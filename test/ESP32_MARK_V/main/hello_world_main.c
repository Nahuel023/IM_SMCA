#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/i2c.h"

// Configuración WiFi
#define SSID "FCAL-Personal"
#define PASSWORD "fcal-uner+2019"

// Pines para LED y motores
#define BLINK_GPIO 2
#define ENA 27
#define ENB 14
#define IN1 26
#define IN2 25
#define IN3 32
#define IN4 33

// PWM para motores
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_RIGHT LEDC_CHANNEL_0
#define LEDC_CHANNEL_LEFT LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 100 // 100 Hz

// I2C Configuración MPU6050
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// Bits de conexión y asignación IP
#define WIFI_CONNECTED_BIT BIT0
#define IP_ASSIGNED_BIT BIT1

static const char *TAG = "ESP32";
static EventGroupHandle_t s_wifi_event_group;
static bool balance_mode = false;

// Variables de compensación MPU6050
float accel_x_offset = 0.04;
float accel_y_offset = 0.04;
float accel_z_offset = 1.15;
float gyro_x_offset = -2.93;
float gyro_y_offset = 1.19;
float gyro_z_offset = 1.28;

// Variables PID y filtro
double kp_pitch = 5, ki_pitch = 0, kd_pitch = 0.9;
double kp_yaw = 0.5, ki_yaw = 0, kd_yaw = 0.5;
double pitchAngle, yawRate, pitchPIDOutput, yawPIDOutput;
double setpointPitchAngle = 0;
double setpointYawRate = 0;
double previousPitchAngle = 0;
#define ANGLE_CHANGE_THRESHOLD 2.0 // Umbral del filtro

// Prototipos de funciones
void wifi_init_sta(void);
esp_err_t root_handler(httpd_req_t *req);
esp_err_t control_handler(httpd_req_t *req);
esp_err_t data_handler(httpd_req_t *req);
esp_err_t start_webserver(void);
void MotorControl(int setMotorRight, int setMotorLeft);
void i2c_init(void);
void calibrate_mpu6050(void);
void read_mpu6050(void *arg);
void balance_control_task(void *arg);

// Manejador de eventos WiFi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Conexión WiFi iniciada");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Desconectado del WiFi, intentando reconectar...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip[16];
        sprintf(ip, IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Conectado con IP: %s", ip);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT);
    }
}

// Inicialización WiFi
void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Inicialización PWM
void pwm_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_right = {
        .gpio_num = ENA,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RIGHT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right));

    ledc_channel_config_t ledc_channel_left = {
        .gpio_num = ENB,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_LEFT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left));
}

// Inicialización I2C y despertar MPU6050
void i2c_init(void) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));

    // Despertar el MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true); // Activar el sensor
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
}

// Calibración del MPU6050
void calibrate_mpu6050(void) {
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
        uint8_t data[14];
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)));
        i2c_cmd_link_delete(cmd);

        int16_t raw_ax = (int16_t)(data[0] << 8 | data[1]);
        int16_t raw_ay = (int16_t)(data[2] << 8 | data[3]);
        int16_t raw_az = (int16_t)(data[4] << 8 | data[5]);
        int16_t raw_gx = (int16_t)(data[8] << 8 | data[9]);
        int16_t raw_gy = (int16_t)(data[10] << 8 | data[11]);
        int16_t raw_gz = (int16_t)(data[12] << 8 | data[13]);

        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;
        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    accel_x_offset = sum_ax / (float)num_samples / 16384.0;
    accel_y_offset = sum_ay / (float)num_samples / 16384.0;
    accel_z_offset = sum_az / (float)num_samples / 16384.0;
    gyro_x_offset = sum_gx / (float)num_samples / 131.0;
    gyro_y_offset = sum_gy / (float)num_samples / 131.0;
    gyro_z_offset = sum_gz / (float)num_samples / 131.0;

    //ESP_LOGI(TAG, "Calibración completa. Offsets:");
    //ESP_LOGI(TAG, "Accel X: %.2f, Y: %.2f, Z: %.2f", accel_x_offset, accel_y_offset, accel_z_offset);
    //ESP_LOGI(TAG, "Gyro X: %.2f, Y: %.2f, Z: %.2f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

// Lectura del MPU6050
void read_mpu6050(void *arg) {
    uint8_t data[14];
    int16_t raw_ax, raw_az, raw_gz;

    while (1) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)));
        i2c_cmd_link_delete(cmd);

        raw_ax = (int16_t)(data[0] << 8 | data[1]);
        raw_az = (int16_t)(data[4] << 8 | data[5]);
        raw_gz = (int16_t)(data[12] << 8 | data[13]);

        double newPitchAngle = atan2(raw_ax / 16384.0 - accel_x_offset, raw_az / 16384.0 - accel_z_offset) * 180 / M_PI;

        if (fabs(newPitchAngle - previousPitchAngle) < ANGLE_CHANGE_THRESHOLD) {
            pitchAngle = newPitchAngle;
        }
        previousPitchAngle = pitchAngle;

        yawRate = (raw_gz / 131.0) - gyro_z_offset;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Control de balance
void balance_control_task(void *arg) {
    while (1) {
        if (balance_mode) {
            double pitchError = setpointPitchAngle - pitchAngle;
            pitchPIDOutput = kp_pitch * pitchError;

            double yawError = setpointYawRate - yawRate;
            yawPIDOutput = kp_yaw * yawError;

            int controlSignalRight = (int)(pitchPIDOutput + yawPIDOutput);
            int controlSignalLeft = (int)(pitchPIDOutput - yawPIDOutput);

            // Ajustar señales de control para asegurar que activen motores
            if (abs(controlSignalRight) < 200) controlSignalRight = (controlSignalRight > 0) ? 200 : -200;
            if (abs(controlSignalLeft) < 200) controlSignalLeft = (controlSignalLeft > 0) ? 200 : -200;

            MotorControl(controlSignalRight*10, controlSignalLeft*10);
        } else {
            MotorControl(0, 0); // Apagar motores si balance_mode está desactivado
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Controlador de motores
void MotorControl(int setMotorRight, int setMotorLeft) {
    uint32_t auxSetMotor;

    auxSetMotor = abs(setMotorRight);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, auxSetMotor);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
    gpio_set_level(IN1, setMotorRight > 0);
    gpio_set_level(IN2, setMotorRight < 0);

    auxSetMotor = abs(setMotorLeft);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, auxSetMotor);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
    gpio_set_level(IN3, setMotorLeft > 0);
    gpio_set_level(IN4, setMotorLeft < 0);
}

// Página de control web
esp_err_t control_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    buf = malloc(buf_len);

    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
        sscanf(buf, "kp_pitch=%lf&ki_pitch=%lf&kd_pitch=%lf&kp_yaw=%lf&ki_yaw=%lf&kd_yaw=%lf&setpointPitchAngle=%lf",
               &kp_pitch, &ki_pitch, &kd_pitch, &kp_yaw, &ki_yaw, &kd_yaw, &setpointPitchAngle);

        if (strstr(buf, "balance_mode=on")) {
            balance_mode = true;
        } else if (strstr(buf, "balance_mode=off")) {
            balance_mode = false;
        }
    }
    free(buf);

    char response[1024];
    snprintf(response, sizeof(response),
             "<!DOCTYPE html><html><body>"
             "<h1>Configuración de PID y Control de Balance</h1>"
             "<form action=\"/control\" method=\"get\">"
             "<h2>Parámetros PID</h2>"
             "Kp Pitch: <input type=\"text\" name=\"kp_pitch\" value=\"%.2f\"><br>"
             "Ki Pitch: <input type=\"text\" name=\"ki_pitch\" value=\"%.2f\"><br>"
             "Kd Pitch: <input type=\"text\" name=\"kd_pitch\" value=\"%.2f\"><br>"
             "Kp Yaw: <input type=\"text\" name=\"kp_yaw\" value=\"%.2f\"><br>"
             "Ki Yaw: <input type=\"text\" name=\"ki_yaw\" value=\"%.2f\"><br>"
             "Kd Yaw: <input type=\"text\" name=\"kd_yaw\" value=\"%.2f\"><br>"
             "<h2>Setpoint Pitch Angle</h2>"
             "<input type=\"text\" name=\"setpointPitchAngle\" value=\"%.2f\"><br>"
             "<input type=\"submit\" value=\"Actualizar PID\">"
             "</form>"
             "<form action=\"/control\" method=\"get\">"
             "<h2>Control de Balance</h2>"
             "<button type=\"submit\" name=\"balance_mode\" value=\"on\">Activar Balance</button>"
             "<button type=\"submit\" name=\"balance_mode\" value=\"off\">Desactivar Balance</button>"
             "</form>"
             "</body></html>", kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, setpointPitchAngle);
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

// Página de datos del sensor
esp_err_t data_handler(httpd_req_t *req) {
    char response[500];
    snprintf(response, sizeof(response),
             "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"1\"></head><body>"
             "<h1>Datos del Sensor MPU6050</h1>"
             "<p>Accel X Offset: %.2f</p><p>Accel Y Offset: %.2f</p><p>Accel Z Offset: %.2f</p>"
             "<p>Gyro X Offset: %.2f</p><p>Gyro Y Offset: %.2f</p><p>Gyro Z Offset: %.2f</p>"
             "</body></html>", accel_x_offset, accel_y_offset, accel_z_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset);
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// Página raíz
esp_err_t root_handler(httpd_req_t *req) {
    const char* resp_str = "<!DOCTYPE html><html><body>"
                           "<h1>Bienvenido al servidor ESP32</h1>"
                           "<p>Visite <a href=\"/control\">Configuración de PID y Control de Balance</a> o <a href=\"/data\">Datos del Sensor</a>.</p>"
                           "</body></html>";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

// Configuración del servidor web
esp_err_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t control_uri = {
            .uri       = "/control",
            .method    = HTTP_GET,
            .handler   = control_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &control_uri);

        httpd_uri_t data_uri = {
            .uri       = "/data",
            .method    = HTTP_GET,
            .handler   = data_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &data_uri);

        ESP_LOGI(TAG, "Servidor web iniciado");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Error al iniciar servidor web");
    return ESP_FAIL;
}

// Función principal
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "NVS inicializado correctamente");

    // Configuración del GPIO para el LED
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Configuración del GPIO de control de motores
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    // Inicializar PWM e I2C
    pwm_init();
    i2c_init();

    // Inicializar WiFi y servidor web
    wifi_init_sta();
    start_webserver();
    ESP_LOGI(TAG, "Servidor web de control iniciado");

    // Crear tareas de lectura de MPU6050 y balance
    xTaskCreate(read_mpu6050, "read_mpu6050", 2048, NULL, 5, NULL);
    xTaskCreate(balance_control_task, "balance_control_task", 2048, NULL, 5, NULL);
}

