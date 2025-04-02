/************************************************************
 * Autor: Nahuel Medina
 * Fecha: 28/03/2025
 *
 * Resumen:
 * Este proyecto consta de dos partes interconectadas:
 *
 * 1. Código para ESP32S3 (Cliente con pantalla y LVGL):
 *    - Se conecta al AP "ESP32_AP" y establece una comunicación TCP
 *      con el servidor (ESP32S) en la IP 192.168.4.1, puerto 3333.
 *    - Utiliza la librería LVGL y Arduino_GFX_Library para generar una 
 *      interfaz gráfica que permite visualizar datos sensados (temperatura, 
 *      humedad y peso) y enviar comandos de configuración (por ejemplo, 
 *      setpoints de temperatura, humedad, merma y velocidad de aire).
 *    - Permite iniciar/parar el sistema mediante un botón que envía comandos
 *      ("arranque" / "parada") al servidor.
 *
 * 2. Código para ESP32S (Servidor de Control):
 *    - Opera en modo Access Point con SSID "ESP32_AP" y levanta un servidor 
 *      TCP en el puerto 3333.
 *    - Lee datos de sensores (AM2320 para temperatura/humedad y opcionalmente 
 *      DS18B20 para temperatura) y envía periódicamente estos datos al cliente.
 *    - Recibe comandos de setpoints y de control (como "arranque", "parada",
 *      "set_temperatura:XX", etc.) para ajustar la lógica de control.
 *    - Controla 5 relés (para recambio de aire, sistema de frío, secador, 
 *      humidificador y resistencia calefactora) aplicando reglas de temporización,
 *      lockouts y condiciones basadas en los valores de sensor y setpoints.
 *
 * Notas Importantes:
 * - La comunicación WiFi se realiza mediante el modo estación en el ESP32S3 y
 *   modo AP en el ESP32S.
 * - Los comandos se envían a través de TCP y se actualizan periódicamente.
 * - La lógica de control incluye intervalos fijos (por ejemplo, recambio de aire 
 *   5 minutos cada 8 horas) y restricciones (como lockout de 10 min para el sistema de frío).
 * - Se utilizan librerías y APIs de ESP-IDF, FreeRTOS, LVGL y otras específicas para 
 *   la lectura de sensores y control de relés.
 *
 * Dependencias:
 * - ESP32 WiFi, FreeRTOS, LWIP, LVGL, Arduino_GFX_Library, ONEWire, DS18B20, driver/i2c.
 *
 * Uso:
 * - Configura los setpoints a través de la interfaz gráfica del ESP32S3.
 * - El servidor ESP32S aplica automáticamente la lógica de control basada en los 
 *   valores de sensor y los comandos recibidos.
 *
 ************************************************************/



/***************************************
 * BLOQUE 1: Librerías, Definiciones Globales y Variables
 ***************************************/
 
 /*
 * Este ESP32 corre en modo AP, crea la red "ESP32_AP",
 * y levanta un servidor TCP en el puerto 3333.
 * 
 * Además, define pines para DS18B20, AM2320 (I2C), y
 * 5 relés (17, 33, 25, 26, 27).
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp32/rom/ets_sys.h"
#include "esp_timer.h"
#include "ONEWire.h"
#include "DS18B20.h"
#include "driver/i2c.h"

#define PORT            3333         // Puerto TCP del servidor
#define BLINK_GPIO      2            // Pin del LED
#define SSID            "ESP32_AP"   // Nombre del AP
#define PASSWORD        "12345678"   // Contraseña del AP
#define DS18B20_PIN     23           // Pin del sensor DS18B20

// Pines y configuración del I2C
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       2000

//Pines de los RELES
#define RELAY_RECAMBIO_AIRE 17
#define RELAY_FRIO          33
#define RELAY_SECADOR       25
#define RELAY_HUMIDIFICADOR 26
#define RELAY_RESISTENCIA   27

// Dirección del sensor AM2320
#define AM2320_SENSOR_ADDR          0x5C

static const char *TAG = "ESP32S_TCP_SERVER";

// Intervalo de lectura para DS18B20 (5 segundos)
uint32_t ds18b20_read_interval_us = 5000000; 
_sOWHandle ow0;
_sDS18B20Handle ds18b20_0;
int16_t tempDS18B20;

// Intervalo de lectura para AM2320 (2 segundos)
uint32_t am2320_read_interval_ms = 5000;     // 5 segundos
// Buffer para recibir datos del AM2320
uint8_t _buf[8];

// Variables de test para otros sensores
float sensorTemperature = 0;  
float sensorHumidity = 0;  
float sensorWeight   = 0; 

// Variables para comandos recibidos
bool systemRunning = false;
float setTemperature = 0.0;
float setHumidity    = 0.0;
float setAirSpeed    = 0.0;
float setMerma 	     = 0.0;

// Variable para indicar si un cliente TCP está conectado
bool tcpClientConnected = false;

/***************************************
 * BLOQUE 2: Funciones para DS18B20 y Placeholders
 ***************************************/
 
 /*
 * - set_ds18b20_read_interval(...) te permite cambiar 
 *   el intervalo de lectura en runtime.
 * - OneWireSetPinInput/Output(...) = callbacks para 
 *   manipular el pin DS18B20_PIN.
 * - DS18B20_Task(...) se llama en ds18b20_task y obtiene
 *   la temperatura en la variable tempDS18B20.
 */


void OneWireSetPinInput(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
}
void OneWireSetPinOutput(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
}
uint8_t OneWireReadPin(void) {
    return gpio_get_level(DS18B20_PIN);
}
void OneWireWritePin(uint8_t value) {
    gpio_set_level(DS18B20_PIN, value);
}
static inline int delayBlocking(int time_us) {
    ets_delay_us(time_us);
    return 1;
}
void set_ds18b20_read_interval(uint32_t interval_sec) {
    ds18b20_read_interval_us = interval_sec * 1000000;
}

/***********************************
 * FUNCIONES PARA EL AM2320
 ***********************************/


/**
 * @brief Función para inicializar el bus I2C en modo maestro.
 * 
 * @return esp_err_t ESP_OK si la configuración es exitosa, de lo contrario un error de ESP-IDF.
 */
static esp_err_t set_i2c(void) {
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
    return ESP_OK;
}

/**
 * @brief Función para despertar el sensor AM2320 antes de realizar una lectura.
 */
static void wake_sensor() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(800);  // Retraso de 800 microsegundos para despertar el sensor
}

/**
 * @brief Función para calcular el CRC16 de los datos recibidos del sensor AM2320.
 */
static uint16_t crc16(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Función para leer los registros del sensor AM2320.
 */
static esp_err_t read_registers(uint8_t reg, uint8_t len) {
    esp_err_t ret;

    // Despertar el sensor
    wake_sensor();

    // Comando de lectura
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true); // Función de lectura (0x03)
    i2c_master_write_byte(cmd, reg, true);  // Dirección del registro inicial
    i2c_master_write_byte(cmd, len, true);  // Número de bytes a leer
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    ets_delay_us(1500);  // Retraso de 1.5 ms según las especificaciones del sensor

    // Leer los datos del sensor
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, _buf, len + 4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // Verificar el CRC
    uint16_t received_crc = (_buf[len + 3] << 8) | _buf[len + 2];
    if (crc16(_buf, len + 2) != received_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

/**
 * @brief Función para obtener los valores de temperatura y humedad del sensor AM2320.
 */
static esp_err_t get_am2320_data(float *temperature, float *humidity) {
    esp_err_t ret = read_registers(0x00, 4); // Leer 4 bytes desde el registro 0x00
    if (ret != ESP_OK) {
        return ret;
    }

    // Convertir la humedad y la temperatura
    uint16_t hum_raw = (_buf[2] << 8) | _buf[3];
    uint16_t temp_raw = (_buf[4] << 8) | _buf[5];

    *humidity = hum_raw / 10.0;
    *temperature = ((temp_raw & 0x7FFF) / 10.0) * ((temp_raw & 0x8000) ? -1 : 1);

    return ESP_OK;
}

/**
 * @brief Ajusta el intervalo de lectura del sensor AM2320.
 */
void set_am2320_read_interval(uint32_t interval_ms) {
    am2320_read_interval_ms = interval_ms;
}

/***************************************
 * BLOQUE 3: Funciones de Conexión WiFi y TCP (Servidor)
 ***************************************/
 
 /*
 * wifi_init_ap():
 *   - Inicializa la interfaz WiFi en modo AP
 *   - Configura SSID y password
 *
 * sendSensorData():
 *   - Envía un string al cliente (DATA:temp=..,hum=..,peso=..)
 */


void wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .password = PASSWORD,
            .ssid_len = strlen(SSID),
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "ESP32S en modo AP. SSID:%s Contraseña:%s", SSID, PASSWORD);
}

// Función para enviar datos sensados al cliente
void sendSensorData(int client_sock) {
    char sensorMsg[128];
    // Se envía con el formato: DATA:temp=XX.XX,hum=YY.YY,peso=ZZZ.ZZ\n
    snprintf(sensorMsg, sizeof(sensorMsg), "DATA:temp=%.2f,hum=%.2f,peso=%.2f\n", sensorTemperature, sensorHumidity, sensorWeight);
    send(client_sock, sensorMsg, strlen(sensorMsg), 0);
}

static void configure_relay(int gpio_num) {
    gpio_reset_pin(gpio_num);
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num, 0); // Apagado inicial
}

static void setRelay(int gpio, bool on)
{
    gpio_set_level(gpio, on ? 1 : 0);
}

/***************************************
 * BLOQUE 4: Tarea TCP Server
 ***************************************/
 
 /*
 * tcp_server_task:
 *   - Crea un socket en listen, 
 *   - accept() un cliente,
 *   - Recibe comandos como "arranque", "parada", 
 *     "set_temperatura:", etc. 
 *   - Cada 10s, envía datos sensados con sendSensorData(...).
 *
 * Nota: Usa un timeout de 200ms en recv() para no bloquear.
 * 
 * Si se desconecta el cliente, vuelve a accept() uno nuevo.
 */


void tcp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // Crear socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error creando socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Llenar estructura de dirección
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    // bind
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Error en bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // listen
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error en listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Servidor TCP esperando conexiones en el puerto %d", PORT);

    while (1) {
        // Aceptar conexión entrante
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Error en accept: errno %d", errno);
            continue;
        }
        tcpClientConnected = true;
        ESP_LOGI(TAG, "Cliente conectado");

        // 1) Configurar un timeout de 200ms (0.2s) para recv(), para no bloquear indefinidamente
        struct timeval timeout;
        timeout.tv_sec = 0;           // 0 segundos
        timeout.tv_usec = 200000;     // 200000 microsegundos = 200ms
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        // 2) Variable para llevar el tiempo de envío periódico
        int64_t lastSendTime = esp_timer_get_time(); // tiempo actual en microsegundos

        while (1) {
            // Intentar recibir datos
            int len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (len < 0) {
                // Si es timeout, simplemente no hay datos en este momento
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // No llegó nada, se ignora
                } else {
                    // Error real de socket
                    ESP_LOGE(TAG, "Error en recv: errno %d", errno);
                    break;
                }
            } else if (len == 0) {
                // Cero bytes significa que el cliente cerró la conexión
                ESP_LOGI(TAG, "Cliente desconectado");
                break;
            } else {
                // Llegaron datos válidos
                rx_buffer[len] = '\0'; // Termina la cadena
                ESP_LOGI(TAG, "Recibido: %s", rx_buffer);

				// Procesamiento de comandos
				if (strcmp(rx_buffer, "arranque") == 0) {
				    systemRunning = true;
				    ESP_LOGI(TAG, "Comando arranque recibido");
				} else if (strcmp(rx_buffer, "parada") == 0) {
				    systemRunning = false;
				    ESP_LOGI(TAG, "Comando parada recibido");
				} else if (strncmp(rx_buffer, "set_temperatura:", 16) == 0) {
				    setTemperature = atof(rx_buffer + 16);
				    ESP_LOGI(TAG, "Set Temperatura: %.2f", setTemperature);
				} else if (strncmp(rx_buffer, "set_humedad:", 12) == 0) {
				    setHumidity = atof(rx_buffer + 12);
				    ESP_LOGI(TAG, "Set Humedad: %.2f", setHumidity);
				} else if (strncmp(rx_buffer, "set_merma:", 10) == 0) {
				    setMerma = atof(rx_buffer + 10);         // <--- NUEVO CAMPO
				    ESP_LOGI(TAG, "Set Merma: %.2f", setMerma);
				} else if (strncmp(rx_buffer, "set_vel_aire:", 13) == 0) {
				    setAirSpeed = atof(rx_buffer + 13);
				    ESP_LOGI(TAG, "Set Velocidad de Aire: %.2f", setAirSpeed);
				} else if (strcmp(rx_buffer, "GET_DATA") == 0) {
				    // Si el cliente lo pide explícitamente, enviamos en ese momento
				    sendSensorData(client_sock);
				} else {
				    // Si se recibe otro comando desconocido, puedes ignorarlo o responder
				    ESP_LOGW(TAG, "Comando desconocido, se ignora o se responde con DATA");
				    // sendSensorData(client_sock);
				}

            }

            // 3) Verificar si ya pasaron 10 segundos para enviar datos
            int64_t now = esp_timer_get_time(); // microsegundos
            if ((now - lastSendTime) >= 10LL * 1000000LL) {
                sendSensorData(client_sock);
                lastSendTime = now;
            }

            // TAREA IMPORTANTE:
            // Podríamos ceder el CPU brevemente para que otras tareas corran
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Cerrar el socket y marcar desconexión
        close(client_sock);
        tcpClientConnected = false;
        ESP_LOGI(TAG, "Socket cerrado, esperando siguiente cliente...");
    }

    close(listen_sock);
    vTaskDelete(NULL);
}


/***************************************
 * BLOQUE 5: Tarea de Lectura del DS18B20
 ***************************************/
 
 /*
 * ds18b20_task:
 *   - Usa DS18B20_Task(...) en un bucle
 *   - Llama DS18B20_StartReadTemp(...) cada X seg
 *   - Actualiza tempDS18B20 al tener un dato correcto
 *   - Ejemplo de lectura final en logs: "DS18B20 Temperature: 25.0000 C"
 */


void ds18b20_task(void *pvParameter) {

    // Configuración del DS18B20
    ow0.DELAYus = &delayBlocking;
    ow0.SETPinInput = &OneWireSetPinInput;
    ow0.SETPinOutput = &OneWireSetPinOutput;
    ow0.ReadPinBit = &OneWireReadPin;
    ow0.WritePinBit = &OneWireWritePin;

    ds18b20_0.OW = &ow0;
    DS18B20_Init(&ds18b20_0, NULL);

    uint32_t lastReadTime = esp_timer_get_time();
    uint32_t currentTime_us;
    const TickType_t delayAfterTempRead = pdMS_TO_TICKS(3000); // 3 segundos de espera
    bool measurementActive = false;

    while (1) {
        currentTime_us = esp_timer_get_time();

        // Si la bandera está activa, realizar la medición
        if (measurementActive) {
            DS18B20_Task(&ds18b20_0, currentTime_us);

            // Verificar el estado y leer la temperatura si está lista
            _eDS18B20Status state = DS18B20_Status(&ds18b20_0);
            if (state == DS18B20_ST_TEMPOK) {
                tempDS18B20 = DS18B20_ReadLastTemp(&ds18b20_0);
                ESP_LOGI(TAG, "DS18B20 Temperature: %d.%04d C", tempDS18B20 / 16, (tempDS18B20 % 16) * 625);

                // Desactivar la bandera y esperar 3 segundos antes de la siguiente medición
                measurementActive = false;
                vTaskDelay(delayAfterTempRead);
                lastReadTime = esp_timer_get_time();
            } else if (state == DS18B20_ST_TEMPCRCERROR) {
                ESP_LOGE(TAG, "DS18B20 CRC Error reading temperature!");
            }

            // Ceder el procesador sin una demora fija
            taskYIELD();
        } else {
            // Verificar si ha pasado el intervalo configurado para iniciar una nueva medición
            if ((currentTime_us - lastReadTime) >= ds18b20_read_interval_us) {
                if (DS18B20_Status(&ds18b20_0) == DS18B20_ST_IDLE) {
                    DS18B20_StartReadTemp(&ds18b20_0);
                    ESP_LOGI(TAG, "DS18B20 START READ TEMP");
                    measurementActive = true;
                } else {
                    ESP_LOGW(TAG, "DS18B20 no está en estado IDLE. Estado actual: %d", DS18B20_Status(&ds18b20_0));
                }
            }

            // Si no es tiempo de iniciar una nueva medición, esperar un poco antes de continuar
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }
}

/***************************************
 * BLOQUE 5.1: Tarea de Lectura del AM2320
 ***************************************/
/**
 * @brief Tarea FreeRTOS para manejar la lectura de temperatura y humedad del sensor AM2320.
 */
void am2320_task(void *pvParameter) {
    float temperature, humidity;

    while (1) {
        esp_err_t ret = get_am2320_data(&temperature, &humidity);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "AM2320 Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
            sensorHumidity = humidity;
            sensorTemperature = temperature;
        } else {
            ESP_LOGE(TAG, "AM2320 Failed to read sensor data: %s", esp_err_to_name(ret));
        }
        vTaskDelay(am2320_read_interval_ms / portTICK_PERIOD_MS);
    }
}

/***************************************
 * Tarea de Control del Sistema
 ***************************************/
 
 /*
 * control_task:
 *   - Maneja la lógica de cada relé según:
 *     (1) systemRunning (arranque/parada)
 *     (2) sensorTemperature y sensorHumidity
 *     (3) setTemperature, setHumidity, setAirSpeed, setMerma
 *   - Aplica tiempos de lockout y duraciones de encendido
 *   - Actualiza LED en BLINK_GPIO si systemRunning == true
 */


void control_task(void *arg)
{
    // Frecuencia de chequeo: cada 10 segundo (ajusta a tu gusto)
    const TickType_t controlDelay = pdMS_TO_TICKS(5000);

    /*
     * Usamos variables estáticas para rastrear estados y temporizadores.
     * El tiempo lo medimos en milisegundos o microsegundos con `esp_timer_get_time()`
     * (retorna µs) o con `xTaskGetTickCount()` en ticks. Aquí usamos microsegundos.
     */
    static bool recambioOn = false;
    static int64_t recambioCycleStart = 0; // arranque de un ciclo de 40 min (ON+OFF)

    static bool frioOn = false;
    static int64_t frioLastOffTime = 0; // momento en que se apagó

    static bool secadorOn = false;
    static int64_t secadorLastOffTime = 0; // momento en que se apagó

    static bool humidificadorOn = false;
    static int64_t humidificadorNextCheck = 0; // cuando volver a chequear
    static int64_t humidificadorOnStart = 0;   // cuando se encendió

    static bool resistenciaOn = false;
    static int64_t resistenciaOnStart = 0; // cuando se encendió
    static int64_t resistenciaNextCheck = 0; // cuando volver a chequear tras OFF

    // Inicialmente, arranca el ciclo de recambio
    recambioCycleStart = esp_timer_get_time();

    while (1)
    {
        // Si el sistema está apagado, todo OFF y resetea estados
        if (!systemRunning) {
            setRelay(RELAY_RECAMBIO_AIRE, false);
            setRelay(RELAY_FRIO, false);
            setRelay(RELAY_SECADOR, false);
            setRelay(RELAY_HUMIDIFICADOR, false);
            setRelay(RELAY_RESISTENCIA, false);

            recambioOn = false;
            frioOn = false;
            secadorOn = false;
            humidificadorOn = false;
            resistenciaOn = false;

            // Esperar 1s y seguir chequeando
            vTaskDelay(controlDelay);
            continue;
        }

        int64_t nowUs = esp_timer_get_time(); // microsegundos actuales
        
         if (systemRunning) {
            // Si el sistema está en modo "RUN"
            gpio_set_level(BLINK_GPIO, 1); // Enciende LED
        } else {
            // Si el sistema está en "STOP"
            gpio_set_level(BLINK_GPIO, 0); // Apaga LED
        }

        /*
         * 1) RECAMBIO DE AIRE
         *   - Ciclo fijo: 5 min ON, 35 min OFF = 40min total
         */
        {
            const int64_t cycleLengthUs = 8LL * 60LL * 60LL * 1000000LL; // 8 horas en microsegundos
    		const int64_t onDurationUs  = 5LL * 60LL * 1000000LL;        // 5 minutos en microsegundos
    
            int64_t elapsed = nowUs - recambioCycleStart;

            if (elapsed >= cycleLengthUs) {
                // Se terminó el ciclo; reiniciamos
                recambioCycleStart = nowUs;
                elapsed = 0;
            }

            // ON los primeros 5 min, OFF el resto
            bool shouldBeOn = (elapsed < onDurationUs);
            if (shouldBeOn != recambioOn) {
                recambioOn = shouldBeOn;
                setRelay(RELAY_RECAMBIO_AIRE, recambioOn);
            }
        }

        /*
         * 2) FRÍO
         *   - ON si temp >= setTemp+3
         *   - OFF si temp <= setTemp
         *   - 10 min lockout después de OFF
         */
        {
            const int64_t lockoutUs = 10LL * 60LL * 1000000LL; // 10 min en µs

            // Si está encendido, chequeamos si se debe apagar
            if (frioOn) {
                if (sensorTemperature <= setTemperature) {
                    // Apagar
                    frioOn = false;
                    setRelay(RELAY_FRIO, false);
                    frioLastOffTime = nowUs;
                    ESP_LOGI(TAG, "FRIO OFF");
                }
            } else {
                // Está off, ver si se enciende
                // Chequear si ya pasó el lockout
                if ((nowUs - frioLastOffTime) >= lockoutUs) {
                    if (sensorTemperature >= (setTemperature + 3.0f)) {
                        // Encender
                        frioOn = true;
                        setRelay(RELAY_FRIO, true);
                        ESP_LOGI(TAG, "FRIO ON");
                    }
                }
            }
        }

        /*
         * 3) SECADOR
         *   - ON si hum >= setHum+5
         *   - OFF si hum <= setHum
         *   - 10 min lockout tras OFF
         */
        {
            const int64_t lockoutUs = 10LL * 60LL * 1000000LL; // 10 min en µs

            if (secadorOn) {
                // ¿Apagar?
                if (sensorHumidity <= setHumidity) {
                    secadorOn = false;
                    setRelay(RELAY_SECADOR, false);
                    secadorLastOffTime = nowUs;
                    ESP_LOGI(TAG, "SECADOR OFF");
                }
            } else {
                // Está off, ver si se enciende
                if ((nowUs - secadorLastOffTime) >= lockoutUs) {
                    if (sensorHumidity >= (setHumidity + 5.0f)) {
                        secadorOn = true;
                        setRelay(RELAY_SECADOR, true);
                        ESP_LOGI(TAG, "SECADOR ON");
                    }
                }
            }
        }

        /*
         * 4) HUMIDIFICADOR
         *   - ON 10s si hum <= setHum-5
         *   - Al encender, contamos 10s, luego OFF
         *   - Luego, esperar 2 min hasta re-chequear
         *   - Sin tiempo mínimo adicional de lockout, salvo esos 2 min
         */
        {
            const int64_t onTimeUs = 5LL * 60LL * 1000000LL; // 5min
            const int64_t offDelayUs = 2LL * 60LL * 1000000LL; // 2 min

            if (!humidificadorOn) {
                // Está apagado, verificar si ya podemos re-chequear
                if (nowUs >= humidificadorNextCheck) {
                    // Analizar si se enciende
                    if (sensorHumidity <= (setHumidity - 5.0f)) {
                        // Encender 10s
                        humidificadorOn = true;
                        humidificadorOnStart = nowUs;
                        setRelay(RELAY_HUMIDIFICADOR, true);
                        ESP_LOGI(TAG, "HUMIDIFICADOR ON");
                    }
                }
            } else {
                // Está encendido; ver si cumplió 10s o si la humedad ya superó setHumidity
                bool timeExpired = (nowUs - humidificadorOnStart) >= onTimeUs;
                bool reachedSetpoint = (sensorHumidity >= setHumidity);

                if (timeExpired || reachedSetpoint) {
                    // Apagar y programar el próximo chequeo dentro de 2 min
                    humidificadorOn = false;
                    setRelay(RELAY_HUMIDIFICADOR, false);
                    ESP_LOGI(TAG, "HUMIDIFICADOR OFF");

                    humidificadorNextCheck = nowUs + offDelayUs; 
                }
            }
        }

        /*
         * 5) RESISTENCIA
         *   - ON si temp <= setTemp - 3
         *   - Permanece ON 30s (salvo que supere setTemp)
         *   - Después OFF, esperar 3 min para volver a evaluar
         */
        {
            const int64_t onTimeUs = 30LL * 1000000LL;         // 30s
            const int64_t postOffDelayUs = 3LL * 60LL * 1000000LL; // 3 min

            // Si estamos esperando la siguiente comprobación
            if (!resistenciaOn) {
                // Revisar si ya podemos encender
                if (nowUs >= resistenciaNextCheck) {
                    // Chequear si la temp está 3° debajo
                    if (sensorTemperature <= (setTemperature - 3.0f)) {
                        // Encender
                        resistenciaOn = true;
                        resistenciaOnStart = nowUs;
                        setRelay(RELAY_RESISTENCIA, true);
                        ESP_LOGI(TAG, "RESISTENCIA ON");
                    }
                }
            } else {
                // Está encendida
                bool timeExpired = ((nowUs - resistenciaOnStart) >= onTimeUs);
                bool exceedSetpoint = (sensorTemperature >= setTemperature);

                if (timeExpired || exceedSetpoint) {
                    // Apagar
                    resistenciaOn = false;
                    setRelay(RELAY_RESISTENCIA, false);
                    ESP_LOGI(TAG, "RESISTENCIA OFF");
                    // Agendamos la próxima revisión en 3 min
                    resistenciaNextCheck = nowUs + postOffDelayUs;
                }
            }
        }

        // Esperar 1s antes de la próxima iteración
        vTaskDelay(controlDelay);
    }
}



/***************************************
 * BLOQUE 6: Función Principal app_main
 ***************************************/
 
 /*
 * app_main:
 *   - Inicia NVS, WiFi en modo AP, I2C, etc.
 *   - Configura pines de relés 
 *   - Crea las tareas:
 *       1) tcp_server_task -> Comandos y envío de datos
 *       2) am2320_task -> Lee Temp/Hum AM2320
 *       3) ds18b20_task -> (desactivada por error de watchdog) Lee Temp DS18B20
 *       4) control_task -> Lógica de relés
 */

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_ap();
    
    ESP_ERROR_CHECK(set_i2c());
    
    /* Configurar pines de relés */
    configure_relay(RELAY_RECAMBIO_AIRE);
    configure_relay(RELAY_FRIO);
    configure_relay(RELAY_SECADOR);
    configure_relay(RELAY_HUMIDIFICADOR);
    configure_relay(RELAY_RESISTENCIA);

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 4, NULL);
    //xTaskCreate(&ds18b20_task, "DS18B20 Task", 2048, NULL, 5, NULL);
	xTaskCreate(&am2320_task, "AM2320 Task", 2048, NULL, 2, NULL);
	xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);

}
