#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "cJSON.h"

#define WIFI_SSID       "ANM2"
#define WIFI_PASS       "anm157523"
#define SERVER_URL      "http://192.168.0.15:8080/api/data"
#define BLINK_GPIO      2
#define BLINK_PERIOD_MS 1000  // 1 segundo

#define WIFI_CONNECTED_BIT BIT0
#define IP_ASSIGNED_BIT    BIT1

static const char *TAG = "ESP32";
static EventGroupHandle_t s_wifi_event_group;

// Prototipos de funciones
void wifi_init_sta(void);
void blink_task(void *pvParameter);
void send_data_task(void *pvParameter);
void app_main(void);

// Manejador de eventos WiFi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Conexión WiFi iniciada");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Desconectado del WiFi, intentando reconectar...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ip4_addr_t ip_addr;
        ip_addr.addr = event->ip_info.ip.addr;
        ESP_LOGI(TAG, "Conectado con IP: %s", ip4addr_ntoa(&ip_addr));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT);
    }
}

// Función para inicializar WiFi en modo estación (STA)
void wifi_init_sta(void)
{
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
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Tarea de parpadeo del LED
void blink_task(void *pvParameter)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(BLINK_PERIOD_MS / portTICK_PERIOD_MS);

        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

// Tarea de envío de datos
void send_data_task(void *pvParameter)
{
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, 
                                               WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT, 
                                               pdFALSE, pdTRUE, portMAX_DELAY);

        if ((bits & (WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT)) == (WIFI_CONNECTED_BIT | IP_ASSIGNED_BIT)) {
            ESP_LOGI(TAG, "Conexión WiFi y asignación de IP verificadas");

            int max_retries = 5;
            int retry_count = 0;
            int delay_ms = 5000;
            bool success = false;

            while (retry_count < max_retries && !success) {
                const char *proceso_actual = "Maduracion";
                bool sistema_frigorifico_frio = (rand() % 2) == 1;
                bool sistema_frigorifico_secado = (rand() % 2) == 1;
                bool humificador = (rand() % 2) == 1;
                bool ventiladores = (rand() % 2) == 1;
                int velocidad_ventiladores = rand() % 101;
                bool resistencia_calefactora = (rand() % 2) == 1;
                bool switch_puerta_abierta = (rand() % 2) == 1;
                double celda_carga = (double)(rand() % 1000) / 10.0;
                double temperatura_control = 25.0;
                double humedad_control = 50.0;

                int cantidad_modulos = 3;
                cJSON *modulos_array = cJSON_CreateArray();
                
                for (int i = 0; i < cantidad_modulos; i++) {
                    cJSON *modulo = cJSON_CreateObject();
                    char module_id[10];
                    snprintf(module_id, sizeof(module_id), "mod_%03d", i + 1);
                    cJSON_AddStringToObject(modulo, "module_id", module_id);
                    cJSON_AddNumberToObject(modulo, "sensor_temperatura", (float)(rand() % 100) / 2.0);
                    cJSON_AddNumberToObject(modulo, "sensor_humedad", (float)(rand() % 100));
                    cJSON_AddNumberToObject(modulo, "carga_bateria", rand() % 101);
                    cJSON_AddItemToArray(modulos_array, modulo);
                }

                cJSON *root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "proceso_actual", proceso_actual);
                cJSON_AddBoolToObject(root, "sistema_frigorifico_frio", sistema_frigorifico_frio);
                cJSON_AddBoolToObject(root, "sistema_frigorifico_secado", sistema_frigorifico_secado);
                cJSON_AddBoolToObject(root, "humificador", humificador);
                cJSON_AddBoolToObject(root, "ventiladores", ventiladores);
                cJSON_AddNumberToObject(root, "velocidad_ventiladores", velocidad_ventiladores);
                cJSON_AddBoolToObject(root, "resistencia_calefactora", resistencia_calefactora);
                cJSON_AddBoolToObject(root, "switch_puerta_abierta", switch_puerta_abierta);
                cJSON_AddNumberToObject(root, "celda_carga", celda_carga);
                cJSON_AddNumberToObject(root, "temperatura_control", temperatura_control);
                cJSON_AddNumberToObject(root, "humedad_control", humedad_control);
                cJSON_AddItemToObject(root, "modulos_sensado", modulos_array);

                char *json_str = cJSON_Print(root);
                if (json_str == NULL) {
                    ESP_LOGE(TAG, "Error al convertir JSON a string");
                    cJSON_Delete(root);
                    vTaskDelay(30000 / portTICK_PERIOD_MS);
                    continue;
                }

                int json_len = strlen(json_str);
                ESP_LOGI(TAG, "Datos JSON creados: %s", json_str);

                esp_http_client_config_t config = {
                    .url = SERVER_URL,
                    .method = HTTP_METHOD_POST,
                    .timeout_ms = 15000,
                };
                esp_http_client_handle_t client = esp_http_client_init(&config);

                esp_http_client_set_header(client, "Content-Type", "application/json");
                esp_http_client_set_post_field(client, json_str, json_len);

                esp_err_t err = esp_http_client_perform(client);
                if (err == ESP_OK) {
                    int http_status = esp_http_client_get_status_code(client);
                    ESP_LOGI(TAG, "Datos enviados con éxito: %s", json_str);
                    ESP_LOGI(TAG, "Código de respuesta del servidor: %d", http_status);
                    success = true;
                } else {
                    ESP_LOGE(TAG, "Error al enviar datos: %s. Intento %d de %d", esp_err_to_name(err), retry_count + 1, max_retries);
                    retry_count++;
                    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
                    delay_ms *= 2;
                }

                free(json_str);
                cJSON_Delete(root);
                esp_http_client_cleanup(client);
            }

            if (!success) {
                ESP_LOGE(TAG, "No se pudo establecer conexión con el servidor después de %d intentos", max_retries);
            }
        } else {
            ESP_LOGE(TAG, "Error en la conexión WiFi o asignación de IP");
        }

        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

// Función principal
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "NVS inicializado correctamente");

    wifi_init_sta();

    ESP_LOGI(TAG, "WiFi inicializado, esperando conexión...");

    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tarea de parpadeo del LED iniciada");

    xTaskCreate(send_data_task, "send_data_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tarea de envío de datos iniciada con mayor memoria asignada");
}


