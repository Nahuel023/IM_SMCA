#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "cJSON.h"

// Credenciales WiFi
#define WIFI_SSID "ANM2"
#define WIFI_PASS "anm157523"

// Configuración de ThingsBoard
#define THINGSBOARD_SERVER "mqtt://thingsboard.cloud:1883"
#define THINGSBOARD_TOKEN "uvxZ4yb9zBkWkjoFoS24"

// Tópico para telemetría (ThingsBoard requiere enviar los datos en este tópico)
#define TELEMETRY_TOPIC "v1/devices/me/telemetry"

static const char *TAG = "THINGSBOARD";

// Manejador global del cliente MQTT
esp_mqtt_client_handle_t mqtt_client = NULL;

/*-----------------------------------------------------------
  Función de manejo de eventos WiFi
-----------------------------------------------------------*/
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if(event_base == WIFI_EVENT) {
        switch(event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "WiFi desconectado. Reintentando conexión...");
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if(event_base == IP_EVENT) {
        if(event_id == IP_EVENT_STA_GOT_IP) {
            ESP_LOGI(TAG, "WiFi conectada, IP asignada.");
        }
    }
}

/*-----------------------------------------------------------
  Inicialización de WiFi
-----------------------------------------------------------*/
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Inicialización de WiFi completada.");
}

/*-----------------------------------------------------------
  Función de manejo de eventos MQTT
-----------------------------------------------------------*/
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t) event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado al broker MQTT");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Desconectado del broker MQTT");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Suscrito, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "Desuscrito, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Publicado, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Datos recibidos en el tópico %.*s: %.*s",
                     event->topic_len, event->topic, event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error en MQTT");
            break;
        default:
            break;
    }
}

/*-----------------------------------------------------------
  Inicialización de MQTT para ThingsBoard con la nueva estructura
-----------------------------------------------------------*/
void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = THINGSBOARD_SERVER,
            },
        },
        .credentials = {
            .username = THINGSBOARD_TOKEN,         // El token se utiliza como username en ThingsBoard
            .client_id = "ESP32_THINGSBOARD",        // Puedes definir un ID específico o dejarlo NULL
            .set_null_client_id = false,
            .authentication = {
                .password = "",                      // La contraseña se deja vacía para ThingsBoard
            },
        },
        .session = {
            .keepalive = 60,                         // Intervalo de keepalive en segundos
            .disable_clean_session = false,          // Mantiene la sesión limpia (true si deseas reconexión limpia)
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

/*-----------------------------------------------------------
  Tarea FreeRTOS para simular y enviar telemetría (temperatura y humedad)
-----------------------------------------------------------*/
void telemetry_task(void *pvParameters)
{
    while (1) {
        // Generar datos ficticios:
        // Temperatura simulada entre 20.0 y 30.0 °C
        float temperature = 25.0f + ((rand() % 1000) / 100.0f) - 5.0f;
        // Humedad simulada entre 47.5 y 52.5 %
        float humidity = 50.0f + ((rand() % 500) / 100.0f) - 2.5f;

        // Crear objeto JSON
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "temperature", temperature);
        cJSON_AddNumberToObject(root, "humidity", humidity);
        char *json_str = cJSON_PrintUnformatted(root);

        ESP_LOGI(TAG, "Publicando telemetría: %s", json_str);

        // Publicar mensaje en el tópico de telemetría
        int msg_id = esp_mqtt_client_publish(mqtt_client, TELEMETRY_TOPIC, json_str, 0, 1, 0);
        ESP_LOGI(TAG, "Mensaje publicado, msg_id=%d", msg_id);

        cJSON_Delete(root);
        free(json_str);

        // Espera 5 segundos entre publicaciones
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/*-----------------------------------------------------------
  Función principal de la aplicación
-----------------------------------------------------------*/
void app_main(void)
{
    // Inicialización de NVS (requerido para WiFi)
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializa WiFi y MQTT
    wifi_init();
    mqtt_app_start();

    // Crea la tarea de telemetría
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}
