#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"

#define PORT 30010
#define SERVER_IP "172.22.241.88"  // IP of your ESP32S3 with the screen
#define BLINK_GPIO 2

static const char *TAG = "TCP_CLIENT";
static int number = 0;

/* Blink Task */
void blink_task(void *pvParameter) {
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);  // Turn on LED
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);  // Turn off LED
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/* Initialize WiFi */
void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            //.ssid = "ANM2",            // Your WiFi SSID
            //.password = "anm157523",   // Your WiFi Password
            //.ssid = "FCAL-Personal",            // Your WiFi SSID
            //.password = "fcal-uner+2019",   // Your WiFi Password
            .ssid = "SMCA",            // Your WiFi SSID
            .password = "12345678",   // Your WiFi Password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Check connection
    for (int i = 0; i < 10; i++) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to WiFi SSID: %s", ap_info.ssid);
            break;
        } else {
            ESP_LOGW(TAG, "Waiting for WiFi connection...");
        }
    }
}

/* TCP Client Task */
void tcp_client_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    while (1) {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Error creating socket: errno %d", errno);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Connecting to server...");
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Error connecting: errno %d", errno);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Connected to server.");
        while (1) {
            char rx_buffer[128];
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                ESP_LOGE(TAG, "Error receiving data: errno %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed.");
                break;
            } else {
                rx_buffer[len] = 0;  // Null-terminate the buffer
                ESP_LOGI(TAG, "Received: %s", rx_buffer);

                // Convert received data to integer
                number = atoi(rx_buffer);
                ESP_LOGI(TAG, "Number received: %d", number);
            }
        }
        close(sock);
        ESP_LOGI(TAG, "Disconnected. Reconnecting...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Retry after 5 seconds
    }
}

void app_main(void) {
    // Initialize NVS and WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();  // Initialize WiFi

    // Create tasks for blinking LED and TCP client
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(tcp_client_task, "tcp_client_task", 4096, NULL, 5, NULL);
}
