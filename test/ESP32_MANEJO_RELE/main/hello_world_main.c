#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "relay_example";

/* GPIO definitions */
#define RELAY_GPIO 33  // Pin para controlar el relé

/* State variables */
static uint8_t relay_state = 0;  // Estado del relé (0 = apagado, 1 = encendido)

/* Function to control the relay */
static void toggle_relay(void) {
    relay_state = !relay_state;  // Alternar el estado del relé
    gpio_set_level(RELAY_GPIO, relay_state);
    ESP_LOGI(TAG, "Relay state: %s", relay_state ? "ON" : "OFF");
}

/* Configures the relay GPIO */
static void configure_relay(void) {
    ESP_LOGI(TAG, "Configuring GPIO for relay control");
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
}

/* Main control task */
static void relay_task(void *arg) {
    while (1) {
        toggle_relay();  // Alternar el estado del relé
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Esperar 5 segundos
    }
}

void app_main(void) {
    /* Configure the relay GPIO */
    configure_relay();

    /* Create task to toggle the relay every 5 seconds */
    xTaskCreate(relay_task, "relay_task", 2048, NULL, 5, NULL);
}
