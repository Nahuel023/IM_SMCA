#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "relay_example";

/* GPIO definitions */
#define RELAY_RECAMBIO_AIRE 17  // 12V
#define RELAY_FRIO 33  // K4 
#define RELAY_SECADOR 25  // K3
#define RELAY_HUMIDIFICADOR 26  // K2
#define RELAY_GPIO4 27  // K1

/* State variables */
static uint8_t relay_states[5] = {0};  // Estado de cada relé (0 = apagado, 1 = encendido)

/* Function to control the relay */
static void toggle_relay(int relay_num, int gpio_num) {
    relay_states[relay_num] = !relay_states[relay_num];  // Alternar el estado del relé
    gpio_set_level(gpio_num, relay_states[relay_num]);
    ESP_LOGI(TAG, "Relay %d state: %s", relay_num, relay_states[relay_num] ? "ON" : "OFF");
}

/* Configures the relay GPIO */
static void configure_relay(int gpio_num) {
    gpio_reset_pin(gpio_num);
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
}

/* Main control task */
static void relay_task(void *arg) {
    while (1) {
        // Alternar los estados de todos los relés
        toggle_relay(0, RELAY_RECAMBIO_AIRE);
        toggle_relay(1, RELAY_FRIO);
        toggle_relay(2, RELAY_SECADOR);
        toggle_relay(3, RELAY_HUMIDIFICADOR);
        //toggle_relay(4, RELAY_GPIO4);

        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Esperar 5 segundos
    }
}

void app_main(void) {
    /* Configure the relay GPIOs */
    configure_relay(RELAY_RECAMBIO_AIRE);
    configure_relay(RELAY_FRIO);
    configure_relay(RELAY_SECADOR);
    configure_relay(RELAY_HUMIDIFICADOR);
    configure_relay(RELAY_GPIO4);

    /* Create task to toggle the relays every 5 seconds */
    xTaskCreate(relay_task, "relay_task", 2048, NULL, 5, NULL);
}
