#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/ets_sys.h"

static const char *TAG = "example";

/* GPIO definitions */
#define BLINK_GPIO 2             // GPIO for the test LED
#define ZERO_CROSS_GPIO 35       // GPIO for the zero-cross detector (P620), input only
#define TRIAC_GPIO 13           // GPIO for triggering the TRIAC (MOC3021), output

/* Timer for controlling triac firing delay */
esp_timer_handle_t triac_timer;

/* Semaphore to signal zero-cross detection */
static SemaphoreHandle_t zero_cross_sem;

/* State variables */
static uint8_t s_led_state = 0;
static volatile bool zero_cross_detected = false;
static uint32_t triac_delay_us = 500;  // Delay for controlling the TRIAC timing

/* Frequency of the AC power supply (change if necessary) */
#define FREQ_HZ 50  // Change to 60 if using 60Hz power supply

/* Function to calculate the delay time based on the angle */
uint32_t calculate_triac_delay(uint32_t angle_degrees) {
    // Período de un semiciclo en microsegundos
    uint32_t semi_period_us = (1000000 / FREQ_HZ) / 2;

    // Calcular el tiempo de retardo basado en el ángulo
    uint32_t delay_us = (semi_period_us * angle_degrees) / 180;

    return delay_us;
}

/* Function to control LED for testing */
static void blink_led(void) {
    gpio_set_level(BLINK_GPIO, s_led_state);
}

/* Configures the blink LED */
static void configure_led(void) {
    ESP_LOGI(TAG, "Configuring GPIO LED for blinking");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

/* Function called when zero-cross detected */
static void IRAM_ATTR zero_cross_isr_handler(void *arg) {
    // Signal that zero-cross was detected (quick, ISR-safe)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(zero_cross_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();  // Yield to higher priority task immediately
    }
}

/* Timer callback to trigger the triac */
static void triac_trigger(void *arg) {
    // Trigger the TRIAC by turning on the GPIO for MOC3021
    gpio_set_level(TRIAC_GPIO, 1);
    ets_delay_us(500);  // Hold for 100 microseconds to ensure triac firing
    gpio_set_level(TRIAC_GPIO, 0);
    ESP_LOGI(TAG, "TRIAC triggered after delay");
}

/* Configures the GPIOs for zero-cross detection and TRIAC control */
static void configure_gpio(void) {
    // Configure zero-cross detector input (GPIO35 is input only)
    gpio_reset_pin(ZERO_CROSS_GPIO);
    gpio_set_direction(ZERO_CROSS_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(ZERO_CROSS_GPIO, GPIO_INTR_POSEDGE);  // Trigger on rising edge (crosses zero)
    
    // Configure TRIAC GPIO output
    gpio_reset_pin(TRIAC_GPIO);
    gpio_set_direction(TRIAC_GPIO, GPIO_MODE_OUTPUT);

    // Install ISR handler for zero-cross detection
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ZERO_CROSS_GPIO, zero_cross_isr_handler, NULL);
}

/* Initialize timer for TRIAC control */
static void configure_timer(void) {
    const esp_timer_create_args_t triac_timer_args = {
        .callback = &triac_trigger,
        .name = "triac_timer"
    };

    esp_err_t err = esp_timer_create(&triac_timer_args, &triac_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TRIAC timer: %s", esp_err_to_name(err));
        triac_timer = NULL;  // Make sure the timer is NULL if initialization failed
    }
}

/* Main control loop */
static void control_task(void *arg) {
    while (1) {
        // Wait for zero-cross signal
        if (xSemaphoreTake(zero_cross_sem, portMAX_DELAY) == pdTRUE) {
            // Zero-cross detected, start timer to fire TRIAC after the delay
          
            
            // The timer controls the firing of the TRIAC after a specific delay (phase control)
            if (triac_timer != NULL) {
                esp_timer_start_once(triac_timer, triac_delay_us);  // Delay in microseconds
            } else {
                ESP_LOGE(TAG, "TRIAC timer not initialized!");
            }
        }
    }
}

void app_main(void) {
    // Create semaphore to signal zero-cross detection
    zero_cross_sem = xSemaphoreCreateBinary();

    // Configurar el ángulo de disparo (por ejemplo, 90 grados)
    uint32_t angle = 80;  // Cambia el ángulo según lo necesites
    triac_delay_us = calculate_triac_delay(angle);  // Calcular el tiempo de retardo correspondiente

    /* Configure the blink LED */
    configure_led();
    
    /* Configure zero-cross detection and TRIAC control */
    configure_gpio();

    /* Configure the timer for controlling TRIAC firing */
    configure_timer();

    /* Create task to handle zero-cross and TRIAC control */
    xTaskCreate(control_task, "control_task", 2048, NULL, 10, NULL);

    // Ciclo principal del LED de prueba
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Parpadeo cada 1 segundo
    }
}

