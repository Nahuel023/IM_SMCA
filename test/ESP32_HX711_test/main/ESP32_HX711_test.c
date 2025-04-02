#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "rom/ets_sys.h"
#include "HX711.h"

static const char *TAG = "HX711_debug";

// Definición de pines para el HX711
#define HX711_DOUT_PIN GPIO_NUM_19
#define HX711_SCK_PIN  GPIO_NUM_18

// Instancia global del driver HX711
_sHX711Handle hx711;

/* Funciones de configuración de pines y temporización */

// Configura el pin DOUT como entrada
void SetHX711PinInput(void) {
    gpio_set_direction(HX711_DOUT_PIN, GPIO_MODE_INPUT);
}

// (No es realmente utilizada, pero se incluye para completar la interfaz)
void SetHX711PinOutput(void) {
    gpio_set_direction(HX711_DOUT_PIN, GPIO_MODE_OUTPUT);
}

// Lee el nivel lógico del pin DOUT
uint8_t ReadHX711Pin(void) {
    return gpio_get_level(HX711_DOUT_PIN);
}

// Escribe en el pin SCK
void WriteHX711Pin(uint8_t value) {
    gpio_set_level(HX711_SCK_PIN, value);
}

// Retardo en microsegundos
void HX711DelayUs(int usDelay) {
    ets_delay_us(usDelay);
}

void app_main(void)
{
    // Configurar los pines de la ESP32 para el HX711
    gpio_set_direction(HX711_DOUT_PIN, GPIO_MODE_INPUT);   // DOUT
    gpio_set_direction(HX711_SCK_PIN,  GPIO_MODE_OUTPUT);    // SCK

    // Asignar las funciones al handle
    hx711.SetPinInput  = SetHX711PinInput;
    hx711.SetPinOutput = SetHX711PinOutput;
    hx711.WritePin     = WriteHX711Pin;
    hx711.ReadPin      = ReadHX711Pin;
    hx711.DelayUs      = HX711DelayUs;

    // Inicializa el HX711 con un factor de escala provisional (se calibrará)
    HX711_Init(&hx711, 1.0f);

    // Realiza el tareo sin peso
    ESP_LOGI(TAG, "Realizando tareo sin peso...");
    HX711_Tare(&hx711, 100); // Promedia 50 muestras
    ESP_LOGI(TAG, "Tarea completada. Valor de tare = %ld", (long)hx711.taskData.tareValue);

    // --- Procedimiento de calibración ---
    // Define el peso conocido en gramos para calibrar (por ejemplo, 500 g)
    const int knownWeight = 385;
    ESP_LOGI(TAG, "Coloque un peso conocido de %d gramos y espere 5 segundos...", knownWeight);
    vTaskDelay(pdMS_TO_TICKS(5000));  // Espera 5 segundos para que se coloque el peso

    // Toma un promedio de lecturas crudas con el peso colocado
    int32_t rawWithWeight = HX711_ReadAverageRaw(&hx711, 10);
    int32_t diff = rawWithWeight - hx711.taskData.tareValue;
    // Calcula el nuevo factor de escala: (raw - tare) / peso_conocido
    float newScaleFactor = diff / (float)knownWeight;
    ESP_LOGI(TAG, "Lectura con peso: %ld, Diff = %ld", (long)rawWithWeight, (long)diff);
    ESP_LOGI(TAG, "Nuevo factor de escala calculado = %f", newScaleFactor);

    // Actualiza el factor de escala en el driver
    hx711.taskData.scaleFactor = newScaleFactor;

    // Pide retirar el peso conocido
    ESP_LOGI(TAG, "Retire el peso conocido y espere 5 segundos para continuar...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    // --- Fin de calibración ---

    // Ciclo principal: lectura y muestra del peso calibrado
    while (1) {
        float weight = HX711_GetWeight(&hx711, 10);
        ESP_LOGI(TAG, "Peso = %.2f g", weight);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


