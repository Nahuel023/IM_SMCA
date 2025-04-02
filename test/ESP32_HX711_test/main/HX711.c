#include "HX711.h"
#include <stdlib.h>

// Inicializa el driver estableciendo un factor de escala (inicial, luego se calibrará)
void HX711_Init(_sHX711Handle *hx711, float scaleFactor) {
    hx711->taskData.tareValue   = 0;
    hx711->taskData.scaleFactor = scaleFactor;
    hx711->taskData.status      = HX711_OK;
}

// Función para tarear la balanza (promediando 'samples' muestras)
_eHX711Status HX711_Tare(_sHX711Handle *hx711, int samples) {
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        int32_t count = 0;

        // Espera a que DOUT esté en LOW
        while (hx711->ReadPin() == 1) { }

        // Lee 24 bits
        for (int j = 0; j < 24; j++) {
            hx711->WritePin(1);
            hx711->DelayUs(1);
            count = (count << 1) | hx711->ReadPin();
            hx711->WritePin(0);
            hx711->DelayUs(1);
        }

        // Pulso 25 para configurar ganancia=128 (canal A)
        hx711->WritePin(1);
        hx711->DelayUs(1);
        hx711->WritePin(0);

        // Extiende el signo (24 bits a 32 bits)
        if (count & 0x800000) {
            count |= 0xFF000000;
        }
        sum += count;
    }
    hx711->taskData.tareValue = (int32_t)(sum / samples);
    return HX711_OK;
}

// Retorna el peso en gramos (aplicando tare y factor de escala)
float HX711_GetWeight(_sHX711Handle *hx711, int samples) {
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        int32_t count = 0;
        while (hx711->ReadPin() == 1) { }
        for (int j = 0; j < 24; j++) {
            hx711->WritePin(1);
            hx711->DelayUs(1);
            count = (count << 1) | hx711->ReadPin();
            hx711->WritePin(0);
            hx711->DelayUs(1);
        }
        hx711->WritePin(1);
        hx711->DelayUs(1);
        hx711->WritePin(0);
        if (count & 0x800000) {
            count |= 0xFF000000;
        }
        sum += count;
    }
    int32_t rawValue = (int32_t)(sum / samples);
    return (rawValue - hx711->taskData.tareValue) / hx711->taskData.scaleFactor;
}

// Lee una sola muestra cruda (24 bits) y extiende el signo
int32_t HX711_ReadRawOnce(_sHX711Handle *hx711) {
    int32_t count = 0;
    while (hx711->ReadPin() == 1) { }
    for (int j = 0; j < 24; j++) {
        hx711->WritePin(1);
        hx711->DelayUs(1);
        count = (count << 1) | hx711->ReadPin();
        hx711->WritePin(0);
        hx711->DelayUs(1);
    }
    hx711->WritePin(1);
    hx711->DelayUs(1);
    hx711->WritePin(0);
    if (count & 0x800000) {
        count |= 0xFF000000;
    }
    return count;
}

// Promedia 'samples' lecturas crudas
int32_t HX711_ReadAverageRaw(_sHX711Handle *hx711, int samples) {
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += HX711_ReadRawOnce(hx711);
    }
    return (int32_t)(sum / samples);
}

