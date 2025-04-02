#ifndef _HX711_H
#define _HX711_H

#include <stdint.h>

// Estados posibles del driver
typedef enum {
    HX711_OK = 0,
    HX711_ERROR = -1,
} _eHX711Status;

// Estructura principal para manejar el HX711
typedef struct {
    // Funciones de hardware que se deben asignar según la plataforma
    void (*SetPinInput)(void);
    void (*SetPinOutput)(void);
    void (*WritePin)(uint8_t value);
    uint8_t (*ReadPin)(void);
    void (*DelayUs)(int usDelay);
    
    // Datos internos
    struct {
        int32_t tareValue;       // Valor obtenido al tarear
        float scaleFactor;       // Factor de escala para convertir a gramos
        _eHX711Status status;    // Estado del driver
    } taskData;
} _sHX711Handle;

// Prototipos de funciones
void HX711_Init(_sHX711Handle *hx711, float scaleFactor);
_eHX711Status HX711_Tare(_sHX711Handle *hx711, int samples);
float HX711_GetWeight(_sHX711Handle *hx711, int samples);

// Funciones auxiliares para lecturas crudas (opcional, para calibración)
int32_t HX711_ReadRawOnce(_sHX711Handle *hx711);
int32_t HX711_ReadAverageRaw(_sHX711Handle *hx711, int samples);

#endif /* _HX711_H */

