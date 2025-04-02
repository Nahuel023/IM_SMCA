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


#include <WiFi.h>
#include <lvgl.h>
#include "ui.h"
#include <Arduino_GFX_Library.h>

/*
 * Este es un "cliente"
 * que se conecta al AP del ESP32S, genera la interfaz
 * gráfica con LVGL, y envía / recibe datos vía TCP.
 */

#define SSID        "ESP32_AP"      // Nombre de red WiFi del ESP32S
#define PASSWORD    "12345678"       // Contraseña de la red
#define SERVER_IP   "192.168.4.1"     // IP del ESP32S en modo AP
#define SERVER_PORT 3333              // Puerto TCP del servidor en el ESP32S
#define TFT_BL      2                 // Control de retroiluminación

/*
 * - "La pantalla es de 800x480, se configuran pines
 *    de la interface RGB (arduino_ESP32RGBPanel)"
 * - "touch.h maneja las rutinas del touch capacitivo (GT911)."
 */

#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);

Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
  bus,
    800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */
);

#endif

#include "touch.h"

static uint32_t screenWidth = 800;
static uint32_t screenHeight = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/*
 * screenWidth / screenHeight
 * Son las dimensiones de la pantalla (800x480).
 * draw_buf y disp_drv se usan en la inicialización de LVGL.
 */

int ValueSetTemperatura = 20;
int ValueSetHumedad = 80;
int ValueSetMerma = 0;
int ValueSetVelAire = 0;
bool ledState = false; // Estado del LED
float temperature = 0.0;

WiFiClient client;

// -- COMENTARIOS DE BLOQUES --

/***************************************
 * Funciones de Display y Touch
 *   - "my_disp_flush": renderiza lo que LVGL dibuja
 *   - "my_touchpad_read": lee el último toque del GT911
 ***************************************/

/***************************************
 * Funciones de Display y Touch
 * - my_disp_flush(...) se llama desde LVGL para redibujar 
 *   la porción de pantalla solicitada.
 * - my_touchpad_read(...) consulta las rutinas de GT911
 *   para obtener la posición X/Y y el estado (tocado/no).
 ***************************************/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch_has_signal()) {
    if (touch_touched()) {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print("Data x ");
      Serial.println(data->point.x);
      Serial.print("Data y ");
      Serial.println(data->point.y);
    } else if (touch_released()) {
      data->state = LV_INDEV_STATE_REL;
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}


/***************************************
 * NUEVAS FUNCIONES DE COMUNICACIÓN
 * - sendCommand(...) = envía un comando genérico al servidor
 * - receiveSensorData(...) = espera "DATA:temp=..,hum=..,peso=.."
 * - checkTCPConnection(...) = reconectar si perdimos el socket
 ***************************************/

void sendCommand(const char* command) {
  if (client.connected()) {
    client.print(command);
    Serial.printf("Comando enviado: %s\n", command);
  } else {
    Serial.println("Conexión TCP perdida. Intentando reconectar...");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      Serial.println("Reconexión TCP exitosa");
      client.print(command);
    } else {
      Serial.println("Error al reconectar TCP");
    }
  }
}

// Función para recibir datos sensados en formato "DATA:temp=XX.XX,hum=YY.YY,peso=ZZZ.ZZ"
void receiveSensorData() {
  if (client.available()) {
    String data = client.readStringUntil('\n');
    // Se espera el formato: DATA:temp=XX.XX,hum=YY.YY,peso=ZZZ.ZZ
    if (data.startsWith("DATA:")) {
      data.remove(0, 5); // Remover "DATA:"
      // Buscamos las posiciones de los parámetros
      int idxTemp = data.indexOf("temp=");
      int idxHum  = data.indexOf("hum=");
      int idxPeso = data.indexOf("peso=");
      if (idxTemp != -1 && idxHum != -1 && idxPeso != -1) {
        // Extraemos los valores
        int comma1 = data.indexOf(',', idxTemp);
        int comma2 = data.indexOf(',', idxHum);
        String tempStr = data.substring(idxTemp + 5, comma1);
        String humStr  = data.substring(idxHum + 4, comma2);
        String pesoStr = data.substring(idxPeso + 5);
        float tempVal = tempStr.toFloat();
        float humVal  = humStr.toFloat();
        float pesoVal = pesoStr.toFloat();
        Serial.printf("Datos recibidos -> Temp: %.2f, Hum: %.2f, Peso: %.2f\n", tempVal, humVal, pesoVal);
        // Actualizar la interfaz (se asume que los nombres de los widgets son los siguientes)
        _ui_label_set_property(ui_LabelTemperatura1, _UI_LABEL_PROPERTY_TEXT, String(tempVal).c_str());
        _ui_label_set_property(ui_ValueHumi, _UI_LABEL_PROPERTY_TEXT, String(humVal).c_str());
        _ui_label_set_property(ui_ValuePeso, _UI_LABEL_PROPERTY_TEXT, String(pesoVal).c_str());
      }
    }
  }
}

// Función para verificar y restablecer la conexión TCP si es necesario
void checkTCPConnection() {
  if (!client.connected()) {
    Serial.println("Conexión TCP perdida, intentando reconectar...");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      Serial.println("Reconexión TCP exitosa");
    } else {
      Serial.println("Error al reconectar TCP");
    }
  }
}

/***************************************
 * Setup
 *  - Se conecta al AP (ESP32_AP)
 *  - Intentar la conexión TCP
 *  - Inicializa la pantalla y LVGL
 *  - Realiza test fillScreen(...) 
 ***************************************/

void setup() {
  Serial.begin(115200);
  Serial.println("LVGL Lab1tech Demo");

  // Configurar el modo WiFi en estación
  WiFi.mode(WIFI_STA); 

  //Configuración de WiFi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conexión WiFi establecida");

  //Intentar conexión TCP
  if (client.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("Conexión TCP establecida");
  } else {
    Serial.println("Error en la conexión TCP");
  }

  // Configuración de pantalla y retroiluminación
  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  ledcSetup(0, 300, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, 255);
#endif

  gfx->fillScreen(RED);
  delay(500);
  gfx->fillScreen(GREEN);
  delay(500);
  gfx->fillScreen(BLUE);
  delay(500);
  gfx->fillScreen(BLACK);
  delay(500);
  
  lv_init();
  pinMode(TOUCH_GT911_RST, OUTPUT);
  digitalWrite(TOUCH_GT911_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_GT911_RST, HIGH);
  delay(10);
  touch_init();

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4);
#endif
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 4);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();
    Serial.println("Setup done");
  }
}


/***************************************
 * Callback del botón "Start/Stop"
 *  - Invocado por el evento LVGL en la UI
 *  - Cambia ledState local
 *  - Envía "arranque"/"parada" al servidor
 ***************************************/

void statusLED(lv_event_t * e)
{
    // Cambiamos el estado ledState
    ledState = !ledState;

    // Cambiar texto
    const char* labelText = ledState ? "STOP" : "START";
    _ui_label_set_property(ui_LabelSTARTSTOP, _UI_LABEL_PROPERTY_TEXT, labelText);

    // Cambiar color del botón según estado
    if (ledState) {
        // Si está en "STOP", color rojo
        lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
        // Enviamos el comando
        sendCommand("arranque");

    } else {
        // Si está en "START", color verde
        lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        // Enviamos el comando
        sendCommand("parada");
    }
}


void BtnMenosTemp(lv_event_t *e) {
  // Decrementar valor de la temperatura
  ValueSetTemperatura--;
  _ui_label_set_property(ui_SetTemperatura,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetTemperatura) + " C").c_str());
}

void BtnMasTemp(lv_event_t *e) {
  // Incrementar valor de la temperatura
  ValueSetTemperatura++;
  _ui_label_set_property(ui_SetTemperatura,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetTemperatura) + " C").c_str());
}

void BtnMenosHum(lv_event_t *e) {
  // Incrementar valor de la humedad
  ValueSetHumedad--;
  _ui_label_set_property(ui_SetHumedad,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetHumedad) + " %").c_str());
}

void BtnMasHum(lv_event_t *e) {
  // Incrementar valor de la humedad
  ValueSetHumedad++;
  _ui_label_set_property(ui_SetHumedad,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetHumedad) + " %").c_str());
}

void BtnMenosMerma(lv_event_t *e) {
  // Incrementar valor de la merma
  ValueSetMerma--;
  _ui_label_set_property(ui_SetMerma,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetMerma) + " %").c_str());
}

void BtnMasMerma(lv_event_t *e) {
  // Incrementar valor de la merma
  ValueSetMerma++;
  _ui_label_set_property(ui_SetMerma,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetMerma) + " %").c_str());
}

void BtnMenosVel(lv_event_t *e) {
  // Incrementar valor de la velocidad del aire
  ValueSetVelAire--;
  _ui_label_set_property(ui_SetVelAire,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetVelAire) + " %").c_str());
}

void BtnMasVel(lv_event_t *e) {
  // Incrementar valor de la velocidad del aire
  ValueSetVelAire++;
  _ui_label_set_property(ui_SetVelAire,_UI_LABEL_PROPERTY_TEXT,(String(ValueSetVelAire) + " %").c_str());
}


void sendAllSetPoints() {

    /*
     * ENVÍA TODOS LOS SETPOINTS (temperatura, humedad, merma, vel aire)
     * SOLO si han cambiado desde la última vez.
     */

    // Variables estáticas para recordar el último valor enviado.
    // Se inicializan con un número improbable para forzar el primer envío.
    static int lastTempSent   = 20;
    static int lastHumSent    = 80;
    static int lastMermaSent  = -99;
    static int lastAireSent   = -99;

    // 1) Asegurar que la temperatura no sea 0 (forzar 10 si es 0)
    if (ValueSetTemperatura == 0) {
        ValueSetTemperatura = 10;
    }

    // 2) Asegurar que la humedad no sea 0 (forzar 10 si es 0)
    if (ValueSetHumedad == 0) {
        ValueSetHumedad = 10;
    }

    // 3) Enviar Temperatura solo si cambió
    if (ValueSetTemperatura != lastTempSent) {
        String command = "set_temperatura:" + String(ValueSetTemperatura);
        sendCommand(command.c_str());
        lastTempSent = ValueSetTemperatura;  // Actualizamos el último valor enviado
    }

    // 4) Enviar Humedad solo si cambió
    if (ValueSetHumedad != lastHumSent) {
        String command = "set_humedad:" + String(ValueSetHumedad);
        sendCommand(command.c_str());
        lastHumSent = ValueSetHumedad;
    }

    // 5) Enviar Merma solo si cambió
    if (ValueSetMerma != lastMermaSent) {
        String command = "set_merma:" + String(ValueSetMerma);
        sendCommand(command.c_str());
        lastMermaSent = ValueSetMerma;
    }

    // 6) Enviar Velocidad de Aire solo si cambió
    if (ValueSetVelAire != lastAireSent) {
        String command = "set_vel_aire:" + String(ValueSetVelAire);
        sendCommand(command.c_str());
        lastAireSent = ValueSetVelAire;
    }
}



/***************************************
 * Loop Principal (se modificó para incluir nuevas funciones de comunicación)
 ***************************************/
void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
  
  checkTCPConnection();    // Verificar la conexión TCP
  receiveSensorData();     // Recibir datos sensados en formato DATA:
  
    // Enviar setpoints cada 10s
  static unsigned long lastSetpointsSend = 0;
  unsigned long now = millis();
  if (now - lastSetpointsSend >= 10000) {
      lastSetpointsSend = now;
      sendAllSetPoints();
  }
}

