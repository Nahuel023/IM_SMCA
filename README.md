# Sistema Modular de Control Ambiental (SMCA) 🚀❄️

Este repositorio contiene el desarrollo completo del proyecto final de Ingeniería Mecatrónica de Nahuel Medina, titulado **Sistema Modular de Control Ambiental (SMCA) para Cámaras Frigoríficas**. Este sistema está diseñado para optimizar las condiciones ambientales, como temperatura y humedad, en entornos industriales, específicamente en cámaras frigoríficas de secado.

## 📘 Descripción del Proyecto
El **SMCA** es un sistema modular y adaptable que combina:
- **🔧 Sensado inalámbrico** de variables ambientales.
- **🖥️ Módulo de control central** para la gestión de actuadores y procesamiento de datos.
- **📊 Interfaz de usuario (HMI)** intuitiva para monitoreo y ajustes en tiempo real, accesible de forma local y remota.

El proyecto aborda problemáticas comunes en la industria de cámaras frigoríficas, como la falta de monitoreo en tiempo real y la necesidad de intervenciones manuales, proponiendo una solución eficiente y escalable.

## ⭐ Características Clave
- **🧠 Microcontrolador ESP32 con FreeRTOS**: Gestión eficiente de tareas en tiempo real, con capacidad dual-core y Wi-Fi/Bluetooth integrado.
- **🌡️ Sensores de alta precisión**: Como el DS18B20 (±0.5°C) y AM2320 (±2% HR), seleccionados por su exactitud en los rangos críticos de operación.
- **⚡ Módulo de potencia ON/OFF y control de potencia modulada**: Utilizando relés de hasta 2500W y TRIACs BT137-600 y BT138-600 para asegurar el funcionamiento de actuadores clave.
- **🌐 Monitoreo y control remoto**: Integración con IoT y comunicación inalámbrica, permitiendo gestión desde cualquier lugar.

## 🎯 Objetivos Específicos
1. 🛠️ Desarrollar y validar módulos de sensado inalámbrico.
2. 🖧 Implementar un módulo de control central para procesar datos y gestionar actuadores.
3. 💻 Diseñar una interfaz de usuario accesible local y remotamente.
4. 🔬 Probar y validar el sistema en entornos industriales reales.

## 🗂️ Estructura del Repositorio
- **/src**: Código fuente del firmware del ESP32.
- **/docs**: Documentación técnica y diagramas.
- **/hardware**: Diseños de PCB y esquemas eléctricos.
- **/tests**: Pruebas y validación del sistema.

## 🔍 Detalles Técnicos
- **ESP32**: Microcontrolador con dual-core a 160-240 MHz, 520 KB de RAM, y conectividad Wi-Fi.
- **FreeRTOS**: Sistema operativo en tiempo real utilizado para la gestión eficiente de tareas.
- **Sensores**: 
  - **DS18B20**: Sensor digital con rango de -55°C a +125°C y precisión de ±0.5°C.
  - **AM2320**: Sensor de humedad con rango de 0-100% HR y precisión de ±2% HR.
- **Módulos de Potencia**: Relés de 2500W para control ON/OFF y TRIACs para modulación de potencia.

## ✅ Conclusiones
El SMCA demuestra ser una solución viable para mejorar la eficiencia y la conservación de productos en cámaras frigoríficas, reduciendo costos operativos y mejorando la calidad del producto final. 🏆

---
Este proyecto se encuentra bajo una licencia Creative Commons **Atribución-NoComercial 4.0 Internacional**.
