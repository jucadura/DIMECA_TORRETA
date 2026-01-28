# DIMECA_TORRETA

## Descripción general

**DIMECA_TORRETA** es un sistema de vigilancia y seguimiento visual basado en **visión por computadora** e **inteligencia artificial**, implementado sobre la **ESP32-P4 AI Vision Board**. El proyecto integra detección de personas en tiempo real con un mecanismo físico **pan–tilt** accionado por un **motor paso a paso (pan)** y un **servomotor (tilt)**, permitiendo orientar dinámicamente la cámara hacia el objetivo detectado.

El sistema está diseñado para operar de forma autónoma: explora el entorno cuando no hay detecciones y, al identificar una persona, detiene el barrido, valida la detección y centra el objetivo dentro del campo visual. Adicionalmente, incorpora finales de carrera para protección mecánica y un control por **MOSFET** que habilita o deshabilita un motor DC externo según el estado del seguimiento.

---

## Plataforma de hardware

- **Microcontrolador:** ESP32-P4 AI Vision Board
- **Cámara:** Compatible con el pipeline de visión del ESP32-P4 (OV5647 / UVC según configuración)
- **Actuadores:**
  - Motor paso a paso (eje X – pan)
  - Servomotor (eje Y – tilt)
- **Sensores:**
  - Dos finales de carrera (izquierdo y derecho, tipo NO a 3.3 V)
- **Etapa de potencia:**
  - MOSFET controlado por GPIO para motor DC externo

---

## Arquitectura del sistema

El proyecto se estructura en tres bloques principales:

1. **Visión por computadora**
   - Captura de video en tiempo real.
   - Inferencia de detección de personas mediante un modelo de IA.
   - Obtención de *bounding boxes* (cajas delimitadoras) y métricas asociadas.

2. **Control y lógica de seguimiento**
   - Selección del objetivo principal (mayor área y score válido).
   - Cálculo del error entre el centro del *bounding box* y el centro de la imagen.
   - Estrategias de *lock*, histéresis y zonas muertas para evitar vibraciones.

3. **Actuación física y seguridad**
   - Control del stepper (pan) y del servo (tilt).
   - Gestión de finales de carrera para evitar colisiones mecánicas.
   - Activación del MOSFET únicamente cuando el objetivo está centrado y estable.

---

## Comportamiento del sistema

- **Modo exploración:**
  - El stepper realiza un barrido horizontal.
  - El servo puede ejecutar un movimiento lento vertical para ampliar el campo de búsqueda.

- **Modo detección:**
  - Al detectar una persona válida, el sistema **detiene el movimiento**.
  - Se valida que la detección sea estable y no un falso positivo.

- **Modo seguimiento:**
  - El stepper corrige el eje X.
  - El servo corrige el eje Y.
  - Ambos buscan alinear el centro del *bounding box* con el centro de la imagen.

- **Modo bloqueo (lock):**
  - Si el objetivo permanece centrado durante un tiempo definido, los actuadores quedan quietos.
  - El MOSFET se activa únicamente en este estado estable.

- **Seguridad mecánica:**
  - Si se activa un final de carrera, el sistema invierte la dirección y se aleja del límite.

---

## Estructura del proyecto

```text
DIMECA_TORRETA/
├── main/
│   ├── turret_control.c
│   ├── turret_control.h
│   └── ...
├── components/
│   ├── pedestrian_detect/
│   └── ...
├── CMakeLists.txt
├── sdkconfig
├── sdkconfig.defaults
├── partitions.csv (si aplica)
└── README.md
```

---

## Configuración y compilación

1. Configurar el target correcto:
   ```bash
   idf.py set-target esp32p4
   ```

2. Configurar el proyecto (opcional):
   ```bash
   idf.py menuconfig
   ```

3. Compilar y flashear:
   ```bash
   idf.py build
   idf.py flash monitor
   ```

---

## Notas importantes

- El proyecto está optimizado para la **ESP32-P4 AI Vision Board**; no se garantiza compatibilidad directa con otros targets.
- El uso de PSRAM es crítico para el procesamiento de imágenes y modelos de IA.
- Se recomienda limitar la frecuencia de inferencia para evitar bloqueos del servidor de video.

---

## Estado del proyecto

Proyecto funcional en fase de integración final, con enfoque en:
- Robustez del seguimiento visual.
- Estabilidad mecánica.
- Reducción de falsas detecciones.

---

## Autoría

Proyecto desarrollado como parte de un sistema experimental de vigilancia y control basado en visión por computadora sobre plataformas embebidas.

