/**
 * ============================================================================
 *  CONTROL DE CO2 EN CÁMARA VOLUMÉTRICA — LAZO PID CON GEMELO DIGITAL
 * ----------------------------------------------------------------------------
 *  Plataforma : Arduino Uno R1
 *  Sensor     : Sensirion SGP30 (I2C: A4 = SDA, A5 = SCL)
 *               Nota: el SGP30 entrega eCO2 (CO2 equivalente estimado a partir
 *               de H2/TVOC), no CO2 real por NDIR. Piso de medición: 400 ppm.
 *  Actuador   : Ventilador extractor 12 V DC 40x40 mm, módulo MOSFET D4184,
 *               señal PWM en D9 (~490 Hz).
 *  Interfaz   : Pulsador en D2 (INPUT_PULLUP, cierra a GND)
 *               LED de estado en D7 (resistencia serie 220 Ω)
 * ----------------------------------------------------------------------------
 *  Arquitectura de software (módulos):
 *    1. Configuración ........ pines, parámetros del PID y constantes.
 *    2. Estado del sistema ... variables globales y máquina de estados.
 *    3. Adquisición .......... lectura del SGP30 con detección de fallos.
 *    4. Controlador PID ...... derivada sobre la medición, anti-windup por
 *                              integración condicional, compensación de la
 *                              zona muerta del ventilador.
 *    5. Interfaz de usuario .. antirrebote del pulsador y patrones del LED.
 *    6. Telemetría ........... Serial Plotter o CSV para la pasarela Python
 *                              hacia Ubidots (gemelo digital).
 *    7. Comandos serie ....... cambio de setpoint y modo manual desde el PC.
 * ----------------------------------------------------------------------------
 *  Comandos por el puerto serie (terminados en salto de línea):
 *    SP:850    -> cambia el setpoint a 850 ppm (aceptado entre 600 y 1000)
 *    MAN:120   -> modo MANUAL: ventilador fijo en PWM 120 (lazo abierto)
 *    MAN:OFF   -> sale del modo manual y vuelve a STANDBY
 *  El Arduino responde con una línea "OK ..." o "ERR ...".
 *  El PID sigue funcionando aunque el PC se desconecte (RNF-01).
 * ----------------------------------------------------------------------------
 *  Máquina de estados:
 *
 *    [inicio] --> PRECALENTAMIENTO --(15 muestras)--> STANDBY
 *                                                     |    ^
 *                                            (botón)  v    |  (botón)
 *                                                    PID_ACTIVO
 *
 *    STANDBY / PID_ACTIVO --(MAN:xxx)--> MANUAL --(MAN:OFF o botón)--> STANDBY
 *
 *    Cualquier estado --(5 lecturas fallidas)--> FALLA_SENSOR (ventilador al 100 %)
 *    FALLA_SENSOR --(lectura válida)--> estado anterior a la falla
 *
 *  Patrones del LED:
 *    PRECALENTAMIENTO : parpadeo lento (1 Hz)
 *    STANDBY          : apagado
 *    PID_ACTIVO       : encendido fijo
 *    FALLA_SENSOR     : parpadeo rápido (~3 Hz)
 *    MANUAL           : encendido con un apagón corto cada segundo
 *
 *  Filtro de la medición: mediana de las 3 últimas lecturas (rechaza picos
 *  aislados del SGP30 sin introducir más de ~1 s de retardo).
 * ============================================================================
 */

#include <Wire.h>
#include "Adafruit_SGP30.h"

// ============================================================================
// 1. CONFIGURACIÓN
// ============================================================================

// --- Pines ---
const uint8_t PIN_PWM_VENTILADOR = 9;   // Señal TRIG/PWM del módulo MOSFET D4184
const uint8_t PIN_LED_ESTADO     = 7;   // LED indicador de estado
const uint8_t PIN_BOTON          = 2;   // Pulsador Marcha/Reposo

// --- Comunicación serie ---
const unsigned long BAUDIOS = 9600;

// Formato de salida serie:
//   1 = "Etiqueta:valor" para el Serial Plotter del Arduino IDE
//   0 = CSV con marca de tiempo y términos P/I/D (pasarela Python -> Ubidots)
#define SALIDA_PLOTTER 0

// --- Muestreo ---
// El algoritmo interno del SGP30 exige llamar a IAQmeasure() cada 1 s.
const unsigned long T_MUESTREO_MS = 1000;
const float DT = T_MUESTREO_MS / 1000.0;          // periodo de muestreo [s]

// --- Precalentamiento ---
// Durante los primeros ~15 s el SGP30 devuelve valores fijos (400 ppm / 0 ppb).
const uint8_t MUESTRAS_PRECALENTAMIENTO = 15;

// --- Controlador PID (síntesis directa, planta FOPTD con tau ≈ 17.5 s) ---
// Unidades: error en ppm, salida en cuentas de PWM (0-255).
const float SETPOINT_INICIAL = 900.0;  // > piso post-ensayo (~650), < 1000 (anteproyecto)
const float SETPOINT_MIN = 600.0;      // límites aceptados por el comando SP:
const float SETPOINT_MAX = 1000.0;     // (RD-02: setpoints respaldados en ASHRAE/OMS)
const float KP = 0.35;     // [PWM / ppm]
const float KI = 0.025;    // [PWM / (ppm·s)]
const float KD = 0.10;     // [PWM·s / ppm]

// --- Actuador ---
const float PWM_MAX = 255.0;
// PWM mínimo al que el ventilador arranca y gira de forma sostenida.
// Medido con prueba_hardware.ino: arranque desde detenido en PWM = 50.
// Se usa arranque + 5 de margen.
const float PWM_MIN = 55.0;

// --- Seguridad ---
const uint8_t MAX_FALLOS_LECTURA = 5;             // fallos consecutivos tolerados
const unsigned long REINTENTO_SENSOR_MS = 5000;   // reintento de sgp.begin()

// --- Interfaz de usuario ---
const unsigned long DEBOUNCE_MS        = 50;
const unsigned long PARPADEO_PRECAL_MS = 500;
const unsigned long PARPADEO_FALLA_MS  = 150;

// ============================================================================
// 2. ESTADO DEL SISTEMA
// ============================================================================

enum Estado : uint8_t {
  PRECALENTAMIENTO = 0,
  STANDBY          = 1,
  PID_ACTIVO       = 2,
  FALLA_SENSOR     = 3,
  MANUAL           = 4
};

Adafruit_SGP30 sgp;

Estado estado             = PRECALENTAMIENTO;
Estado estadoAntesDeFalla = PRECALENTAMIENTO;
bool sensorInicializado   = false;

// Controlador
float setpoint          = SETPOINT_INICIAL;  // [ppm] modificable con SP:xxx
int   pwmManual         = 0;     // PWM fijo del modo MANUAL [0-255]
float termI             = 0.0;   // contribución integral acumulada [PWM]
float terminoP          = 0.0;   // último término proporcional [PWM]
float terminoD          = 0.0;   // último término derivativo [PWM]
float medicionPrevia    = 0.0;   // eCO2 de la muestra anterior [ppm]
bool  hayMedicionPrevia = false;
int   pwmSalida         = 0;     // PWM aplicado al ventilador [0-255]

// Adquisición
float   co2Actual          = 0.0;   // [ppm] filtrado (mediana de 3)
float   co2Crudo           = 0.0;   // [ppm] lectura directa del SGP30
float   bufferCO2[3]       = {0, 0, 0};
uint8_t nBuffer            = 0;     // lecturas válidas en el buffer (0-3)
uint8_t idxBuffer          = 0;
float   tvocActual         = 0.0;   // [ppb]
uint8_t fallosConsecutivos = 0;
uint8_t muestrasPrecal     = 0;

// Comandos serie
char    bufferCmd[16];
uint8_t lenCmd = 0;

// Temporización
unsigned long tUltimaMuestra   = 0;
unsigned long tUltimoReintento = 0;

// --- Prototipos ---
void cambiarEstado(Estado nuevo);
void entrarEnFalla();
void cicloMuestreo();
bool leerSensor();
int  calcularPID(float medicion);
void reiniciarPID();
void aplicarPWM(int valor);
void gestionarBoton();
void actualizarLed();
void enviarTelemetria();
float filtroMediana(float x);
void leerComandos();
void procesarComando(char *cmd);

// ============================================================================
// PROGRAMA PRINCIPAL
// ============================================================================

void setup() {
  Serial.begin(BAUDIOS);

  pinMode(PIN_PWM_VENTILADOR, OUTPUT);
  pinMode(PIN_LED_ESTADO, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  aplicarPWM(0);
  digitalWrite(PIN_LED_ESTADO, LOW);

#if !SALIDA_PLOTTER
  Serial.println(F("t_ms,co2_ppm,tvoc_ppb,setpoint_ppm,pwm,P,I,D,estado,co2_crudo_ppm"));
#endif

  sensorInicializado = sgp.begin();
#if defined(WIRE_HAS_TIMEOUT)
  // Si se desconecta el sensor, el bus I2C no se queda bloqueado esperando:
  // tras 25 ms la lectura falla y actúa el modo seguro (FALLA_SENSOR).
  Wire.setWireTimeout(25000, true);
#endif
  if (sensorInicializado) {
    cambiarEstado(PRECALENTAMIENTO);
  } else {
    Serial.println(F("ERR: SGP30 no detectado. Revisa A4/A5 y la alimentacion."));
    entrarEnFalla();
  }

  tUltimaMuestra   = millis();
  tUltimoReintento = millis();
}

void loop() {
  leerComandos();
  gestionarBoton();
  actualizarLed();

  if (millis() - tUltimaMuestra >= T_MUESTREO_MS) {
    tUltimaMuestra += T_MUESTREO_MS;   // periodo fijo, sin deriva acumulada
    cicloMuestreo();
  }
}

// ============================================================================
// MÁQUINA DE ESTADOS
// ============================================================================

/**
 * Cambia el estado del sistema y ejecuta las acciones de entrada del estado.
 * @param nuevo  Estado destino.
 */
void cambiarEstado(Estado nuevo) {
  if (nuevo == PID_ACTIVO && estado != PID_ACTIVO) {
    reiniciarPID();                    // arranque sin acumulación previa
  }
  if (nuevo == PRECALENTAMIENTO) {
    muestrasPrecal = 0;
  }
  if (nuevo == STANDBY || nuevo == PRECALENTAMIENTO) {
    aplicarPWM(0);
  }
  estado = nuevo;
}

/**
 * Entra en el estado seguro FALLA_SENSOR: sin medición confiable se asume el
 * peor caso (CO2 alto) y se ventila al máximo.
 */
void entrarEnFalla() {
  if (estado != FALLA_SENSOR) {
    estadoAntesDeFalla = estado;
  }
  estado = FALLA_SENSOR;
  hayMedicionPrevia = false;           // evita un pico derivativo al recuperar
  aplicarPWM((int)PWM_MAX);
  Serial.println(F("ERR: falla de sensor -> ventilacion al 100 %"));
}

/**
 * Tarea periódica (1 Hz): adquiere la medición, ejecuta la lógica del estado
 * actual y envía la telemetría.
 */
void cicloMuestreo() {
  // --- Sensor nunca detectado: reintentar la inicialización periódicamente ---
  if (!sensorInicializado) {
    if (millis() - tUltimoReintento >= REINTENTO_SENSOR_MS) {
      tUltimoReintento = millis();
      if (sgp.begin()) {
        sensorInicializado = true;
        fallosConsecutivos = 0;
        Serial.println(F("INFO: SGP30 detectado, iniciando precalentamiento"));
        cambiarEstado(PRECALENTAMIENTO);
      }
    }
    enviarTelemetria();
    return;
  }

  // --- Adquisición ---
  if (!leerSensor()) {
    if (fallosConsecutivos < 255) fallosConsecutivos++;
    if (fallosConsecutivos >= MAX_FALLOS_LECTURA && estado != FALLA_SENSOR) {
      entrarEnFalla();
    }
    // Por debajo del umbral se mantiene la última salida aplicada.
    enviarTelemetria();
    return;
  }
  fallosConsecutivos = 0;

  // --- Lógica por estado ---
  switch (estado) {
    case PRECALENTAMIENTO:
      aplicarPWM(0);
      muestrasPrecal++;
      if (muestrasPrecal >= MUESTRAS_PRECALENTAMIENTO) {
        cambiarEstado(STANDBY);
      }
      break;

    case STANDBY:
      aplicarPWM(0);
      break;

    case PID_ACTIVO:
      aplicarPWM(calcularPID(co2Actual));
      break;

    case MANUAL:
      aplicarPWM(pwmManual);             // lazo abierto: PWM fijo
      break;

    case FALLA_SENSOR:
      // El sensor volvió a responder: retomar el estado previo a la falla.
      Serial.println(F("INFO: sensor recuperado"));
      cambiarEstado(estadoAntesDeFalla);
      if (estado == PID_ACTIVO) {
        aplicarPWM(calcularPID(co2Actual));
      } else if (estado == MANUAL) {
        aplicarPWM(pwmManual);
      }
      break;
  }

  // La medición previa se actualiza en TODOS los estados: así, al activar el
  // PID, la derivada parte de un valor real y no produce un salto.
  medicionPrevia    = co2Actual;
  hayMedicionPrevia = true;

  enviarTelemetria();
}

// ============================================================================
// 3. ADQUISICIÓN
// ============================================================================

/**
 * Lee eCO2 y TVOC del SGP30.
 * @return true si la lectura fue válida; false si la comunicación I2C falló.
 */
bool leerSensor() {
  if (!sgp.IAQmeasure()) {
    return false;
  }
  co2Crudo   = sgp.eCO2;
  co2Actual  = filtroMediana(co2Crudo);
  tvocActual = sgp.TVOC;
  return true;
}

/**
 * Filtro de mediana de 3 muestras: descarta picos aislados del sensor.
 * Mientras el buffer se llena, devuelve la lectura sin filtrar.
 * @param x  Nueva lectura [ppm].
 * @return   Mediana de las 3 últimas lecturas [ppm].
 */
float filtroMediana(float x) {
  bufferCO2[idxBuffer] = x;
  idxBuffer = (idxBuffer + 1) % 3;
  if (nBuffer < 3) {
    nBuffer++;
    return x;
  }
  float a = bufferCO2[0], b = bufferCO2[1], c = bufferCO2[2];
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================================
// 4. CONTROLADOR PID
// ============================================================================

/**
 * PID discreto de acción directa (a mayor CO2, mayor extracción).
 *  - Derivada sobre la medición: sin salto al activar el lazo.
 *  - Anti-windup por integración condicional: la integral no se acumula
 *    cuando la salida está saturada en la misma dirección del error, y su
 *    contribución se limita a [0, PWM_MAX] (el ventilador no puede inyectar CO2).
 *  - Compensación de zona muerta: el ventilador no gira por debajo de PWM_MIN.
 *
 * @param medicion  eCO2 medido [ppm].
 * @return          PWM a aplicar [0-255].
 */
int calcularPID(float medicion) {
  float error = medicion - setpoint;

  // Proporcional
  terminoP = KP * error;

  // Derivativo (sobre la medición; de/dt = dy/dt porque el setpoint es fijo)
  terminoD = hayMedicionPrevia ? KD * (medicion - medicionPrevia) / DT : 0.0;

  // Integral con integración condicional
  float iCandidata = constrain(termI + KI * error * DT, 0.0, PWM_MAX);
  float uCandidata = terminoP + iCandidata + terminoD;
  bool saturaArriba = (uCandidata > PWM_MAX) && (error > 0);
  bool saturaAbajo  = (uCandidata < 0.0)     && (error < 0);
  if (!saturaArriba && !saturaAbajo) {
    termI = iCandidata;
  }

  float u = constrain(terminoP + termI + terminoD, 0.0, PWM_MAX);

  // Zona muerta: por debajo de PWM_MIN/2 se apaga; entre PWM_MIN/2 y PWM_MIN
  // se lleva a PWM_MIN para que el ventilador realmente gire.
  if (u < PWM_MIN / 2.0) {
    u = 0.0;
  } else if (u < PWM_MIN) {
    u = PWM_MIN;
  }

  return (int)(u + 0.5);
}

/** Reinicia la memoria del controlador al activar el lazo. */
void reiniciarPID() {
  termI    = 0.0;
  terminoP = 0.0;
  terminoD = 0.0;
}

/**
 * Escribe el PWM en el MOSFET y lo registra para la telemetría.
 * @param valor  Ciclo útil en cuentas [0-255].
 */
void aplicarPWM(int valor) {
  pwmSalida = constrain(valor, 0, 255);
  analogWrite(PIN_PWM_VENTILADOR, pwmSalida);
}

// ============================================================================
// 5. INTERFAZ DE USUARIO
// ============================================================================

/**
 * Antirrebote del pulsador. Cada pulsación confirmada alterna entre STANDBY y
 * PID_ACTIVO (desde MANUAL vuelve a STANDBY). Se ignora durante
 * PRECALENTAMIENTO y FALLA_SENSOR.
 */
void gestionarBoton() {
  static bool lecturaPrevia  = HIGH;
  static bool estadoEstable  = HIGH;
  static unsigned long tCambio = 0;

  bool lectura = digitalRead(PIN_BOTON);
  if (lectura != lecturaPrevia) {
    tCambio = millis();
    lecturaPrevia = lectura;
  }

  if ((millis() - tCambio) > DEBOUNCE_MS && lectura != estadoEstable) {
    estadoEstable = lectura;
    if (estadoEstable == LOW) {               // flanco de presión confirmado
      if (estado == STANDBY) {
        cambiarEstado(PID_ACTIVO);
      } else if (estado == PID_ACTIVO || estado == MANUAL) {
        cambiarEstado(STANDBY);
      }
    }
  }
}

/** Actualiza el LED según el estado, sin bloquear (basado en millis()). */
void actualizarLed() {
  unsigned long ahora = millis();
  bool encendido = false;

  switch (estado) {
    case PRECALENTAMIENTO: encendido = (ahora / PARPADEO_PRECAL_MS) % 2; break;
    case STANDBY:          encendido = false;                            break;
    case PID_ACTIVO:       encendido = true;                             break;
    case FALLA_SENSOR:     encendido = (ahora / PARPADEO_FALLA_MS) % 2;  break;
    case MANUAL:           encendido = (ahora % 1000) < 800;             break;
  }
  digitalWrite(PIN_LED_ESTADO, encendido ? HIGH : LOW);
}

// ============================================================================
// 6. TELEMETRÍA
// ============================================================================

/** Envía una línea de datos por el puerto serie en el formato configurado. */
void enviarTelemetria() {
#if SALIDA_PLOTTER
  Serial.print(F("CO2_ppm:"));     Serial.print(co2Actual, 0);
  Serial.print(F(",CO2_crudo:"));  Serial.print(co2Crudo, 0);
  Serial.print(F(",Setpoint:"));   Serial.print(setpoint, 0);
  Serial.print(F(",PWM:"));        Serial.print(pwmSalida);
  Serial.print(F(",Estado:"));     Serial.println((int)estado);
#else
  Serial.print(millis());          Serial.print(',');
  Serial.print(co2Actual, 0);      Serial.print(',');
  Serial.print(tvocActual, 0);     Serial.print(',');
  Serial.print(setpoint, 0);       Serial.print(',');
  Serial.print(pwmSalida);         Serial.print(',');
  Serial.print(terminoP, 2);       Serial.print(',');
  Serial.print(termI, 2);          Serial.print(',');
  Serial.print(terminoD, 2);       Serial.print(',');
  Serial.print((int)estado);       Serial.print(',');
  Serial.println(co2Crudo, 0);
#endif
}

// ============================================================================
// 7. COMANDOS SERIE
// ============================================================================

/** Acumula caracteres del puerto serie y procesa cada línea completa. */
void leerComandos() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lenCmd > 0) {
        bufferCmd[lenCmd] = '\0';
        procesarComando(bufferCmd);
        lenCmd = 0;
      }
    } else if (lenCmd < sizeof(bufferCmd) - 1) {
      bufferCmd[lenCmd++] = toupper(c);
    }
  }
}

/**
 * Interpreta un comando:  SP:<ppm>  |  MAN:<pwm>  |  MAN:OFF
 * @param cmd  Texto del comando en mayúsculas, sin salto de línea.
 */
void procesarComando(char *cmd) {
  if (strncmp(cmd, "SP:", 3) == 0) {
    float valor = atof(cmd + 3);
    if (valor >= SETPOINT_MIN && valor <= SETPOINT_MAX) {
      setpoint = valor;
      Serial.print(F("OK SP="));
      Serial.println(setpoint, 0);
    } else {
      Serial.print(F("ERR SP fuera de rango ["));
      Serial.print(SETPOINT_MIN, 0); Serial.print('-');
      Serial.print(SETPOINT_MAX, 0); Serial.println(']');
    }
  } else if (strcmp(cmd, "MAN:OFF") == 0) {
    if (estado == MANUAL) cambiarEstado(STANDBY);
    Serial.println(F("OK MANUAL OFF -> STANDBY"));
  } else if (strncmp(cmd, "MAN:", 4) == 0) {
    // El valor debe ser un número de 1 a 3 dígitos (0-255)
    const char *arg = cmd + 4;
    bool numeroValido = strlen(arg) >= 1 && strlen(arg) <= 3;
    for (const char *c = arg; *c && numeroValido; c++) {
      if (!isdigit(*c)) numeroValido = false;
    }
    if (!numeroValido) {
      Serial.println(F("ERR MAN: usa MAN:0-255 o MAN:OFF"));
    } else if (estado == STANDBY || estado == PID_ACTIVO || estado == MANUAL) {
      pwmManual = constrain(atoi(cmd + 4), 0, 255);
      estado = MANUAL;
      aplicarPWM(pwmManual);
      Serial.print(F("OK MANUAL PWM="));
      Serial.println(pwmManual);
    } else {
      Serial.println(F("ERR MANUAL no disponible en este estado"));
    }
  } else {
    Serial.println(F("ERR comando desconocido (usa SP:xxx, MAN:xxx, MAN:OFF)"));
  }
}
