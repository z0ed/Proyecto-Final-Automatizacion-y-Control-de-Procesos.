"""
===============================================================================
 PUENTE IoT + GEMELO DIGITAL — Control de CO2 en cámara volumétrica
-------------------------------------------------------------------------------
 Lee la telemetría CSV del Arduino (control_co2.ino con SALIDA_PLOTTER 0),
 ejecuta en paralelo un modelo físico de la cámara (el "gemelo"), compara la
 predicción con la medición, guarda todo en un CSV local y publica un resumen
 en Ubidots.

 Requisitos:
     pip install pyserial

 Uso:
     1. En control_co2.ino poner  #define SALIDA_PLOTTER 0  y subirlo.
     2. Cerrar el Monitor Serie / Serial Plotter del Arduino IDE.
     3. Ajustar la sección CONFIGURACIÓN (puerto, token de Ubidots, modelo).
     4. python puente_gemelo.py        (Ctrl+C para terminar)

 Comandos (escribir en esta misma terminal y presionar Enter):
   Al Arduino:
     SP:850     cambia el setpoint (600-1000 ppm)
     MAN:120    ventilador fijo en PWM 120 (ensayo en lazo abierto)
     MAN:OFF    sale del modo manual
   A la capa de IA (Gemini):
     IA:AHORA   consulta a Gemini inmediatamente
     IA:OFF     pausa la IA (el setpoint queda fijo)   IA:ON  la reactiva
     OCUP:2     declara 2 ocupantes en el apartamento (0 = vacío)

 Nota: al abrir el puerto, el Arduino se reinicia y repite los 15 s de
 precalentamiento. Es normal.
-------------------------------------------------------------------------------
 MODELO DEL GEMELO (balance de masa de CO2 en la cámara)

     V·dC/dt = − Q(u)·(C − C_ext) − Q_fuga·(C − C_ext)

     Q(u)/V  = (u / PWM_REF) / TAU_REF     si el ventilador gira (u ≥ PWM_MIN)
             = 0                          si está detenido
     Q_fuga/V = 1 / TAU_FUGA              (renovación natural por rendijas)

 Se integra con la solución exacta en cada paso (estable para cualquier dt):
     C[k+1] = C_ext + (C[k] − C_ext)·exp(−(Q(u)/V + 1/TAU_FUGA)·dt)

 El PWM entra con un retardo THETA_S (tiempo muerto del FOPTD).
-------------------------------------------------------------------------------
 CAPA DE IA (RF-05): cada IA_PERIODO_S segundos se envía a Gemini un resumen
 (hora, ocupantes declarados, CO2 promedio y tendencia, uso del ventilador,
 setpoint actual). Gemini propone un setpoint que se ACOTA a [600, 1000] ppm
 (RD-02) antes de enviarlo al Arduino como "SP:xxx". Si Gemini no responde, el
 Arduino conserva el último setpoint (RNF-01). Cada decisión queda registrada
 en datos/decisiones_ia_*.csv.

 SINCRONIZACIÓN: el gemelo no "ve" la exhalación (perturbación no medida).
 Mientras el sistema está en Standby, el gemelo copia la medición. En el
 instante en que se activa el PID, ambos parten del mismo valor y desde ahí
 evolucionan por separado: la curva real según la física y la virtual según
 las ecuaciones. El residuo r = real − gemelo mide qué tan bien predice el
 modelo; si crece de forma sostenida indica error de modelo o una falla.
===============================================================================
"""

import csv
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from datetime import datetime

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Falta pyserial. Instálalo con:  pip install pyserial")

# =============================================================================
# CONFIGURACIÓN
# =============================================================================

# --- Puerto serie ---
PUERTO = "COM5"
BAUDIOS = 9600

# --- Ubidots (dejar el token vacío para trabajar solo en local) ---
UBIDOTS_TOKEN = "BBUS-2b2aOCtETKvrJPoIJg8L8I3yyAViKw"                      # Token de tu cuenta (API Credentials)
UBIDOTS_DISPOSITIVO = "camara-co2"      # Se crea solo en el primer envío
UBIDOTS_URL = "https://industrial.api.ubidots.com/api/v1.6/devices/"
UBIDOTS_PERIODO_S = 12                  # Cuenta STEM: ~4000 datos/día, 1 envío/s

# --- Modelo del gemelo (AJUSTAR con los datos de tus pruebas) ---
# Valores calibrados con sesion_20261108_163217.csv (decaimiento con PWM ~245):
C_EXT = 410.0        # [ppm] CO2 del aire de renovación (medido en modo manual
                     #       sin perturbación: 410-440 ppm). Tras saturar el sensor
                     #       soplando directo, el SGP30 queda ~650 por memoria.
TAU_REF = 32.0       # [s] constante de tiempo identificada (ajuste: 31-34 s)...
PWM_REF = 245.0      # ...con este PWM promedio aplicado
TAU_FUGA = 600.0     # [s] renovación natural con el ventilador apagado
THETA_S = 2.0        # [s] tiempo muerto entre el PWM y el efecto en el sensor
PWM_MIN = 55.0       # PWM mínimo de giro (medido: arranque = 50)

# --- Validez de las lecturas ---
SATURACION_PPM = 50000.0  # el SGP30 satura en ~57330 ppm: esas lecturas se
                          # excluyen del gemelo y de las estadísticas

# --- Detección de fallas por residuo ---
UMBRAL_RESIDUO = 150.0    # [ppm]
TIEMPO_ALERTA_S = 15      # segundos seguidos sobre el umbral para alertar

# --- Capa de IA: Gemini (RF-05) ---
GEMINI_API_KEY = "AQ.Ab8RN6JC_JMi5sMkyZ51wata4IH2YHFpvVzQ9r7PoVcFJ0rt-g"                     # Clave de Google AI Studio (vacía = IA desactivada)
GEMINI_MODELO = "gemini-flash-lite-latest"  # si no existe, se elige otro automáticamente
GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/"
IA_PERIODO_S = 300          # una consulta cada 5 min (la cuenta gratuita tiene límite diario)
IA_PRIMERA_S = 60           # primera consulta tras 60 s de datos
IA_SP_MIN = 600             # rango seguro: igual al que acepta el Arduino (RD-02)
IA_SP_MAX = 1000
IA_CAMBIO_MIN = 25          # [ppm] no se envían cambios menores (evita oscilar)
OCUPACION_INICIAL = 1       # ocupantes declarados al iniciar (se cambia con OCUP:n)

# --- Archivos ---
CARPETA_DATOS = "datos"

ESTADOS = {0: "PRECAL", 1: "STANDBY", 2: "PID", 3: "FALLA", 4: "MANUAL"}
ACTIVOS = (2, 4)   # estados con ventilador comandado: el gemelo simula
CAMPOS_ARDUINO = ["t_ms", "co2_ppm", "tvoc_ppb", "setpoint_ppm",
                  "pwm", "P", "I", "D", "estado", "co2_crudo_ppm"]


# =============================================================================
# GEMELO DIGITAL
# =============================================================================

class GemeloCamara:
    """Modelo de balance de masa de la cámara, alimentado con el PWM real."""

    def __init__(self):
        self.c = C_EXT
        self.historial_pwm = deque()   # (t_s, pwm) para aplicar el retardo
        self.u_retardado = 0.0         # PWM que "ve" la cámara tras el retardo
        self.t_previo = None

    def sincronizar(self, c_medido, t_s):
        """Iguala el estado del gemelo a la medición (inicio de un ensayo)."""
        self.c = c_medido
        self.t_previo = t_s
        self.historial_pwm.clear()
        self.u_retardado = 0.0         # en Standby el ventilador está apagado

    def _pwm_retardado(self, t_s):
        """PWM aplicado THETA_S segundos antes del instante t_s."""
        while self.historial_pwm and self.historial_pwm[0][0] <= t_s - THETA_S:
            self.u_retardado = self.historial_pwm.popleft()[1]
        return self.u_retardado

    def paso(self, t_s, pwm):
        """Avanza el modelo hasta t_s con la entrada pwm. Devuelve C predicho."""
        self.historial_pwm.append((t_s, pwm))
        if self.t_previo is None:
            self.t_previo = t_s
            return self.c
        dt = max(0.0, t_s - self.t_previo)
        self.t_previo = t_s

        u = self._pwm_retardado(t_s)
        k_vent = (u / PWM_REF) / TAU_REF if u >= PWM_MIN else 0.0
        k_total = k_vent + 1.0 / TAU_FUGA
        self.c = C_EXT + (self.c - C_EXT) * math.exp(-k_total * dt)
        return self.c


# =============================================================================
# ESTADÍSTICAS DE CADA ENSAYO (desde que se activa el PID hasta que se apaga)
# =============================================================================

class Ensayo:
    def __init__(self, numero, t_inicio, c_inicio):
        self.numero = numero
        self.t_inicio = t_inicio
        self.c_inicio = c_inicio
        self.reales, self.gemelos = [], []

    def agregar(self, real, gemelo):
        self.reales.append(real)
        self.gemelos.append(gemelo)

    def resumen(self, t_fin):
        n = len(self.reales)
        if n < 3:
            return f"Ensayo {self.numero}: muy corto para evaluar."
        err = [r - g for r, g in zip(self.reales, self.gemelos)]
        rmse = math.sqrt(sum(e * e for e in err) / n)
        media = sum(self.reales) / n
        den = math.sqrt(sum((r - media) ** 2 for r in self.reales))
        num = math.sqrt(sum(e * e for e in err))
        ajuste = 100.0 * (1.0 - num / den) if den > 0 else float("nan")
        return (f"Ensayo {self.numero}: duración {t_fin - self.t_inicio:.0f} s | "
                f"CO2 inicial {self.c_inicio:.0f} ppm | RMSE {rmse:.1f} ppm | "
                f"error máx {max(abs(e) for e in err):.0f} ppm | "
                f"ajuste {ajuste:.1f} %")


# =============================================================================
# UBIDOTS
# =============================================================================

def enviar_ubidots(datos):
    """Publica un diccionario {variable: valor} en el dispositivo de Ubidots."""
    if not UBIDOTS_TOKEN:
        return
    cuerpo = json.dumps(datos).encode("utf-8")
    peticion = urllib.request.Request(
        UBIDOTS_URL + UBIDOTS_DISPOSITIVO,
        data=cuerpo,
        headers={"X-Auth-Token": UBIDOTS_TOKEN,
                 "Content-Type": "application/json"},
        method="POST")
    try:
        with urllib.request.urlopen(peticion, timeout=3) as resp:
            if resp.status not in (200, 201):
                print(f"  [Ubidots] respuesta {resp.status}")
    except urllib.error.HTTPError as e:
        print(f"  [Ubidots] error HTTP {e.code}: {e.read()[:120]!r}")
    except Exception as e:  # sin internet, timeout, etc.
        print(f"  [Ubidots] no se pudo enviar: {e}")


# =============================================================================
# CAPA DE IA (GEMINI)
# =============================================================================

class EstadoCompartido:
    """Datos que comparten el hilo principal, el teclado y la IA."""

    def __init__(self):
        self.lock = threading.Lock()
        self.lock_puerto = threading.Lock()
        self.muestras = deque()            # (t_pc, co2, pwm, estado, sp, tvoc, t_arduino)
        self.ocupacion = OCUPACION_INICIAL
        self.ia_activa = True
        self.forzar = threading.Event()

    def agregar(self, co2, pwm, estado, sp, tvoc, t_arduino):
        ahora = time.time()
        with self.lock:
            self.muestras.append((ahora, co2, pwm, estado, sp, tvoc, t_arduino))
            while self.muestras and self.muestras[0][0] < ahora - 600:
                self.muestras.popleft()

    def resumen(self):
        """Resumen de los últimos 5 min para la IA, o None si hay pocos datos."""
        with self.lock:
            m = [x for x in self.muestras
                 if x[0] >= time.time() - 300 and x[1] < SATURACION_PPM]
            ocup = self.ocupacion
        if len(m) < 30:
            return None
        co2 = [x[1] for x in m]
        pwm = [x[2] for x in m]
        # Tendencia: pendiente por mínimos cuadrados de los últimos 3 min
        # (con el reloj del Arduino, que marca exactamente 1 muestra/s)
        u = [x for x in m if x[6] >= m[-1][6] - 180]
        tm = sum(x[6] for x in u) / len(u)
        cm = sum(x[1] for x in u) / len(u)
        den = sum((x[6] - tm) ** 2 for x in u) or 1.0
        pendiente = sum((x[6] - tm) * (x[1] - cm) for x in u) / den * 60.0
        ahora = datetime.now()
        return {
            "hora": ahora.strftime("%H:%M"),
            "dia": ["lunes", "martes", "miércoles", "jueves", "viernes",
                    "sábado", "domingo"][ahora.weekday()],
            "ocupantes": ocup,
            "co2_actual": round(co2[-1]),
            "co2_promedio_5min": round(sum(co2) / len(co2)),
            "tendencia_ppm_min": round(pendiente, 1),
            "pwm_promedio": round(sum(pwm) / len(pwm)),
            "ventilador_encendido_pct": round(100 * sum(p > 0 for p in pwm) / len(pwm)),
            "modo": ESTADOS.get(m[-1][3], str(m[-1][3])),
            "setpoint_actual": round(m[-1][4]),
        }


def construir_prompt(r):
    return (
        "Eres la capa de optimización (IA) de un sistema de control de calidad del "
        "aire interior para un apartamento tipo estudio del coliving Colivo (Bogotá), "
        "emulado en una cámara a escala. Un controlador PID local regula el CO2 con un "
        "ventilador; tú SOLO propones el punto de consigna (setpoint) de CO2.\n"
        "Objetivo: equilibrar la salud y el rendimiento cognitivo de los ocupantes "
        "con el consumo energético del ventilador.\n"
        "Reglas obligatorias:\n"
        f"- El setpoint es un entero entre {IA_SP_MIN} y {IA_SP_MAX} ppm (límites del "
        "proyecto basados en ASHRAE 62.1 y la OMS).\n"
        "- Con ocupantes en horario de actividad (estudio o trabajo), prefiere "
        "valores bajos (650-800 ppm).\n"
        "- Con el espacio vacío o en descanso nocturno, puedes subirlo (850-1000 ppm) "
        "para ahorrar energía.\n"
        "- Si el CO2 sube rápido (tendencia positiva alta), no relajes el setpoint.\n"
        "Datos actuales:\n"
        f"- Hora local: {r['hora']} ({r['dia']})\n"
        f"- Ocupantes declarados: {r['ocupantes']}\n"
        f"- CO2 actual: {r['co2_actual']} ppm; promedio 5 min: {r['co2_promedio_5min']} ppm; "
        f"tendencia: {r['tendencia_ppm_min']} ppm/min\n"
        f"- Ventilador: PWM promedio {r['pwm_promedio']}/255, encendido "
        f"{r['ventilador_encendido_pct']} % del tiempo\n"
        f"- Modo del controlador: {r['modo']}\n"
        f"- Setpoint actual: {r['setpoint_actual']} ppm\n"
        'Responde SOLO con JSON: {"setpoint_ppm": <entero>, '
        '"razon": "<máximo 25 palabras en español>"}'
    )


def _peticion_gemini(url, cuerpo=None):
    datos = json.dumps(cuerpo).encode("utf-8") if cuerpo is not None else None
    req = urllib.request.Request(
        url, data=datos, method="POST" if datos else "GET",
        headers={"x-goog-api-key": GEMINI_API_KEY,
                 "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read().decode("utf-8"))


def elegir_modelo():
    """Busca un modelo 'flash' disponible para esta clave (prefiere 'lite')."""
    lista = _peticion_gemini(GEMINI_URL + "models?pageSize=200").get("models", [])
    excluir = ("image", "tts", "audio", "embedding", "live", "native", "vision")
    candidatos = [m["name"].split("/", 1)[-1] for m in lista
                  if "generateContent" in m.get("supportedGenerationMethods", [])
                  and "flash" in m["name"]
                  and not any(x in m["name"] for x in excluir)]
    if not candidatos:
        raise RuntimeError("la clave no tiene modelos 'flash' disponibles")
    candidatos.sort(key=lambda n: ("lite" not in n, "latest" not in n, n))
    return candidatos[0]


def consultar_gemini(resumen, modelo):
    """Devuelve (setpoint_propuesto, razon, modelo_usado). Lanza excepción si falla."""
    cuerpo = {"contents": [{"role": "user",
                            "parts": [{"text": construir_prompt(resumen)}]}],
              "generationConfig": {"responseMimeType": "application/json",
                                   "temperature": 0.2}}
    try:
        resp = _peticion_gemini(f"{GEMINI_URL}models/{modelo}:generateContent", cuerpo)
    except urllib.error.HTTPError as e:
        if e.code == 404:                       # modelo inexistente: elegir otro
            modelo = elegir_modelo()
            print(f"  [IA] usando el modelo disponible: {modelo}")
            resp = _peticion_gemini(f"{GEMINI_URL}models/{modelo}:generateContent", cuerpo)
        elif e.code == 429:
            raise RuntimeError("límite de la cuenta gratuita alcanzado (429)")
        elif e.code in (400, 401, 403):
            raise RuntimeError(f"clave de API inválida o sin permiso ({e.code})")
        else:
            raise
    partes = resp["candidates"][0]["content"]["parts"]
    texto = "".join(p.get("text", "") for p in partes)
    texto = texto[texto.find("{"): texto.rfind("}") + 1]
    j = json.loads(texto)
    return float(j["setpoint_ppm"]), str(j.get("razon", "")).strip()[:200], modelo


def enviar_comando(puerto, comp, texto):
    with comp.lock_puerto:
        puerto.write((texto + "\n").encode("ascii", errors="ignore"))


def hilo_ia(puerto, comp, ruta_log):
    """Consulta periódicamente a Gemini y aplica el setpoint acotado."""
    modelo = GEMINI_MODELO
    with open(ruta_log, "w", newline="", encoding="utf-8") as f:
        log = csv.writer(f)
        log.writerow(["hora_pc", "ocupantes", "co2_promedio_5min", "tendencia_ppm_min",
                      "pwm_promedio", "sp_anterior", "sp_propuesto", "sp_aplicado",
                      "razon", "modelo", "resultado"])
        espera = IA_PRIMERA_S
        while True:
            comp.forzar.wait(timeout=espera)
            comp.forzar.clear()
            espera = IA_PERIODO_S
            if not comp.ia_activa:
                continue
            r = comp.resumen()
            if r is None:
                print("  [IA] aún no hay suficientes datos (se necesitan ~30 s)")
                espera = 30
                continue
            hora = datetime.now().isoformat(timespec="seconds")
            base = [hora, r["ocupantes"], r["co2_promedio_5min"],
                    r["tendencia_ppm_min"], r["pwm_promedio"], r["setpoint_actual"]]
            try:
                propuesto, razon, modelo = consultar_gemini(r, modelo)
            except Exception as e:
                print(f"  [IA] sin respuesta de Gemini ({e}). "
                      f"Se mantiene SP={r['setpoint_actual']} ppm.")
                log.writerow(base + ["", r["setpoint_actual"], "", modelo, f"error: {e}"])
                f.flush()
                continue
            aplicado = int(round(min(max(propuesto, IA_SP_MIN), IA_SP_MAX)))
            if abs(aplicado - r["setpoint_actual"]) >= IA_CAMBIO_MIN:
                enviar_comando(puerto, comp, f"SP:{aplicado}")
                accion = "aplicado"
            else:
                aplicado = r["setpoint_actual"]
                accion = "sin cambio"
            nota = "" if propuesto == aplicado or accion == "sin cambio" \
                else f" (acotado desde {propuesto:.0f})"
            print(f"\n  [IA] Gemini propone {propuesto:.0f} ppm -> SP={aplicado}{nota} "
                  f"[{accion}]\n  [IA] Razón: {razon}\n")
            log.writerow(base + [propuesto, aplicado, razon, modelo, accion])
            f.flush()


# =============================================================================
# LECTURA Y PROCESAMIENTO
# =============================================================================

def interpretar_linea(linea):
    """Convierte una línea CSV del Arduino en dict, o None si no es de datos."""
    partes = linea.split(",")
    if len(partes) != len(CAMPOS_ARDUINO):
        return None
    try:
        valores = [float(p) for p in partes]
    except ValueError:
        return None          # encabezado u otra línea de texto
    d = dict(zip(CAMPOS_ARDUINO, valores))
    d["estado"] = int(d["estado"])
    return d


def hilo_teclado(puerto, comp):
    """Atiende lo que se escriba en esta terminal: comandos de IA u órdenes
    que se reenvían al Arduino (SP:850, MAN:120, MAN:OFF)."""
    while True:
        try:
            texto = input().strip()
        except (EOFError, OSError):
            return
        if not texto:
            continue
        orden = texto.upper()
        if orden == "IA:AHORA":
            if GEMINI_API_KEY:
                print("  [IA] consultando a Gemini...")
                comp.forzar.set()
            else:
                print("  [IA] desactivada: falta GEMINI_API_KEY")
        elif orden in ("IA:ON", "IA:OFF"):
            comp.ia_activa = orden == "IA:ON"
            print(f"  [IA] {'activada' if comp.ia_activa else 'en pausa'}")
        elif orden.startswith("OCUP:"):
            try:
                comp.ocupacion = max(0, int(orden[5:]))
                print(f"  [IA] ocupantes declarados: {comp.ocupacion}")
            except ValueError:
                print("  Usa OCUP:<número>, por ejemplo OCUP:2")
        else:
            enviar_comando(puerto, comp, texto)
            print(f"  [PC -> Arduino] {texto}")


def main():
    os.makedirs(CARPETA_DATOS, exist_ok=True)
    nombre = datetime.now().strftime("sesion_%Y%m%d_%H%M%S.csv")
    ruta_csv = os.path.join(CARPETA_DATOS, nombre)

    print(f"Abriendo {PUERTO} a {BAUDIOS} baudios...")
    try:
        puerto = serial.Serial(PUERTO, BAUDIOS, timeout=2)
    except serial.SerialException as e:
        sys.exit(f"No se pudo abrir {PUERTO}: {e}\n"
                 "¿Está abierto el Monitor Serie del Arduino IDE? Ciérralo.")

    if UBIDOTS_TOKEN:
        datos_dia = 6 * 86400 / UBIDOTS_PERIODO_S
        print(f"Ubidots activo: 6 variables cada {UBIDOTS_PERIODO_S} s "
              f"(~{datos_dia / 24:.0f} datos/hora; la cuenta STEM permite ~4000/día).")
    else:
        print("Ubidots desactivado (UBIDOTS_TOKEN vacío). Solo registro local.")
    print(f"Guardando datos en {ruta_csv}")
    comp = EstadoCompartido()
    if GEMINI_API_KEY:
        ruta_ia = os.path.join(CARPETA_DATOS, nombre.replace("sesion_", "decisiones_ia_"))
        print(f"IA activa: Gemini cada {IA_PERIODO_S / 60:g} min "
              f"(setpoint acotado a {IA_SP_MIN}-{IA_SP_MAX} ppm). Registro: {ruta_ia}")
        threading.Thread(target=hilo_ia, args=(puerto, comp, ruta_ia), daemon=True).start()
    else:
        print("IA desactivada (GEMINI_API_KEY vacía).")
    print("Comandos: SP:850 · MAN:120 · MAN:OFF · IA:AHORA · IA:OFF/ON · OCUP:n")
    print("Ctrl+C para terminar.\n")
    threading.Thread(target=hilo_teclado, args=(puerto, comp), daemon=True).start()

    gemelo = GemeloCamara()
    estado_previo = None
    ensayo, n_ensayos = None, 0
    segundos_alerta = 0
    t_ultimo_envio = 0.0

    with open(ruta_csv, "w", newline="", encoding="utf-8") as f:
        escritor = csv.writer(f)
        escritor.writerow(["hora_pc"] + CAMPOS_ARDUINO +
                          ["co2_gemelo", "residuo", "alerta"])
        try:
            while True:
                bruto = puerto.readline()
                if not bruto:
                    continue
                linea = bruto.decode("utf-8", errors="ignore").strip()
                d = interpretar_linea(linea)
                if d is None:
                    if linea:
                        print(f"  [Arduino] {linea}")
                    continue

                t_s = d["t_ms"] / 1000.0
                c_real = d["co2_ppm"]
                estado = d["estado"]

                valida = c_real < SATURACION_PPM

                # --- Gemelo: sincronizar fuera del PID, simular dentro ---
                activo = estado in ACTIVOS
                if activo and (estado_previo not in ACTIVOS or estado != estado_previo):
                    if ensayo is not None:     # cambio directo PID <-> MANUAL
                        print("\n<<< " + ensayo.resumen(t_s) + "\n")
                    n_ensayos += 1
                    ensayo = None              # se crea con la 1a lectura válida
                    segundos_alerta = 0
                    print(f"\n>>> {ESTADOS[estado]} activado: ensayo {n_ensayos}\n")

                if activo and ensayo is None and valida:
                    gemelo.sincronizar(c_real, t_s)
                    ensayo = Ensayo(n_ensayos, t_s, c_real)
                    print(f"    gemelo sincronizado en {c_real:.0f} ppm")

                if activo and (ensayo is None or not valida):
                    # Lectura saturada: no se compara (el gemelo sigue con el PWM)
                    if ensayo is not None:
                        gemelo.paso(t_s, d["pwm"])
                    c_gemelo = gemelo.c if ensayo is not None else c_real
                    residuo = 0.0
                    print(f"    lectura saturada ({c_real:.0f} ppm) excluida")
                elif activo:
                    c_gemelo = gemelo.paso(t_s, d["pwm"])
                    residuo = c_real - c_gemelo
                    ensayo.agregar(c_real, c_gemelo)
                    segundos_alerta = segundos_alerta + 1 \
                        if abs(residuo) > UMBRAL_RESIDUO else 0
                else:
                    if estado_previo in ACTIVOS and ensayo is not None:
                        print("\n<<< " + ensayo.resumen(t_s) + "\n")
                        ensayo = None
                    gemelo.sincronizar(c_real, t_s)
                    c_gemelo, residuo, segundos_alerta = c_real, 0.0, 0

                alerta = 1 if segundos_alerta >= TIEMPO_ALERTA_S else 0
                if alerta and segundos_alerta == TIEMPO_ALERTA_S:
                    print("  !!! ALERTA: el sistema real se aleja del gemelo "
                          f"más de {UMBRAL_RESIDUO:.0f} ppm (¿ventilador "
                          "obstruido, cámara abierta o modelo desajustado?)")
                estado_previo = estado

                # --- Registro local (todas las muestras) ---
                escritor.writerow([datetime.now().isoformat(timespec="seconds")] +
                                  [d[k] for k in CAMPOS_ARDUINO] +
                                  [round(c_gemelo, 1), round(residuo, 1), alerta])
                f.flush()
                comp.agregar(c_real, d["pwm"], estado, d["setpoint_ppm"], d["tvoc_ppb"], t_s)

                print(f"t={t_s:7.0f}s  {ESTADOS.get(estado, estado):7s}  "
                      f"CO2 real={c_real:6.0f}  gemelo={c_gemelo:6.0f}  "
                      f"r={residuo:+6.0f}  PWM={d['pwm']:3.0f}")

                # --- Publicación en Ubidots (submuestreada) ---
                ahora = time.time()
                if ahora - t_ultimo_envio >= UBIDOTS_PERIODO_S:
                    t_ultimo_envio = ahora
                    enviar_ubidots({
                        "co2-real": c_real,
                        "co2-gemelo": round(c_gemelo, 1),
                        "pwm": d["pwm"],
                        "estado": estado,
                        "residuo": round(residuo, 1),
                        "setpoint": d["setpoint_ppm"],
                    })

        except KeyboardInterrupt:
            print("\nFinalizando...")
            if ensayo is not None:
                print("<<< " + ensayo.resumen(t_s))
        finally:
            puerto.close()

    print(f"Datos guardados en: {os.path.abspath(ruta_csv)}")


if __name__ == "__main__":
    main()
