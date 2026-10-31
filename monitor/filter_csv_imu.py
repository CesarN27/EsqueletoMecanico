"""Filtro del monitor serie de PlatformIO para el IMU.

Mientras el monitor está abierto (desde que lo abres hasta Ctrl+C):
  - guarda cada muestra que manda el ESP32 ("D,t_ms,ax,ay,az,gx,gy,gz,temp,motor,enc_a,enc_b") en un CSV:
      datos/captura_<fecha>_<hora>.csv   (una por cada vez que abres el monitor)
      datos/captura_ultima.csv           (copia con nombre fijo, se sobrescribe cada vez)
  - muestra en pantalla una tabla de 10 Hz (promedio de 100 ms) en lugar de 100 líneas por segundo
  - deja pasar tal cual el resto de los mensajes (motores, diagnóstico, etc.)

Se activa con `monitor_filters = default, csv_imu` en platformio.ini.
"""
import atexit
import io
import os
from datetime import datetime

from platformio.device.monitor.filters.base import DeviceMonitorFilterBase

ENCABEZADO_CSV = "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,motor,enc_a,enc_b"
CAMPOS = 11  # t_ms + 6 ejes + temperatura + motor + 2 encoders
VENTANA_MS = 100  # cada línea de la tabla promedia 100 ms
PERIODO_MUESTRA_MS = 10  # el ESP32 muestrea a 100 Hz
LINEAS_POR_ENCABEZADO = 40


class CsvImu(DeviceMonitorFilterBase):
    NAME = "csv_imu"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._archivos = []  # el de la sesión y el de "ultima"
        self._ruta_sesion = None
        self._filas = 0
        self._resto = ""  # parte de una línea que aún no termina
        self._reiniciar_ventana()
        self._lineas_tabla = 0
        self._t_anterior = None

    def __call__(self):
        # miniterm llama esto una vez cuando arranca el monitor
        if not self._archivos:
            self._abrir()
        return self

    def __del__(self):
        self._cerrar()

    # ---------- archivos ----------
    def _abrir(self):
        proyecto = self.project_dir or os.getcwd()
        carpeta = os.path.join(proyecto, "datos")
        os.makedirs(carpeta, exist_ok=True)
        sello = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._ruta_sesion = os.path.join(carpeta, "captura_%s.csv" % sello)
        ruta_ultima = os.path.join(carpeta, "captura_ultima.csv")
        for ruta in (self._ruta_sesion, ruta_ultima):
            # buffering=1: se escribe línea por línea, así un Ctrl+C no pierde datos
            archivo = io.open(ruta, "w", encoding="utf-8", newline="\n", buffering=1)
            archivo.write(ENCABEZADO_CSV + "\n")
            self._archivos.append(archivo)
        atexit.register(self._cerrar)
        print("--- Guardando CSV en %s (se cierra con Ctrl+C)" % self._ruta_sesion)

    def _cerrar(self):
        if not self._archivos:
            return
        for archivo in self._archivos:
            try:
                archivo.close()
            except OSError:
                pass
        self._archivos = []
        try:
            print("--- CSV guardado: %d filas en %s" % (self._filas, self._ruta_sesion))
        except (OSError, ValueError):
            pass

    # ---------- procesamiento ----------
    def rx(self, text):
        self._resto += text
        salida = []
        while "\n" in self._resto:
            linea, self._resto = self._resto.split("\n", 1)
            salida.append(self._procesar_linea(linea.rstrip("\r")))
        return "".join(salida)

    def _procesar_linea(self, linea):
        if not linea.startswith("D,"):
            return linea + "\n"  # no es una muestra: pasa tal cual

        campos = linea[2:].split(",")
        try:
            t = int(campos[0])
            valores = [float(c) for c in campos[1:]]
            if len(campos) != CAMPOS:
                raise ValueError
        except ValueError:
            return linea + "\n"  # línea dañada: se muestra pero no se guarda

        fila = ",".join(campos) + "\n"
        for archivo in self._archivos:
            archivo.write(fila)
        self._filas += 1

        # Si el tiempo retrocede, la placa se reinició: la ventana empieza de nuevo
        if self._t_anterior is not None and t < self._t_anterior:
            self._reiniciar_ventana()
        self._t_anterior = t
        return self._acumular(t, valores)

    def _reiniciar_ventana(self):
        self._n = 0
        self._t0 = 0
        self._suma = [0.0] * (CAMPOS - 1)

    def _acumular(self, t, valores):
        if self._n == 0:
            self._t0 = t
        for i, v in enumerate(valores):
            self._suma[i] += v
        self._n += 1

        if t - self._t0 + PERIODO_MUESTRA_MS < VENTANA_MS:
            return ""
        prom = [s / self._n for s in self._suma]
        self._reiniciar_ventana()

        texto = ""
        if self._lineas_tabla % LINEAS_POR_ENCABEZADO == 0:
            texto += "%8s %8s %8s %8s %9s %9s %9s %6s %6s %7s\n" % (
                "t_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "T_C", "MOTOR", "enc_a")
        self._lineas_tabla += 1
        # motor: 0 / 1 (fraccion si cambio dentro de los 100 ms); enc_b solo va en el CSV
        texto += "%8d %8.3f %8.3f %8.3f %9.1f %9.1f %9.1f %6.1f %6.2f %7.0f\n" % (t, *prom[:9])
        return texto
