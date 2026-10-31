"""Vista en vivo del IMU: donde esta el sensor en el espacio y cuando enciende el motor (rojo).

  En vivo (cierra antes el monitor de PlatformIO: solo un programa puede usar el puerto):
      python tools/monitor_vivo.py --puerto COM5
  Reproducir un CSV (para probar sin placa):
      python tools/monitor_vivo.py --csv datos/captura_20261029_113355.csv [--velocidad 2]

Necesita: pip install pyserial matplotlib numpy
Guarda lo recibido en datos/vivo_<fecha>.csv (mismo formato que el monitor de PlatformIO).
"""
import argparse, collections, os, threading, time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VENTANA = 600      # muestras visibles (6 s a 100 Hz)
ESTELA = 150       # muestras de la estela 3D
ENCABEZADO = "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,motor,enc_a,enc_b"
datos = collections.deque(maxlen=VENTANA)   # filas de 11 valores
lock = threading.Lock()
total = [0]

def agregar(v):
    with lock:
        datos.append(v)
        total[0] += 1

def hilo_serie(puerto, baudios, salida):
    import serial
    with serial.Serial(puerto, baudios, timeout=1) as s, open(salida, "w", encoding="utf-8") as f:
        f.write(ENCABEZADO + "\n")
        while True:
            l = s.readline().decode("utf-8", "ignore").strip()
            if not l.startswith("D,"):
                if l: print(l)          # mensajes del firmware (MOTOR ON/OFF, etc.)
                continue
            try:
                v = [float(x) for x in l[2:].split(",")]
            except ValueError:
                continue
            if len(v) == 8: v += [0, 0, 0]
            if len(v) == 11:
                f.write(l[2:] + "\n"); agregar(v)

def hilo_csv(ruta, velocidad):
    d = np.genfromtxt(ruta, delimiter=",", skip_header=1)
    if d.shape[1] == 8: d = np.hstack([d, np.zeros((len(d), 3))])
    t0 = time.time()
    for fila in d:
        espera = (fila[0] - d[0, 0]) / 1000 / velocidad - (time.time() - t0)
        if espera > 0: time.sleep(espera)
        agregar(list(fila))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto"); ap.add_argument("--baudios", type=int, default=115200)
    ap.add_argument("--csv"); ap.add_argument("--velocidad", type=float, default=1.0)
    ap.add_argument("--sin-ventana", action="store_true", help="prueba: guarda un PNG y sale")
    a = ap.parse_args()
    if a.csv:
        threading.Thread(target=hilo_csv, args=(a.csv, a.velocidad), daemon=True).start()
    elif a.puerto:
        os.makedirs("datos", exist_ok=True)
        salida = os.path.join("datos", datetime.now().strftime("vivo_%Y%m%d_%H%M%S.csv"))
        print("Guardando en", salida)
        threading.Thread(target=hilo_serie, args=(a.puerto, a.baudios, salida), daemon=True).start()
    else:
        ap.error("indica --puerto COMx o --csv archivo")

    fig = plt.figure(figsize=(13, 7))
    a3 = fig.add_subplot(1, 2, 1, projection="3d")
    aa = fig.add_subplot(2, 2, 2); ag = fig.add_subplot(2, 2, 4, sharex=aa)
    a3.set_xlim(-1.5, 1.5); a3.set_ylim(-1.5, 1.5); a3.set_zlim(-1.5, 1.5)
    a3.set_xlabel("ax (g)"); a3.set_ylabel("ay (g)"); a3.set_zlabel("az (g)")
    a3.set_title("Aceleracion (hacia donde apunta el sensor)")
    estela = a3.scatter([], [], [], s=6)
    punta, = a3.plot([], [], [], "o", ms=12)
    flecha, = a3.plot([], [], [], "-", lw=3)
    lineas_a = [aa.plot([], [], lw=1, label=n)[0] for n in ("ax", "ay", "az")]
    lineas_g = [ag.plot([], [], lw=1, label=n)[0] for n in ("gx", "gy", "gz")]
    aa.set_ylim(-2.1, 2.1); aa.set_ylabel("g"); aa.legend(loc="upper left", ncol=3)
    ag.set_ylim(-260, 260); ag.set_ylabel("grados/s"); ag.set_xlabel("tiempo (s)"); ag.legend(loc="upper left", ncol=3)
    titulo = fig.suptitle("")
    franjas = []

    def actualizar(_):
        with lock:
            d = np.array(datos)
        if len(d) < 2: return
        t = (d[:, 0] - d[-1, 0]) / 1000
        on = d[:, 8] > 0
        r = d[-ESTELA:]; ron = r[:, 8] > 0
        estela._offsets3d = (r[:, 1], r[:, 2], r[:, 3])
        estela.set_color(np.where(ron[:, None], [[1, 0, 0, 1]], [[0.3, 0.47, 0.65, 0.35]]))
        c = "red" if on[-1] else "tab:blue"
        punta.set_data_3d([r[-1, 1]], [r[-1, 2]], [r[-1, 3]]); punta.set_color(c)
        flecha.set_data_3d([0, r[-1, 1]], [0, r[-1, 2]], [0, r[-1, 3]]); flecha.set_color(c)
        for i, l in enumerate(lineas_a): l.set_data(t, d[:, 1 + i])
        for i, l in enumerate(lineas_g): l.set_data(t, d[:, 4 + i])
        aa.set_xlim(t[0], 0)
        for f in franjas: f.remove()
        franjas.clear()
        for ax_ in (aa, ag):
            franjas.append(ax_.fill_between(t, -300, 300, where=on, color="red", alpha=0.2, step="mid"))
        titulo.set_text(("MOTOR ENCENDIDO" if on[-1] else "motor apagado") + f"   |   muestras: {total[0]}")
        titulo.set_color("red" if on[-1] else "black")

    ani = FuncAnimation(fig, actualizar, interval=50, cache_frame_data=False)
    if a.sin_ventana:
        time.sleep(8); actualizar(0); fig.savefig(a.csv + ".png", dpi=60); return
    plt.show()

if __name__ == "__main__":
    main()
