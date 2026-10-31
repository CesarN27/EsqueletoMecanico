"""Graficas 3D del IMU (MPU6050) a partir del CSV capturado con el monitor serie.

Funciona en tu PC y en Google Colab (en Colab sube antes captura_ultima.csv al panel de archivos).

Uso en PC:  python graficar_3d.py                      (lee datos\captura_ultima.csv)
            python graficar_3d.py ruta\al\archivo.csv

Ventana 1: vectores de aceleracion (g) y giroscopio (grados/s) en 3D, coloreados por tiempo.
Ventana 2: animacion de la orientacion estimada del sensor (filtro complementario tipo Mahony).
En PC se pueden girar con el mouse; en Colab salen como imagen y la animacion con un reproductor.
"""
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------- Configuracion ----------------
EN_COLAB = "google.colab" in sys.modules
# En Colab los archivos subidos quedan en /content
RUTA = "/content/captura_ultima.csv" if EN_COLAB else r"C:\Users\thech\EsqueletoMecanico\datos\captura_ultima.csv"
VER_VECTORES = True
VER_ORIENTACION = True
VELOCIDAD = 1.0        # animacion: 1.0 = tiempo real, 0.5 = camara lenta, 2.0 = el doble de rapido
CALIBRAR_SESGO = True  # resta el offset del giroscopio usando el inicio, si el sensor esta quieto
KP = 2.0               # cuanto corrige la inclinacion el acelerometro (mas alto = confia mas en el)
RANGO_ACEL_G = (0.75, 1.35)  # solo corrige con el acelerometro si |a| esta cerca de 1 g


# ---------------- Datos ----------------
def cargar(ruta):
    df = pd.read_csv(ruta).dropna().reset_index(drop=True)
    saltos = np.where(np.diff(df["t_ms"].to_numpy()) < 0)[0]
    if len(saltos):  # la placa se reinicio durante la captura: el tiempo retrocede
        df = df.iloc[saltos[-1] + 1:].reset_index(drop=True)
        print("Aviso: la placa se reinicio en la captura, se usa solo el ultimo tramo.")
    if len(df) < 20:
        sys.exit("El archivo tiene muy pocas filas para graficar.")
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    return df


# ---------------- Ventana 1: vectores en 3D ----------------
def _panel(fig, posicion, x, y, z, t, titulo, unidad, esfera=False):
    ax = fig.add_subplot(1, 2, posicion, projection="3d")
    ax.plot(x, y, z, color="gray", lw=0.6, alpha=0.5)
    puntos = ax.scatter(x, y, z, c=t, cmap="viridis", s=6)
    ax.scatter(x[0], y[0], z[0], color="green", s=70, label="inicio")
    ax.scatter(x[-1], y[-1], z[-1], color="red", s=70, label="fin")
    if esfera:  # esfera de 1 g: con el sensor quieto el punto queda sobre ella
        u, v = np.mgrid[0:2 * np.pi:24j, 0:np.pi:12j]
        ax.plot_wireframe(np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v),
                          color="lightgray", alpha=0.4, linewidth=0.4)
    lim = 1.1 * np.nanmax(np.abs([x, y, z]))
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (%s)" % unidad)
    ax.set_ylabel("Y (%s)" % unidad)
    ax.set_zlabel("Z (%s)" % unidad)
    ax.set_title(titulo)
    ax.legend(loc="upper left")
    fig.colorbar(puntos, ax=ax, shrink=0.6, pad=0.1, label="tiempo (s)")


def ventana_vectores(df):
    fig = plt.figure("Vectores del IMU", figsize=(14, 6.5))
    t = df["t_s"].to_numpy()
    _panel(fig, 1, df["ax_g"].to_numpy(), df["ay_g"].to_numpy(), df["az_g"].to_numpy(), t,
           "Aceleracion (direccion de la gravedad)", "g", esfera=True)
    _panel(fig, 2, df["gx_dps"].to_numpy(), df["gy_dps"].to_numpy(), df["gz_dps"].to_numpy(), t,
           "Giroscopio (velocidad de giro)", "grados/s")
    fig.suptitle("%d muestras, %.1f s" % (len(df), t[-1]))
    fig.tight_layout()
    return fig


# ---------------- Ventana 2: orientacion estimada ----------------
def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def _rotacion(w):
    """Matriz de rotacion para un giro w (vector, radianes) en ejes del sensor (Rodrigues)."""
    angulo = np.linalg.norm(w)
    if angulo < 1e-12:
        return np.eye(3)
    K = _skew(w / angulo)
    return np.eye(3) + np.sin(angulo) * K + (1 - np.cos(angulo)) * (K @ K)


def _actitud_inicial(a):
    """Rotacion cuyo 'arriba' coincide con la gravedad medida en la primera muestra."""
    a = a / np.linalg.norm(a)
    h = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    r0 = h - (h @ a) * a
    r0 /= np.linalg.norm(r0)
    return np.vstack([r0, np.cross(a, r0), a])


def estimar_orientacion(df):
    """Devuelve una matriz 3x3 por muestra: columnas = ejes X, Y, Z del sensor en el mundo.
    El mundo tiene Z hacia arriba. Sin magnetometro, el giro sobre la vertical (yaw) se va
    desviando poco a poco: es normal, el acelerometro no puede corregirlo."""
    a = df[["ax_g", "ay_g", "az_g"]].to_numpy()
    w = np.radians(df[["gx_dps", "gy_dps", "gz_dps"]].to_numpy())
    t = df["t_s"].to_numpy()

    if CALIBRAR_SESGO:
        inicio = np.degrees(w[t < 0.5])
        # Solo es offset si al inicio el sensor esta quieto (poca variacion) y el valor es chico
        if len(inicio) >= 10 and inicio.std(axis=0).max() < 1.0 and np.abs(inicio.mean(axis=0)).max() < 20:
            w = w - np.radians(inicio.mean(axis=0))

    R = _actitud_inicial(a[0])
    resultado = np.empty((len(df), 3, 3))
    resultado[0] = R
    arriba = np.array([0.0, 0.0, 1.0])
    for i in range(1, len(df)):
        dt = min(t[i] - t[i - 1], 0.1)
        giro = w[i].copy()
        norma = np.linalg.norm(a[i])
        if RANGO_ACEL_G[0] < norma < RANGO_ACEL_G[1]:
            giro += KP * np.cross(a[i] / norma, R.T @ arriba)  # empuja la inclinacion hacia la medida
        R = R @ _rotacion(giro * dt)
        U, _, Vt = np.linalg.svd(R)  # mantiene R como rotacion valida
        R = U @ Vt
        resultado[i] = R
    return resultado


CARAS = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4)]


def ventana_orientacion(df):
    Rs = estimar_orientacion(df)
    t = df["t_s"].to_numpy()
    modulo = np.sqrt(df["ax_g"] ** 2 + df["ay_g"] ** 2 + df["az_g"] ** 2).to_numpy()
    giro = np.sqrt(df["gx_dps"] ** 2 + df["gy_dps"] ** 2 + df["gz_dps"] ** 2).to_numpy()

    # Cuadros de la animacion: uno cada ~33 ms de pantalla (cada 100 ms en Colab, para que pese menos)
    base = 0.1 if EN_COLAB else 0.033
    paso = base * VELOCIDAD
    indices = np.searchsorted(t, np.arange(0, t[-1], paso))
    indices = np.minimum(indices, len(t) - 1)

    # En Colab cada cuadro se guarda como imagen dentro de la pagina: menos dpi = menos peso
    fig = plt.figure("Orientacion estimada", figsize=(8, 8), dpi=55 if EN_COLAB else 100)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.4, 1.4)
    ax.set_zlim(-1.4, 1.4)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("Este-Oeste")
    ax.set_ylabel("Norte-Sur")
    ax.set_zlabel("Arriba")
    ax.set_title("Orientacion del sensor (rojo X, verde Y, azul Z)")
    for eje, color in zip(np.eye(3), ("#ffaaaa", "#aaddaa", "#aaaaff")):  # ejes fijos del mundo
        ax.plot([0, 1.3 * eje[0]], [0, 1.3 * eje[1]], [0, 1.3 * eje[2]], color=color, lw=1, ls="--")

    semi = np.array([0.9, 0.6, 0.12])  # medidas relativas del modulo
    vertices = np.array([[sx, sy, sz] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]) * semi
    vertices = vertices[[0, 1, 3, 2, 4, 5, 7, 6]]  # orden de anillo para dibujar las caras
    caja = Poly3DCollection([], facecolor="#8ec8e8", edgecolor="k", alpha=0.55)
    ax.add_collection3d(caja)
    ejes = [ax.plot([], [], [], color=c, lw=3)[0] for c in ("red", "green", "blue")]
    estela, = ax.plot([], [], [], color="orange", lw=1, alpha=0.8)
    texto = fig.text(0.5, 0.94, "", ha="center")

    def dibujar(k):
        i = indices[k]
        R = Rs[i]
        mundo = vertices @ R.T
        caja.set_verts([mundo[list(cara)] for cara in CARAS])
        for n, linea in enumerate(ejes):
            punta = R[:, n] * 1.25
            linea.set_data_3d([0, punta[0]], [0, punta[1]], [0, punta[2]])
        desde = indices[max(0, k - 60)]  # estela: donde estuvo la punta del eje X
        punta_x = Rs[desde:i + 1, :, 0] * 1.25
        estela.set_data_3d(punta_x[:, 0], punta_x[:, 1], punta_x[:, 2])
        texto.set_text("t = %5.2f s    |a| = %.2f g    giro = %3.0f grados/s" % (t[i], modulo[i], giro[i]))
        return [caja, estela, texto, *ejes]

    dibujar(0)
    animacion = FuncAnimation(fig, dibujar, frames=len(indices), interval=base * 1000, repeat=True)
    fig._animacion = animacion  # sin esta referencia, matplotlib la borra y se congela
    return fig


# ---------------- Programa ----------------
def main():
    # Solo se toma como archivo un argumento que termine en .csv (Colab mete argumentos propios)
    archivos = [a for a in sys.argv[1:] if a.lower().endswith(".csv")]
    ruta = archivos[0] if archivos else RUTA
    if not os.path.exists(ruta):
        sys.exit("No encuentro el archivo: %s\n%s" % (ruta, (
            "En Colab: sube captura_ultima.csv con el icono de carpeta (panel izquierdo) y revisa que el "
            "nombre coincida." if EN_COLAB else "Revisa la ruta en RUTA o pasala como argumento.")))
    df = cargar(ruta)
    print("Archivo: %s" % ruta)
    print("%d muestras, %.1f s (%.0f Hz de media)" % (len(df), df["t_s"].iloc[-1],
                                                      len(df) / df["t_s"].iloc[-1]))
    if VER_VECTORES:
        ventana_vectores(df)
    if EN_COLAB:
        plt.show()  # muestra ya la figura de vectores como imagen
    if VER_ORIENTACION:
        fig = ventana_orientacion(df)
        if EN_COLAB:  # en Colab la animacion no se reproduce sola: se muestra con un reproductor
            from IPython.display import HTML, display
            plt.rcParams["animation.embed_limit"] = 200  # MB; el valor por defecto es muy bajo
            display(HTML(fig._animacion.to_jshtml()))
            plt.close(fig)
    if not EN_COLAB:
        plt.show()


if __name__ == "__main__":
    main()
