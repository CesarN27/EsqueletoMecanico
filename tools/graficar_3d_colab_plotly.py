"""Graficas 3D INTERACTIVAS del IMU (MPU6050) con Plotly, pensadas para Google Colab.

Antes: sube tu CSV (icono de carpeta del panel izquierdo) y ajusta RUTA.
Se pueden girar con el mouse, acercar con la rueda y ver los valores al pasar el cursor.

Figura 1: aceleracion (g) y giroscopio (grados/s) en 3D, coloreados por tiempo.
Figura 2: animacion de la orientacion estimada del sensor, con boton de reproducir, pausa y
          barra de tiempo (la camara se puede girar mientras se reproduce).
"""
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ---------------- Configuracion ----------------
RUTA = "/content/captura_ultima.csv"
VER_VECTORES = True
VER_ORIENTACION = True
VELOCIDAD = 1.0        # animacion: 1.0 = tiempo real, 0.5 = camara lenta, 2.0 = el doble de rapido
CALIBRAR_SESGO = True  # resta el offset del giroscopio usando el inicio, si el sensor esta quieto
KP = 2.0               # cuanto corrige la inclinacion el acelerometro (mas alto = confia mas en el)
RANGO_ACEL_G = (0.75, 1.35)  # solo corrige con el acelerometro si |a| esta cerca de 1 g
PASO_ANIMACION_S = 0.1       # un cuadro de animacion cada 0.1 s de datos (menos = mas pesado)


# ---------------- Datos ----------------
def cargar(ruta):
    df = pd.read_csv(ruta).dropna().reset_index(drop=True)
    saltos = np.where(np.diff(df["t_ms"].to_numpy()) < 0)[0]
    if len(saltos):  # la placa se reinicio durante la captura: el tiempo retrocede
        df = df.iloc[saltos[-1] + 1:].reset_index(drop=True)
        print("Aviso: la placa se reinicio en la captura, se usa solo el ultimo tramo.")
    if len(df) < 20:
        raise SystemExit("El archivo tiene muy pocas filas para graficar.")
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    return df


# ---------------- Figura 1: vectores en 3D ----------------
def _nube(x, y, z, t, nombre, unidad, mostrar_barra):
    """Linea tenue + puntos coloreados por tiempo; al pasar el mouse muestra t y los valores."""
    return go.Scatter3d(
        x=x, y=y, z=z, mode="lines+markers", name=nombre, showlegend=False,
        line=dict(color="rgba(120,120,120,0.35)", width=1),
        marker=dict(size=3, color=t, colorscale="Viridis", showscale=mostrar_barra,
                    colorbar=dict(title="tiempo (s)", x=1.03, len=0.7)),
        customdata=t,
        hovertemplate="t = %{customdata:.2f} s<br>X = %{x:.3f} " + unidad +
                      "<br>Y = %{y:.3f} " + unidad + "<br>Z = %{z:.3f} " + unidad + "<extra></extra>")


def _extremos(x, y, z, fila_col):
    return [
        go.Scatter3d(x=[x[0]], y=[y[0]], z=[z[0]], mode="markers", name="inicio",
                     marker=dict(size=8, color="green"), showlegend=(fila_col == 1)),
        go.Scatter3d(x=[x[-1]], y=[y[-1]], z=[z[-1]], mode="markers", name="fin",
                     marker=dict(size=8, color="red"), showlegend=(fila_col == 1)),
    ]


def figura_vectores(df):
    t = df["t_s"].to_numpy()
    ax_, ay_, az_ = (df[c].to_numpy() for c in ("ax_g", "ay_g", "az_g"))
    gx_, gy_, gz_ = (df[c].to_numpy() for c in ("gx_dps", "gy_dps", "gz_dps"))

    fig = make_subplots(rows=1, cols=2, specs=[[{"type": "scene"}, {"type": "scene"}]],
                        subplot_titles=("Aceleracion (direccion de la gravedad)",
                                        "Giroscopio (velocidad de giro)"))

    # Esfera de 1 g: con el sensor quieto el punto queda sobre ella
    u, v = np.mgrid[0:2 * np.pi:40j, 0:np.pi:20j]
    fig.add_trace(go.Surface(x=np.cos(u) * np.sin(v), y=np.sin(u) * np.sin(v), z=np.cos(v),
                             colorscale=[[0, "lightgray"], [1, "lightgray"]], showscale=False,
                             opacity=0.12, hoverinfo="skip"), row=1, col=1)
    fig.add_trace(_nube(ax_, ay_, az_, t, "aceleracion", "g", False), row=1, col=1)
    for trazo in _extremos(ax_, ay_, az_, 1):
        fig.add_trace(trazo, row=1, col=1)

    fig.add_trace(_nube(gx_, gy_, gz_, t, "giroscopio", "grados/s", True), row=1, col=2)
    for trazo in _extremos(gx_, gy_, gz_, 2):
        fig.add_trace(trazo, row=1, col=2)

    def escena(x, y, z, unidad):
        lim = 1.1 * float(np.nanmax(np.abs([x, y, z])))
        eje = dict(range=[-lim, lim])
        return dict(xaxis=dict(title="X (%s)" % unidad, **eje), yaxis=dict(title="Y (%s)" % unidad, **eje),
                    zaxis=dict(title="Z (%s)" % unidad, **eje), aspectmode="cube")

    fig.update_layout(scene=escena(ax_, ay_, az_, "g"), scene2=escena(gx_, gy_, gz_, "grados/s"),
                      title="%d muestras, %.1f s" % (len(df), t[-1]), height=620,
                      margin=dict(l=0, r=0, t=70, b=0), legend=dict(x=0.02, y=0.95))
    return fig


# ---------------- Figura 2: orientacion estimada ----------------
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


def _caja():
    """Vertices (anillo inferior 0-3, superior 4-7) y triangulos de una caja que representa el modulo."""
    semi = np.array([0.9, 0.6, 0.12])  # medidas relativas del modulo
    anillo = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    vertices = np.vstack([np.column_stack([anillo, np.full(4, -1)]),
                          np.column_stack([anillo, np.full(4, 1)])]) * semi
    triangulos = [(0, 1, 2), (0, 2, 3), (4, 5, 6), (4, 6, 7)]
    for k in range(4):
        sig = (k + 1) % 4
        triangulos += [(k, sig, 4 + sig), (k, 4 + sig, 4 + k)]
    i, j, k = zip(*triangulos)
    return vertices, list(i), list(j), list(k)


def figura_orientacion(df):
    Rs = estimar_orientacion(df)
    t = df["t_s"].to_numpy()
    modulo = np.sqrt(df["ax_g"] ** 2 + df["ay_g"] ** 2 + df["az_g"] ** 2).to_numpy()
    giro = np.sqrt(df["gx_dps"] ** 2 + df["gy_dps"] ** 2 + df["gz_dps"] ** 2).to_numpy()

    paso = PASO_ANIMACION_S * VELOCIDAD
    indices = np.minimum(np.searchsorted(t, np.arange(0, t[-1], paso)), len(t) - 1)
    duracion_ms = int(PASO_ANIMACION_S * 1000)  # tiempo en pantalla de cada cuadro

    vertices, tri_i, tri_j, tri_k = _caja()
    colores = ("red", "green", "blue")

    def cuadro(k):
        """Datos de las 6 trazas que cambian: caja, 3 ejes del sensor, estela y texto."""
        i = indices[k]
        R = Rs[i]
        mundo = vertices @ R.T
        trazas = [go.Mesh3d(x=mundo[:, 0], y=mundo[:, 1], z=mundo[:, 2])]
        for n in range(3):
            punta = R[:, n] * 1.25
            trazas.append(go.Scatter3d(x=[0, punta[0]], y=[0, punta[1]], z=[0, punta[2]]))
        desde = indices[max(0, k - 60)]  # estela: donde estuvo la punta del eje X
        punta_x = Rs[desde:i + 1, :, 0] * 1.25
        trazas.append(go.Scatter3d(x=punta_x[:, 0], y=punta_x[:, 1], z=punta_x[:, 2]))
        trazas.append(go.Scatter3d(
            x=[0], y=[0], z=[1.5],
            text=["t = %.2f s | |a| = %.2f g | giro = %.0f grados/s" % (t[i], modulo[i], giro[i])]))
        return trazas

    # Cuadro inicial: trazas dinamicas (0-5) y luego los ejes fijos del mundo (6-8)
    inicial = cuadro(0)
    fig = go.Figure(data=[
        go.Mesh3d(x=inicial[0].x, y=inicial[0].y, z=inicial[0].z, i=tri_i, j=tri_j, k=tri_k,
                  color="lightblue", opacity=0.6, flatshading=True, name="sensor", hoverinfo="skip"),
        *[go.Scatter3d(x=inicial[1 + n].x, y=inicial[1 + n].y, z=inicial[1 + n].z, mode="lines",
                       line=dict(color=colores[n], width=8), name="eje " + "XYZ"[n], hoverinfo="skip")
          for n in range(3)],
        go.Scatter3d(x=inicial[4].x, y=inicial[4].y, z=inicial[4].z, mode="lines",
                     line=dict(color="orange", width=3), name="estela eje X", hoverinfo="skip"),
        go.Scatter3d(x=inicial[5].x, y=inicial[5].y, z=inicial[5].z, text=inicial[5].text,
                     mode="text", showlegend=False, hoverinfo="skip"),
        *[go.Scatter3d(x=[0, 1.3 * e[0]], y=[0, 1.3 * e[1]], z=[0, 1.3 * e[2]], mode="lines",
                       line=dict(color=c, width=1, dash="dash"), showlegend=False, hoverinfo="skip")
          for e, c in zip(np.eye(3), ("#ffaaaa", "#aaddaa", "#aaaaff"))],
    ])

    fig.frames = [go.Frame(data=cuadro(k), traces=[0, 1, 2, 3, 4, 5], name=str(k))
                  for k in range(len(indices))]

    animar = {"frame": {"duration": duracion_ms, "redraw": True}, "transition": {"duration": 0}}
    detener = {"frame": {"duration": 0, "redraw": False}, "mode": "immediate", "transition": {"duration": 0}}
    pasos = [dict(method="animate", label=("%.0f s" % t[i]) if k % 10 == 0 else "",
                  args=[[str(k)], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate",
                                   "transition": {"duration": 0}}])
             for k, i in enumerate(indices)]

    eje = dict(range=[-1.5, 1.5])
    fig.update_layout(
        title="Orientacion del sensor (rojo X, verde Y, azul Z). Gira la vista con el mouse.",
        height=720, margin=dict(l=0, r=0, t=60, b=0), uirevision="fijo",
        scene=dict(xaxis=dict(title="Este-Oeste", **eje), yaxis=dict(title="Norte-Sur", **eje),
                   zaxis=dict(title="Arriba", **eje), aspectmode="cube"),
        updatemenus=[dict(type="buttons", direction="left", showactive=False, x=0.02, y=0.0,
                          xanchor="left", yanchor="top", pad=dict(t=40),
                          buttons=[dict(label="Reproducir", method="animate",
                                        args=[None, dict(animar, fromcurrent=True)]),
                                   dict(label="Pausa", method="animate", args=[[None], detener])])],
        sliders=[dict(active=0, x=0.22, len=0.76, y=0.0, yanchor="top", pad=dict(t=30),
                      currentvalue=dict(prefix="t = ", visible=True), steps=pasos)])
    return fig


# ---------------- Programa ----------------
df = cargar(RUTA)
print("Archivo: %s" % RUTA)
print("%d muestras, %.1f s (%.0f Hz de media)" % (len(df), df["t_s"].iloc[-1], len(df) / df["t_s"].iloc[-1]))

if VER_VECTORES:
    figura_vectores(df).show()
if VER_ORIENTACION:
    figura_orientacion(df).show()
