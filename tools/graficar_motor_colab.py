"""Muestra en que posicion / coordenadas del IMU esta encendido el motor (Google Colab, Plotly).
CSV con columnas: t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,motor,enc_a,enc_b
Pega todo en una celda de Colab; si no encuentra el archivo te pide subirlo."""
import os
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

RUTA = "captura_20261029_113355.csv"  # nombre del CSV subido a Colab
COLOR_MOTOR = "red"       # puntos con el motor girando
COLOR_REPOSO = "#4c78a8"  # puntos con el motor apagado

if not os.path.exists(RUTA):
    from google.colab import files
    RUTA = next(iter(files.upload()))

df = pd.read_csv(RUTA).dropna().reset_index(drop=True)
df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
df["on"] = df["motor"] > 0
# Angulo del muslo (giro sobre el eje Y del IMU), el mismo que usa el firmware
df["theta"] = np.degrees(np.arctan2(df["ax_g"], -df["az_g"]))
on, off = df[df["on"]], df[~df["on"]]
print(f"{len(df)} muestras | motor encendido {len(on)} ({100*len(on)/len(df):.0f}%) = {len(on)/100:.1f} s")

# Tramos continuos con el motor encendido
cambios = np.flatnonzero(np.diff(df["on"].astype(int)))
ini = [0] * int(df["on"].iloc[0]) + list(cambios[df["on"].iloc[cambios + 1].to_numpy()] + 1)
fin = list(cambios[~df["on"].iloc[cambios + 1].to_numpy()] + 1) + [len(df)] * int(df["on"].iloc[-1])
tramos = list(zip(ini, fin))
print(f"{len(tramos)} encendidos del motor:")
for i, (a, b) in enumerate(tramos, 1):
    s = df.iloc[a:b]
    print(f"  #{i}: {s.t_s.iloc[0]:6.2f}-{s.t_s.iloc[-1]:6.2f} s | theta {s.theta.iloc[0]:6.1f} -> {s.theta.iloc[-1]:6.1f} deg"
          f" | ax {s.ax_g.mean():5.2f} ay {s.ay_g.mean():5.2f} az {s.az_g.mean():5.2f} g")


def nube(cx, cy, cz, unidad, nombre):
    """Dos nubes de puntos: motor apagado (azul, tenue) y motor encendido (rojo)."""
    hover = "t = %{customdata:.2f} s<br>X = %{x:.3f}<br>Y = %{y:.3f}<br>Z = %{z:.3f}<extra>%{fullData.name}</extra>"
    return [
        go.Scatter3d(x=off[cx], y=off[cy], z=off[cz], mode="markers", name="motor apagado",
                     marker=dict(size=2.5, color=COLOR_REPOSO, opacity=0.45),
                     customdata=off["t_s"], hovertemplate=hover),
        go.Scatter3d(x=on[cx], y=on[cy], z=on[cz], mode="markers", name="MOTOR ENCENDIDO",
                     marker=dict(size=3.5, color=COLOR_MOTOR), customdata=on["t_s"],
                     hovertemplate=hover)]


# ---- Figura 1: aceleracion y giroscopio en 3D ----
fig = make_subplots(rows=1, cols=2, specs=[[{"type": "scatter3d"}] * 2],
                    subplot_titles=("Aceleracion (g): hacia donde apunta el sensor", "Giroscopio (grados/s)"))
for j, (cols, nombre) in enumerate([(("ax_g", "ay_g", "az_g"), "acel"), (("gx_dps", "gy_dps", "gz_dps"), "giro")], 1):
    for k, tr in enumerate(nube(*cols, "", nombre)):
        tr.showlegend = j == 1
        fig.add_trace(tr, row=1, col=j)
fig.update_scenes(xaxis_title="X", yaxis_title="Y", zaxis_title="Z", aspectmode="cube")
fig.update_layout(height=650, title="Posicion del IMU: rojo = motor encendido", legend=dict(itemsizing="constant"))
fig.show()

# ---- Figura 2: en el tiempo (franjas rojas = motor encendido) ----
fig2 = make_subplots(rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.05,
                     subplot_titles=("Angulo del muslo theta (deg)", "Aceleracion (g)", "Giroscopio (grados/s)"))
fig2.add_trace(go.Scatter(x=df["t_s"], y=df["theta"], name="theta", line=dict(color="black")), row=1, col=1)
for c, col in zip(("ax_g", "ay_g", "az_g"), ("#1f77b4", "#ff7f0e", "#2ca02c")):
    fig2.add_trace(go.Scatter(x=df["t_s"], y=df[c], name=c, line=dict(color=col, width=1)), row=2, col=1)
for c, col in zip(("gx_dps", "gy_dps", "gz_dps"), ("#1f77b4", "#ff7f0e", "#2ca02c")):
    fig2.add_trace(go.Scatter(x=df["t_s"], y=df[c], name=c, line=dict(color=col, width=1)), row=3, col=1)
for a, b in tramos:
    fig2.add_vrect(x0=df["t_s"].iloc[a], x1=df["t_s"].iloc[b - 1], fillcolor="red", opacity=0.18, line_width=0)
fig2.update_xaxes(title_text="tiempo (s)", row=3, col=1)
fig2.update_layout(height=800, title="Franjas rojas = motor encendido")
fig2.show()

# ---- Figura 3: angulo vs velocidad angular (cada levantarse es un lazo) ----
fig3 = go.Figure()
fig3.add_trace(go.Scatter(x=off["theta"], y=off["gy_dps"], mode="markers", name="motor apagado",
                          marker=dict(size=3, color=COLOR_REPOSO, opacity=0.4)))
fig3.add_trace(go.Scatter(x=on["theta"], y=on["gy_dps"], mode="markers", name="MOTOR ENCENDIDO",
                          marker=dict(size=4, color=COLOR_MOTOR)))
fig3.update_layout(height=500, title="En que angulo y velocidad se enciende el motor",
                   xaxis_title="theta (deg)", yaxis_title="gy (grados/s)")
fig3.show()
