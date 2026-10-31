"""Simula en la PC el detector de levantarse (misma logica que src/levantarse.cpp) sobre un CSV del IMU.
Uso: python tools/sim_levantarse.py datos/captura_ultima3.csv [salida.png]"""
import sys, math
import numpy as np

# --- Parametros (espejo de levantarse.cpp) ---
ALFA_GY = 0.15         # LPF de gy (100 Hz -> ~2.8 Hz)
ALFA_TH = 0.10         # LPF del angulo
GY_SUBE = 10.0         # dps: gy filtrado por encima = subiendo
GY_PARA = 3.0          # dps: por debajo = ya no sube
MS_CONFIRMAR = 80      # gy > GY_SUBE sostenido para arrancar
MS_PARAR = 100         # gy < GY_PARA sostenido para apagar
ARRANQUE_DEG = 2.0     # el angulo ya debe haber subido esto sobre el valle
SUBIDA_MIN_DEG = 8.0   # subida minima para considerarlo un levantarse
BAJA_DEG = 5.0         # desde el pico: ya se esta sentando -> re-armar
CAIDA_PICO_DEG = 3.0   # apaga si el angulo cae esto desde el pico
FUGA_DEG_S = 0.3       # el valle sube lento para seguir la deriva
MOTOR_MAX_MS = 4000

def correr(t, a, g):
    thf = gyf = valle = pico = 0.0
    est = 'IDLE'; conf = parar = 0.0; t_ini = 0.0; primera = True
    motor = np.zeros(len(t), bool); ev = []; th_o = np.zeros(len(t)); gy_o = np.zeros(len(t))
    for i in range(len(t)):
        th = math.degrees(math.atan2(a[i, 0], -a[i, 2]))
        if primera: thf = th; valle = th; primera = False
        dt = (t[i] - t[i-1]) if i else 10.0
        thf += ALFA_TH * (th - thf); gyf += ALFA_GY * (g[i, 1] - gyf)
        if est == 'IDLE':
            valle = thf if thf < valle else valle + FUGA_DEG_S * dt / 1000
            conf = conf + dt if gyf > GY_SUBE else 0
            if conf >= MS_CONFIRMAR and thf - valle >= ARRANQUE_DEG:
                est = 'UP'; pico = thf; t_ini = t[i]; parar = 0; ev.append((t[i], 'MOTOR ON'))
        elif est == 'UP':
            pico = max(pico, thf)
            parar = parar + dt if (gyf < GY_PARA or thf < pico - CAIDA_PICO_DEG) else 0
            if parar >= MS_PARAR or t[i] - t_ini > MOTOR_MAX_MS:
                sub = pico - valle
                if sub >= SUBIDA_MIN_DEG: est = 'ARRIBA'; ev.append((t[i], f'MOTOR OFF (arriba, subio {sub:.0f} deg)'))
                else: est = 'IDLE'; valle = thf; conf = 0; ev.append((t[i], f'MOTOR OFF (falsa alarma, {sub:.0f} deg)'))
        else:  # ARRIBA
            pico = max(pico, thf)
            if thf < pico - BAJA_DEG: est = 'IDLE'; valle = thf; conf = 0
        motor[i] = est == 'UP'; th_o[i] = thf; gy_o[i] = gyf
    return motor, ev, th_o, gy_o

if __name__ == '__main__':
    d = np.genfromtxt(sys.argv[1], delimiter=',', skip_header=1)
    t = d[:, 0] - d[0, 0]; a = d[:, 1:4]; g = d[:, 4:7]
    motor, ev, th, gy = correr(t, a, g)
    for e in ev: print(f'{e[0]/1000:6.2f} s  {e[1]}')
    if len(sys.argv) > 2:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        f, ax = plt.subplots(3, 1, figsize=(14, 8), sharex=True); ts = t / 1000
        ax[0].plot(ts, th); ax[0].set_ylabel('theta (deg)'); ax[1].plot(ts, gy); ax[1].set_ylabel('gy filtrado'); ax[1].axhline(GY_SUBE, c='r', ls=':'); ax[1].axhline(GY_PARA, c='orange', ls=':')
        ax[2].fill_between(ts, motor, step='post', color='g'); ax[2].set_ylabel('MOTOR')
        for x in ax: x.grid()
        for x in (ax[0], ax[2]):
            x.fill_between(ts, *x.get_ylim(), where=motor, color='g', alpha=.15)
        plt.tight_layout(); plt.savefig(sys.argv[2], dpi=70)
