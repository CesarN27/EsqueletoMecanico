// Comandos por el monitor serie, atendidos en su propia tarea
#pragma once

// Lanza la tarea que lee el serie. Comandos:
//   <numero>  milisegundos de giro de los motores
//   s              mostrar / ocultar los datos del IMU en el monitor
//   t / e          diagnóstico completo del IMU / estado y estadísticas
//   v / x / p / r  volcar CSV del IMU / borrar / pausar / reanudar
void comandosIniciar();
