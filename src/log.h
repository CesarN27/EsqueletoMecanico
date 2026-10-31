// Salida serie protegida con mutex para que varias tareas no mezclen sus líneas
#pragma once

void logIniciar();
void logPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Reserva el puerto serie para una tarea (ej. un volcado largo): las demás esperan en logPrintf.
// Dentro del bloqueo se escribe directo a Serial, no con logPrintf.
void logBloquear();
void logLiberar();
