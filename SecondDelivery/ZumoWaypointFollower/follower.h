// Seguidor de trayectoria por tramos (Second Delivery, partes C y D).
//
// C portable (sin Pico SDK): se compila igual en el Zumo y en el simulador de PC
// (../sim/sim_follower.c). Así lo que se ajusta en la simulación es exactamente
// lo que corre en el robot.
//
// Idea:
//   1. A partir de la tabla de waypoints se arma la ruta como una lista de tramos:
//      RECTA entre dos waypoints con el mismo rumbo, ARCO entre dos con rumbo distinto
//      (entrada y salida de cada vuelta). El centro y el radio del arco se calculan
//      con los dos waypoints medidos.
//   2. En cada medición nueva de la cámara (x, y, θ):
//      - se localiza el robot sobre el tramo actual (avance t ∈ [0,1], error lateral e_y);
//      - si t ≥ 1 se pasa al siguiente tramo, en orden y sin saltarse waypoints;
//      - el rumbo de referencia es la tangente del tramo, corregida por el error lateral;
//      - ω = ω_feedforward (v/R en arcos) + k_θ·(error de rumbo);
//      - la velocidad sale de un perfil: rápido en rectas, frena antes de cada arco;
//      - se convierte (v, ω) a velocidad de rueda izquierda/derecha.
//   3. Al terminar el último tramo (llegar a P14) se detiene.
#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>
#include "waypoints.h"

typedef struct {
    float x;      // m
    float y;      // m
    float theta;  // rad, 0 = +x, antihorario positivo
} pose_t;

typedef enum { SEG_LINE = 0, SEG_ARC = 1 } seg_type_t;

typedef struct {
    seg_type_t type;
    uint8_t from, to;   // índices de waypoint (0-based)
    float ax, ay;       // punto inicial (waypoint "from")
    float bx, by;       // punto final (waypoint "to")
    float length;       // m (recta: |B-A|; arco: R·span)
    // Solo recta:
    float ux, uy;       // vector unitario A→B
    // Solo arco:
    float cx, cy, r;    // centro y radio
    int8_t dir;         // +1 = izquierda (antihorario), -1 = derecha (horario)
    float phi0;         // ángulo polar del punto inicial respecto al centro
    float span;         // ángulo total recorrido (rad, > 0)
} segment_t;

typedef struct {
    float v_straight;   // m/s   velocidad máxima en rectas
    float v_arc;        // m/s   velocidad en arcos
    float accel;        // m/s²  aceleración máxima
    float decel;        // m/s²  frenado para llegar a v_arc al inicio del arco
    float k_theta;      // 1/s   ganancia de rumbo
    // Peso del error lateral: k_e = k_theta / (4·ζ²·v). Linealizando, el error
    // lateral cumple ë + k_θ·ė + k_θ·k_e·v·e = 0; con esta k_e el amortiguamiento
    // queda en ζ a cualquier velocidad (si k_e fuera fija, al acelerar oscilaría).
    float zeta;         // -    amortiguamiento deseado (0.7–1.0)
    float k_e_max;      // 1/m   límite de k_e a velocidades bajas
    float approach_max; // rad   máximo ángulo de aproximación a la ruta (atan(k_e·e_y))
    float omega_max;    // rad/s límite de giro
    float wheel_base;   // m     separación entre orugas (medir en el Zumo)
    float wheel_max;    // m/s   velocidad máxima de una oruga
    float latency;      // s     retardo de la cámara a compensar (0 = no compensar)
} follower_params_t;

#define FOLLOWER_MAX_SEGMENTS WAYPOINT_COUNT

typedef struct {
    follower_params_t p;
    segment_t seg[FOLLOWER_MAX_SEGMENTS];
    int n_seg;
    int cur;            // tramo actual
    bool done;
    float v_cmd;        // velocidad lineal comandada (con rampa)
    float omega_cmd;    // última ω comandada
    // Diagnóstico de la última actualización:
    float t;            // avance en el tramo actual [0..1]
    float e_y;          // error lateral (m), + = robot a la izquierda de la ruta
    float e_theta;      // error de rumbo (rad)
} follower_t;

// Parámetros por defecto (punto de partida; ajustar con el simulador y en pista).
follower_params_t follower_default_params(void);

// Arma los tramos de una vuelta completa empezando en el waypoint start_idx.
void follower_init(follower_t *f, const follower_params_t *p,
                   const waypoint_t *wp, int n_wp, int start_idx);

// Procesa una pose nueva. dt = tiempo desde la pose anterior (s).
// Devuelve las velocidades de las orugas en m/s. Devuelve false cuando terminó la vuelta
// (en ese caso v_left = v_right = 0).
bool follower_update(follower_t *f, const pose_t *pose, float dt,
                     float *v_left, float *v_right);

// Proyecta (x, y) sobre un tramo: avance t (0 = inicio, 1 = fin), error lateral e_y
// (+ = a la izquierda de la ruta) y rumbo de la tangente.
void follower_project(const segment_t *s, float x, float y,
                      float *t, float *e_y, float *path_theta);

float follower_wrap_pi(float a);

// Comando para cuando faltan poses (hueco de WiFi): misma velocidad y solo el giro
// propio del tramo (v/R en un arco, 0 en una recta), sin la corrección de rumbo,
// que sin pose nueva se pasaría.
void follower_coast(const follower_t *f, float *v_left, float *v_right);

#endif
