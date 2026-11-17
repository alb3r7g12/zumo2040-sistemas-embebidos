#include "follower.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)
#define TWO_PI (2.0f * (float)M_PI)

float follower_wrap_pi(float a) {
    while (a > (float)M_PI)   a -= TWO_PI;
    while (a <= -(float)M_PI) a += TWO_PI;
    return a;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

follower_params_t follower_default_params(void) {
    follower_params_t p = {
        .v_straight = 0.30f,
        .v_arc      = 0.15f,
        .accel      = 0.60f,
        .decel      = 0.60f,
        // Ganancias elegidas con el barrido del simulador (ver SecondDelivery/README.md):
        // k_theta más alta oscila por el retardo y la baja frecuencia de la cámara.
        // Cámara a ~8 Hz (reporte Wireless). Con retardo de 100 ms k_theta=5 va bien,
        // pero con 200 ms oscila; 4 / 0.8 aguanta ambos casos.
        .k_theta    = 4.0f,
        .zeta       = 0.8f,
        .k_e_max    = 30.0f,
        // Vuelta 4 (17/11): con 6.5 cm de error a 0.08 m/s el ángulo pedido llegaba a ~50°
        // y el Zumo se cruzaba al otro lado (zigzag). 35° corrige igual sin pasarse.
        .approach_max = 0.61f,
        .omega_max  = 6.0f,
        .wheel_base = 0.085f,
        .wheel_max  = 0.60f,
        .latency    = 0.05f,
    };
    return p;
}

// Arco entre A (rumbo θa) y B (rumbo θb). El sentido de giro sale de en qué lado
// del rumbo inicial queda B (producto cruz); así funciona también para medias
// vueltas de 180°, donde la diferencia de rumbos no dice hacia dónde girar.
static void build_arc(segment_t *s, float ta, float tb) {
    float hx = cosf(ta), hy = sinf(ta);
    float dx = s->bx - s->ax, dy = s->by - s->ay;
    float cross = hx * dy - hy * dx;
    s->dir = (cross >= 0.0f) ? 1 : -1;

    // Normal hacia el centro desde cada waypoint: izquierda del rumbo si dir=+1.
    float nax = -sinf(ta) * s->dir, nay = cosf(ta) * s->dir;
    float nbx = -sinf(tb) * s->dir, nby = cosf(tb) * s->dir;
    // Centro = A + R·na = B + R·nb  →  R por mínimos cuadrados.
    float mx = nax - nbx, my = nay - nby;
    float den = mx * mx + my * my;
    float r = (den > 1e-6f) ? -((s->ax - s->bx) * mx + (s->ay - s->by) * my) / den
                            : 0.5f * sqrtf(dx * dx + dy * dy);
    if (r < 0.02f) r = 0.5f * sqrtf(dx * dx + dy * dy);
    s->r = r;
    s->cx = 0.5f * ((s->ax + r * nax) + (s->bx + r * nbx));
    s->cy = 0.5f * ((s->ay + r * nay) + (s->by + r * nby));

    s->phi0 = atan2f(s->ay - s->cy, s->ax - s->cx);
    float phi1 = atan2f(s->by - s->cy, s->bx - s->cx);
    float span = (phi1 - s->phi0) * s->dir;          // ángulo en el sentido de giro
    while (span <= 0.0f)   span += TWO_PI;
    while (span > TWO_PI)  span -= TWO_PI;
    s->span = span;
    s->length = r * span;
}

void follower_init(follower_t *f, const follower_params_t *p,
                   const waypoint_t *wp, int n_wp, int start_idx) {
    f->p = *p;
    f->n_seg = 0;
    for (int k = 0; k < n_wp && k < FOLLOWER_MAX_SEGMENTS; k++) {
        int i = (start_idx + k) % n_wp;
        int j = (i + 1) % n_wp;
        segment_t *s = &f->seg[f->n_seg++];
        s->from = (uint8_t)i;
        s->to = (uint8_t)j;
        s->ax = wp[i].x; s->ay = wp[i].y;
        s->bx = wp[j].x; s->by = wp[j].y;
        float ta = DEG2RAD(wp[i].theta_deg), tb = DEG2RAD(wp[j].theta_deg);
        float dx = s->bx - s->ax, dy = s->by - s->ay;
        float len = sqrtf(dx * dx + dy * dy);
        if (fabsf(follower_wrap_pi(tb - ta)) < DEG2RAD(30)) {
            s->type = SEG_LINE;
            s->length = len;
            s->ux = dx / len; s->uy = dy / len;
        } else {
            s->type = SEG_ARC;
            build_arc(s, ta, tb);
        }
    }
    f->cur = 0;
    f->done = false;
    f->v_cmd = 0.0f;
    f->omega_cmd = 0.0f;
    f->t = f->e_y = f->e_theta = 0.0f;
}

// Proyecta la pose sobre el tramo: avance t, error lateral e_y y rumbo de la tangente.
void follower_project(const segment_t *s, float x, float y,
                    float *t, float *e_y, float *path_theta) {
    if (s->type == SEG_LINE) {
        float px = x - s->ax, py = y - s->ay;
        *t = (px * s->ux + py * s->uy) / s->length;
        *e_y = s->ux * py - s->uy * px;             // + = a la izquierda
        *path_theta = atan2f(s->uy, s->ux);
    } else {
        float rx = x - s->cx, ry = y - s->cy;
        float d = sqrtf(rx * rx + ry * ry);
        float phi = atan2f(ry, rx);
        // Avance angular centrado a la mitad del arco, para que un robot un poco
        // antes del inicio no parezca haber terminado (ambigüedad de ±π).
        float half = 0.5f * s->span;
        float trav = follower_wrap_pi((phi - s->phi0) * s->dir - half) + half;
        *t = trav / s->span;
        *e_y = s->dir * (s->r - d);
        *path_theta = phi + s->dir * 0.5f * (float)M_PI;
    }
}

bool follower_update(follower_t *f, const pose_t *pose, float dt,
                     float *v_left, float *v_right) {
    *v_left = *v_right = 0.0f;
    if (f->done) return false;

    // Compensación de retardo: se adelanta la pose con el último comando.
    float x = pose->x, y = pose->y, th = pose->theta;
    if (f->p.latency > 0.0f) {
        float L = f->p.latency;
        th += f->omega_cmd * L * 0.5f;
        x += f->v_cmd * cosf(th) * L;
        y += f->v_cmd * sinf(th) * L;
        th += f->omega_cmd * L * 0.5f;
    }

    // Localizar en el tramo y avanzar mientras se haya pasado el final.
    float t, e_y, path_th;
    const segment_t *s;
    for (;;) {
        s = &f->seg[f->cur];
        follower_project(s, x, y, &t, &e_y, &path_th);
        if (t < 1.0f) break;
        f->cur++;
        if (f->cur >= f->n_seg) {           // llegó al waypoint final
            f->done = true;
            f->v_cmd = f->omega_cmd = 0.0f;
            f->t = 1.0f;
            return false;
        }
    }

    // Perfil de velocidad.
    float v_target;
    if (s->type == SEG_ARC) {
        v_target = f->p.v_arc;
    } else {
        float remain = (1.0f - clampf(t, 0.0f, 1.0f)) * s->length;
        float v_brake = sqrtf(f->p.v_arc * f->p.v_arc + 2.0f * f->p.decel * remain);
        v_target = fminf(f->p.v_straight, v_brake);
    }
    if (v_target > f->v_cmd) f->v_cmd = fminf(v_target, f->v_cmd + f->p.accel * dt);
    else                     f->v_cmd = v_target;   // frenar es inmediato
    float v = f->v_cmd;

    // Dirección: rumbo de referencia = tangente − atan(k_e·e_y).
    float k_e = f->p.k_theta / (4.0f * f->p.zeta * f->p.zeta * fmaxf(v, 0.05f));
    if (k_e > f->p.k_e_max) k_e = f->p.k_e_max;
    float th_ref = path_th - clampf(atanf(k_e * e_y), -f->p.approach_max, f->p.approach_max);
    float e_th = follower_wrap_pi(th_ref - th);
    float omega_ff = (s->type == SEG_ARC) ? s->dir * v / s->r : 0.0f;
    float omega = clampf(omega_ff + f->p.k_theta * e_th, -f->p.omega_max, f->p.omega_max);
    f->omega_cmd = omega;

    // (v, ω) → orugas. Si una satura, se escalan ambas para conservar la curvatura.
    float vl = v - omega * f->p.wheel_base * 0.5f;
    float vr = v + omega * f->p.wheel_base * 0.5f;
    float m = fmaxf(fabsf(vl), fabsf(vr));
    if (m > f->p.wheel_max) { vl *= f->p.wheel_max / m; vr *= f->p.wheel_max / m; }

    f->t = t; f->e_y = e_y; f->e_theta = e_th;
    *v_left = vl; *v_right = vr;
    return true;
}

void follower_coast(const follower_t *f, float *v_left, float *v_right) {
    *v_left = *v_right = 0.0f;
    if (f->done) return;
    const segment_t *s = &f->seg[f->cur];
    float v = f->v_cmd;
    float omega = (s->type == SEG_ARC) ? s->dir * v / s->r : 0.0f;
    *v_left = v - omega * f->p.wheel_base * 0.5f;
    *v_right = v + omega * f->p.wheel_base * 0.5f;
}
