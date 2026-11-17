// Simulador en PC del seguidor de trayectoria.
//
// Compila el MISMO follower.c que corre en el Zumo y lo prueba contra un modelo
// de robot diferencial (uniciclo) con:
//   - respuesta de motores de primer orden (tau),
//   - asimetría entre orugas (una más lenta que la otra),
//   - cámara con frecuencia limitada, retardo y ruido en x, y, θ.
//
// Uso:   ./sim_follower [v_recta v_arco [hz_camara retardo_s]] > vuelta.csv
// Salida: CSV por stdout (t,x,y,theta,seg,vl,vr,e_true) y un resumen por stderr.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "follower.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float gauss(float sigma) {           // Box-Muller
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

// Distancia real del robot a la ruta: mínimo sobre todos los tramos.
static float path_error(const follower_t *f, float x, float y) {
    float best = 1e9f;
    for (int i = 0; i < f->n_seg; i++) {
        float t, e, th;
        const segment_t *s = &f->seg[i];
        follower_project(s, x, y, &t, &e, &th);
        if (t < 0.0f || t > 1.0f) {              // fuera del tramo: distancia al extremo
            float ex = (t < 0.0f) ? s->ax : s->bx, ey = (t < 0.0f) ? s->ay : s->by;
            float d = sqrtf((x - ex) * (x - ex) + (y - ey) * (y - ey));
            e = (e < 0.0f) ? -d : d;
        }
        if (fabsf(e) < fabsf(best)) best = e;
    }
    return best;
}

// Exporta la ruta deseada (los mismos tramos que arma el Zumo) muestreada cada ~5 mm.
static int dump_route(void) {
    follower_params_t p = follower_default_params();
    follower_t f;
    follower_init(&f, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);
    printf("x,y,seg\n");
    for (int i = 0; i < f.n_seg; i++) {
        const segment_t *s = &f.seg[i];
        int n = (int)(s->length / 0.005f) + 1;
        for (int k = 0; k <= n; k++) {
            float u = (float)k / n;
            if (s->type == SEG_LINE) {
                printf("%.4f,%.4f,%d\n", s->ax + u * (s->bx - s->ax), s->ay + u * (s->by - s->ay), i);
            } else {
                float phi = s->phi0 + s->dir * u * s->span;
                printf("%.4f,%.4f,%d\n", s->cx + s->r * cosf(phi), s->cy + s->r * sinf(phi), i);
            }
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc >= 2 && argv[1][0] == '-' && argv[1][1] == '-') return dump_route();
    follower_params_t p = follower_default_params();
    float cam_hz = 8.0f, cam_delay = 0.12f;   // ~8 poses/s medido en Wireless
    if (argc >= 3) { p.v_straight = atof(argv[1]); p.v_arc = atof(argv[2]); }
    if (argc >= 5) { cam_hz = atof(argv[3]); cam_delay = atof(argv[4]); }
    p.latency = cam_delay;
    // Ajuste de ganancias sin recompilar: ZETA=0.9 K_TH=5 ./sim_follower ...
    if (getenv("ZETA")) p.zeta = atof(getenv("ZETA"));
    if (getenv("K_TH")) p.k_theta = atof(getenv("K_TH"));
    if (getenv("ACC"))  p.accel = p.decel = atof(getenv("ACC"));
    if (getenv("APPROACH")) p.approach_max = atof(getenv("APPROACH"));
    // Ráfagas de WiFi (medidas el 17/11): cada BURST_T s hay un hueco de BURST s sin
    // poses; las que llegan durante el hueco se entregan juntas al final y el Zumo
    // usa solo la última. COAST_MS > 0: tras ese hueco el Zumo pasa a follower_coast().
    float burst = getenv("BURST") ? atof(getenv("BURST")) : 0.0f;
    float burst_t = getenv("BURST_T") ? atof(getenv("BURST_T")) : 0.8f;
    float coast_ms = getenv("COAST_MS") ? atof(getenv("COAST_MS")) : 0.0f;
    float t_max = getenv("T_MAX") ? atof(getenv("T_MAX")) : 60.0f;

    const float noise_xy = 0.005f;                    // σ = 5 mm (≈ ±1 cm)
    const float noise_th = 0.5f * (float)M_PI / 180;  // σ = 0.5° (≈ ±1°)
    const float tau = 0.08f;                          // s, respuesta de motores
    const float asym = 0.95f;                         // oruga izquierda 5 % más lenta
    const float dt = 0.001f;
    srand(1);

    follower_t f;
    follower_init(&f, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);

    // Arranca sobre P14, con rumbo medido.
    const waypoint_t *w0 = &WAYPOINTS[WAYPOINT_START];
    float x = w0->x, y = w0->y, th = w0->theta_deg * (float)M_PI / 180.0f;
    float vl = 0, vr = 0, cl = 0, cr = 0;

    // Cola de poses retrasadas (retardo de cámara).
    enum { QN = 64 };
    pose_t q[QN]; float qt[QN]; int qh = 0, qn = 0;
    float next_cam = 0.0f, last_upd = 0.0f, max_err = 0.0f, sum_err2 = 0.0f;
    long n_err = 0;
    int last_seg = -1;

    printf("t,x,y,theta,seg,vl,vr,e_true\n");
    float t;
    bool coasting = false;
    for (t = 0.0f; t < t_max; t += dt) {
        // Cámara: toma la foto ahora, llega "cam_delay" después.
        if (t >= next_cam) {
            int i = (qh + qn) % QN;
            q[i].x = x + gauss(noise_xy);
            q[i].y = y + gauss(noise_xy);
            q[i].theta = th + gauss(noise_th);
            qt[i] = t + cam_delay;
            if (burst > 0.0f) {                       // llega dentro de un hueco: se retiene
                float ph = fmodf(qt[i], burst_t);
                if (ph < burst) qt[i] += burst - ph;
            }
            if (qn < QN) qn++;
            next_cam += 1.0f / cam_hz;
        }
        if (qn > 0 && t >= qt[qh]) {                  // llega una pose al Zumo
            pose_t meas = q[qh];
            qh = (qh + 1) % QN; qn--;
            while (qn > 0 && t >= qt[qh]) { meas = q[qh]; qh = (qh + 1) % QN; qn--; }  // ráfaga
            coasting = false;
            bool running = follower_update(&f, &meas, t - last_upd, &cl, &cr);
            last_upd = t;
            if (running && f.cur != last_seg) {
                const segment_t *s = &f.seg[f.cur];
                fprintf(stderr, "t=%5.2f s  tramo %2d: P%d -> P%d (%s)\n", t, f.cur,
                        s->from + 1, s->to + 1, s->type == SEG_LINE ? "recta" : "arco");
                last_seg = f.cur;
            }
            if (!running) break;
        }
        if (coast_ms > 0.0f && !coasting && t - last_upd > coast_ms / 1000.0f) {
            follower_coast(&f, &cl, &cr);
            coasting = true;
        }
        // Planta: motores de primer orden + uniciclo.
        vl += (cl * asym - vl) * dt / tau;
        vr += (cr - vr) * dt / tau;
        float v = 0.5f * (vl + vr), w = (vr - vl) / p.wheel_base;
        x += v * cosf(th) * dt;
        y += v * sinf(th) * dt;
        th += w * dt;

        float e = path_error(&f, x, y);
        if (e < 1e8f) {
            if (fabsf(e) > max_err) max_err = fabsf(e);
            sum_err2 += e * e; n_err++;
        }
        if (((long)(t / dt)) % 20 == 0)
            printf("%.3f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.4f\n", t, x, y, th, f.cur, cl, cr, e);
    }

    float dx = x - w0->x, dy = y - w0->y;
    fprintf(stderr, "\nResumen: v_recta=%.2f v_arco=%.2f camara=%.0f Hz retardo=%.0f ms\n",
            p.v_straight, p.v_arc, cam_hz, cam_delay * 1000);
    fprintf(stderr, "  vuelta %s en %.2f s, tramos recorridos %d/%d\n",
            f.done ? "COMPLETA" : "INCOMPLETA", t, f.done ? f.n_seg : f.cur, f.n_seg);
    fprintf(stderr, "  error lateral max %.1f mm, RMS %.1f mm\n",
            max_err * 1000, sqrtf(sum_err2 / (n_err ? n_err : 1)) * 1000);
    fprintf(stderr, "  distancia final a P14: %.1f mm\n", sqrtf(dx * dx + dy * dy) * 1000);
    return f.done ? 0 : 1;
}
