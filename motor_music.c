/*
 * motor_music.c — Motor titresim melodisi oynaticisi + Ceddin Deden melodisi
 *
 * Osilayon mekanizmasi
 * --------------------
 * Her tick'te (1 ms) Bresenham biriktiricisi 1000 us ileri sarılır.
 * Birikim, notanin yarim periyodunu (half_period_us = 500000 / freq_hz)
 * astığında yön degistirilir ve birikim tasinir.  Bu yontem, 1 ms'lik sabit
 * tick sinirlamasi altinda bile dogru ortalama frekans uretir.
 *
 * Sessizlik (MUS_REST) sirasında surucu hedef hizi 0 RPM'de tutulur.
 *
 * Melodi verisi
 * ------------
 * g_ceddin_deden[] dizisi, kamu mali geleneksel Osmanli mehter marsının
 * Re minoru üzerindeki ana temasinı kodlar.  Nota frekans degerleri standart
 * (A4 = 440 Hz) esas alinarak hesaplanmistir; ancak 1 ms tick cozunurlugu
 * nedeniyle surucu ciktisi en yakin elde edilebilir frekansa yuvarlaniir.
 */

#include "motor_music.h"
#include "cia402_app.h"  /* cia402_set_target_rpm() */

/* ── Oynatici yardimcilari ───────────────────────────────────────────────── */

static void load_note(motor_music_t *p)
{
    p->note_ms_left = p->notes[p->note_idx].dur_ms;
    p->osc_acc_us   = 0U;
    p->osc_phase    = false;
}

/* ── Genel API ───────────────────────────────────────────────────────────── */

void motor_music_start(motor_music_t *p,
                       const mus_note_t *notes, uint16_t len,
                       uint16_t node_id, float ampl_rpm, uint16_t gap_ms, bool loop)
{
    p->notes    = notes;
    p->len      = len;
    p->node_id  = node_id;
    p->ampl_rpm = ampl_rpm;
    p->gap_ms   = gap_ms;
    p->loop     = loop;

    p->note_idx    = 0U;
    p->osc_acc_us  = 0U;
    p->osc_phase   = false;
    p->playing     = (len > 0U);
    if (p->playing) { load_note(p); }
}

void motor_music_stop(motor_music_t *p)
{
    p->playing = false;
    cia402_set_target_rpm(p->node_id, 0.0f);
}

bool motor_music_is_playing(const motor_music_t *p)
{
    return p->playing;
}

void motor_music_tick(motor_music_t *p)
{
    if (!p->playing) { return; }

    const mus_note_t *note = &p->notes[p->note_idx];

    /* Gap bolgesi: notanin sonundaki son gap_ms milisaniye sessizdir.
     * Bu sure icerisinde hiz 0'a cekilir; surucu manyetik titresimi keser
     * ve bir sonraki nota baslangicinda tiz bir gecis olusturur. Boylece
     * arka arkaya gelen ayni frekanstaki notalar da ayirt edilebilir hale gelir.
     * Nota suresi gap_ms'den kisa ya da esitse gap atlanir (sessizlik notasi gibi davranir). */
    const bool in_gap = (p->gap_ms > 0U) && (p->note_ms_left <= (uint32_t)p->gap_ms);

    if (in_gap || note->freq_hz == MUS_REST || note->freq_hz == 0U) {
        cia402_set_target_rpm(p->node_id, 0.0f);
    } else {
        /* Bresenham: yarim periyot asimini kontrol et (us cinsinden). */
        const uint32_t half_us = 500000UL / (uint32_t)note->freq_hz;
        p->osc_acc_us += 1000U; /* 1 ms tick */
        if (p->osc_acc_us >= half_us) {
            p->osc_acc_us -= half_us;
            p->osc_phase   = !p->osc_phase;
        }
        const float rpm = p->osc_phase ? p->ampl_rpm : -p->ampl_rpm;
        cia402_set_target_rpm(p->node_id, rpm);
    }

    /* Sure sayaci ---------------------------------------------------------- */
    if (p->note_ms_left > 0U) { p->note_ms_left--; }

    if (p->note_ms_left == 0U) {
        p->note_idx++;
        if (p->note_idx >= p->len) {
            if (p->loop) {
                p->note_idx = 0U;
            } else {
                p->playing = false;
                cia402_set_target_rpm(p->node_id, 0.0f);
                return;
            }
        }
        load_note(p);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Ceddin Deden — Geleneksel Osmanli Mehter Marsi
 *
 * Ton    : Re minoru (D Doryan / naturel minoru)
 * Tempo  : 120 BPM  → ceyreklik nota = MUS_Q = 500 ms
 * Olcu   : 4/4
 * Kaynak : Kamu mali geleneksel Osmanli askeri muzigi; telif hakki yoktur.
 *
 * Yapi:
 *   [A] Ana tema          (olcular  1–8,  tekrarlanir)
 *   [B] Orta tema / kopru (olcular  9–16, tekrarlanir)
 *   [A] Ana tema tekrari  (olcular  1–8)
 *   [C] Kapanış fanfari   (olcular 17–20)
 *
 * Not: 1 ms SYNC periyodu nedeniyle surucu ciktisi frekanslari en yakin
 * elde edilebilir degere yuvarlaniir (bkz. motor_music.h dokumantasyonu).
 * ═══════════════════════════════════════════════════════════════════════════ */

#define N(f,d) { (f), (d) }  /* okunabilirlik kisaltmasi */

const mus_note_t g_ceddin_deden[] = {

    /* ── [A] Ana tema — olcu 1-2 ────────────────────────────────────────── */
    /* Karakteristik acilis fanfari: noktalı ritim, cift vuruş */
    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),  /* D4. D4 */
    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),  /* D4. D4 */
    N(MUS_A3,  MUS_H ),                        /* A3(yarimlık) */

    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),  /* D4. D4 */
    N(MUS_F4,  MUS_DX), N(MUS_E4,  MUS_SX),  /* F4. E4 */
    N(MUS_D4,  MUS_H ),                        /* D4(yarimlık) */

    /* ── [A] Ana tema — olcu 3-4 ────────────────────────────────────────── */
    N(MUS_E4,  MUS_Q ), N(MUS_F4,  MUS_Q ),  /* E4 F4 */
    N(MUS_G4,  MUS_DX), N(MUS_F4,  MUS_SX), N(MUS_E4, MUS_Q),

    N(MUS_D4,  MUS_Q ), N(MUS_A3,  MUS_Q ),  /* D4 A3 */
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),  /* Bb3 A3 */

    /* ── [A] Ana tema — olcu 5-6 ────────────────────────────────────────── */
    N(MUS_G3,  MUS_DX), N(MUS_G3,  MUS_SX),  /* G3. G3 */
    N(MUS_A3,  MUS_DX), N(MUS_Bb3, MUS_SX),  /* A3. Bb3 */
    N(MUS_A3,  MUS_H ),                        /* A3 */

    N(MUS_G3,  MUS_Q ), N(MUS_F3,  MUS_Q ),  /* G3 F3 */
    N(MUS_E3,  MUS_Q ), N(MUS_D3,  MUS_Q ),  /* E3 D3 (inen basamak) */

    /* ── [A] Ana tema — olcu 7-8 (kapanış) ─────────────────────────────── */
    N(MUS_A3,  MUS_DX), N(MUS_A3,  MUS_SX),  /* A3. A3 */
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),  /* Bb3 A3 */

    N(MUS_D4,  MUS_H ), N(MUS_REST, MUS_Q ), N(MUS_REST, MUS_Q),

    /* ── [A] Ana tema tekrari — olcu 1-8 ────────────────────────────────── */
    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),
    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),
    N(MUS_A3,  MUS_H ),

    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),
    N(MUS_F4,  MUS_DX), N(MUS_E4,  MUS_SX),
    N(MUS_D4,  MUS_H ),

    N(MUS_E4,  MUS_Q ), N(MUS_F4,  MUS_Q ),
    N(MUS_G4,  MUS_DX), N(MUS_F4,  MUS_SX), N(MUS_E4, MUS_Q),

    N(MUS_D4,  MUS_Q ), N(MUS_A3,  MUS_Q ),
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),

    N(MUS_G3,  MUS_DX), N(MUS_G3,  MUS_SX),
    N(MUS_A3,  MUS_DX), N(MUS_Bb3, MUS_SX),
    N(MUS_A3,  MUS_H ),

    N(MUS_G3,  MUS_Q ), N(MUS_F3,  MUS_Q ),
    N(MUS_E3,  MUS_Q ), N(MUS_D3,  MUS_Q ),

    N(MUS_A3,  MUS_DX), N(MUS_A3,  MUS_SX),
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),

    N(MUS_D4,  MUS_H ), N(MUS_REST, MUS_Q ), N(MUS_REST, MUS_Q),

    /* ── [B] Orta tema — olcu 9-10 ──────────────────────────────────────── */
    N(MUS_F4,  MUS_DX), N(MUS_F4,  MUS_SX),  /* F4. F4 */
    N(MUS_G4,  MUS_Q ), N(MUS_F4,  MUS_Q ),  /* G4 F4 */

    N(MUS_E4,  MUS_Q ), N(MUS_D4,  MUS_Q ),  /* E4 D4 */
    N(MUS_E4,  MUS_H ),                        /* E4 */

    /* ── [B] Orta tema — olcu 11-12 ─────────────────────────────────────── */
    N(MUS_F4,  MUS_Q ), N(MUS_E4,  MUS_Q ),  /* F4 E4 */
    N(MUS_D4,  MUS_Q ), N(MUS_C4,  MUS_Q ),  /* D4 C4 */

    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),  /* Bb3 A3 */
    N(MUS_G3,  MUS_H ),                        /* G3 */

    /* ── [B] Orta tema — olcu 13-14 ─────────────────────────────────────── */
    N(MUS_A3,  MUS_DX), N(MUS_A3,  MUS_SX),  /* A3. A3 */
    N(MUS_Bb3, MUS_Q ), N(MUS_C4,  MUS_Q ),  /* Bb3 C4 */

    N(MUS_D4,  MUS_Q ), N(MUS_C4,  MUS_Q ),  /* D4 C4 */
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),  /* Bb3 A3 */

    /* ── [B] Orta tema — olcu 15-16 (kapanış) ───────────────────────────── */
    N(MUS_G3,  MUS_Q ), N(MUS_A3,  MUS_Q ),  /* G3 A3 */
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),  /* Bb3 A3 */

    N(MUS_D4,  MUS_H ), N(MUS_REST, MUS_Q ), N(MUS_REST, MUS_Q),

    /* ── [B] Orta tema tekrari — olcu 9-16 ──────────────────────────────── */
    N(MUS_F4,  MUS_DX), N(MUS_F4,  MUS_SX),
    N(MUS_G4,  MUS_Q ), N(MUS_F4,  MUS_Q ),

    N(MUS_E4,  MUS_Q ), N(MUS_D4,  MUS_Q ),
    N(MUS_E4,  MUS_H ),

    N(MUS_F4,  MUS_Q ), N(MUS_E4,  MUS_Q ),
    N(MUS_D4,  MUS_Q ), N(MUS_C4,  MUS_Q ),

    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),
    N(MUS_G3,  MUS_H ),

    N(MUS_A3,  MUS_DX), N(MUS_A3,  MUS_SX),
    N(MUS_Bb3, MUS_Q ), N(MUS_C4,  MUS_Q ),

    N(MUS_D4,  MUS_Q ), N(MUS_C4,  MUS_Q ),
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),

    N(MUS_G3,  MUS_Q ), N(MUS_A3,  MUS_Q ),
    N(MUS_Bb3, MUS_Q ), N(MUS_A3,  MUS_Q ),

    N(MUS_D4,  MUS_H ), N(MUS_REST, MUS_Q ), N(MUS_REST, MUS_Q),

    /* ── [C] Kapanis fanfari — olcu 17-20 ───────────────────────────────── */
    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),  /* D4. D4 */
    N(MUS_F4,  MUS_DX), N(MUS_E4,  MUS_SX),  /* F4. E4 */
    N(MUS_D4,  MUS_Q ), N(MUS_A3,  MUS_Q ),  /* D4 A3 */

    N(MUS_D4,  MUS_DX), N(MUS_D4,  MUS_SX),  /* D4. D4 */
    N(MUS_E4,  MUS_Q ), N(MUS_F4,  MUS_Q ),  /* E4 F4 */
    N(MUS_G4,  MUS_H ),                        /* G4 */

    N(MUS_F4,  MUS_Q ), N(MUS_E4,  MUS_Q ),  /* F4 E4 */
    N(MUS_D4,  MUS_Q ), N(MUS_A3,  MUS_Q ),  /* D4 A3 */

    N(MUS_D4,  MUS_W ),                        /* D4 tam nota — bitis */
    N(MUS_REST, MUS_H),                         /* son sessizlik        */
};

#undef N

const uint16_t g_ceddin_deden_len =
    (uint16_t)(sizeof(g_ceddin_deden) / sizeof(g_ceddin_deden[0]));
