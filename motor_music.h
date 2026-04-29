#ifndef MOTOR_MUSIC_H
#define MOTOR_MUSIC_H

/*
 * motor_music.h — Motor titresim melodisi oynatici
 *
 * Mekanizma
 * ---------
 * CiA 402 Profil Hiz modundaki bir servo surucu, hizi +ampl ve -ampl RPM
 * arasinda nota frekansinda titrestirildiginde motor sargisi hoparlor gibi
 * davranir ve duyulabilir ton uretir.  Osilayon, biriktiricili Bresenham
 * tamsayi aritmetigi kullanirak 1 ms'lik SYNC periyodundan hassas nota
 * frekanslari uretir.
 *
 * Entegrasyon
 * -----------
 * 1. app_canopen_init() icinde motor_music_start() cagrilir.
 * 2. co_transmit_process() icinde senkronizasyon olayindan ONCE
 *    motor_music_tick() cagrilir; boylece guncel RPM hedefi her SYNC'te
 *    suruculere iletilir.
 *
 * Hizlanma rampalari
 * ------------------
 * Profile velocity mode'daki 0x6083/0x6084 ivme/yavaslamasi titresimi
 * sondurur.  Melodiyi caldirilacak surucu icin cia402_set_accel() araciligiyla
 * mumkun olan en yuksek deger ayarlanmali (orn. 10 000 000 count/s^2), melodiden
 * sonra eski deger geri yuklenmesi onerilir.
 *
 * Frekans cozunurlugu (1 ms tick)
 * --------------------------------
 * Elde edilebilir frekans: f = 1 000 000 / (2 * acc_hedefi_us)
 * Ornekler:
 *   D3  (147 Hz) — 3 ms yarim periyot → gercek 167 Hz
 *   G3  (196 Hz) — 3 ms yarim periyot → gercek 167 Hz
 *   A3  (220 Hz) — 2 ms yarim periyot → gercek 250 Hz
 *   D4  (294 Hz) — 2 ms yarim periyot → gercek 250 Hz
 *   A4  (440 Hz) — 1 ms yarim periyot → gercek 500 Hz
 * Bresenham birikim yontemi, hatalarin ortalamasini alarak dogru frekansda
 * titresim saglar; ancak 1 ms'lik darbe titresimi (jitter) kacinilamaz.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Nota frekans sabitleri (Hz) ─────────────────────────────────────────── */
#define MUS_REST   0U    /* sessizlik                  */
#define MUS_C3   131U
#define MUS_D3   147U
#define MUS_Eb3  156U
#define MUS_E3   165U
#define MUS_F3   175U
#define MUS_G3   196U
#define MUS_Ab3  208U
#define MUS_A3   220U
#define MUS_Bb3  233U
#define MUS_B3   247U
#define MUS_C4   262U
#define MUS_Db4  277U
#define MUS_D4   294U
#define MUS_Eb4  311U
#define MUS_E4   330U
#define MUS_F4   349U
#define MUS_Gb4  370U
#define MUS_G4   392U
#define MUS_Ab4  415U
#define MUS_A4   440U
#define MUS_Bb4  466U

/* ── Sure sabitleri — 120 BPM = 500 ms/ceyrek ───────────────────────────── */
#define MUS_SX    125U   /* on altilık nota          */
#define MUS_E     250U   /* sekizlik nota             */
#define MUS_DX    375U   /* noktali on altilık nota  */
#define MUS_Q     500U   /* ceyreklik nota            */
#define MUS_DQ    750U   /* noktali ceyreklik nota   */
#define MUS_H    1000U   /* yarimlık nota             */
#define MUS_DH   1500U   /* noktali yarimlık nota    */
#define MUS_W    2000U   /* tam nota                  */

/* ── Veri tipleri ────────────────────────────────────────────────────────── */
typedef struct {
    uint16_t freq_hz;   /* nota frekansi (Hz); 0 = sessizlik */
    uint16_t dur_ms;    /* nota suresi (ms)                   */
} mus_note_t;

typedef struct {
    /* Yapilandirma — motor_music_start() ile doldurulur */
    const mus_note_t *notes;
    uint16_t          len;
    uint16_t          node_id;    /* RPM hedefinin ayarlanacagi CANopen dugum kimligi */
    float             ampl_rpm;   /* +/- titresim genliği (RPM); 20–60 onerilir      */
    bool              loop;       /* true ise melodi bittikten sonra basa doner       */

    /* Calisma zamani durumu — motor_music_start() ile sifirlaniir */
    uint16_t note_idx;
    uint32_t note_ms_left;
    uint32_t osc_acc_us;    /* Bresenham birikim sayaci (mikrosaniye)  */
    bool     osc_phase;     /* mevcut osilayon yonu (+ veya -)          */
    bool     playing;
} motor_music_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * Oynatmayi baslatir.  Daha once durumda olan bir oynaticiyi yeniden
 * baslatir (loop = false iken bitmis bile olsa).
 *
 * @param p         Kullanici tarafindan ayrilmis ve sifirlanmis motor_music_t
 * @param notes     Nota dizisi (ROM veya RAM'de)
 * @param len       Dizi uzunlugu (not sayisi)
 * @param node_id   Melodinin gonderilecegi CANopen dugum kimligi
 * @param ampl_rpm  Titresim genliği; surucu akiminı sinirlandirmak icin kucuk tutun
 * @param loop      true → melodi bitince basa don
 */
void motor_music_start(motor_music_t *p,
                       const mus_note_t *notes, uint16_t len,
                       uint16_t node_id, float ampl_rpm, bool loop);

/**
 * Her 1 ms'de bir cagrilmasi gereken ana tick fonksiyonu.
 * co_transmit_process() icinde sync olayindan ONCE cagrilmalidir.
 * cia402_set_target_rpm() araciligiyla surucu RPM hedefini gunceller.
 */
void motor_music_tick(motor_music_t *p);

/**
 * Oynatmayi aninda durdurur, surucu hizini 0'a ayarlar.
 */
void motor_music_stop(motor_music_t *p);

/** Oynatici hala calisiyorsa true doner. */
bool motor_music_is_playing(const motor_music_t *p);

/* ── Hazir melodi: Ceddin Deden (Mehter Marsi) ───────────────────────────── */
/* Geleneksel Osmanli mehter marsi — kamu malı, hic bir telif hakki yok.
 * Re minoru, 120 BPM, 4/4 olcu. */
extern const mus_note_t  g_ceddin_deden[];
extern const uint16_t    g_ceddin_deden_len;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MUSIC_H */
