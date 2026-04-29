#ifndef __CIA402_APP_H__
#define __CIA402_APP_H__

#include "canopen.h"
#include "app_config.h"

/* ── Surucu tipi ─────────────────────────────────────────────────────────── */
/* app_canopen_init() fonksiyonunun bir dugume hangi baslatma tarifini uygulayacagini secer:
 *   DRIVER_ZEROERR — CiA 402 Profil Hiz modu (ZeroErr eRob aktuator).
 *                    Tam PDO haritalama + profil rampasi SDO + kalp atisi.
 *   DRIVER_DELTA   — CiA 402 Profil Hiz modu (Delta ASDA servo).
 *                    Ayni PDO duzeni; counts_per_rpm_s farkli hesaplanir.
 *   DRIVER_WAT     — Ayni veri yolunu paylasan CiA-402 disi surucu. Master yalnizca
 *                    asgari kalp atisi yapilandirmasini yazar; PDO haritalama ve
 *                    CiA 402 durum makinesi atlanir. */
typedef enum {
    DRIVER_ZEROERR,
    DRIVER_DELTA,
	DRIVER_WAT
} driver_type_t;

/* ── Dugum basina yapilandirma ───────────────────────────────────────────── */
typedef struct {
    uint8_t       node_id;             /* CANopen dugum kimligi (1..127) */
    uint32_t      default_accel;       /* [count/s²] baslangicta 0x6083'e bir kez yazilir;
                                          calisma zamaninda cia402_set_accel() ile guncellenir. */
    uint32_t      default_decel;       /* [count/s²] baslangicta 0x6084'e bir kez yazilir;
                                          calisma zamaninda cia402_set_accel() ile guncellenir. */
    uint32_t      encoder_counts_per_rev; /* Motor devri basina enkoder count sayisi (CPR).
                                             Ham CPR degerini girin, orn.:
                                               524288   — ZeroErr 19-bit mutlak enkoder
                                               16777216 — Delta ASDA-B3 24-bit optik enkoder
                                               131072   — Delta ASDA-B3 17-bit manyetik enkoder
                                               10000    — 2500-PPR artimli ×4
                                             DRIVER_DELTA icin hiz komutlari count/s degil
                                             0.1-RPM birimi kullanir; bu alan yalnizca
                                             cia402_get_pos_vel() icindeki konum
                                             raporlamasini etkiler. */
    uint16_t      reductor_ratio;      /* Reduktor orani: cikis mili basina motor devri sayisi.
                                          Dogrudan tahrik icin 1 kullanin.
                                          counts_per_rpm_s onbellegine katlanir; boylece
                                          genel API'ye saglanan hedef/RPM degerleri CIKIS
                                          milinde yorumlanirken enkoder motor tarafinda sayar. */
    driver_type_t driver_type;         /* baslatma tarifini secer (yukariya bakin) */
} cia402_cfg_t;

/* Ornek sayisinin derleme zamani ust siniri. example_embedded.c icindeki
 * (m_node[], m_sdo_tbl[], g_*[], vb.) depolama bu sinira gore boyutlandirilir;
 * baslangicta cia402_node_count <= CIA402_MAX_NODES kontrol edilir.
 * Daha fazla surucu gerekiyorsa bunu ve canopen.h icindeki
 * CO_MAX_RPDO / CO_MAX_TPDO degerlerini birlikte artirin. */
#ifndef CIA402_MAX_NODES
#warning "CIA402_MAX_NODES is not defined"
#endif

/* Yapilandirma tablosu — uygulamanizda bir kez tanimlayin, orn.:
 *
 *   const cia402_cfg_t cia402_nodes[] = {
 *       { 0x05U, 50000U, 200000U,  524288U, 1U, DRIVER_ZEROERR },  /* ZeroErr 19-bit */
 *       { 0x06U, 50000U, 200000U, 16777216U, 1U, DRIVER_DELTA   },  /* Delta ASDA-B3 24-bit */
 *   };
 *   const uint8_t cia402_node_count =
 *       (uint8_t)(sizeof(cia402_nodes) / sizeof(cia402_nodes[0]));
 *
 * example_embedded.c bunlara yalnizca extern ile basvurur; dolayisiyla uygulama
 * tabloyu stack kaynagini degistirmeden herhangi bir ceviri biriminden saglayabilir. */
extern const cia402_cfg_t cia402_nodes[];
extern const uint8_t      cia402_node_count;

/* ── Genel API ───────────────────────────────────────────────────────────── */
co_error_t app_canopen_init(void);
void       app_canopen_loop(void);
void       co_transmit_process(void);

void cia402_set_target_rpm(uint16_t node_id, float target_rpm);
void cia402_set_accel(uint16_t node_id, uint32_t accel_counts_s2, uint32_t decel_counts_s2);
void cia402_get_pos_vel(uint16_t node_id, float *pos_deg, float *vel_rpm, float *torque_pct);

#endif /* __CIA402_APP_H__ */
