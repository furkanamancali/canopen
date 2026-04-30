#ifndef __CIA402_APP_H__
#define __CIA402_APP_H__

#include "canopen.h"
#include "canopen_port.h"
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
    uint8_t          node_id;          /* CANopen dugum kimligi (1..127) */
    co_port_handle_t can_handle;       /* Bu dugumun bagli oldugu CAN cevre birimi
                                          (orn. &hfdcan1, &hfdcan2, &hcan1).
                                          Ayni handle'a sahip dugumler ayni CAN
                                          veri yolunu paylaşır; farkli handle'lar
                                          bagimsiz veri yollarini temsil eder ve
                                          her biri kendi master CANopen ornegini alir. */
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
#  error "CIA402_MAX_NODES must be defined in app_config.h or via a -D compiler flag. " \
         "Set it to the maximum total slave count across all CAN buses. " \
         "Also ensure CO_MAX_RPDO >= CIA402_MAX_NODES * 2 and CO_MAX_TPDO >= CIA402_MAX_NODES * 2."
#endif

/* Yapilandirma tablosu — uygulamanizda bir kez tanimlayin, orn.:
 *
 *   extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;   // or CAN_HandleTypeDef for F4
 *
 *   const cia402_cfg_t cia402_nodes[] = {
 *       { 0x05U, &hfdcan1,  50000U, 200000U,  524288U, 1U, DRIVER_ZEROERR },  // ZeroErr, bus 1
 *       { 0x06U, &hfdcan2,  50000U, 200000U, 16777216U, 1U, DRIVER_DELTA   },  // Delta,   bus 2
 *   };
 *   const uint8_t cia402_node_count =
 *       (uint8_t)(sizeof(cia402_nodes) / sizeof(cia402_nodes[0]));
 *
 * example_embedded.c bunlara yalnizca extern ile basvurur; dolayisiyla uygulama
 * tabloyu stack kaynagini degistirmeden herhangi bir ceviri biriminden saglayabilir. */
extern const cia402_cfg_t cia402_nodes[];
extern const uint8_t      cia402_node_count;

/* ── Master CiA 402 durum makinesi ──────────────────────────────────────────
 * co_diag_info_t.detail.drive uygulama katmanina gorunur olmasi icin burada
 * tanimlanir. */
typedef enum {
    MASTER_CIA402_IDLE = 0,
    MASTER_CIA402_FAULT_RESET,
    MASTER_CIA402_WAIT_FAULT_CLEAR,
    MASTER_CIA402_SHUTDOWN,
    MASTER_CIA402_SWITCH_ON,
    MASTER_CIA402_ENABLE,
    MASTER_CIA402_RUNNING,
    MASTER_CIA402_FAULT,
} master_cia402_state_t;

/* ── Yapilandirilmis tanisal geri cagirma ────────────────────────────────── */
typedef enum {
    CO_DIAG_EMCY_RECEIVED      = 0, /* kole EMCY cercevesi gonderdi; detail.emcy dolu */
    CO_DIAG_HB_TIMEOUT         = 1, /* kole kalp atisi zaman asimina ugradi */
    CO_DIAG_HB_RECOVERED       = 2, /* kole kalp atisi geri geldi */
    CO_DIAG_SDO_TIMEOUT        = 3, /* SDO yazmasi zaman asimina ugradi; detail.sdo_timeout dolu */
    CO_DIAG_SDO_ABORT          = 4, /* kole SDO'yu reddetti; detail.sdo_abort dolu */
    CO_DIAG_DRIVE_STATE_CHANGE = 5, /* CiA 402 master FSM gecisi; detail.drive dolu */
    CO_DIAG_NMT_STATE_CHANGE   = 6, /* kole NMT durumu degisti; detail.nmt dolu */
} co_diag_event_t;

typedef struct {
    co_diag_event_t event;
    uint8_t         node_id;  /* hangi kole (0 = gecerli degil) */
    union {
        struct {
            uint16_t emcy_code;
            uint8_t  error_reg;
            uint8_t  msef;        /* manufacturer-specific error sub-code (byte 3) */
        } emcy;
        struct {
            uint16_t index;
            uint8_t  subindex;
            uint32_t abort_code;
        } sdo_abort;
        struct {
            uint16_t index;
            uint8_t  subindex;
            bool     is_init;     /* true = baslangic SDO dizisi, false = calisma zamani */
        } sdo_timeout;
        struct {
            master_cia402_state_t from;
            master_cia402_state_t to;
        } drive;
        struct {
            co_nmt_state_t state;
        } nmt;
    } detail;
} co_diag_info_t;

/* Tanisal geri cagirma tipi.
 * Birden fazla baglam tarafindan tetiklenebilir (ISR, ana dongu, zamanlayici);
 * kisa, bloke edici olmayan uygulamalar yazilmalidir (orn. halka tamponu gunlugu). */
typedef void (*co_diag_cb_t)(const co_diag_info_t *info, void *user);

/* Tanisal geri cagirmayi kaydet. app_canopen_init()'den once cagrilabilir;
 * is parcacigi-guvenlidir (tek bir isaretci atamasiyla gerceklenir). */
void app_canopen_set_diag_cb(co_diag_cb_t cb, void *user);

/* ── Genel API ───────────────────────────────────────────────────────────── */
co_error_t app_canopen_init(void);
void       app_canopen_loop(void);
void       co_transmit_process(void);

void cia402_set_target_rpm(uint16_t node_id, float target_rpm);
void cia402_set_accel(uint16_t node_id, uint32_t accel_counts_s2, uint32_t decel_counts_s2);
void cia402_get_pos_vel(uint16_t node_id, float *pos_deg, float *vel_rpm, float *torque_pct);

#endif /* __CIA402_APP_H__ */
