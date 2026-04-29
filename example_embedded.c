/*
 * CANopen Master — STM32H7 FDCAN (cok ornekli)
 *
 * Bir veya daha fazla CiA 402 Profil Hiz aktuatorunu (ve DRIVER_WAT isaretli
 * CiA-402 disi suruculeri taslak olarak) kontrol eder.
 *
 * Topoloji
 * ────────
 *   STM32H7 (master, dugum 0x7F) ←──CAN──→ kole[0] (dugum 0x05)
 *                                            kole[1] (dugum 0x06)  ← istege bagli
 *                                            ...
 *
 * Ornek tablosu, example_embedded.h'de bildirilen extern'ler araciligiyla
 * uygulama tarafindan saglanir (cia402_nodes[] / cia402_node_count).
 * Bu kaynak dosya artik yapilandirma dizisine sahip degildir — herhangi bir
 * ceviri biriminde tanimlayin, stack otomatik olarak alir.
 *
 * Her CiA 402 ornegi master'da iki TPDO ve iki RPDO kullanir:
 *   ornek i → tpdo_num = i*2    (controlword, mod, hedef_hiz → kole RPDO1)
 *           → rpdo_num = i*2    (statusword, mod_goster, hiz_gercek ← kole TPDO1)
 *           → rpdo_num = i*2+1  (konum_gercek, tork_gercek ← kole TPDO2)
 * Ivme/yavaslama SDO ile yazilir (0x6083/0x6084): baslangicta bir kez,
 * calisma zamaninda cia402_set_accel() ile sdo_rt_run() uzerinden kuyruklanir.
 *
 * OD yedek depolamasi orneklerin cakismamasi icin saticiya ozgu indeksler kullanir:
 *   Ornek i TPDO verisi: OD indeks 0x4000 + i  (alt 1–5)
 *   Ornek i RPDO verisi: OD indeks 0x4100 + i  (alt 1–5)
 */

#include "canopen.h"
#include "canopen_stm32h7_fdcan.h"
#include "cia402_app.h"
#include "utils/utils.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* ── Master yapilandirmasi ───────────────────────────────────────────────── */
#define MASTER_NODE_ID              0x7FU
#define MASTER_HEARTBEAT_MS         50U
#define SLAVE_HB_TIMEOUT_MS          150U
#define SYNC_PERIOD_US              1000UL
#define CIA402_MAX_VEL_RPM            3000
#define CIA402_STATE_TIMEOUT_MS       2000U
#define CIA402_FAULT_RESET_MS         10U
#define CIA402_RPDO_WATCHDOG_MS       100U
#define NMT_RESET_DELAY_MS     500U
#define NMT_PREOP_DELAY_MS     100U
/* Surucu yapilandirildiktan sonra PRE_OPERATIONAL durumunun gercek bir kole
 * dususu sayilmasi icin en az bu kadar sure devam etmesi gerekir; aksi hâlde
 * SDO yapilandirmasi yeniden calistirilmaz. Deger kalp atisi periyodunun cok
 * uzerinde tutulur; boylece iki OP cercevesi arasindaki tek bir bozuk/kayip
 * PRE_OP kalp atisi tek basina master_started'i sifirlayip 28 adimli SDO
 * dizisini yeniden baslatamaz. */
#define NMT_PREOP_DEBOUNCE_MS  150U
#define SDO_TIMEOUT_MS         500U
#define SDO_RT_QUEUE_SIZE      4U   /* ornek basina maksimum kuyruklanmis calisma zamani SDO yazmasi */

#define CW_SHUTDOWN             0x0006U
#define CW_SWITCH_ON            0x0007U
#define CW_ENABLE_OPERATION     0x000FU
#define CW_QUICK_STOP           0x000BU
#define CW_FAULT_RESET          0x0080U
#define CW_FAULT_RESET_CLEAR    0x0000U
#define CIA402_MODE_PROFILE_VEL   ((int8_t)3)

/* ═══════════════════════════════════════════════════════════════════════════
 * ORNEK YAPILANDIRMASI
 *
 * cia402_nodes[] / cia402_node_count extern'leri (example_embedded.h'de
 * bildirilen) uygulamadan gelir — onerilen kalip icin o basliga bakin.
 * Bu dosyadaki tum donguler cia402_node_count kullanir; ornek basina tum
 * depolama CIA402_MAX_NODES'a gore boyutlandirilir. app_canopen_init()
 * baska bir sey yapmadan once cia402_node_count'u aralik kontrolunden gecirir;
 * canopen.h icindeki CO_MAX_RPDO / CO_MAX_TPDO her biri
 * >= cia402_node_count * 2 olmalidir.
 * ═══════════════════════════════════════════════════════════════════════════ */

_Static_assert(CIA402_MAX_NODES * 2U <= CO_MAX_RPDO,
    "CIA402_MAX_NODES too high — raise CO_MAX_RPDO and CO_MAX_TPDO in canopen.h");

/* ── CANopen stack ornegi ────────────────────────────────────────────────── */
static co_node_t      canopen_node;
static co_stm32_ctx_t can_ctx;

/* ── Ornek basina uzak SDO yapilandirma tablolari ────────────────────────── */
typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    uint8_t  size;
} sdo_entry_t;

/* Ornek basina baslangic SDO tablosu. SDO_STEPS_MAX depolama boyutudur;
 * gercek giris sayisi driver_type'a gore degisir ve sdo_cfg_run()'in
 * sona ulastigini anlayabilmesi icin m_sdo_steps[i]'ye kaydedilir. */
#define SDO_STEPS_MAX 28U
static sdo_entry_t m_sdo_tbl[CIA402_MAX_NODES][SDO_STEPS_MAX];
static uint8_t          m_sdo_steps[CIA402_MAX_NODES];

static void build_sdo_table(uint8_t i)
{
    const uint8_t  nid = cia402_nodes[i].node_id;
    sdo_entry_t *t = m_sdo_tbl[i];
    uint32_t n = 0U;

    if ((cia402_nodes[i].driver_type == DRIVER_ZEROERR) ||
        (cia402_nodes[i].driver_type == DRIVER_DELTA  )   ) {
        const uint32_t r1d = 0x80000000UL | (0x200UL + nid);
        const uint32_t r1e =                 0x200UL + nid;
        const uint32_t t1d = 0x80000000UL | (0x180UL + nid);
        const uint32_t t1e =                 0x180UL + nid;
        const uint32_t t2d = 0x80000000UL | (0x280UL + nid);
        const uint32_t t2e =                 0x280UL + nid;

        /* kole RPDO1 */
        t[n++] = (sdo_entry_t){ 0x1400U, 0x01U, r1d,          4U };
        t[n++] = (sdo_entry_t){ 0x1400U, 0x02U, 0xFFU,         1U };
        t[n++] = (sdo_entry_t){ 0x1600U, 0x00U, 0U,            1U };
        t[n++] = (sdo_entry_t){ 0x1600U, 0x01U, 0x60400010UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1600U, 0x02U, 0x60600008UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1600U, 0x03U, 0x60FF0020UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1600U, 0x00U, 3U,            1U };
        t[n++] = (sdo_entry_t){ 0x1400U, 0x01U, r1e,           4U };
        t[n++] = (sdo_entry_t){ 0x1400U, 0x05U, 500U,          2U };
        /* kole TPDO1 */
        t[n++] = (sdo_entry_t){ 0x1800U, 0x01U, t1d,          4U };
        t[n++] = (sdo_entry_t){ 0x1800U, 0x02U, 0x01U,         1U };
        t[n++] = (sdo_entry_t){ 0x1A00U, 0x00U, 0U,            1U };
        t[n++] = (sdo_entry_t){ 0x1A00U, 0x01U, 0x60410010UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1A00U, 0x02U, 0x60610008UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1A00U, 0x03U, 0x606C0020UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1A00U, 0x00U, 3U,            1U };
        t[n++] = (sdo_entry_t){ 0x1800U, 0x01U, t1e,           4U };
        /* kole TPDO2 */
        t[n++] = (sdo_entry_t){ 0x1801U, 0x01U, t2d,          4U };
        t[n++] = (sdo_entry_t){ 0x1801U, 0x02U, 0x01U,         1U };
        t[n++] = (sdo_entry_t){ 0x1A01U, 0x00U, 0U,            1U };
        t[n++] = (sdo_entry_t){ 0x1A01U, 0x01U, 0x60640020UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1A01U, 0x02U, 0x60770010UL, 4U };
        t[n++] = (sdo_entry_t){ 0x1A01U, 0x00U, 2U,            1U };
        t[n++] = (sdo_entry_t){ 0x1801U, 0x01U, t2e,           4U };
        /* Profil rampasi — bir kez yazilir; ivme/yavaslama artik PDO'ya eslenmez */
        t[n++] = (sdo_entry_t){ 0x6083U, 0x00U, cia402_nodes[i].default_accel, 4U };
        t[n++] = (sdo_entry_t){ 0x6084U, 0x00U, cia402_nodes[i].default_decel, 4U };
        /* Kalp atisi */
        t[n++] = (sdo_entry_t){ 0x1017U, 0x00U, 50U,           2U };
        t[n++] = (sdo_entry_t){ 0x1016U, 0x01U,
                                      ((uint32_t)MASTER_NODE_ID << 16) | 150U, 4U };
    } else {
        /* DRIVER_WAT — yer tutucu. Koleyi her CANopen surucusunun ihtiyac duydugu
         * asgari duzeyde cevrimici getirin: master'in izleyebilecegi bir kalp atisi
         * ureticisi ve master'a geri isaret eden bir HB tuketici girisi.
         * Burada PDO haritalama yapilandirilmamistir — WAT nesne sozlugu
         * bilindiginde uygulamanin bu dali (ve app_canopen_init icindeki PDO
         * kurulumunu) genisletmesi gerekecektir. */
        t[n++] = (sdo_entry_t){ 0x1017U, 0x00U, 50U,           2U };
        t[n++] = (sdo_entry_t){ 0x1016U, 0x01U,
                                      ((uint32_t)MASTER_NODE_ID << 16) | 150U, 4U };
    }

    m_sdo_steps[i] = (uint8_t)n;
}

/* ── Ornek basina calisma zamani durumu ──────────────────────────────────── */
typedef enum {
    SDO_CFG_IDLE = 0,
    SDO_CFG_RESET_DELAY,
    SDO_CFG_NMT_DELAY,
    SDO_CFG_WAIT_RESPONSE,
    SDO_CFG_DONE,
} sdo_cfg_state_t;

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

#ifdef DEBUG
/* Iki'nin kuvveti; CiA 402 durum makinesi gecislerinin ornek basina halka tamponu.
 * "SOD'da takili kaldi" / hata-sifirlama dongusu sonrasi incelemede
 * tam etkinlestirme dizisini ve birkac hatayi sarmadan yakalayacak buyuklukte.
 * Yalnizca -DDEBUG altinda derlenir; surum yapilarinda flash, RAM veya
 * calisma zamani maliyeti yoktur; surum derlemesinde cia402_log_transition()
 * tek bir drive_state atamasina indirgenir (asagiya bakin). */
#define CIA402_TRANSITION_LOG_SIZE 16U

typedef struct {
    uint32_t ts_ms;        /* gecis anindaki now_ms() degeri */
    uint16_t statusword;   /* gecisteki surucu 0x6041 ("SOD'da takili" sorusturmalarinda ayirt edici) */
    uint8_t  from;         /* gecis oncesi master_cia402_state_t */
    uint8_t  to;           /* gecis sonrasi master_cia402_state_t  */
} cia402_transition_t;
#endif

typedef struct {
	driver_type_t driver_type_st;

    /* OD yedek: TPDO1 — master → kole komutlari */
    uint16_t controlword;
    int8_t   mode_of_op;
    int32_t  target_vel;
    /* OD yedek: RPDO1 — kole → master geri bildirimi */
    uint16_t statusword;
    int8_t   mode_display;
    int32_t  velocity_act;
    /* OD yedek: RPDO2 — kole → master konum/tork */
    int32_t  position_act;
    int16_t  torque_act;

    sdo_cfg_state_t sdo_cfg_state;
    uint32_t             sdo_cfg_step;
    uint32_t             sdo_cfg_step_ms;
    bool                 sdo_cfg_resp_ok;

    /* Calisma zamani SDO yazma kuyrugu — cia402_set_accel() vb. tarafindan doldurulur,
     * sdo_rt_run() tarafindan yanit dongusu basina bir giris olacak sekilde bosaltilir.
     * Yalnizca master_started = true oldugunda etkin (baslangic yapilandirmasi tamamlandi). */
    sdo_entry_t sdo_rt_queue[SDO_RT_QUEUE_SIZE];
    uint8_t          sdo_rt_head;     /* sonraki yazma yuvasi */
    uint8_t          sdo_rt_tail;     /* sonraki okuma yuvasi */
    bool             sdo_rt_active;   /* gecerli giris icin yanit bekleniyor */
    bool             sdo_rt_resp_ok;
    uint32_t         sdo_rt_ms;       /* timestamp of last send */
    /* Gecerli giris icin son hizlandirilmis SDO yukleme yuku (data[4..7], little-endian).
     * Yalnizca ucustaki giris bir okuma oldugunda ve yanit bir yukleme onayi (SCS 0x40)
     * oldugunda anlamlidir; yazmalar bu alani degistirmez.
     * Yaygin 1–4 baytlik CiA 402 nesneleri icin boyutlandirilmistir
     * (controlword, statusword, mode_of_op, vb.). */
    uint32_t         sdo_rt_resp_value;
    /* Calisma zamani yolunda gozlemlenen son SDO iptali. Sifirdan farkliysa
     * en son calisma zamani SDO yazmasi surucu tarafindan reddedilmistir;
     * iptal cercevesinin data[4..7] alani ve basarisiz girisin indeks/alt-indeksi.
     * Tam ornek sifirlamada temizlenir; m_node[] icinde dolasmadan gorunur
     * olmasi icin g_sdo_rt_abort_code[]'a yansitilir. */
    uint32_t         sdo_rt_abort_code;
    uint16_t         sdo_rt_abort_index;
    uint8_t          sdo_rt_abort_subindex;

    master_cia402_state_t drive_state;
    uint32_t              state_entry_ms;
    uint32_t              last_rpdo1_ms;

    bool           master_started;
    bool           hb_lost;
    co_nmt_state_t nmt_state;

    /* PRE_OP kararli hâle getirme. Baslatilmis bir surucu PRE_OPERATIONAL bildirdiginde
     * on_slave_nmt_change tarafindan kurulur; OPERATIONAL'a donuldugunde veya
     * ana dongu NMT_PREOP_DEBOUNCE_MS sonunda dususu onayladiginda temizlenir. */
    bool           pre_op_pending;
    uint32_t       pre_op_entry_ms;

    /* app_canopen_init() zamaninda cia402_nodes[i]'den turetilen ornek basina
     * enkoder sabitleri. counts_per_rpm_s surucuye ozgudur:
     *   DRIVER_ZEROERR: cpr * reduktor_orani / 60  (cikis RPM'i basina count/s)
     *   DRIVER_DELTA:   10  * reduktor_orani        (0.1-RPM birimi)
     * SYNC-tick donusumunun cia402_apply_target_internal() icinde ve
     * cia402_get_pos_vel() icindeki kayan nokta matematiginin her cagrida
     * yeniden hesaplamamasi icin bir kez onbellege alinir. */
    int32_t        encoder_counts_per_rev;
    int16_t        ratio;
    int32_t        counts_per_rpm_s;

#ifdef DEBUG
    /* CiA 402 gecis gecmisi. Her drive_state degisiminde cia402_log_transition()
     * tarafindan yazilir; en son giris (transition_log_head - 1) & (SIZE - 1)
     * konumundadir. count, SIZE'da doygunlasir ve hata ayiklayicinin "log henuz
     * sarilmadi" ile "log dolu" arasinda ayrim yapmasina olanak tanir.
     * node_reset() tarafindan temizlenmez — sifirlamayi atlatmak zaten amacidir;
     * "neden IDLE'a donduk" sorusunun sonradan incelenmesi onceki durumu gerektirir. */
    cia402_transition_t transition_log[CIA402_TRANSITION_LOG_SIZE];
    uint8_t           transition_log_head;
    uint8_t           transition_log_count;
#endif
} node_state_t;

static node_state_t m_node[CIA402_MAX_NODES];

/* Ornek basina son EMCY — hatalari tanimlamak icin hata ayiklayicida inceleyin. */
volatile uint16_t g_emcy_code[CIA402_MAX_NODES];
volatile uint8_t  g_emcy_reg[CIA402_MAX_NODES];

/* Ornek basina son calisma zamani SDO iptali. Sifirdan farkli abort_code, bir calisma
 * zamani yazmasinin (orn. cia402_set_accel) reddedildigi anlamina gelir;
 * indeks/alt-indeks basarisiz girisi tanimlar. Uygulama veya ornek sifirlamasi
 * temizleyene kadar kalicidir. */
volatile uint32_t g_sdo_rt_abort_code[CIA402_MAX_NODES];
volatile uint16_t g_sdo_rt_abort_index[CIA402_MAX_NODES];
volatile uint8_t  g_sdo_rt_abort_subindex[CIA402_MAX_NODES];

/* Herhangi bir gorev baglamindan ayarlanan ornek basina hedef RPM. */
volatile float32_t g_target_rpm[CIA402_MAX_NODES];

/* ── CiA 402 statusword yardimcilari ────────────────────────────────────── */
#define SW_RTSO  0x0001U
#define SW_SO    0x0002U
#define SW_OE    0x0004U
#define SW_FAULT 0x0008U
#define SW_QS    0x0020U
#define SW_SOD   0x0040U
#define SW_MASK  (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD)

static inline bool cia402_sw_fault_reaction_active(uint16_t sw)
{
    return (sw & (SW_FAULT | SW_OE | SW_SO | SW_RTSO))
               == (SW_FAULT | SW_OE | SW_SO | SW_RTSO);
}
static inline bool cia402_sw_ready_to_switch_on(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_QS);
}
static inline bool cia402_sw_switched_on(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_SO | SW_QS);
}
static inline bool cia402_sw_operation_enabled(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_SO | SW_OE | SW_QS);
}
static inline bool cia402_sw_fault(uint16_t sw) { return (sw & SW_FAULT) != 0U; }

static inline uint32_t now_ms(void)
{
    return canopen_node.iface.millis(canopen_node.iface.user);
}

/* ── Temel kritik bolum ──────────────────────────────────────────────────── */
/* PRIMASK kaydet-geri-yukle korumasi — her tarafta uc Cortex-M komutu ve
 * bir mutex'in aksine ISR baglamindan cagrilabilir. sdo_rt_enqueue() ve
 * cia402_set_target_rpm() icindeki uretici tarafi guncellemelerini, bir RTOS
 * gorevini guncelleme ortasinda oncelikli kesen daha yuksek oncelikli bir ISR
 * dahil, diger her oncelige karsi atomik hâle getirmek icin kullanilir.
 * PRIMASK'i kaydetmek (cikista korce etkinlestirmek yerine) yardimcilari
 * ic ice gecme icin guvenli tutar: kesmeler zaten devre disiyken alinan bir
 * kritik bolum, cikista cagiranin bekledigi gibi devre disi birakilmis birakir. */
static inline uint32_t co_enter_critical(void)
{
    const uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}
static inline void co_exit_critical(uint32_t saved_primask)
{
    __set_PRIMASK(saved_primask);
}

/* ── Ornek arama ─────────────────────────────────────────────────────────── */
static int8_t node_find(uint8_t node_id)
{
    for (uint8_t i = 0U; i < cia402_node_count; ++i) {
        if (cia402_nodes[i].node_id == node_id) { return (int8_t)i; }
    }
    return -1;
}

/* ── Gecis kaydi ─────────────────────────────────────────────────────────── */
/* CiA 402 durum makinesi gecisini ornek basina halkaya kaydeder ve drive_state'i
 * taahhut eder. Oz-kenarlar (X→X) atilir — cia402 adim makinesi siklikla ayni
 * durumu tick'ler arasinda yeniden onaylar ve bu gurultu gercek gecisleri
 * bastirir. Oz-kenar atlamada da dahil olmak uzere drive_state her zaman
 * taahhut edilir; boylece cagiranlarin ayri bir atamaya ihtiyaci olmaz.
 *
 * Surum derlemeleri (-DDEBUG ayarli degil) bunu tek bir alan atamasina
 * indirger — halka depolamasi yok, now_ms() cagrisi yok, dal yok — dolayisiyla
 * cia402_step / on_hb_event / ana dongu icindeki cagri noktalari kendi
 * #ifdef sarmalayicilarina ihtiyac duymaz. */
#ifdef DEBUG
static void cia402_log_transition(uint8_t i, master_cia402_state_t to)
{
    node_state_t *e = &m_node[i];
    if (e->drive_state == to) { return; }

    cia402_transition_t *r = &e->transition_log[e->transition_log_head];
    r->ts_ms      = now_ms();
    r->statusword = e->statusword;
    r->from       = (uint8_t)e->drive_state;
    r->to         = (uint8_t)to;

    e->transition_log_head =
        (uint8_t)((e->transition_log_head + 1U) & (CIA402_TRANSITION_LOG_SIZE - 1U));
    if (e->transition_log_count < CIA402_TRANSITION_LOG_SIZE) {
        e->transition_log_count++;
    }
    e->drive_state = to;
}
#else
static inline void cia402_log_transition(uint8_t i, master_cia402_state_t to)
{
    m_node[i].drive_state = to;
}
#endif

/* ── Ornek sifirlama ─────────────────────────────────────────────────────── */
static void node_reset(uint8_t i, bool full)
{
    m_node[i].target_vel     = 0;
    m_node[i].controlword    = CW_SHUTDOWN;
    cia402_log_transition(i, MASTER_CIA402_IDLE);
    m_node[i].master_started = false;
    m_node[i].sdo_rt_head    = 0U;
    m_node[i].sdo_rt_tail    = 0U;
    m_node[i].sdo_rt_active  = false;
    m_node[i].pre_op_pending = false;
    if (full) {
        m_node[i].sdo_cfg_state = SDO_CFG_IDLE;
        m_node[i].sdo_cfg_step  = 0U;
        m_node[i].hb_lost       = false;
    }
}

/* Is parcacigi: herhangi bir baglam (ISR, RTOS gorevi, ana dongu). Her oncelikteki
 * es zamanli cagiranlar guvenlidir — yuva yazmasi temel bir IRQ-devre-disi kritik
 * bolumuyle sarilmistir; dolayisiyla daha yuksek oncelikli bir onceliklendirici
 * yirtik bir deger gozlemleyemez veya uretemez; volatile, co_transmit_process()'in
 * bir sonraki SYNC tick'inde kilitlenen degeri almasini garanti eder. */
void cia402_set_target_rpm(uint16_t node_id_u16, float target_rpm_f32)
{
    const int8_t idx = node_find((uint8_t)node_id_u16);
    if (idx < 0) { return; }
    const uint32_t pm = co_enter_critical();
    g_target_rpm[idx] = target_rpm_f32;
    co_exit_critical(pm);
}

static void cia402_apply_target_internal(uint8_t instance, float rpm)
{
    if (instance >= cia402_node_count) { return; }
    if (rpm >  CIA402_MAX_VEL_RPM) { rpm =  CIA402_MAX_VEL_RPM; }
    if (rpm < -CIA402_MAX_VEL_RPM) { rpm = -CIA402_MAX_VEL_RPM; }
	if (DRIVER_ZEROERR == m_node[instance].driver_type_st) {
		m_node[instance].target_vel = rpm * m_node[instance].counts_per_rpm_s;
		/* tork: CiA 402 0x6077 nominal torkunun %0.1 birimindedir */
	} else if(DRIVER_DELTA == m_node[instance].driver_type_st) {
		m_node[instance].target_vel = rpm * m_node[instance].ratio * 10.0f;
	}

}

/* ── SDO gonderme yardimcisi ─────────────────────────────────────────────── */
static void sdo_send(uint8_t node_id, uint16_t index, uint8_t subindex,
                          uint32_t value, uint8_t size)
{
    uint8_t cmd;
    switch (size) {
        case 1U: cmd = 0x2FU; break;
        case 2U: cmd = 0x2BU; break;
        case 3U: cmd = 0x27U; break;
        default: cmd = 0x23U; break;
    }
    co_can_frame_t f;
    f.cob_id  = 0x600U + node_id;
    f.len     = 8U;
    f.data[0] = cmd;
    f.data[1] = (uint8_t)(index & 0xFFU);
    f.data[2] = (uint8_t)(index >> 8U);
    f.data[3] = subindex;
    f.data[4] = (uint8_t)(value         & 0xFFU);
    f.data[5] = (uint8_t)((value >>  8U) & 0xFFU);
    f.data[6] = (uint8_t)((value >> 16U) & 0xFFU);
    f.data[7] = (uint8_t)((value >> 24U) & 0xFFU);
    (void)canopen_node.iface.send(canopen_node.iface.user, &f);
}

/* ── NMT durum degisikligi ───────────────────────────────────────────────── */
static void on_slave_nmt_change(uint8_t i, co_nmt_state_t st)
{
    switch (st) {
    case CO_NMT_INITIALIZING:
        if (m_node[i].sdo_cfg_state == SDO_CFG_RESET_DELAY) { break; }
        node_reset(i, true);
        break;
    case CO_NMT_PRE_OPERATIONAL:
        /* Kendi baslangicimizda beklenen durum (NMT sifirlama / 0x80 gonderdik) —
         * ucustaki yapilandirma dizisine dokunmayin. */
        if (m_node[i].sdo_cfg_state == SDO_CFG_NMT_DELAY ||
            m_node[i].sdo_cfg_state == SDO_CFG_RESET_DELAY) {
            break;
        }
        /* Baslatilmis bir surucuden beklenmedik dusus — durumu yikmak yerine
         * kararli hâle getiriciyi kurun. Kole NMT_PREOP_DEBOUNCE_MS icinde
         * OPERATIONAL'a donerse bu gecici bir HB olarak degerlendirilir ve
         * master kesintisiz devam eder; aksi hâlde ana dongu tam yeniden
         * yapilandirmaya gecer. Idempotent: pencere icindeki ikinci bir
         * PRE_OP gecisi zamanlayiciyi yeniden baslatmaz. */
        if (!m_node[i].pre_op_pending) {
            m_node[i].pre_op_pending  = true;
            m_node[i].pre_op_entry_ms = now_ms();
        }
        break;
    case CO_NMT_STOPPED:
        node_reset(i, true);
        break;
    case CO_NMT_OPERATIONAL:
        /* Kole OP'ye dondu — bekleyen yeniden yapilandirmayi iptal edin. */
        m_node[i].pre_op_pending = false;
        break;
    }
}

/* ── on_rx_frame kancasi ─────────────────────────────────────────────────── */
static void on_rx_frame(void *user, const co_can_frame_t *frame)
{
    (void)user;

    /* Kalp atisi: ornek basina NMT durumunu takip et */
    if (frame->cob_id >= 0x701U && frame->cob_id <= 0x77FU && frame->len >= 1U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x700U);
        const int8_t  idx = node_find(nid);
        if (idx >= 0) {
            const co_nmt_state_t st = (co_nmt_state_t)(frame->data[0] & 0x7FU);
            if (st != m_node[idx].nmt_state) {
                m_node[idx].nmt_state = st;
                on_slave_nmt_change((uint8_t)idx, st);
            }
        }
        return;
    }

    /* EMCY (COB-ID = 0x080 + dugum_kimligi): ornek basina hata kodunu yakala */
    if (frame->cob_id >= 0x081U && frame->cob_id <= 0x0FFU && frame->len >= 3U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x080U);
        const int8_t  idx = node_find(nid);
        if (idx >= 0) {
            g_emcy_code[idx] = (uint16_t)frame->data[0]
                                  | ((uint16_t)frame->data[1] << 8);
            g_emcy_reg[idx]  = frame->data[2];
        }
        return;
    }

    /* SDO yaniti (COB-ID = 0x580 + dugum_kimligi): bekleyen ornegi sinyal.
     *
     * SCS bitleri (data[0] & 0xE0):
     *   0x60 = hizlandirilmis/segmentli indirme onayi → basari (yazma ack)
     *   0x40 = yukleme yaniti                         → basari (okuma sonucu;
     *                                                    yuk data[4..7]'de)
     * data[0] == 0x80                                 → iptal
     *
     * Iptaller sessizce basari olarak degerlendirilmemelidir: rt kuyrugu
     * cagiranin haberi olmadan ilerlerdi. 4 baytlik iptal kodunu (data[4..7])
     * ve basarisiz girisin indeks/alt-indeksini ornek basina alanlara ve
     * g_sdo_rt_abort_*[] yuzeylerine kaydedin. Iptal durumunda kuyruk hâlâ
     * ilerler — yeniden denemek ayni kodu uretir — ancak hata artik gozlemlenebilir.
     *
     * Yukleme (okuma) onaylari icin hizlandirilmis yuku sdo_rt_resp_value'ya
     * da saklariz; boylece gelecekteki bir okuma API'si kuyruk ilerlemeden ve
     * giris uzerine yazilmadan once buradan alabilir. */
    if (frame->cob_id >= 0x581U && frame->cob_id <= 0x5FFU && frame->len >= 1U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x580U);
        const int8_t  idx = node_find(nid);
        if (idx >= 0) {
            const uint8_t scs           = (uint8_t)(frame->data[0] & 0xE0U);
            const bool    is_dl_confirm = (scs == 0x60U);
            const bool    is_ul_confirm = (scs == 0x40U);
            const bool    is_abort      = (frame->data[0] == 0x80U);
            if (is_dl_confirm || is_ul_confirm || is_abort) {
                if (m_node[idx].sdo_cfg_state == SDO_CFG_WAIT_RESPONSE) {
                    m_node[idx].sdo_cfg_resp_ok = true;
                } else if (m_node[idx].sdo_rt_active) {
                    if (is_abort && frame->len >= 8U) {
                        const uint32_t code = (uint32_t)frame->data[4]
                                            | ((uint32_t)frame->data[5] << 8)
                                            | ((uint32_t)frame->data[6] << 16)
                                            | ((uint32_t)frame->data[7] << 24);
                        const sdo_entry_t *w =
                            &m_node[idx].sdo_rt_queue[m_node[idx].sdo_rt_tail];
                        m_node[idx].sdo_rt_abort_code     = code;
                        m_node[idx].sdo_rt_abort_index    = w->index;
                        m_node[idx].sdo_rt_abort_subindex = w->subindex;
                        g_sdo_rt_abort_code[idx]     = code;
                        g_sdo_rt_abort_index[idx]    = w->index;
                        g_sdo_rt_abort_subindex[idx] = w->subindex;
                    } else if (is_ul_confirm && frame->len >= 8U) {
                        m_node[idx].sdo_rt_resp_value =
                              (uint32_t)frame->data[4]
                            | ((uint32_t)frame->data[5] << 8)
                            | ((uint32_t)frame->data[6] << 16)
                            | ((uint32_t)frame->data[7] << 24);
                    }
                    m_node[idx].sdo_rt_resp_ok = true;
                }
            }
        }
    }
}

/* ── SDO yapilandirma durum makinesi (ornek basina) ──────────────────────── */
static bool sdo_cfg_run(uint8_t i)
{
    node_state_t *e  = &m_node[i];
    if (e->sdo_cfg_state == SDO_CFG_DONE) { return true; }

    const uint32_t t   = now_ms();
    const uint8_t  nid = cia402_nodes[i].node_id;

    if (e->sdo_cfg_state == SDO_CFG_IDLE) {
        (void)co_nmt_master_send(&canopen_node, 0x82U, nid);
        e->sdo_cfg_step    = 0U;
        e->sdo_cfg_resp_ok = false;
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = SDO_CFG_RESET_DELAY;
        return false;
    }

    if (e->sdo_cfg_state == SDO_CFG_RESET_DELAY) {
        if ((t - e->sdo_cfg_step_ms) < NMT_RESET_DELAY_MS) { return false; }
        (void)co_nmt_master_send(&canopen_node, 0x80U, nid);
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = SDO_CFG_NMT_DELAY;
        return false;
    }

    if (e->sdo_cfg_state == SDO_CFG_NMT_DELAY) {
        if ((t - e->sdo_cfg_step_ms) < NMT_PREOP_DELAY_MS) { return false; }
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = SDO_CFG_WAIT_RESPONSE;
        const sdo_entry_t *w = &m_sdo_tbl[i][0];
        sdo_send(nid, w->index, w->subindex, w->value, w->size);
        return false;
    }

    /* SDO_CFG_YANIT_BEKLE */
    if (!e->sdo_cfg_resp_ok) {
        if ((t - e->sdo_cfg_step_ms) >= SDO_TIMEOUT_MS) {
            e->sdo_cfg_state = SDO_CFG_IDLE;
            e->sdo_cfg_step  = 0U;
        }
        return false;
    }

    e->sdo_cfg_resp_ok = false;
    e->sdo_cfg_step++;
    if (e->sdo_cfg_step >= m_sdo_steps[i]) {
        e->sdo_cfg_state = SDO_CFG_DONE;
        return true;
    }
    e->sdo_cfg_step_ms = t;
    const sdo_entry_t *w = &m_sdo_tbl[i][e->sdo_cfg_step];
    sdo_send(nid, w->index, w->subindex, w->value, w->size);
    return false;
}

/* ── Calisma zamani SDO yazma kuyrugu ───────────────────────────────────────
 * sdo_rt_enqueue() genel API'den cagrilir (orn. cia402_set_accel).
 * sdo_rt_run() her ana dongu tick'inde cagrilir; bir seferde bir giris gonderir
 * ve bir sonrakine gecmeden once SDO yanitini bekler.
 * Yalnizca master_started = true oldugunda calisir; dolayisiyla ayni SDO
 * kanalini kullanan baslangic yapilandirma dizisiyle hicbir zaman yarismaz. */

/* Calisma zamani SDO kuyrugunun uretici tarafi. Yuva yazmasi + bas ilerlemesi
 * IRQ-devre-disi kritik bolumuyle sarilmistir; dolayisiyla her oncelikteki
 * es zamanli ureticiler — guncelleme ortasinda bir RTOS gorevini onceliklendiren
 * bir ISR dahil — yazimlarini ayni yuvaya serpistiremez veya bas isaretcisini
 * yarisarak bir kuyrugu kaybedemez. Tuketici (sdo_rt_run()) yalnizca tail'i
 * degistirir ve bu nedenle korumaya ihtiyac duymaz; ancak bundan yararlanir:
 * okudugu her bas degeri tam olarak yayimlanmistir. */
static bool sdo_rt_enqueue(uint8_t i, uint16_t index, uint8_t subindex,
                                 uint32_t value, uint8_t size)
{
    node_state_t *e   = &m_node[i];
    const uint32_t pm = co_enter_critical();
    const uint8_t next = (uint8_t)((e->sdo_rt_head + 1U) % SDO_RT_QUEUE_SIZE);
    if (next == e->sdo_rt_tail) {
        co_exit_critical(pm);
        return false;                                   /* dolu — cagiran yeniden denemeli */
    }
    e->sdo_rt_queue[e->sdo_rt_head] =
        (sdo_entry_t){ index, subindex, value, size };
    e->sdo_rt_head = next;
    co_exit_critical(pm);
    return true;
}

static void sdo_rt_run(uint8_t i)
{
    node_state_t *e = &m_node[i];
    if (!e->master_started) { return; }

    const uint32_t t = now_ms();

    if (e->sdo_rt_active) {
        if (e->sdo_rt_resp_ok) {
            /* Yanit alindi — girisi tuket ve devam et. */
            e->sdo_rt_resp_ok = false;
            e->sdo_rt_active  = false;
            e->sdo_rt_tail    = (uint8_t)((e->sdo_rt_tail + 1U) % SDO_RT_QUEUE_SIZE);
        } else if ((t - e->sdo_rt_ms) >= SDO_TIMEOUT_MS) {
            /* Zaman asimi — girisi dusur ve geri kalanla devam et. */
            e->sdo_rt_active = false;
            e->sdo_rt_tail   = (uint8_t)((e->sdo_rt_tail + 1U) % SDO_RT_QUEUE_SIZE);
        }
        return;
    }

    if (e->sdo_rt_tail == e->sdo_rt_head) { return; }  /* kuyruk bos */

    const sdo_entry_t *w = &e->sdo_rt_queue[e->sdo_rt_tail];
    sdo_send(cia402_nodes[i].node_id, w->index, w->subindex, w->value, w->size);
    e->sdo_rt_active  = true;
    e->sdo_rt_resp_ok = false;
    e->sdo_rt_ms      = t;
}

/* ── CiA 402 durum makinesi (ornek basina) ───────────────────────────────── */
static void cia402_step(uint8_t i)
{
    node_state_t *e   = &m_node[i];
    const uint32_t t  = now_ms();
    const uint16_t sw = e->statusword;

    switch (e->drive_state) {

    case MASTER_CIA402_IDLE:
        if (cia402_sw_fault_reaction_active(sw)) {
            break;
        } else if (cia402_sw_fault(sw)) {
            e->controlword    = CW_FAULT_RESET;
            cia402_log_transition(i, MASTER_CIA402_FAULT_RESET);
            e->state_entry_ms = t;
        } else if (sw != 0U) {
            /* Sifirdan farkli, hata olmayan her statusword; Quick Stop Active
             * (0x0003) ve QS biti ayarli Switch On Disabled (0x0060) dahil.
             * CiA 402 Tablo 7, QS'yi bircok durumda "onemsiz" olarak isaretler;
             * dolayisiyla kesin maske kontrolleri beklenmedik bitlerle gecerli
             * kelimeleri kacirir. CW hemen ayarlanir; boylece bir sonraki master
             * TPDO'su Shutdown komutunu tasir — bu olmadan surucu bir olay
             * zamanlayicisi dongusu icin eski CW'yi gorur; bu da surucunun
             * Switch On Disabled'da "oyalanmasinin" tipik nedenidir. */
            e->controlword    = CW_SHUTDOWN;
            cia402_log_transition(i, MASTER_CIA402_SHUTDOWN);
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_FAULT_RESET:
        if ((t - e->state_entry_ms) >= CIA402_FAULT_RESET_MS) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            cia402_log_transition(i, MASTER_CIA402_WAIT_FAULT_CLEAR);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_FAULT_RESET;
        }
        break;

    case MASTER_CIA402_WAIT_FAULT_CLEAR:
        if (!cia402_sw_fault(sw)) {
            e->controlword    = CW_SHUTDOWN;
            cia402_log_transition(i, MASTER_CIA402_SHUTDOWN);
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= CIA402_STATE_TIMEOUT_MS) {
            e->controlword    = CW_FAULT_RESET;
            cia402_log_transition(i, MASTER_CIA402_FAULT_RESET);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_FAULT_RESET_CLEAR;
        }
        break;

    case MASTER_CIA402_SHUTDOWN:
        if (cia402_sw_ready_to_switch_on(sw)) {
            e->controlword    = CW_SWITCH_ON;
            cia402_log_transition(i, MASTER_CIA402_SWITCH_ON);
            e->state_entry_ms = t;
        } else if (cia402_sw_fault(sw)) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            cia402_log_transition(i, MASTER_CIA402_FAULT);
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= CIA402_STATE_TIMEOUT_MS) {
            e->controlword    = CW_FAULT_RESET;
            cia402_log_transition(i, MASTER_CIA402_FAULT_RESET);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_SHUTDOWN;
        }
        break;

    case MASTER_CIA402_SWITCH_ON:
        if (cia402_sw_switched_on(sw)) {
            e->controlword    = CW_ENABLE_OPERATION;
            cia402_log_transition(i, MASTER_CIA402_ENABLE);
            e->state_entry_ms = t;
        } else if (cia402_sw_fault(sw)) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            cia402_log_transition(i, MASTER_CIA402_FAULT);
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= CIA402_STATE_TIMEOUT_MS) {
            e->controlword    = CW_FAULT_RESET;
            cia402_log_transition(i, MASTER_CIA402_FAULT_RESET);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_SWITCH_ON;
        }
        break;

    case MASTER_CIA402_ENABLE:
        if (cia402_sw_operation_enabled(sw)) {
            cia402_log_transition(i, MASTER_CIA402_RUNNING);
            e->state_entry_ms = t;
            e->controlword    = CW_ENABLE_OPERATION;
        } else if (cia402_sw_fault(sw)) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            cia402_log_transition(i, MASTER_CIA402_FAULT);
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= CIA402_STATE_TIMEOUT_MS) {
            e->controlword    = CW_FAULT_RESET;
            cia402_log_transition(i, MASTER_CIA402_FAULT_RESET);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_ENABLE_OPERATION;
        }
        break;

    case MASTER_CIA402_RUNNING:
        if (cia402_sw_fault(sw)) {
            e->target_vel     = 0;
            e->controlword    = CW_FAULT_RESET_CLEAR;
            cia402_log_transition(i, MASTER_CIA402_FAULT);
            e->state_entry_ms = t;
        } else if (!cia402_sw_operation_enabled(sw)) {
            /* Surucu Hata bildirmeden Operation Enabled'dan cikti
             * (orn. Quick Stop Active, kablo yeniden takilmasindan sonra Switch On
             * Disabled veya bir engelleme girisi). Hizi sifirlayin ve IDLE'a
             * geri donun; boylece durum makinesi bir sonraki RPDO1'de canli
             * statusword'u yeniden okur ve dogru CiA 402 yeniden etkinlestirme
             * dizisini calistirir. */
            e->target_vel     = 0;
            e->controlword    = CW_SHUTDOWN;
            cia402_log_transition(i, MASTER_CIA402_IDLE);
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_ENABLE_OPERATION;
        }
        break;

    case MASTER_CIA402_FAULT:
        e->target_vel = 0;
        if (cia402_sw_fault_reaction_active(sw)) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            e->state_entry_ms = t;
            break;
        }
        if ((t - e->state_entry_ms) < CIA402_FAULT_RESET_MS) {
            e->controlword = CW_FAULT_RESET;
        } else if (!cia402_sw_fault(sw)) {
            e->controlword    = CW_SHUTDOWN;
            cia402_log_transition(i, MASTER_CIA402_SHUTDOWN);
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= CIA402_STATE_TIMEOUT_MS) {
            /* Hata sifirlama dongusunu yeniden baslat. */
            e->controlword    = CW_FAULT_RESET;
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_FAULT_RESET_CLEAR;
        }
        break;
    }
}

/* ── RPDO cerceve geri cagirmasi ─────────────────────────────────────────── */
static void on_rpdo_frame(co_node_t *node, uint8_t rpdo_num, void *user)
{
    (void)node; (void)user;
    const uint8_t i = rpdo_num / 2U;
    if (i >= cia402_node_count) { return; }
    if (!((cia402_nodes[i].driver_type == DRIVER_ZEROERR) ||
         (cia402_nodes[i].driver_type == DRIVER_DELTA  ))   ) { return; }
    if (rpdo_num % 2U == 0U) {   /* RPDO1: statusword + mode + velocity */
        m_node[i].last_rpdo1_ms = now_ms();
        cia402_step(i);
    }
    /* RPDO2 (konum + tork) anlik islem gerektirmez */
}

/* ── Kalp atisi tuketici geri cagirmasi ──────────────────────────────────── */
static void on_hb_event(co_node_t *node, uint8_t slave_node_id,
                        co_hb_event_t event, void *user)
{
    (void)node; (void)user;
    const int8_t idx = node_find(slave_node_id);
    if (idx < 0) { return; }
    const uint8_t i = (uint8_t)idx;

    if (event == CO_HB_EVENT_TIMEOUT) {
        /* Surucu baslatilmadan once tetiklenen zaman asimlarini yoksay.
         * sdo_cfg_run() ilk adim olarak bir NMT sifirlamasi (0x82) gonderir;
         * bu surucuyu ~500 ms boyunca cevrimdisi alir — SLAVE_HB_TIMEOUT_MS'den
         * daha uzun. Bu beklenen bosluk SDO yapilandirma dizisini bozmamalidir.
         * Baslangici hic tamamlayamamis suruculerin kurtarilmasi, bu yoldan
         * degil, NMT kalp atisi geri cagirmalari araciligiyla
         * on_slave_nmt_change() tarafindan ele alinir. */
        if (!m_node[i].master_started) { return; }
        m_node[i].target_vel  = 0;
        m_node[i].controlword = CW_QUICK_STOP;
        (void)co_send_tpdo(&canopen_node, (uint8_t)(i * 2U));
        m_node[i].controlword    = CW_SHUTDOWN;
        cia402_log_transition(i, MASTER_CIA402_IDLE);
        m_node[i].sdo_cfg_state  = SDO_CFG_IDLE;
        m_node[i].sdo_cfg_step   = 0U;
        m_node[i].master_started = false;
        m_node[i].hb_lost        = true;
    } else {
        if (!m_node[i].hb_lost) { return; }
        m_node[i].hb_lost        = false;
        m_node[i].sdo_cfg_state  = SDO_CFG_IDLE;
        m_node[i].sdo_cfg_step   = 0U;
        m_node[i].master_started = false;
        cia402_log_transition(i, MASTER_CIA402_IDLE);
        m_node[i].state_entry_ms = now_ms();
    }
}

/* ── NMT Iletisim Sifirlama geri cagirmasi ───────────────────────────────── */
static void on_reset_communication(void *user)
{
    (void)user;
    for (uint8_t i = 0U; i < cia402_node_count; ++i) {
        node_reset(i, true);
    }
}


/* ── Genel API ───────────────────────────────────────────────────────────── */

/* Is parcacigi: herhangi bir baglam. Ana dongu RPDO geri cagirma yolundan
 * yazilan m_node[] PDO geri bildirim alanlarini okur; konum/hizin 32-bit
 * okumalari es zamanli bir RPDO guncellemesine gore yirtilabilir.
 * Telemetri icin uygundur, alt-dongu senkronizasyonu icin degil. */
void cia402_get_pos_vel(uint16_t node_id_u16, float *const p_pos_deg_f32,
                      float *const p_vel_rpm_f32, float *const p_torque_pct_f32)
{
    const int8_t idx = node_find((uint8_t)node_id_u16);
    if (idx < 0) { return; }
	const int32_t cpr = m_node[idx].encoder_counts_per_rev ;
	/* Olceklendirmeden once isaretli enkoder sayilarini [0, counts_per_rev)
	 * araligina sar. C, negatif modulu sifira dogru kirpar; dolayisiyla
	 * sonucun her iki isaret icin [0, 360) araligina dusmesi icin acik bir
	 * geri-ekleme gerekir. */
	int32_t mod_counts = m_node[idx].position_act % cpr ;
	if (mod_counts < 0) { mod_counts += cpr; }
	*p_pos_deg_f32 = (float)mod_counts * (360.0f / (float)cpr);

	if (DRIVER_ZEROERR == m_node[idx].driver_type_st) {
        /* hiz: count/s ÷ counts_per_rpm_s = RPM */
		*p_vel_rpm_f32    = (float)m_node[idx].velocity_act / (float)m_node[idx].counts_per_rpm_s;
	} else if(DRIVER_DELTA == m_node[idx].driver_type_st) {
		*p_vel_rpm_f32    = (float)m_node[idx].velocity_act / (10.0f * m_node[idx].ratio);
	}

	/* tork: CiA 402 0x6077 nominal torkunun %0.1 birimindedir */
    *p_torque_pct_f32 = (float)m_node[idx].torque_act * 0.1f;
}

/* Is parcacigi: herhangi bir baglam (ISR, RTOS gorevi, ana dongu). Es zamanli
 * cagiranlar sdo_rt_enqueue() icindeki IRQ-devre-disi kritik bolumu tarafindan
 * serilestirilir; iki uretici bas isaretcisini yarisamaz. Tuketici
 * (app_canopen_loop() icindeki sdo_rt_run()) yalnizca tail'i degistirir ve
 * bu nedenle bu yolla hicbir zaman cakismaz. */
void cia402_set_accel(uint16_t node_id_u16, uint32_t accel_plus_s, uint32_t decel_plus_s)
{
    const int8_t idx = node_find((uint8_t)node_id_u16);
    if (idx < 0) { return; }
    (void)sdo_rt_enqueue((uint8_t)idx, 0x6083U, 0x00U, accel_plus_s, 4U);
    (void)sdo_rt_enqueue((uint8_t)idx, 0x6084U, 0x00U, decel_plus_s, 4U);
}

/* ── Baslatma ────────────────────────────────────────────────────────────── */
/* Is parcacigi: sistem baslatma sirasinda bir kez cagrilir; FDCAN kesmeleri
 * etkinlestirilmeden ve app_canopen_loop() / co_transmit_process() baslamadan once. */
co_error_t app_canopen_init(void)
{
    co_error_t err;

    /* Tabloyu dolasmadan once uygulama tarafindan saglanan tabloyu aralik
     * kontrolunden gecirin. Sayi farkli bir ceviri biriminden gelir; dolayisiyla
     * depolama siniri yalnizca baslangicta uygulanabilir, derleyici tarafindan degil. */
    if (cia402_node_count == 0U || cia402_node_count > CIA402_MAX_NODES) {
        return CO_ERROR_INVALID_ARGS;
    }

    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx,
                    MASTER_NODE_ID, MASTER_HEARTBEAT_MS);
    canopen_node.iface.on_reset_communication = on_reset_communication;
    canopen_node.iface.on_rx_frame            = on_rx_frame;

    for (uint8_t i = 0U; i < cia402_node_count; ++i) {
        build_sdo_table(i);

        /* Enkoder sabitlerini bir kez onbellege al. counts_per_rpm_s, disli
         * indirimini RPM↔count donusumune katar; boylece genel API'ye saglanan
         * degerler CIKIS milinde yorumlanir. 64-bit ara deger, buyuk CPR × oran
         * carpimlarinda tasmaya karsi korur; sonuc herhangi bir gercekci aktuator
         * icin int32_t'ye sigar. */
        const uint32_t cpr   = cia402_nodes[i].encoder_counts_per_rev;
        const uint16_t ratio = cia402_nodes[i].reductor_ratio;
        m_node[i].encoder_counts_per_rev = (int32_t)cpr * ratio;
        m_node[i].ratio = ratio;
        /* counts_per_rpm_s: cikis mili RPM'inden bu surucunun kullandigi hiz
         * kayit birimine donusum faktoru.
         *   DRIVER_ZEROERR: 0x60FF / 0x606C enkoder count/s biriminde
         *     → 1 RPM_cikis = cpr * oran / 60 count/s
         *   DRIVER_DELTA (ASDA-B3): 0x60FF / 0x606C, 0.1-RPM biriminde
         *     → 1 RPM_cikis = oran motor RPM = oran * 10, 0.1-RPM biriminde */
        if (cia402_nodes[i].driver_type == DRIVER_DELTA) {
            m_node[i].counts_per_rpm_s = (int32_t)ratio * 10;
        } else {
            m_node[i].counts_per_rpm_s =
                (int32_t)(((uint64_t)cpr * (uint64_t)ratio) / 60U);
        }

        m_node[i].controlword = CW_SHUTDOWN;
        m_node[i].mode_of_op  = CIA402_MODE_PROFILE_VEL;
        m_node[i].drive_state = MASTER_CIA402_IDLE;
        m_node[i].nmt_state     = CO_NMT_INITIALIZING;
        m_node[i].driver_type_st = cia402_nodes[i].driver_type;
        /* Asagidaki OD yedek depolama ve PDO haritalama CiA 402 duzenini varsayar
         * (controlword 0x6040, statusword 0x6041, hedef hiz 0x60FF, vb.).
         * CiA-402 disi bir surucu, build_sdo_table()'dan yalnizca kalp atisi
         * iceren bir taslak SDO tablosu alir; aksi hâlde dokunulmaz birakilir —
         * alternatif surucunun OD'si tanimlandiginda bu dali genisletin. */
        if (!((cia402_nodes[i].driver_type == DRIVER_ZEROERR) ||
              (cia402_nodes[i].driver_type == DRIVER_DELTA  ))   ) {
            continue;
        }

        /* Saticiya ozgu OD indeksleri ornekler arasindaki cakismayi onler.
         * Ornek i TPDO verisi: 0x4000+i,  RPDO verisi: 0x4100+i  (alt 1–5) */
        const uint16_t ti = (uint16_t)(0x4000U + i);
        const uint16_t ri = (uint16_t)(0x4100U + i);

        err = co_od_add(&canopen_node, ti, 0x01U,
                        (uint8_t *)&m_node[i].controlword,   2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ti, 0x02U,
                        (uint8_t *)&m_node[i].mode_of_op,    1U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ti, 0x03U,
                        (uint8_t *)&m_node[i].target_vel,    4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }

        err = co_od_add(&canopen_node, ri, 0x01U,
                        (uint8_t *)&m_node[i].statusword,    2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x02U,
                        (uint8_t *)&m_node[i].mode_display,  1U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x03U,
                        (uint8_t *)&m_node[i].velocity_act,  4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x04U,
                        (uint8_t *)&m_node[i].position_act,  4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x05U,
                        (uint8_t *)&m_node[i].torque_act,    2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }

        /* RPDO iletisim + haritalama. Ornek i, rpdo_num = i*2 ve i*2+1 kullanir. */
        const uint8_t  rn0      = (uint8_t)(i * 2U);
        const uint8_t  rn1      = (uint8_t)(i * 2U + 1U);
        const uint32_t rcob0    = 0x180UL + cia402_nodes[i].node_id;
        const uint32_t rcob1    = 0x280UL + cia402_nodes[i].node_id;
        const uint8_t  rpdo_syn = 0x01U;
        const uint8_t  z = 0U, two = 2U, three = 3U;

        if (co_od_write(&canopen_node, 0x1400U + rn0, 0x01U,
                        (const uint8_t *)&rcob0, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1400U + rn0, 0x02U,
                        &rpdo_syn, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t r1m1 = ((uint32_t)ri << 16) | (0x01UL << 8) | 16UL;
        const uint32_t r1m2 = ((uint32_t)ri << 16) | (0x02UL << 8) |  8UL;
        const uint32_t r1m3 = ((uint32_t)ri << 16) | (0x03UL << 8) | 32UL;
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x00U, &z,    1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x01U, (const uint8_t *)&r1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x02U, (const uint8_t *)&r1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x03U, (const uint8_t *)&r1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x00U, &three, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        if (co_od_write(&canopen_node, 0x1400U + rn1, 0x01U,
                        (const uint8_t *)&rcob1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1400U + rn1, 0x02U,
                        &rpdo_syn, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t r2m1 = ((uint32_t)ri << 16) | (0x04UL << 8) | 32UL;
        const uint32_t r2m2 = ((uint32_t)ri << 16) | (0x05UL << 8) | 16UL;
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x00U, &z,   1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x01U, (const uint8_t *)&r2m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x02U, (const uint8_t *)&r2m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x00U, &two,  1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        /* TPDO iletisim + haritalama. Ornek i yalnizca tpdo_num = i*2 kullanir. */
        const uint8_t  tn0   = (uint8_t)(i * 2U);
        const uint32_t tcob0 = 0x200UL + cia402_nodes[i].node_id;
        const uint8_t  tev   = 0xFFU;
        const uint16_t tms   = 1U;

        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x01U,
                        (const uint8_t *)&tcob0, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x02U, &tev, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x05U,
                        (const uint8_t *)&tms, 2U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t t1m1 = ((uint32_t)ti << 16) | (0x01UL << 8) | 16UL;
        const uint32_t t1m2 = ((uint32_t)ti << 16) | (0x02UL << 8) |  8UL;
        const uint32_t t1m3 = ((uint32_t)ti << 16) | (0x03UL << 8) | 32UL;
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x00U, &z,    1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x01U, (const uint8_t *)&t1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x02U, (const uint8_t *)&t1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x03U, (const uint8_t *)&t1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x00U, &three, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    }

    co_set_rpdo_frame_hook(&canopen_node, on_rpdo_frame, NULL);
    co_set_hb_event_hook(&canopen_node,  on_hb_event,   NULL);

    /* SYNC ureticisi: master her 1 ms'de COB-ID 0x080 uretir */
    const uint32_t sync_cob_id   = 0x40000080UL;
    const uint32_t sync_cycle_us = SYNC_PERIOD_US;
    if (co_od_write(&canopen_node, 0x1005U, 0x00U,
                    (const uint8_t *)&sync_cob_id,   4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1006U, 0x00U,
                    (const uint8_t *)&sync_cycle_us, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    /* Kalp atisi tuketicisi: kole ornegi basina bir alt-indeks */
    const uint8_t hb_count = cia402_node_count;
    if (co_od_write(&canopen_node, 0x1016U, 0x00U,
                    &hb_count, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    for (uint8_t i = 0U; i < cia402_node_count; ++i) {
        const uint32_t hb = ((uint32_t)cia402_nodes[i].node_id << 16)
                          | (uint32_t)SLAVE_HB_TIMEOUT_MS;
        if (co_od_write(&canopen_node, 0x1016U, (uint8_t)(i + 1U),
                        (const uint8_t *)&hb, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    }

    return CO_ERROR_NONE;
}

/* ── FDCAN RX kesmesi ────────────────────────────────────────────────────── */

/* Is parcacigi: yalnizca FDCAN RX FIFO0 ISR. HAL tarafindan kesme baglamindan
 * cagrilir; gorev kodundan cagirmayin. FDCAN FIFO'sunu app_canopen_loop()
 * tarafindan tuketilen SPSC halka tamponuna bosaltir. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (hfdcan == &hfdcan1) {
        co_stm32_rx_isr(&canopen_node, &can_ctx);
    }
}

/* ── Ana dongu ───────────────────────────────────────────────────────────── */
/* Is parcacigi: yalnizca ana dongu / tek gorev baglami. Yeniden girisli degil.
 * HAL_FDCAN_RxFifo0Callback() tarafindan doldurulan SPSC halkasini tuketir ve
 * ISR kalp atisi zaman damgasi hizli yolu disindaki her m_node[] mutasyonuna
 * sahiptir; dolayisiyla CiA 402 durum makinesini suren tek baglam olmalidir. */
void app_canopen_loop(void)
{
	/* 1. Kuyruklanmis cerceveleri ilet (kalp atisi zaman damgalari ISR tarafindan zaten guncellendi). */
	co_stm32_drain_rx(&canopen_node, &can_ctx);

	/* 2. Master PRE_OPERATIONAL'a ulastiginda kendisini OPERATIONAL'a yukselt. */
	if (canopen_node.nmt_state == CO_NMT_PRE_OPERATIONAL) {
		co_nmt_set_state(&canopen_node, CO_NMT_OPERATIONAL);
	}

	/* 3. Ornek basina baslangic: SDO yapilandirma → NMT Baslatma. */
	if (canopen_node.nmt_state == CO_NMT_OPERATIONAL) {
		for (uint8_t i = 0U; i < cia402_node_count; ++i) {
			if (!m_node[i].master_started && sdo_cfg_run(i)) {
				(void)co_nmt_master_send(&canopen_node, 0x01U,
										cia402_nodes[i].node_id);
				m_node[i].master_started = true;
			}
		}
	}

	/* 4. Baslatilmis her CiA 402 ornegi icin CiA 402 etkinlestirme dizisini ilerlет.
	 *    on_rpdo_frame() hizli yanit icin RPDO1 alindiginda da bunu cagirir;
	 *    ancak ana donguden surmek, RPDO1 cerceveleri kacirilsa dahi (orn. surucu
	 *    SOD'dan cikmakta yavas) ilerlemeyi ve zaman asimi islemeyi garanti eder.
	 *    Durum makinesi idempotent'tir — ayni statusword ve durumla tekrarlanan
	 *    cagrilar guvenlidir. CiA-402 disi ornekler burada atlanir; varsa durum
	 *    makineleri uygulamanin sorumlulugundadir. */
	for (uint8_t i = 0U; i < cia402_node_count; ++i) {
		if ((m_node[i].master_started                       ) &&
		    ((cia402_nodes[i].driver_type == DRIVER_ZEROERR) ||
		     (cia402_nodes[i].driver_type == DRIVER_DELTA  ))   ) {
			cia402_step(i);
		}
	}

	/* 5. RPDO izleme: RUNNING durumunda ancak CIA402_RPDO_WATCHDOG_MS sure
	 *    RPDO1 alinmadiysa Quick Stop gonder; surucunun kendi olay zamanlayicisi
	 *    yedek durdurma saglar. */
	const uint32_t now = now_ms();
	for (uint8_t i = 0U; i < cia402_node_count; ++i) {
		if (m_node[i].master_started &&
			m_node[i].drive_state == MASTER_CIA402_RUNNING &&
			(now - m_node[i].last_rpdo1_ms) >= CIA402_RPDO_WATCHDOG_MS) {
			m_node[i].target_vel  = 0;
			m_node[i].controlword = CW_QUICK_STOP;
			(void)co_send_tpdo(&canopen_node, (uint8_t)(i * 2U));
			cia402_log_transition(i, MASTER_CIA402_IDLE);
		}
	}

	/* 5b. PRE_OP kararli hâle getirme. Onaylanan kole dususu (PRE_OP pencereyi
	 *     asti) → durumu yik ve SDO yapilandirmasini yeniden oynat.
	 *     Pencere icinde OP'ye geri donen gecici dususler on_slave_nmt_change
	 *     tarafindan temizlenir ve bu dala hic ulasmaz. */
	for (uint8_t i = 0U; i < cia402_node_count; ++i) {
		if (m_node[i].pre_op_pending &&
			(now - m_node[i].pre_op_entry_ms) >= NMT_PREOP_DEBOUNCE_MS) {
			node_reset(i, false);                       /* clears pre_op_pending */
			m_node[i].sdo_cfg_state = SDO_CFG_IDLE;
			m_node[i].sdo_cfg_step  = 0U;
		}
	}

	/* 6. Her ornek icin calisma zamani SDO yazma kuyrugunu bosalt. */
	for (uint8_t i = 0U; i < cia402_node_count; ++i) {
		sdo_rt_run(i);
	}
}

/* Cagirma politikasi — cagri noktasini baglamadan once bunu okuyun:
 *
 *   • Her SYNC_PERIOD_US'da (varsayilan olarak 1 ms) cagrilmalidir. Bu,
 *     stack'in iletim yarisidir — SYNC cercevesini uretir, g_target_rpm[]'yi
 *     OD'ye kilitler ve her surucuye controlword + hedef hizi tasiyan eszamanli
 *     TPDO'lari yayar. Bir tick atlamak kolelerin o dongu icin eski CW/hedef
 *     gormesi anlamina gelir; surekli atlama master'in kendi HB ureticisini ve
 *     surucu tarafindaki RPDO olay zamanlayicisini tetikler ve suruculeri
 *     "iletisim kesildi" hatasiyla hata durumuna sokar.
 *
 *   • Yuksek oncelikli bir baglamdan calismalidir — bir donanim zamanlayicisi
 *     ISR'si veya en yuksek oncelikli RTOS gorevi. app_canopen_loop() ile dusuk
 *     oncelikli bir ana dongude birlestirmek tehlikelidir: herhangi bir uzun
 *     suren gorev (gunlukleme, flash yazma, yavas bir veri yolundan SDO yukleme)
 *     SYNC periyodunu titretir ve yukaridaki belirtileri yeniden uretir.
 *
 *   • app_canopen_loop() ile yurutme baglamini paylasmalidir — her ikisi de
 *     kilit olmadan m_node[] ve canopen_node SDO/PDO durumuna erisir; dolayisiyla
 *     birbirini onceliklendiremezler. Pratikte bu su anlama gelir: her ikisini
 *     de ayni zamanlayici/gorevden calistirin ya da farkli onceliklerde
 *     yasiyorlarsa m_node[] ve canopen_node'u kritik bolum / mutex ile koruyun.
 */
void co_transmit_process(void)
{
    /* 5. Her SYNC tick'inde uygulama hedef hizlarini ilet. */
    if (canopen_node.sync_event_pending) {
        canopen_node.sync_event_pending = false;
        for (uint8_t i = 0U; i < cia402_node_count; ++i) {
            cia402_apply_target_internal(i, g_target_rpm[i]);
        }
    }

    /* CANopen stack'i calistir: SYNC, TPDO'lar, kalp atisi, HB tuketici zaman asimi. */
	co_process(&canopen_node);
}
