#include "cia402_app.h"

/*
 * Dugum yapilandirma tablosu.
 *
 * Motor muzik modu icin ivme/yavaslamalar maksimuma ayarlanmistir (10 000 000
 * count/s^2); bu sayede surucu, nota osilayon yonu degisikliklerini rampasiz
 * uygular ve motor sargisi duyulabilir ton uretir.
 * Normal surucuye geri donulmek istendiginde cia402_set_accel() ile
 * calisma zamaninda dusurulabilir.
 *
 * Encoder:  ZeroErr eRob — 19-bit mutlak enkoder (524 288 count/devir)
 * Reduktor: Dogrudan tahrik (oran = 1)
 */
const cia402_cfg_t cia402_nodes[] = {
    /*  node_id   accel          decel         encoder_cpr   ratio   driver         */
    {   0x01U,  10000000U,    10000000U,       524288U,      1U,   DRIVER_ZEROERR  },
};

const uint8_t cia402_node_count =
    (uint8_t)(sizeof(cia402_nodes) / sizeof(cia402_nodes[0]));
