#include "canopen.h"
#include "cia402.h"

#include <stdio.h>
#include <string.h>

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))
#define TEST_ASSERT(cond)                                                                                               \
    do {                                                                                                                \
        if (!(cond)) {                                                                                                  \
            fprintf(stderr, "ASSERT FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                   \
            return 1;                                                                                                   \
        }                                                                                                               \
    } while (0)

typedef struct {
    uint32_t now_ms;
    co_can_frame_t tx[256];
    size_t tx_count;
    unsigned reset_comm_count;
    unsigned reset_app_count;
    unsigned reset_any_count;
    co_reset_type_t last_reset_type;
} test_bus_t;

static co_error_t fake_send(void *user, const co_can_frame_t *frame)
{
    test_bus_t *bus = (test_bus_t *)user;
    if (bus->tx_count >= ARRAY_LEN(bus->tx)) {
        return CO_ERROR_HW;
    }
    bus->tx[bus->tx_count++] = *frame;
    return CO_ERROR_NONE;
}

static uint32_t fake_millis(void *user)
{
    test_bus_t *bus = (test_bus_t *)user;
    return bus->now_ms;
}

static void fake_on_reset_comm(void *user)
{
    test_bus_t *bus = (test_bus_t *)user;
    bus->reset_comm_count++;
}

static void fake_on_reset_app(void *user)
{
    test_bus_t *bus = (test_bus_t *)user;
    bus->reset_app_count++;
}

static void fake_on_reset_any(void *user, co_reset_type_t type)
{
    test_bus_t *bus = (test_bus_t *)user;
    bus->reset_any_count++;
    bus->last_reset_type = type;
}

static void setup_node(co_node_t *node, test_bus_t *bus, uint8_t node_id, uint16_t heartbeat_ms)
{
    memset(bus, 0, sizeof(*bus));
    const co_if_t iface = {
        .user = bus,
        .send = fake_send,
        .millis = fake_millis,
        .on_reset_communication = fake_on_reset_comm,
        .on_reset_application = fake_on_reset_app,
        .on_reset = fake_on_reset_any,
    };
    co_init(node, &iface, node_id, heartbeat_ms);
}

static const co_can_frame_t *last_tx(const test_bus_t *bus)
{
    if (bus->tx_count == 0U) {
        return NULL;
    }
    return &bus->tx[bus->tx_count - 1U];
}

static int test_nmt_bootup_reset(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 100);

    TEST_ASSERT(node.nmt_state == CO_NMT_INITIALIZING);
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_PRE_OPERATIONAL);
    TEST_ASSERT(bus.tx_count == 1U);
    TEST_ASSERT(bus.tx[0].cob_id == 0x705U && bus.tx[0].data[0] == 0x00U);

    co_can_frame_t nmt = {.cob_id = 0x000U, .len = 2U, .data = {0x01U, 0x00U}};
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_OPERATIONAL);

    nmt.data[0] = 0x02U;
    nmt.data[1] = 0x09U;
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_OPERATIONAL);

    nmt.data[0] = 0x02U;
    nmt.data[1] = 0x05U;
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_STOPPED);

    nmt.data[0] = 0x80U;
    nmt.data[1] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_PRE_OPERATIONAL);

    nmt.data[0] = 0x81U;
    nmt.data[1] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_INITIALIZING);
    TEST_ASSERT(bus.reset_comm_count == 1U);
    TEST_ASSERT(bus.reset_app_count == 0U);
    TEST_ASSERT(bus.last_reset_type == CO_RESET_COMMUNICATION);

    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_PRE_OPERATIONAL);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x705U && last_tx(&bus)->data[0] == 0x00U);

    nmt.data[0] = 0x82U;
    TEST_ASSERT(co_on_can_rx(&node, &nmt) == CO_ERROR_NONE);
    TEST_ASSERT(node.nmt_state == CO_NMT_INITIALIZING);
    TEST_ASSERT(bus.reset_comm_count == 2U);
    TEST_ASSERT(bus.reset_app_count == 1U);
    TEST_ASSERT(bus.last_reset_type == CO_RESET_APPLICATION);

    return 0;
}

static int test_sdo_expedited_segmented_block(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 0);
    (void)co_process(&node);

    uint16_t u16 = 0x0000U;
    uint8_t seg[10] = {0};
    uint8_t blk[14] = {0};
    TEST_ASSERT(co_od_add(&node, 0x2000, 0x00, (uint8_t *)&u16, sizeof(u16), true, true) == CO_ERROR_NONE);
    TEST_ASSERT(co_od_add(&node, 0x2001, 0x00, seg, sizeof(seg), true, true) == CO_ERROR_NONE);
    TEST_ASSERT(co_od_add(&node, 0x2002, 0x00, blk, sizeof(blk), true, true) == CO_ERROR_NONE);

    co_can_frame_t fr = {.cob_id = 0x605U, .len = 8U};

    /* expedited download */
    memset(&fr.data[0], 0, 8);
    fr.data[0] = 0x2BU;
    fr.data[1] = 0x00U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 0x34U;
    fr.data[5] = 0x12U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(u16 == 0x1234U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x60U);

    /* expedited upload */
    memset(&fr.data[0], 0, 8);
    fr.data[0] = 0x40U;
    fr.data[1] = 0x00U;
    fr.data[2] = 0x20U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x4BU);
    TEST_ASSERT(last_tx(&bus)->data[4] == 0x34U && last_tx(&bus)->data[5] == 0x12U);

    /* abort unknown object */
    fr.data[0] = 0x40U;
    fr.data[1] = 0xFFU;
    fr.data[2] = 0x21U;
    fr.data[3] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);

    /* segmented download success (10 bytes) */
    fr.data[0] = 0x21U;
    fr.data[1] = 0x01U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 10U;
    fr.data[5] = fr.data[6] = fr.data[7] = 0U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x60U);

    memset(&fr.data[0], 0, 8);
    fr.data[0] = 0x00U;
    for (unsigned i = 0; i < 7; ++i) {
        fr.data[1 + i] = (uint8_t)(i + 1U);
    }
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x30U);

    memset(&fr.data[0], 0, 8);
    fr.data[0] = 0x19U;
    fr.data[1] = 8U;
    fr.data[2] = 9U;
    fr.data[3] = 10U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x20U);
    for (unsigned i = 0; i < 10; ++i) {
        TEST_ASSERT(seg[i] == (uint8_t)(i + 1U));
    }

    /* segmented upload success */
    fr.data[0] = 0x40U;
    fr.data[1] = 0x01U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x41U);

    fr.data[0] = 0x60U;
    memset(&fr.data[1], 0, 7);
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x00U);
    TEST_ASSERT(last_tx(&bus)->data[1] == 1U && last_tx(&bus)->data[7] == 7U);

    fr.data[0] = 0x70U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT((last_tx(&bus)->data[0] & 0x01U) == 0x01U);
    TEST_ASSERT(last_tx(&bus)->data[1] == 8U && last_tx(&bus)->data[3] == 10U);

    /* segmented toggle abort */
    fr.data[0] = 0x21U;
    fr.data[1] = 0x01U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 10U;
    fr.data[5] = fr.data[6] = fr.data[7] = 0U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    fr.data[0] = 0x10U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);

    /* block download success */
    memset(blk, 0, sizeof(blk));
    fr.data[0] = 0xC0U;
    fr.data[1] = 0x02U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 2U;
    fr.data[5] = fr.data[6] = fr.data[7] = 0U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0xA4U);

    fr.data[0] = 0x01U;
    for (unsigned i = 0; i < 7; ++i) {
        fr.data[1 + i] = (uint8_t)(i + 1U);
    }
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    fr.data[0] = 0x82U;
    for (unsigned i = 0; i < 7; ++i) {
        fr.data[1 + i] = (uint8_t)(i + 8U);
    }
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0xA2U);

    fr.data[0] = 0xC1U;
    memset(&fr.data[1], 0, 7);
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0xA1U);
    for (unsigned i = 0; i < 14; ++i) {
        TEST_ASSERT(blk[i] == (uint8_t)(i + 1U));
    }

    /* block download abort (sequence mismatch) */
    fr.data[0] = 0xC0U;
    fr.data[1] = 0x02U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 4U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    fr.data[0] = 0x02U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);

    /* block upload success */
    fr.data[0] = 0xA0U;
    fr.data[1] = 0x02U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 3U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(bus.tx[bus.tx_count - 3].data[0] == 0xC6U);
    TEST_ASSERT((bus.tx[bus.tx_count - 2].data[0] & 0x7FU) == 1U);
    TEST_ASSERT((bus.tx[bus.tx_count - 1].data[0] & 0x7FU) == 2U);

    fr.data[0] = 0xA2U;
    fr.data[1] = 2U;
    fr.data[2] = 3U;
    memset(&fr.data[3], 0, 5);
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT((last_tx(&bus)->data[0] & 0xE1U) == 0xC1U);

    /* block upload abort on invalid ack command */
    fr.data[0] = 0xA0U;
    fr.data[1] = 0x02U;
    fr.data[2] = 0x20U;
    fr.data[3] = 0x00U;
    fr.data[4] = 2U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    fr.data[0] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);

    return 0;
}

static int test_sdo_multi_channel_concurrency_and_timeout(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 0);
    (void)co_process(&node);

    uint8_t ch0_obj[10] = {0};
    uint8_t ch1_obj[10] = {0};
    TEST_ASSERT(co_od_add(&node, 0x2100, 0x00, ch0_obj, sizeof(ch0_obj), true, true) == CO_ERROR_NONE);
    TEST_ASSERT(co_od_add(&node, 0x2101, 0x00, ch1_obj, sizeof(ch1_obj), true, true) == CO_ERROR_NONE);

    const uint32_t ch1_rx = 0x625U;
    const uint32_t ch1_tx = 0x5A5U;
    TEST_ASSERT(co_od_write(&node, 0x1201, 0x01, (const uint8_t *)&ch1_rx, sizeof(ch1_rx)) == 0U);
    TEST_ASSERT(co_od_write(&node, 0x1201, 0x02, (const uint8_t *)&ch1_tx, sizeof(ch1_tx)) == 0U);

    co_can_frame_t ch0 = {.cob_id = 0x605U, .len = 8U};
    co_can_frame_t ch1 = {.cob_id = ch1_rx, .len = 8U};

    /* Start segmented download on both channels. */
    ch0.data[0] = 0x21U;
    ch0.data[1] = 0x00U;
    ch0.data[2] = 0x21U;
    ch0.data[3] = 0x00U;
    ch0.data[4] = 10U;
    ch0.data[5] = ch0.data[6] = ch0.data[7] = 0U;
    TEST_ASSERT(co_on_can_rx(&node, &ch0) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x585U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x60U);

    ch1.data[0] = 0x21U;
    ch1.data[1] = 0x01U;
    ch1.data[2] = 0x21U;
    ch1.data[3] = 0x00U;
    ch1.data[4] = 10U;
    ch1.data[5] = ch1.data[6] = ch1.data[7] = 0U;
    TEST_ASSERT(co_on_can_rx(&node, &ch1) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == ch1_tx);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x60U);

    TEST_ASSERT(node.sdo_channels[0].state != 0U);
    TEST_ASSERT(node.sdo_channels[1].state != 0U);

    /* Channel 0 first segment only (keep pending for timeout). */
    memset(ch0.data, 0, sizeof(ch0.data));
    ch0.data[0] = 0x00U;
    for (unsigned i = 0; i < 7; ++i) {
        ch0.data[1 + i] = (uint8_t)(i + 1U);
    }
    TEST_ASSERT(co_on_can_rx(&node, &ch0) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x585U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x30U);

    /* Channel 1 complete transfer independently. */
    memset(ch1.data, 0, sizeof(ch1.data));
    ch1.data[0] = 0x00U;
    for (unsigned i = 0; i < 7; ++i) {
        ch1.data[1 + i] = (uint8_t)(i + 11U);
    }
    TEST_ASSERT(co_on_can_rx(&node, &ch1) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == ch1_tx);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x30U);

    memset(ch1.data, 0, sizeof(ch1.data));
    ch1.data[0] = 0x19U;
    ch1.data[1] = 18U;
    ch1.data[2] = 19U;
    ch1.data[3] = 20U;
    TEST_ASSERT(co_on_can_rx(&node, &ch1) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == ch1_tx);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x20U);
    for (unsigned i = 0; i < 10; ++i) {
        TEST_ASSERT(ch1_obj[i] == (uint8_t)(i + 11U));
    }

    /* Timeout should abort only channel 0 and use channel-0 TX COB-ID. */
    bus.now_ms = 1000U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x585U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);
    TEST_ASSERT(node.sdo_channels[0].state == 0U);
    TEST_ASSERT(node.sdo_channels[1].state == 0U);

    /* Start channel 1 upload and timeout channel 1 specifically. */
    bus.now_ms = 2000U;
    memset(ch1.data, 0, sizeof(ch1.data));
    ch1.data[0] = 0x40U;
    ch1.data[1] = 0x01U;
    ch1.data[2] = 0x21U;
    ch1.data[3] = 0x00U;
    TEST_ASSERT(co_on_can_rx(&node, &ch1) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == ch1_tx);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x41U);

    bus.now_ms = 3000U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == ch1_tx);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x80U);

    return 0;
}

static int test_pdo_remap_and_runtime(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 0);
    (void)co_process(&node);

    uint16_t rpdo_target = 0U;
    uint16_t tpdo_source = 0xBEEFU;
    TEST_ASSERT(co_od_add(&node, 0x3000, 0x00, (uint8_t *)&rpdo_target, sizeof(rpdo_target), true, true) == CO_ERROR_NONE);
    TEST_ASSERT(co_od_add(&node, 0x3001, 0x00, (uint8_t *)&tpdo_source, sizeof(tpdo_source), true, true) == CO_ERROR_NONE);

    uint8_t cnt = 0U;
    TEST_ASSERT(co_od_write(&node, 0x1600, 0x00, &cnt, 1U) == 0U);
    uint32_t bad_map = (0x3999UL << 16U) | 0x0010UL;
    TEST_ASSERT(co_od_write(&node, 0x1600, 0x01, (const uint8_t *)&bad_map, 4U) == CO_SDO_ABORT_NO_OBJECT);

    uint32_t rpdo_map = (0x3000UL << 16U) | 0x0010UL;
    TEST_ASSERT(co_od_write(&node, 0x1600, 0x01, (const uint8_t *)&rpdo_map, 4U) == 0U);
    cnt = 1U;
    TEST_ASSERT(co_od_write(&node, 0x1600, 0x00, &cnt, 1U) == 0U);

    TEST_ASSERT(co_od_write(&node, 0x1A00, 0x00, &(uint8_t){0}, 1U) == 0U);
    uint32_t tpdo_map = (0x3001UL << 16U) | 0x0010UL;
    TEST_ASSERT(co_od_write(&node, 0x1A00, 0x01, (const uint8_t *)&tpdo_map, 4U) == 0U);
    cnt = 1U;
    TEST_ASSERT(co_od_write(&node, 0x1A00, 0x00, &cnt, 1U) == 0U);

    co_can_frame_t fr = {.cob_id = node.rpdo_cfg[0].cob_id.can_id, .len = 2U, .data = {0xCDU, 0xABU}};
    TEST_ASSERT(co_on_can_rx(&node, &fr) == CO_ERROR_NONE);
    TEST_ASSERT(rpdo_target == 0xABCDU);

    node.nmt_state = CO_NMT_OPERATIONAL;
    TEST_ASSERT(co_send_tpdo(&node, 0U) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == node.tpdo_cfg[0].cob_id.can_id);
    TEST_ASSERT(last_tx(&bus)->len == 2U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0xEFU && last_tx(&bus)->data[1] == 0xBEU);

    return 0;
}

static int test_heartbeat_and_emcy(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 100);

    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(bus.tx_count == 1U); /* boot-up */

    bus.now_ms = 99U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(bus.tx_count == 1U);

    bus.now_ms = 100U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x705U && last_tx(&bus)->data[0] == CO_NMT_PRE_OPERATIONAL);

    TEST_ASSERT(co_fault_raise(&node, CO_FAULT_CAN_TX, 0x8110U, CO_ERROR_REG_COMMUNICATION, 0x2AU, NULL) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x085U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x10U && last_tx(&bus)->data[1] == 0x81U);
    TEST_ASSERT(last_tx(&bus)->data[2] == CO_ERROR_REG_COMMUNICATION);
    TEST_ASSERT(last_tx(&bus)->data[3] == 0x2AU);

    TEST_ASSERT(co_fault_clear(&node, CO_FAULT_CAN_TX, 0x55U, NULL) == CO_ERROR_NONE);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x085U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x00U && last_tx(&bus)->data[1] == 0x00U);

    return 0;
}

static int test_heartbeat_consumer_timeout_recovery_and_jitter(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 0);
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE); /* boot-up */

    uint8_t hb_cons_count = 1U;
    uint32_t hb_cons_entry = ((uint32_t)0x22U << 16U) | 100U;
    TEST_ASSERT(co_od_write(&node, 0x1016, 0x00, &hb_cons_count, sizeof(hb_cons_count)) == 0U);
    TEST_ASSERT(co_od_write(&node, 0x1016, 0x01, (const uint8_t *)&hb_cons_entry, sizeof(hb_cons_entry)) == 0U);
    TEST_ASSERT(node.hb_consumers[0].enabled);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_UNKNOWN);

    bus.now_ms = 250U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_UNKNOWN);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) == 0U);

    bus.now_ms = 10U;
    co_can_frame_t hb = {.cob_id = 0x722U, .len = 1U, .data = {CO_NMT_OPERATIONAL}};
    TEST_ASSERT(co_on_can_rx(&node, &hb) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_ACTIVE);
    TEST_ASSERT(node.hb_runtime[0].remote_state == CO_NMT_OPERATIONAL);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) == 0U);

    bus.now_ms = 110U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_ACTIVE);

    bus.now_ms = 111U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_TIMEOUT);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) != 0U);
    TEST_ASSERT(node.faults[CO_FAULT_HEARTBEAT_CONSUMER].active);

    bus.now_ms = 115U;
    TEST_ASSERT(co_on_can_rx(&node, &hb) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_ACTIVE);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) == 0U);
    TEST_ASSERT(!node.faults[CO_FAULT_HEARTBEAT_CONSUMER].active);

    bus.now_ms = 210U;
    TEST_ASSERT(co_on_can_rx(&node, &hb) == CO_ERROR_NONE);
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_ACTIVE);

    bus.now_ms = 309U;
    TEST_ASSERT(co_on_can_rx(&node, &hb) == CO_ERROR_NONE);
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_ACTIVE);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) == 0U);

    bus.now_ms = 520U;
    TEST_ASSERT(co_process(&node) == CO_ERROR_NONE);
    TEST_ASSERT(node.hb_runtime[0].state == CO_HB_CONSUMER_TIMEOUT);
    TEST_ASSERT((node.error_register & CO_ERROR_REG_COMMUNICATION) != 0U);

    return 0;
}

static uint16_t expected_statusword(cia402_state_t state)
{
    uint16_t sw = 0U;
    const int ready = (state == CIA402_READY_TO_SWITCH_ON || state == CIA402_SWITCHED_ON ||
                       state == CIA402_OPERATION_ENABLED || state == CIA402_QUICK_STOP);
    const int switched_on = (state == CIA402_SWITCHED_ON || state == CIA402_OPERATION_ENABLED || state == CIA402_QUICK_STOP);
    const int op_enabled = (state == CIA402_OPERATION_ENABLED);
    const int fault = (state == CIA402_FAULT || state == CIA402_FAULT_REACTION_ACTIVE);
    const int voltage_enabled = (state == CIA402_READY_TO_SWITCH_ON || state == CIA402_SWITCHED_ON ||
                                 state == CIA402_OPERATION_ENABLED || state == CIA402_QUICK_STOP ||
                                 state == CIA402_FAULT_REACTION_ACTIVE);
    const int quick_stop = (state == CIA402_OPERATION_ENABLED || state == CIA402_SWITCHED_ON ||
                            state == CIA402_READY_TO_SWITCH_ON);
    const int switch_on_disabled = (state == CIA402_SWITCH_ON_DISABLED);

    sw |= (uint16_t)(ready ? (1U << 0) : 0U);
    sw |= (uint16_t)(switched_on ? (1U << 1) : 0U);
    sw |= (uint16_t)(op_enabled ? (1U << 2) : 0U);
    sw |= (uint16_t)(fault ? (1U << 3) : 0U);
    sw |= (uint16_t)(voltage_enabled ? (1U << 4) : 0U);
    sw |= (uint16_t)(quick_stop ? (1U << 5) : 0U);
    sw |= (uint16_t)(switch_on_disabled ? (1U << 6) : 0U);
    sw |= (1U << 9); /* remote defaults true */
    return sw;
}

static int test_cia402_transition_matrix_and_statusword(void)
{
    co_node_t node;
    test_bus_t bus;
    setup_node(&node, &bus, 0x05, 0);
    (void)co_process(&node);

    cia402_axis_t axis;
    cia402_init(&axis);
    cia402_bind_od(&axis, &node);

    TEST_ASSERT(axis.state == CIA402_SWITCH_ON_DISABLED);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    cia402_apply_controlword(&axis, 0x0006U);
    TEST_ASSERT(axis.state == CIA402_READY_TO_SWITCH_ON);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    cia402_apply_controlword(&axis, 0x0007U);
    TEST_ASSERT(axis.state == CIA402_SWITCHED_ON);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    cia402_apply_controlword(&axis, 0x000FU);
    TEST_ASSERT(axis.state == CIA402_OPERATION_ENABLED);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    axis.quick_stop_option_code = 5U;
    cia402_apply_controlword(&axis, 0x0002U);
    TEST_ASSERT(axis.state == CIA402_QUICK_STOP);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    axis.quick_stop_option_code = 2U;
    cia402_apply_controlword(&axis, 0x0000U);
    TEST_ASSERT(axis.state == CIA402_SWITCH_ON_DISABLED);
    TEST_ASSERT(axis.statusword == expected_statusword(axis.state));

    /* Drive into fault via unsupported mode while operation enabled. */
    cia402_apply_controlword(&axis, 0x0006U);
    cia402_apply_controlword(&axis, 0x0007U);
    cia402_apply_controlword(&axis, 0x000FU);
    axis.mode_of_operation = CIA402_MODE_NONE;
    cia402_step(&axis);
    TEST_ASSERT(axis.state == CIA402_FAULT);
    TEST_ASSERT((axis.statusword & (1U << 3)) != 0U);
    TEST_ASSERT(last_tx(&bus)->cob_id == 0x085U);
    TEST_ASSERT(last_tx(&bus)->data[0] == 0x10U && last_tx(&bus)->data[1] == 0xFFU);

    cia402_apply_controlword(&axis, 0x0080U);
    TEST_ASSERT(axis.state == CIA402_SWITCH_ON_DISABLED);
    TEST_ASSERT((axis.statusword & (1U << 3)) == 0U);

    return 0;
}

int main(void)
{
    struct {
        const char *name;
        int (*fn)(void);
    } tests[] = {
        {"nmt_bootup_reset", test_nmt_bootup_reset},
        {"sdo_expedited_segmented_block", test_sdo_expedited_segmented_block},
        {"sdo_multi_channel_concurrency_and_timeout", test_sdo_multi_channel_concurrency_and_timeout},
        {"pdo_remap_and_runtime", test_pdo_remap_and_runtime},
        {"heartbeat_and_emcy", test_heartbeat_and_emcy},
        {"heartbeat_consumer_timeout_recovery_and_jitter", test_heartbeat_consumer_timeout_recovery_and_jitter},
        {"cia402_transition_matrix_and_statusword", test_cia402_transition_matrix_and_statusword},
    };

    int failures = 0;
    for (size_t i = 0; i < ARRAY_LEN(tests); ++i) {
        const int rc = tests[i].fn();
        if (rc == 0) {
            printf("[PASS] %s\n", tests[i].name);
        } else {
            printf("[FAIL] %s\n", tests[i].name);
            failures++;
        }
    }

    if (failures == 0) {
        printf("All host-side tests passed (%zu).\n", ARRAY_LEN(tests));
        return 0;
    }

    printf("Host-side tests failed: %d\n", failures);
    return 1;
}
