#include <zephyr/input/input.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/sensor.h>
// #include <zephyr/logging/log.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101
static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(5);
#define MOUSE_BTN_LEFT 0
#define MOUSE_BTN_RIGHT 1
#define MOUSE_BTN_WHEEL 2
#define MOUSE_BTN_BACK 3
#define MOUSE_BTN_FORWARD 4

enum mouse_report_idx {
    MOUSE_BTN_REPORT_IDX = 0,
    MOUSE_X_REPORT_IDX = 1,
    MOUSE_Y_REPORT_IDX = 2,
    MOUSE_WHEEL_REPORT_IDX = 3,
    MOUSE_REPORT_COUNT = 4,
};

/* Number of pixels by which the cursor is moved when a button is pushed. */
#define MOVEMENT_SPEED 5
/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT 3
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN 3
/* Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN 3
/* Length of Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN 1
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX 0
/* Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_INDEX 1
/* Index of Mouse Input Report containing media player data. */
#define INPUT_REP_MPLAYER_INDEX 2
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID 1
/* Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID 2
/* Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID 3

/* HIDs queue size. */
#define HIDS_QUEUE_SIZE 10

const struct device *const qdec_dev = DEVICE_DT_GET(DT_ALIAS(qdec0));
static uint8_t report[INPUT_REP_BUTTONS_LEN] = {0};
static bool report_updated = false;
static bool usb_mode = true;
static enum usb_dc_status_code usb_status;

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    usb_status = status;
}

static ALWAYS_INLINE void rwup_if_suspended(void)
{
    if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
        if (usb_status == USB_DC_SUSPEND) {
            usb_wakeup_request();
            return;
        }
    }
}

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
            INPUT_REP_BUTTONS_LEN,
            INPUT_REP_MOVEMENT_LEN,
            INPUT_REP_MEDIA_PLAYER_LEN);

static struct k_work hids_work;

struct mouse_pos {
    int16_t x_val;
    int16_t y_val;
};

/* Mouse movement queue. */
K_MSGQ_DEFINE(hids_queue,
              sizeof(struct mouse_pos),
              HIDS_QUEUE_SIZE,
              4);

#if CONFIG_BT_DIRECTED_ADVERTISING
/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue,
              sizeof(bt_addr_le_t),
              CONFIG_BT_MAX_PAIRED,
              4);
#endif

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode {
    struct bt_conn *conn;
    bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static struct k_work adv_work;

static struct k_work pairing_work;
struct pairing_data_mitm {
    struct bt_conn *conn;
    unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue,
              sizeof(struct pairing_data_mitm),
              CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
              4);

#if CONFIG_BT_DIRECTED_ADVERTISING
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
    int err;

    /* Filter already connected peers. */
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn) {
            const bt_addr_le_t *dst = bt_conn_get_dst(conn_mode[i].conn);

            if (!bt_addr_le_cmp(&info->addr, dst)) {
                return;
            }
        }
    }

    err = k_msgq_put(&bonds_queue, (void *)&info->addr, K_NO_WAIT);
    if (err) {
        printk("No space in the queue for the bond.\n");
    }
}
#endif

static void advertising_continue(void)
{
    struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
    bt_addr_le_t addr;

    if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
        char addr_buf[BT_ADDR_LE_STR_LEN];

        adv_param = *BT_LE_ADV_CONN_DIR(&addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

        int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

        if (err) {
            printk("Directed advertising failed to start\n");
            return;
        }

        bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
        printk("Direct advertising to %s started\n", addr_buf);
    } else
#endif
    {
        int err;

        adv_param = *BT_LE_ADV_CONN;
        adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            return;
        }

        printk("Regular advertising started\n");
    }
}

static void advertising_start(void)
{
#if CONFIG_BT_DIRECTED_ADVERTISING
    k_msgq_purge(&bonds_queue);
    bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

    k_work_submit(&adv_work);
}

static void advertising_process(struct k_work *work)
{
    advertising_continue();
}

static void pairing_process(struct k_work *work)
{
    int err;
    struct pairing_data_mitm pairing_data;

    char addr[BT_ADDR_LE_STR_LEN];

    err = k_msgq_peek(&mitm_queue, &pairing_data);
    if (err) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
                      addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
    printk("Press Button 1 to confirm, Button 2 to reject.\n");
}

static void insert_conn_object(struct bt_conn *conn)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            conn_mode[i].conn = conn;
            conn_mode[i].in_boot_mode = false;

            return;
        }
    }

    printk("Connection object could not be inserted %p\n", conn);
}

static bool is_conn_slot_free(void)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            return true;
        }
    }

    return false;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        if (err == BT_HCI_ERR_ADV_TIMEOUT) {
            printk("Direct advertising to %s timed out\n", addr);
            k_work_submit(&adv_work);
        } else {
            printk("Failed to connect to %s (%u)\n", addr, err);
        }
        return;
    }

    printk("Connected %s\n", addr);

    err = bt_hids_connected(&hids_obj, conn);

    if (err) {
        printk("Failed to notify HID service about connection\n");
        return;
    }

    insert_conn_object(conn);

    if (is_conn_slot_free()) {
        advertising_start();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected from %s (reason %u)\n", addr, reason);

    err = bt_hids_disconnected(&hids_obj, conn);

    if (err) {
        printk("Failed to notify HID service about disconnection\n");
    }

    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            conn_mode[i].conn = NULL;
            break;
        }
    }

    advertising_start();
}

#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        printk("Security failed: %s level %u err %d\n", addr, level,
               err);
    }
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
    .security_changed = security_changed,
#endif
};

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
                                struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    size_t i;

    for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            break;
        }
    }

    if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
        printk("Boot mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = true;
        break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
        printk("Report mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = false;
        break;

    default:
        break;
    }
}

static void hid_init(void)
{
    int err;
    struct bt_hids_init_param hids_init_param = {0};
    struct bt_hids_inp_rep *hids_inp_rep;
    static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_BUTTONS_LEN, 8)] = {0};

    static const uint8_t report_map[] = {
        0x05, 0x01, /* Usage Page (Generic Desktop) */
        0x09, 0x02, /* Usage (Mouse) */

        0xA1, 0x01, /* Collection (Application) */

        /* Report ID 1: Mouse buttons + scroll/pan */
        0x85, 0x01,          /* Report Id 1 */
        0x09, 0x01,          /* Usage (Pointer) */
        0xA1, 0x00,          /* Collection (Physical) */
        0x95, 0x05,          /* Report Count (3) */
        0x75, 0x01,          /* Report Size (1) */
        0x05, 0x09,          /* Usage Page (Buttons) */
        0x19, 0x01,          /* Usage Minimum (01) */
        0x29, 0x05,          /* Usage Maximum (05) */
        0x15, 0x00,          /* Logical Minimum (0) */
        0x25, 0x01,          /* Logical Maximum (1) */
        0x81, 0x02,          /* Input (Data, Variable, Absolute) */
        0x95, 0x01,          /* Report Count (1) */
        0x75, 0x03,          /* Report Size (3) */
        0x81, 0x01,          /* Input (Constant) for padding */
        0x75, 0x08,          /* Report Size (8) */
        0x95, 0x01,          /* Report Count (1) */
        0x05, 0x01,          /* Usage Page (Generic Desktop) */
        0x09, 0x38,          /* Usage (Wheel) */
        0x15, 0x81,          /* Logical Minimum (-127) */
        0x25, 0x7F,          /* Logical Maximum (127) */
        0x81, 0x06,          /* Input (Data, Variable, Relative) */
        0x05, 0x0C,          /* Usage Page (Consumer) */
        0x0A, 0x38, 0x02, /* Usage (AC Pan) */
        0x95, 0x01,          /* Report Count (1) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0xC0,              /* End Collection (Physical) */

        /* Report ID 2: Mouse motion */
        0x85, 0x02,          /* Report Id 2 */
        0x09, 0x01,          /* Usage (Pointer) */
        0xA1, 0x00,          /* Collection (Physical) */
        0x75, 0x0C,          /* Report Size (12) */
        0x95, 0x02,          /* Report Count (2) */
        0x05, 0x01,          /* Usage Page (Generic Desktop) */
        0x09, 0x30,          /* Usage (X) */
        0x09, 0x31,          /* Usage (Y) */
        0x16, 0x01, 0xF8, /* Logical maximum (2047) */
        0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
        0x81, 0x06,          /* Input (Data, Variable, Relative) */
        0xC0,              /* End Collection (Physical) */
        0xC0,              /* End Collection (Application) */

        /* Report ID 3: Advanced buttons */
        0x05, 0x0C, /* Usage Page (Consumer) */
        0x09, 0x01, /* Usage (Consumer Control) */
        0xA1, 0x01, /* Collection (Application) */
        0x85, 0x03, /* Report Id (3) */
        0x15, 0x00, /* Logical minimum (0) */
        0x25, 0x01, /* Logical maximum (1) */
        0x75, 0x01, /* Report Size (1) */
        0x95, 0x01, /* Report Count (1) */

        0x09, 0xCD,          /* Usage (Play/Pause) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xB5,          /* Usage (Scan Next Track) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xB6,          /* Usage (Scan Previous Track) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */

        0x09, 0xEA,          /* Usage (Volume Down) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x09, 0xE9,          /* Usage (Volume Up) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x25, 0x02, /* Usage (AC Forward) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0x0A, 0x24, 0x02, /* Usage (AC Back) */
        0x81, 0x06,          /* Input (Data,Value,Relative,Bit Field) */
        0xC0              /* End Collection */
    };

    hids_init_param.rep_map.data = report_map;
    hids_init_param.rep_map.size = sizeof(report_map);

    hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init_param.info.b_country_code = 0x00;
    hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE |
                                  BT_HIDS_NORMALLY_CONNECTABLE);

    hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
    hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
    hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_inp_rep++;
    hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
    hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
    hids_inp_rep->rep_mask = mouse_movement_mask;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_inp_rep++;
    hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
    hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_init_param.is_mouse = true;
    hids_init_param.pm_evt_handler = hids_pm_evt_handler;

    err = bt_hids_init(&hids_obj, &hids_init_param);
    __ASSERT(err == 0, "HIDS initialization failed\n");
}

static void num_comp_reply(bool accept) {
    struct pairing_data_mitm pairing_data;
    struct bt_conn *conn;

    if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
        return;
    }

    conn = pairing_data.conn;

    if (accept) {
        bt_conn_auth_passkey_confirm(conn);
        printk("Numeric Match, conn %p\n", conn);
    } else {
        bt_conn_auth_cancel(conn);
        printk("Numeric Reject, conn %p\n", conn);
    }

    bt_conn_unref(pairing_data.conn);

    if (k_msgq_num_used_get(&mitm_queue)) {
        k_work_submit(&pairing_work);
    }
}

#if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    int err;

    struct pairing_data_mitm pairing_data;

    pairing_data.conn = bt_conn_ref(conn);
    pairing_data.passkey = passkey;

    err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
    if (err) {
        printk("Pairing queue is full. Purge previous data.\n");
    }

    /* In the case of multiple pairing requests, trigger
     * pairing confirmation which needed user interaction only
     * once to avoid display information about all devices at
     * the same time. Passkey confirmation for next devices will
     * be proccess from queue after handling the earlier ones.
     */
    if (k_msgq_num_used_get(&mitm_queue) == 1) {
        k_work_submit(&pairing_work);
    }
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct pairing_data_mitm pairing_data;

    if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
        return;
    }

    if (pairing_data.conn == conn) {
        bt_conn_unref(pairing_data.conn);
        k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */

void ble_init() {
    int err;

    printk("Starting Bluetooth Peripheral HIDS mouse example\n");

    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err) {
            printk("Failed to register authorization callbacks.\n");
            return;
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err) {
            printk("Failed to register authorization info callbacks.\n");
            return;
        }
    }

    /* DIS initialized at system boot with SYS_INIT macro. */
    hid_init();

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    k_work_init(&adv_work, advertising_process);
    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        k_work_init(&pairing_work, pairing_process);
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    advertising_start();
}

void switch_mode() {
    usb_mode = !usb_mode;
    if (usb_mode) {
        usb_enable(status_cb);
    } else {
        ble_init();
    }
}

static void input_cb(struct input_event *evt)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            continue;
        }

        if (conn_mode[i].in_boot_mode) {
        } else {
            switch (evt->code) {
            case INPUT_KEY_0:
                WRITE_BIT(report[0], 0, evt->value);
                report_updated = true;
                break;
            case INPUT_KEY_1:
                switch_mode();
                break;
            case INPUT_KEY_2:
                num_comp_reply(true); // Confirm pairing
                break;
            case INPUT_KEY_3:
                num_comp_reply(false); // Reject pairing
                break;
            case INPUT_KEY_4:
                switch_mode();
                break;
            default:
                return;
            }
        }
    }

    if (usb_mode) {
        switch (evt->code) {
        case INPUT_KEY_0:
            WRITE_BIT(report[0], 0, evt->value);
            report_updated = true;
            break;
        case INPUT_KEY_1:
            if (evt->value == 1) {
                switch_mode();
            }
            break;
        case INPUT_KEY_2:
            num_comp_reply(true); // Confirm pairing
            break;
        case INPUT_KEY_3:
            num_comp_reply(false); // Reject pairing
            break;
        case INPUT_KEY_4:
            switch_mode();
            break;
        default:
            return;
        }
    }
}

static void qdec_fetch_and_handle(void)
{
    struct sensor_value val;
    int rc;

    rc = sensor_sample_fetch(qdec_dev);
    if (rc != 0) {
        printk("Failed to fetch QDEC sample (%d)\n", rc);
        return;
    }

    rc = sensor_channel_get(qdec_dev, 35, &val);
    if (rc != 0) {
        printk("Failed to get QDEC data (%d)\n", rc);
        return;
    }

    if (val.val1 > 0) {
        report[1] = 5;
        report_updated = true;
    } else if (val.val1 < 0) {
        report[1] = -5;
        report_updated = true;
    }
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

int main(void)
{
    const struct device *hid_dev;
    int ret;

    if (!device_is_ready(qdec_dev)) {
        printk("Qdec device is not ready\n");
        return 0;
    }

    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        printk("Cannot get USB HID Device\n");
        return 0;
    }

    usb_hid_register_device(hid_dev,
                            hid_report_desc, sizeof(hid_report_desc),
                            NULL);

    usb_hid_init(hid_dev);

    ret = usb_enable(status_cb);
    if (ret != 0) {
        printk("Failed to enable USB");
        return 0;
    }

    while (1) {
        qdec_fetch_and_handle();
        if (report_updated) {
            if (usb_mode) {
                ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
                if (ret) {
                    printk("HID write error, %d", ret);
                }

                report[0] = 0U;
                report[1] = 0U;
                report[2] = 0U;
            } else {
                for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
                    if (conn_mode[i].conn) {
                        bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                                            INPUT_REP_BUTTONS_INDEX,
                                            report, sizeof(report), NULL);
                    }
                }
                report[0] = 0U;
                report[1] = 0U;
                report[2] = 0U;
            }
            report_updated = false;
        }
        k_msleep(10);
    }
}
