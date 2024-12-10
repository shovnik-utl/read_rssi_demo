#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>

/* Advertising. */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)), /* Advertise a random service for now. */
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Connection/Disconnection. */
static bool ble_connected = false;
static struct bt_conn *default_conn;
static uint16_t default_conn_handle;

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;

	if (err) {
		printk("Connection failed, err 0x%02x.\n", err);
	}
    else
    {
		default_conn = bt_conn_ref(conn);
		ret = bt_hci_get_conn_handle(default_conn,
					     &default_conn_handle);
		if (ret) {
			printk("No connection handle (err %d).\n", ret);
		}
        else {
			bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
			printk("Connected via connection (%d) at %s.\n", default_conn_handle, addr);
		}
	}
    ble_connected = true;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x.\n", reason);
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
    ble_connected = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* Initialization. */
static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d).\n", err);
		return;
	}
	printk("Bluetooth initialized.\n");

	/* Start advertising. */        
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d).\n", err);
		return;
	}
}

/* Read RSSI value by querying HCI. */
static void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer.\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		printk("Read RSSI err: %d.\n", err);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}

int main(void)
{
    int err;
    int8_t rssi = 0;
    int16_t rssi_avg = 0;
	default_conn = NULL;
    int8_t sample[6] = { 0 };
    const int nreadings = sizeof(sample);
    const k_timeout_t rssi_sample_interval = K_MSEC(50); /* Adjust based on experimental results. */
    const k_timeout_t rssi_burst_interval = K_SECONDS(1);  /* Adjust based on power requirements. */

	printk("Starting RSSI Test Demo (%d).\n", nreadings);

	/* Initialize the Bluetooth Subsystem. */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d).\n", err);
	}

    /* Read and log the RSSI value periodically in a loop while connection persists. */
    for (;;)
    {
        if (ble_connected) {

            /* A single raw reading for the RSSI is often inaccurate. We can try
               to address this by reading in bursts (ie taking a sample of N
               readings) and then applying statistical treatment to the sample.
               Here that treatment is just an arithmetic mean.   
            */
            rssi_avg = 0;
            for (int i = 0; i < nreadings; i++) {
                read_conn_rssi(default_conn_handle, &rssi);
                sample[i] = rssi; /* Save individual readings for display later. */
                rssi_avg += rssi;

                /* We need to introduce a little bit of delay here to allow for the
                   RSSI query to return a value that is (even if slightly)
                   "different" from the last measured value. If we sample
                   in quick succession without any delay what ends up happening is
                   the same value is returned on every query meaning their average
                   is also the same and it does not have the effect of smoothing
                   out erroneous values - defeating the purpose of wanting to
                   take an average (or applying any other statistical method to
                   the data) in the first place! */
                k_sleep(rssi_sample_interval);
            }
            rssi_avg /= nreadings;
            printk("Connected, RSSI (avg) = %d dBm ", rssi);

            /* Display the readings obtained in the burst. */
            printk("[");
            for (int i = 0; i < nreadings; i++) {
                printk("%d ", sample[i]);
            }
            printk("]\n");
        }
        k_sleep(rssi_burst_interval);
    }

    return 0;
}
