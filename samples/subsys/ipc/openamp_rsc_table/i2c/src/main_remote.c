/*
 * Copyright (c) 2019, STMICROLECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/kernel.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>
#include <mmio_table.h>

/* Display / Framebuffer */
#include <zephyr/display/cfb.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define SHM_DEVICE_NAME	"shm"
#define I2C_REMOTE_DISP_ADDR 0x3D

/* constant derivated from linker symbols */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (4096)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));
struct rpmsg_device *rpdev;

static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(arduino_i2c));

#ifdef CONFIG_SSD1306
static const struct device *const dev_display = DEVICE_DT_GET(DT_NODELABEL(ssd1306_zephyr));
#endif

#ifdef CONFIG_HTS221
static const struct device *const hts221 = DEVICE_DT_GET(DT_NODELABEL(hts221_x_nucleo_iks01a2));
#endif

#ifdef CONFIG_LPS22HB
static const struct device *const lps22hb =
			DEVICE_DT_GET(DT_NODELABEL(lps22hb_press_x_nucleo_iks01a2));
#endif

static uint16_t display_rows;
static uint8_t display_ppt;
static uint8_t display_font_width;
static uint8_t display_font_height;
static bool display_available;
static bool first_write;
static uint8_t reg_addr;

static struct sensor_value temp1, temp2, hum, press;

static bool hts221_available = true;
static bool lps22hb_available = true;

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		{.virt = NULL}, /* shared memory */
		{.virt = NULL}, /* rsc_table memory */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct metal_io_region *shm_io;
static struct rpmsg_virtio_shm_pool shpool;

static struct metal_io_region *rsc_io;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;

static K_SEM_DEFINE(data_sem, 0, 1);

#define TEST_DATA_SIZE	20

/**** MMIO ****/

/* This marks a buffer as continuing via the next field. */
#define VIRTQ_DESC_F_NEXT	0
/* This marks a buffer as device write-only (otherwise device read-only). */
#define VIRTQ_DESC_F_WRITE	1

#define BitSet(BIT, VALUE) (VALUE & (1 << BIT))

uint64_t features = 1 | ((uint64_t)1 << 32) | ((uint64_t)1 << 33);
uint64_t driver_features;

K_SEM_DEFINE(my_sem, 0, 1);

void mmio_interrupt(void)
{
	static struct fw_mmio_table *mmio;
	static uint32_t last_status;

	if (!mmio)
		mmio_table_get(&mmio);

	if (last_status != mmio->status) {
		printf("MMIO status: %d\n", mmio->status);
		last_status = mmio->status;
	}

	if (mmio->status == 0) {
		driver_features = 0;
		mmio->queueReady = 0;
	} else if (mmio->status == 3) {
		mmio->deviceFeatures = (uint32_t)(features >> (mmio->deviceFeaturesSel * 32));
		driver_features |=
			((uint64_t)mmio->driverFeatures << (mmio->driverFeaturesSel * 32));
	} else if (mmio->status == 11) {
		if (driver_features != features) {
			printf("DriverFeatures does not match our features :(\n");
			mmio->status &= ~(uint32_t)8;
			return;
		}

		mmio->queueNumMax = 16;

		if (!device_is_ready(i2c_dev)) {
			printf("I2C device is not ready\n");
			return;
		}
	} else if (mmio->status == 15) {
		k_sem_give(&my_sem);
	}
}


/**** ENDMMIO ****/

void stop_display(void)
{
	display_blanking_on(dev_display);
}

static void platform_ipm_callback(const struct device *dev,
				  void *context, uint32_t id, volatile void *data)
{
	if (id == 2) {
		/* receive shutdown */
		stop_display();
		return;
	}

	if (id == 5) {
		mmio_interrupt();
		return;
	}

	LOG_ERR("IPM IRQ\n");
	k_sem_give(&data_sem);
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	unsigned int i = 1000;

	while (i) {
		if (k_sem_take(&data_sem, K_NO_WAIT) == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		} else {
			k_sleep(K_MSEC(1));
			i--;
		}
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	ipm_send(ipm_handle, 0, id, NULL, 0);
	LOG_DBG("%s: msg sent\n", __func__);

	return 0;
}

void init_sensors(void)
{
	/* search in devicetree if sensors are referenced */

#if CONFIG_HTS221
	if (!device_is_ready(hts221)) {
		LOG_ERR("Could not get HTS221 device\n");
		hts221_available = false;
	}
#endif
#if CONFIG_LPS22HB
	if (!device_is_ready(lps22hb)) {
		LOG_ERR("Could not get LPS22HB device\n");
		lps22hb_available = false;
	}
#endif

	/* set LSM6DSL accel/gyro sampling frequency to 104 Hz */
	struct sensor_value odr_attr;

	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

}

int init_display(void)
{
	int ret;

	/* init display */
	if (!device_is_ready(dev_display)) {
		LOG_ERR("Display Device not found\n");
		return -ENODEV;
	} else {
		display_available = true;

		ret = display_set_pixel_format(dev_display, PIXEL_FORMAT_MONO10);
		if (ret) {
			LOG_ERR("Failed to set format PIXEL_FORMAT_MONO10\n");
			return ret;
		}

		ret = cfb_framebuffer_init(dev_display);
		if (ret) {
			LOG_ERR("Framebuffer initialization failed!\n");
			return ret;
		}
		cfb_framebuffer_clear(dev_display, true);

		display_blanking_off(dev_display);

		display_rows = cfb_get_display_parameter(dev_display,
							 CFB_DISPLAY_ROWS);
		display_ppt = cfb_get_display_parameter(dev_display,
							CFB_DISPLAY_PPT);
		for (int idx = 0; idx < 42; idx++) {
			if (cfb_get_font_size(dev_display, idx,
					      &display_font_width,
					      &display_font_height)) {
				break;
			}
			cfb_framebuffer_set_font(dev_display, idx);
			LOG_DBG("idx: %d font width %d, font height %d\n",
				idx, display_font_width, display_font_height);
		}
		/* for idx: 0 font width 10, font height 16*/
		cfb_framebuffer_set_font(dev_display, 0);
		cfb_get_font_size(dev_display, 0, &display_font_width,
				  &display_font_height);

		LOG_DBG("x_res %d, y_res %d, ppt %d, rows %d, cols %d\n",
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_WIDTH),
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_HEIGH),
			display_ppt,
			display_rows,
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_COLS));
	}

	return 0;
}

int init_remote_display(void)
{
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("failed to bind remote display\n");
		return -ENODEV;
	}

	return 0;
}

int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_device *device;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d\n", status);
		return -1;
	}

	status = metal_register_generic_device(&shm_device);
	if (status) {
		LOG_ERR("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (status) {
		LOG_ERR("metal_device_open failed: %d\n", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		LOG_ERR("Failed to get shm_io region\n");
		return -1;
	}

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		LOG_ERR("Failed to get rsc_io region\n");
		return -1;
	}

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready\n");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev\r\n");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0\r\n");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1\r\n");
		goto failed;
	}

	rpmsg_virtio_init_shm_pool(&shpool, NULL, SHM_SIZE);
	ret =  rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, &shpool);

	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev\r\n");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

static int rpmsg_target_write_requested(struct i2c_target_config *config)
{

	LOG_DBG("%s\n", __func__);
	first_write = true;

	return 0;
}

static int rpmsg_target_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	LOG_DBG("%s: val=0x%x\n", __func__, *val);
	reg_addr = *val;
	/* Increment will be done in the read_processed callback */

	return 0;
}

static int rpmsg_target_write_received(struct i2c_target_config *config, uint8_t val)
{
	int ret;

	if (first_write) {
		reg_addr = val;
		first_write = false;
	} else {
		LOG_DBG("%s: write reg=0x%x val=0x%x\n", __func__, reg_addr,
			val);
		ret = i2c_reg_write_byte(i2c_dev, I2C_REMOTE_DISP_ADDR,
					 reg_addr, val);
		if (ret < 0)
			LOG_DBG("%s: failed to write reg=0x%x val=0x%x\n",
				__func__, reg_addr, val);
	}

	return 0;
}

static int rpmsg_target_read_processed(struct i2c_target_config *config, uint8_t *val)
{

	LOG_DBG("%s: reg=0x%x val=0x%x\n", __func__, reg_addr, *val);

	i2c_reg_read_byte(i2c_dev, I2C_REMOTE_DISP_ADDR, reg_addr++, val);

	/* Increment will be done in the next read_processed callback
	 * In case of STOP, the byte won't be taken in account
	 */

	return 0;
}

static int rpmsg_target_stop(struct i2c_target_config *config)
{

	LOG_DBG("%s: stop\n", __func__);
	first_write = true;

	return 0;
}

static const struct i2c_target_callbacks i2c_rpmsg_callbacks = {
	.write_requested = rpmsg_target_write_requested,
	.read_requested = rpmsg_target_read_requested,
	.write_received = rpmsg_target_write_received,
	.read_processed = rpmsg_target_read_processed,
	.stop = rpmsg_target_stop,
};

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	unsigned char *msg;
	const struct device *i2c_rpmsg_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_rpmsg));
	struct i2c_target_config config;
	int len;
	int ret = 0;
	char message[128];
	char display_buf[16];
	uint8_t line = 0;

	LOG_INF("\r\nOpenAMP[remote] I2C demo started\n");
	if (!i2c_rpmsg_dev)
		LOG_ERR(" failed to get rpmsg I2C dev");

	config.address = I2C_REMOTE_DISP_ADDR;
	config.callbacks = &i2c_rpmsg_callbacks;

	/* Attach application target */
	ret = i2c_target_register(i2c_rpmsg_dev, &config);
	if (ret) {
		LOG_ERR("Failed to register I2C SLAVE %#x\n", config.address);
		goto task_end;
	}

	init_remote_display();
	init_sensors();
	init_display();

	if (display_available) {
		line = 0;
		LOG_ERR("--init display screen --\n");

		cfb_framebuffer_clear(dev_display, false);
		cfb_print(dev_display, "Zephyr", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "   Waiting ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "     TTY   ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "    data   ", 0,
			  line++ * display_font_height);
		cfb_framebuffer_finalize(dev_display);
	}

	while (1) {
		unsigned int line = 0;

		if (display_available)
			cfb_framebuffer_clear(dev_display, false);

		/* Get sensor samples */
		if (hts221_available && sensor_sample_fetch(hts221) < 0)
			printf("HTS221 Sensor sample update error\n");
		if (lps22hb_available && sensor_sample_fetch(lps22hb) < 0)
			printf("LPS22HB Sensor sample update error\n");

		/* Get sensor data */
		if (hts221_available) {
			sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP,
					   &temp1);
			sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
		}
		if (lps22hb_available) {
			sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &press);
			sensor_channel_get(lps22hb, SENSOR_CHAN_AMBIENT_TEMP,
					   &temp2);
		}

		receive_message(&msg, &len);
		/* Display sensor data */

		/* Erase previous */
		if (display_available) {
			cfb_print(dev_display, "Zephyr", 0,
				  line++ * display_font_height);

			if (hts221_available) {
				/* temperature */
				snprintf(display_buf, sizeof(display_buf), "T: %d.%1d*C",
					(int)sensor_value_to_double(&temp1),
					(int)(sensor_value_to_double(&temp1) * 10) % 10);

				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);

				/* humidity */
				snprintf(display_buf, sizeof(display_buf), "H: %d.%1d%%",
					 (int)sensor_value_to_double(&hum),
					 (int)(sensor_value_to_double(&hum) * 10) % 10);

				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);
			} else {
				cfb_print(dev_display, "T: N/A", 0, line++ * display_font_height);
				cfb_print(dev_display, "H: N/A", 0, line++ * display_font_height);
			}

			if (lps22hb_available) {
				/* pressure */
				/* lps22hb temperature */
				len = snprintf(message, sizeof(message),
						"LPS22HB: Temperature: %.1f C\n",
						sensor_value_to_double(&temp2));

				snprintf(display_buf, sizeof(display_buf), "P: %d.%02dkpa",
					(int)(sensor_value_to_double(&press) * 10.0),
					(int)(sensor_value_to_double(&press) * 1000) % 100);

				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);
			} else {
				cfb_print(dev_display, "P: N/A", 0, line++ * display_font_height);
			}

			cfb_framebuffer_finalize(dev_display);
		}

		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_DBG("sensor task ended\n");
}

static int openamp_init(const struct device *dev)
{
	int ret;

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform\n");
		goto error_case;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device\n");
		goto error_case;
	}

	LOG_DBG("Init OpenAMP rpdev = %p\n", rpdev);
	return 0;

error_case:
	cleanup_system();

	return 1;
}

void main(void)
{

	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

}

SYS_INIT(openamp_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
