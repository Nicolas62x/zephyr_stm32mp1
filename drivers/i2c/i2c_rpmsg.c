/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rpmsg_i2c

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <openamp/open_amp.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_target);

#if 0
#undef LOG_ERR
#define LOG_ERR printk
#undef LOG_DBG
#define LOG_DBG printk
#endif

#define RPMSG_I2C_NAME "rpmsg_i2c"
#define I2C_MASTER_READ   0x1

extern struct rpmsg_device *rpdev;

#define RPMSG_I2C_HEADER_SIZE 100

struct rpmsg_i2c_msg {
	uint8_t addr;                      /*!< i2c target addr                */
	uint32_t len;                      /*!< Data Buffer size              */
	uint8_t result;                    /*!< return value for the master   */
	uint8_t buf[RPMSG_I2C_HEADER_SIZE];  /*!< i2c Data buffer             */
} rpmsg_i2c_msg;

#define RMSG_I2C_HEADER_SIZE (sizeof(struct rpmsg_i2c_msg)  - \
				  RPMSG_I2C_HEADER_SIZE)

struct i2c_rpmsg_data {
	sys_slist_t targets;
	struct rpmsg_endpoint ept;          /*!< rpmsg endpoint */
	struct rpmsg_virtio_device *rvdev;  /*!< the rpmsg virtio device  */
};

#define DEV_DATA(dev)							\
	((struct i2c_rpmsg_data * const)(dev)->data)

static struct i2c_target_config *find_address(struct i2c_rpmsg_data *data, uint16_t address)
{
	struct i2c_target_config *cfg = NULL;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE(&data->targets, node) {
		cfg = CONTAINER_OF(node, struct i2c_target_config, node);

		if (cfg->address == address) {
			return cfg;
		}
	}

	return NULL;
}

static int i2c_rpmsg_msg_sendack(struct rpmsg_endpoint *ept,
				 struct rpmsg_i2c_msg *msg, unsigned int ack)
{
	int res;

	msg->len = 0;
	msg->result = ack ? ack : 2;

	res = rpmsg_send(ept, msg, RMSG_I2C_HEADER_SIZE);
	if (res < 0) {
		return res;
	}

	return 0;
}

static int i2c_rpmsg_msg_write(struct rpmsg_endpoint *ept,
			       struct rpmsg_i2c_msg *msg,
			       struct i2c_target_config *cfg)
{
	unsigned int len = 0U;
	uint8_t *buf = msg->buf;
	int ret;

	cfg->callbacks->write_requested(cfg);

	LOG_DBG("%s: write %d bytes for target %#x\n", __func__, msg->len,
		cfg->address);
	len = msg->len;
	while (len) {

		ret = cfg->callbacks->write_received(cfg, *buf);
		if (ret) {
			goto error;
		}
		buf++;
		len--;
	}

	cfg->callbacks->stop(cfg);

	i2c_rpmsg_msg_sendack(ept, msg, 1);
	return 0;
error:
	LOG_DBG("%s: NACK", __func__);

	i2c_rpmsg_msg_sendack(ept, msg, 0);
	return -EIO;
}

static int i2c_rpmsg_msg_read(struct rpmsg_endpoint *ept,
			      struct rpmsg_i2c_msg *msg,
			      struct i2c_target_config *cfg)

{
	unsigned int len = msg->len;
	uint8_t *buf = msg->buf;

	LOG_DBG("%s: read %d bytes for target %#x\n", __func__, msg->len,
		cfg->address);
	if (!msg->len) {
		return 0;
	}

	cfg->callbacks->read_requested(cfg, buf);
	buf++;
	len--;

	while (len) {
		cfg->callbacks->read_processed(cfg, buf);
		buf++;
		len--;
	}

	cfg->callbacks->stop(cfg);

	rpmsg_send(ept, msg, len + RMSG_I2C_HEADER_SIZE);

	return 0;
}

static int i2c_rpmsg_target_event(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	struct i2c_rpmsg_data *d_data = CONTAINER_OF(ept, struct i2c_rpmsg_data,
						     ept);
	struct rpmsg_i2c_msg *msg = data;
	struct i2c_target_config *cfg;
	uint8_t target;
	int ret;

	target = (msg->addr & (~I2C_MASTER_READ)) >> 1;
	cfg = find_address(d_data, target);
	if (!cfg) {
		i2c_rpmsg_msg_sendack(ept, msg, 0);
		return 0;
	}

	if (msg->addr & I2C_MASTER_READ)
		ret = i2c_rpmsg_msg_read(ept, msg, cfg);
	else
		ret = i2c_rpmsg_msg_write(ept, msg, cfg);

	return 0;
}

int i2c_rpmsg_runtime_configure(const struct device *dev, uint32_t config)
{
	return 0;
}

static int i2c_rpmsg_target_register(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);

	if (!config) {
		return -EINVAL;
	}

	/* Check the address is unique */
	if (find_address(data, config->address)) {
		return -EINVAL;
	}

	sys_slist_append(&data->targets, &config->node);

	LOG_DBG("%s: register target for address 0x%x\n", __func__,
		config->address);

	return 0;
}

static int i2c_rpmsg_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);

	if (!config) {
		return -EINVAL;
	}

	if (!sys_slist_find_and_remove(&data->targets, &config->node)) {
		return -EINVAL;
	}

	LOG_DBG("%s: unregister target for address 0x%x\n", __func__,
		config->address);

	return 0;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_rpmsg_runtime_configure,
	.target_register = i2c_rpmsg_target_register,
	.target_unregister = i2c_rpmsg_target_unregister,
};

static int i2c_rpmsg_init(const struct device *dev)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);
	int status;

	sys_slist_init(&data->targets);

	/* Create a endpoint for rmpsg communication */
	status = rpmsg_create_ept(&data->ept, rpdev, RPMSG_I2C_NAME,
				  RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
				  i2c_rpmsg_target_event, NULL);

	if (status < 0) {
		LOG_ERR("Failed to create rpmsg endpoint\n");
		return -EPIPE;
	}

	return 0;
}

#define I2C_DEVICE_INIT_RPMSG(n)				\
	static struct i2c_rpmsg_data i2c_rpmsg_data_ ## n;	\
								\
	I2C_DEVICE_DT_INST_DEFINE(n,				\
			i2c_rpmsg_init, NULL,			\
			&i2c_rpmsg_data_ ## n,			\
			NULL, POST_KERNEL,			\
			CONFIG_I2C_INIT_PRIORITY,		\
			&api_funcs);				\

DT_INST_FOREACH_STATUS_OKAY(I2C_DEVICE_INIT_RPMSG)
