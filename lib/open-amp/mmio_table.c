#include <zephyr/kernel.h>
#include <mmio_table.h>

#define __resource Z_GENERIC_SECTION(.mmio_table)

static struct fw_mmio_table __resource mmio_table = {
	.magic = 0x74726976,
	.version = 2,
	.deviceType = 0x22,
	.vendoId = 42,

	.deviceFeatures = 0,
	.deviceFeaturesSel = 0,
	.driverFeatures = 0,
	.driverFeaturesSel = 0,
	.queueSel = 0,
	.queueNumMax = 0,
	.queueNum = 0,
	.queueReady = 0,
	.queueNotify = 0,
	.interruptStatus = 0,
	.interruptACK = 0,
	.status = 0,
	.queueDesc = 0,
	.queueDriver = 0,
	.queueDevice = 0,
	.configGeneration = 0
};

void mmio_table_get(struct fw_mmio_table **table_ptr)
{
	*table_ptr = &mmio_table;
}
