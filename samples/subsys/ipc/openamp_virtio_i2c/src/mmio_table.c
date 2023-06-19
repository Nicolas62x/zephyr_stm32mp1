#include <zephyr/kernel.h>
#include "mmio_table.h"

#define __resource Z_GENERIC_SECTION(.mmio_table)

static struct fw_mmio_table __resource mmio_table = {
	.magic = 0x74726976,
	.version = 2,
	.deviceType = 0x22,
};

void mmio_table_get(struct fw_mmio_table **table_ptr)
{
	*table_ptr = &mmio_table;
}
