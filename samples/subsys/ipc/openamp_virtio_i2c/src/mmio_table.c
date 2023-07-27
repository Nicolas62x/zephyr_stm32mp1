#include <openamp/virtio_mmio_dev.h>

#define __resource Z_GENERIC_SECTION(.mmio_table)

static struct mmio_table __resource __attribute__((used)) mmio_table = EMPTY_MMIO_TABLE;
