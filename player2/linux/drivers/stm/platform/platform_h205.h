#ifndef PLATFORM_H205_H
#define PLATFORM_H205_H

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <linux/irq.h>
#endif

static struct resource h264pp_resource_h205[] =
{
	[0] = {
		.start = 0xFE540000, //TODO: get correct value
		.end   = 0xFE5FFFFF, //TODO: get correct value
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = evt2irq(0x14a0), //TODO: get correct value
		.end   = evt2irq(0x14a0), //TODO: get correct value
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device h264pp_device_h205 =
{
	.name          = "h264pp",
	.id            = -1,
	.num_resources = ARRAY_SIZE(h264pp_resource_h205),
	.resource      = &h264pp_resource_h205
};

static struct platform_device *platform_h205[] __initdata =
{
	&h264pp_device_h205,
};

static __init int platform_init_h205(void)
{
//    register_board_drivers();
	return platform_add_devices(platform_h205, sizeof(platform_h205) / sizeof(struct platform_device*));
}

module_init(platform_init_h205);

#endif
