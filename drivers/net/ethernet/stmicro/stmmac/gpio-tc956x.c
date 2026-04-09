// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

/*
 * The Toshiba TC9564 implements a PCIe Gen 3 switch that connects an upstream
 * x4 port to two downstream PCIe x2 ports.  It incorporates an internal
 * endpoint as well, which implements two Synopsys XGMAC Ethernet interfaces.
 *
 * A set of 35 GPIOs are also implemented by an embedded GPIO controller.  A
 * set of three registers controls the first 32 GPIOs (other than 20 and 21,
 * which are reserved).  Three other registers control GPIOs 32 through 36.
 * GPIOs 22-24, 27-28, 31, and 34 are treated as "input only".
 */

#include <linux/auxiliary_bus.h>
#include <linux/dev_printk.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME		"tc956x-gpio"

#define TC956X_GPIO_COUNT	37	/* Number of GPIOs (20-21 reserved) */

/* The GPIO offsets are relative to 0x1200 in TC9564 SFR space */
#define GPIO_IN0_OFFSET		0x00		/* Input value (0-31) */
#define GPIO_EN0_OFFSET		0x08		/* 0: out; 1: in (0-31) */
#define GPIO_OUT0_OFFSET	0x10		/* Output value (0-31) */

#define GPIO_IN1_OFFSET		0x04		/* Input value (32-36) */
#define GPIO_EN1_OFFSET		0x0c		/* 0: out; 1: in (32-36) */
#define GPIO_OUT1_OFFSET	0x14		/* Output value (32-36) */

/*
 * struct tc9564_gpio - Information related to the embedded GPIO controller
 * @chip:		GPIO chip structure
 * @regmap:		MMIO register map for SFR GPIO region access
 * @input_only:		Bitmap indicating which GPIOs are input-only
 */
struct tc9564_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
	DECLARE_BITMAP(input_only, TC956X_GPIO_COUNT);
};

static int tc9564_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct tc9564_gpio *gpio = gpiochip_get_data(gc);
	u32 reg;
	u32 val;

	if (test_bit(offset, gpio->input_only))
		return GPIO_LINE_DIRECTION_IN;

	reg = offset < 32 ? GPIO_EN0_OFFSET : GPIO_EN1_OFFSET;

	regmap_read(gpio->regmap, reg, &val);
	if (val & BIT(offset % 32))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int tc9564_gpio_direction_input(struct gpio_chip *gc,
				       unsigned int offset)
{
	u32 reg = offset < 32 ? GPIO_EN0_OFFSET : GPIO_EN1_OFFSET;
	struct tc9564_gpio *gpio = gpiochip_get_data(gc);
	u32 mask = BIT(offset % 32);

	return regmap_update_bits(gpio->regmap, reg, mask, mask);
}

static int tc9564_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	struct tc9564_gpio *gpio = gpiochip_get_data(gc);
	u32 vreg;
	u32 dreg;
	u32 mask;

	if (test_bit(offset, gpio->input_only))
		return -EINVAL;

	if (offset < 32) {
		vreg = GPIO_OUT0_OFFSET;
		dreg = GPIO_EN0_OFFSET;
	} else {
		vreg = GPIO_OUT1_OFFSET;
		dreg = GPIO_EN1_OFFSET;
	}
	mask = BIT(offset % 32);

	/* Set output value first, then direction */
	regmap_update_bits(gpio->regmap, vreg, mask, value ? mask : 0);

	return regmap_update_bits(gpio->regmap, dreg, mask, 0);
}

static int tc9564_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	u32 reg = offset < 32 ? GPIO_IN0_OFFSET : GPIO_IN1_OFFSET;
	struct tc9564_gpio *gpio = gpiochip_get_data(gc);
	u32 val;

	regmap_read(gpio->regmap, reg, &val);

	return val & BIT(offset % 32) ? 1 : 0;
}

static int tc9564_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	u32 reg = offset < 32 ? GPIO_OUT0_OFFSET : GPIO_OUT1_OFFSET;
	struct tc9564_gpio *gpio = gpiochip_get_data(gc);
	u32 mask = BIT(offset % 32);

	return regmap_update_bits(gpio->regmap, reg, mask, value ? mask : 0);
}

static int tc9564_gpio_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	/* GPIOs 20 and 21 are reserved (and not usable) */
	bitmap_fill(valid_mask, ngpios);
	bitmap_clear(valid_mask, 20, 2);

	return 0;
}

static int tc9564_gpio_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct tc9564_gpio *gpio;
	struct gpio_chip *gc;

	dev_info(dev, " === %s starting\n", __func__);
	if (!dev->platform_data)
		return -EINVAL;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	gpio->regmap = dev->platform_data;

	/* Mark GPIOs 22, 23, 24, 27, 28, 31, and 34 are input only */
	bitmap_set(gpio->input_only, 22, 3);
	bitmap_set(gpio->input_only, 27, 2);
	set_bit(31, gpio->input_only);
	set_bit(34, gpio->input_only);

	gc = &gpio->chip;

	gc->label = "tc9564-gpio";
	gc->parent = dev->parent;

	gc->get_direction = tc9564_gpio_get_direction;
	gc->direction_input = tc9564_gpio_direction_input;
	gc->direction_output = tc9564_gpio_direction_output;
	gc->get = tc9564_gpio_get;
	gc->set = tc9564_gpio_set;
	gc->init_valid_mask = tc9564_gpio_init_valid_mask;

	gc->base = -1;
	gc->ngpio = TC956X_GPIO_COUNT;
	gc->can_sleep = false;

	dev_set_drvdata(dev, gpio);

	dev_info(dev, " === %s finishing\n", __func__);

	return devm_gpiochip_add_data(dev, gc, gpio);
}

static const struct auxiliary_device_id tc964_gpio_ids[] = {
	{ .name = "tc956x_pci." DRIVER_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, tc964_gpio_ids);

static int tc9564_gpio_suspend(struct device *dev)
{
	dev_info(dev, " === %s\n", __func__);
	return 0;
}

static int tc9564_gpio_resume(struct device *dev)
{
	dev_info(dev, " === %s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(tc9564_gpio_pm_ops, tc9564_gpio_suspend,
			 tc9564_gpio_resume);

static struct auxiliary_driver tc9564_gpio_driver = {
	.name		= DRIVER_NAME,
	.probe          = tc9564_gpio_probe,
	.id_table       = tc964_gpio_ids,
	.driver = {
		.name		= DRIVER_NAME,
		.pm		= &tc9564_gpio_pm_ops,
		/* .owner	= THIS_MODULE, */
		/* .probe_type	= PROBE_PREFER_ASYNCHRONOUS, */
	},
};
module_auxiliary_driver(tc9564_gpio_driver);

MODULE_DESCRIPTION("Toshiba TC9564 PCIe GPIO Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("auxiliary:" DRIVER_NAME);
