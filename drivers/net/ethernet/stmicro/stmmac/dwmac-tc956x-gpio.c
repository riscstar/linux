// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/string_choices.h>

#define TC956X_GPIO_COUNT	37	/* Number of GPIOs (20-21 reserved) */

/* The GPIO offsets are relative to 0x1200 in SFR space */
#define GPIO_IN0_OFFSET		0x00		/* Input value (0-31) */
#define GPIO_IN1_OFFSET		0x04		/* Input value (32-36) */
#define GPIO_EN0_OFFSET		0x08		/* 0: out; 1: in (0-31) */
#define GPIO_EN1_OFFSET		0x0c		/* 0: out; 1: in (32-36) */
#define GPIO_OUT0_OFFSET	0x10		/* Output value (0-31) */
#define GPIO_OUT1_OFFSET	0x14		/* Output value (32-36) */

/*
 * struct tc956x_gpio - Information related to the embedded GPIO controller
 * @chip:		GPIO chip structure
 * @regmap:		MMIO register map for SFR GPIO region access
 * @input_only:		Bitmap indicating which GPIOs are input-only
 */
struct tc956x_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
	DECLARE_BITMAP(input_only, TC956X_GPIO_COUNT);
};

static int tc956x_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	printk(" === %s %u\n", __func__, offset);
	/* Probably not needed */
	return 0;
}

static void tc956x_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
	/* Probably not needed */
	printk(" === %s %u\n", __func__, offset);
}


static int tc956x_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct tc956x_gpio *gpio = gpiochip_get_data(gc);
	u32 reg;
	u32 val;

	printk(" === %s %u\n", __func__, offset);

	if (test_bit(offset, gpio->input_only))
		return GPIO_LINE_DIRECTION_IN;

	reg = offset < 32 ? GPIO_EN0_OFFSET : GPIO_EN1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);
	if (val & BIT(offset % 32))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int tc956x_gpio_direction_input(struct gpio_chip *gc,
				       unsigned int offset)
{
	struct tc956x_gpio *gpio = gpiochip_get_data(gc);
	u32 reg;
	u32 val;
	u32 new;

	printk(" === %s %u\n", __func__, offset);

	reg = offset < 32 ? GPIO_EN0_OFFSET : GPIO_EN1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);
	new = val | BIT(offset % 32);		/* Set line for input */
	if (new != val)
		regmap_write(gpio->regmap, reg, val);

	return 0;
}

static int tc956x_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	struct tc956x_gpio *gpio = gpiochip_get_data(gc);
	u32 mask = BIT(offset % 32);
	u32 reg;
	u32 val;
	u32 new;

	printk(" === %s %u %s\n", __func__, offset, str_on_off(!!value));

	if (test_bit(offset, gpio->input_only))
		return -EINVAL;

	reg = offset < 32 ? GPIO_OUT0_OFFSET : GPIO_OUT1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);
	if (value)
		new = val | mask;
	else
		new = val & ~mask;
	if (new != val)
		regmap_write(gpio->regmap, reg, val);

	reg = offset < 32 ? GPIO_EN0_OFFSET : GPIO_EN1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);
	new = val & ~mask;			/* Set line for output */
	if (new != val)
		regmap_write(gpio->regmap, reg, val);

	return 0;
}

static int tc956x_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct tc956x_gpio *gpio = gpiochip_get_data(gc);
	u32 reg;
	u32 val;

	printk(" === %s %u\n", __func__, offset);

	reg = offset < 32 ? GPIO_IN0_OFFSET : GPIO_IN1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);

	return val & BIT(offset % 32) ? 1 : 0;
}

static int tc956x_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct tc956x_gpio *gpio = gpiochip_get_data(gc);
	u32 mask = BIT(offset % 32);
	u32 reg;
	u32 val;
	u32 new;

	printk(" === %s %u %s\n", __func__, offset, str_on_off(!!value));

	reg = offset < 32 ? GPIO_OUT0_OFFSET : GPIO_OUT1_OFFSET;
	regmap_read(gpio->regmap, reg, &val);
	if (value)
		new = val | mask;
	else
		new = val & ~mask;
	if (new != val)
		regmap_write(gpio->regmap, reg, val);

	return 0;
}

static int tc956x_gpio_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	printk(" === %s\n", __func__);

	/* GPIOs 20 and 21 are reserved (and not usable) */
	bitmap_fill(valid_mask, TC956X_GPIO_COUNT);
	bitmap_clear(valid_mask, 20, 2);

	return 0;
}

static int tc956x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc956x_gpio *gpio;
	struct gpio_chip *gc;

	printk(" === %s\n", __func__);

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->regmap = dev_get_regmap(dev->parent, "tc956x-gpio");
	if (!gpio->regmap)
		return -EINVAL;

	/*
	 * "Up to 35 GPIOs by share function"
	 *
	 * 00-19
	 * 20-21				reserved
	 * 22, 23, 24, 27, 28, 31		input only
	 * 32, 33
	 * 34					input only
	 * 35, 36
	 *
	 * GPIOI0  	0x1200
	 * 	GPIO 00-19, 22-31 input value (input mode)
	 *	20-21 are reserved
	 * GPIOE0  	0x1208
	 * 	00-19, 25-26, 29-30	0 output, 1 input
	 *	20-21 are reserved
	 * 	22-24, 27-28, 31	0 output low, 1 input or Hi-Z
	 * 	25-26, 29-30		0 output, 1 input
	 * GPIOO0  	0x1210
	 * 	00-19, 25-26, 29-30	output value
	 *	20-21 are reserved
	 *	22-24, 27-28, 31  	reserved	 (not output)
	 *
	 * GPIOI1	0x1204
	 * 	GPIO 32-36 input value (input mode)
	 * GPIOE1	0x120c
	 * 	32-33, 35-36		0 output, 1 input
	 * 	34			0 output low, 1 input or Hi-Z
	 * GPIOO1	0x1214
	 * 	32-33, 35-36		output value
	 * 	34			reserved	(not output)
	 *
	 */

	/* GPIOs 22, 23, 24, 27, 28, 31, and 34 are input only */
	bitmap_set(gpio->input_only, 22, 3);
	bitmap_set(gpio->input_only, 27, 2);
	set_bit(31, gpio->input_only);
	set_bit(34, gpio->input_only);

	gc = &gpio->chip;

	gc->label = "tc956x-gpio";
	gc->parent = dev->parent;

	gc->request = tc956x_gpio_request;
	gc->free = tc956x_gpio_free;
	gc->get_direction = tc956x_gpio_get_direction;
	gc->direction_input = tc956x_gpio_direction_input;
	gc->direction_output = tc956x_gpio_direction_output;
	gc->get = tc956x_gpio_get;
	gc->set = tc956x_gpio_set;
	gc->init_valid_mask = tc956x_gpio_init_valid_mask;

	gc->base = -1;
	gc->ngpio = TC956X_GPIO_COUNT;
	gc->can_sleep = false;

	return devm_gpiochip_add_data(dev, gc, gpio);
}

static struct platform_driver tc956x_gpio_driver = {
	.probe = tc956x_gpio_probe,
	.driver = {
		.name = "tc956x-gpio",
	},
};

module_platform_driver(tc956x_gpio_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe GPIO Driver");
MODULE_LICENSE("GPL");
