// SPDX-License-Identifier: GPL-2.0
/*
 * ADG1414 SPST Switch Driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/regmap.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define ADG1414_MAX_DEVICES		4

// struct adg1414_state {
// 	struct gpio_chip chip;
// 	u32 buf;

// 	__be32 tx __aligned(ARCH_KMALLOC_MINALIGN);
// };

// static int adg1414_write(void *context, const void *data, size_t count)
// {
// 	struct adg1414_state *st = context;
// 	u32 value = *(const u32 *)data;
// 	struct spi_device *spi = to_spi_device(st->chip.parent);

// 	struct spi_transfer xfer = {
// 		.tx_buf = &st->tx,
// 		.len = st->chip.ngpio / 8,
// 	};

// 	st->tx = cpu_to_be32(value << (32 - st->chip.ngpio));

// 	return spi_sync_transfer(spi, &xfer, 1);
// 	// return spi_write(spi, &value, count);
// }

// static void adg1414_set(struct gpio_chip *chip, unsigned int offset, int value)
// {
// 	struct adg1414_state *st = gpiochip_get_data(chip);
// 	u32 buffer;
// 	int ret;

// 	if (value)
// 		buffer |= BIT(offset);
//     	else
// 		buffer &= ~BIT(offset);

// 	adg1414_write(st, &buffer, sizeof(buffer));
// }

// static int adg1414_get(struct gpio_chip *chip, unsigned int offset)
// {
// 	struct adg1414_state *st = gpiochip_get_data(chip);
// 	int value;

// 	value = st->buf & BIT(offset);

// 	return value;
// }

// static int adg1414_get_direction(struct gpio_chip *chip, unsigned int offset)
// {
// 	return GPIO_LINE_DIRECTION_OUT;
// }

// static const struct regmap_bus adg1414_regmap_bus = {
// 	.write = adg1414_write,
// 	// .read = adg1414_get,
// 	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
// 	.val_format_endian_default = REGMAP_ENDIAN_BIG,
// };

static int adg1414_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct regmap *regmap;
	struct adg1414_state *st;
	struct gpio_desc *reset;
	u32 num_devices;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	spi_set_drvdata(spi, st);

	regmap = devm_regmap_init(dev, &adg1414_regmap_bus, st, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Use reset pin to reset the device */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to get reset gpio");

	if (reset) {
		fsleep(1);
		gpiod_set_value_cansleep(reset, 0);
	}

	num_devices = 1;
	ret = device_property_read_u32(dev, "#daisy-chained-devices",
				       &num_devices);
	if (!ret) {
		if (!num_devices || num_devices > ADG1414_MAX_DEVICES)
			return dev_err_probe(dev, ret,
			       "Failed to get daisy-chained-devices property\n");
	}

	printk(KERN_INFO "num_devices: %d\n", num_devices);

	st->chip.label = "adg1414";
	st->chip.parent = dev;
	st->chip.get_direction = adg1414_get_direction;
	st->chip.set = adg1414_set;
	// st->chip.get = adg1414_get;
	st->chip.base = -1;
	st->chip.ngpio =  num_devices * 8;
	st->chip.can_sleep = true;

	return devm_gpiochip_add_data(dev, &st->chip, st);
}

static const struct of_device_id adg1414_of_match[] = {
	{ .compatible = "adi,adg1414" },
	{ }
};
MODULE_DEVICE_TABLE(of, adg1414_of_match);

static struct spi_driver adg1414_driver = {
	.driver = {
		.name = "adg1414",
		.of_match_table = adg1414_of_match,
	},
	.probe = adg1414_probe,
};
module_spi_driver(adg1414_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("ADG1414 SPST Switch Driver");
MODULE_LICENSE("GPL");
