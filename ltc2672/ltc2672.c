// SPDX-License-Identifier: GPL-2.0
/*
 * LTC2672 Current Output 16-Bit Softspan DAC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#define LTC2672_DAC_CHANNELS			5

#define LTC2672_CMD_CH_CODE(x)			(0x00 + (x))
#define LTC2672_CMD_CODE_PWRUP_UPD_CH(x)	(0x30 + (x))
#define LTC2672_CMD_PWRDWN_CH(x)		(0x40 + (x))
#define LTC2672_CMD_SPAN_TO_CH(x)		(0x60 + (x))

#define LTC2672_CMD_TOGGLE_SEL			0xC0
#define LTC2672_CMD_TOGGLE_GLBL			0xD0

// #define LTC2672_ADDR_MASK                       GENMASK(3, 0)
// #define LTC2672_CMD_MASK                        GENMASK(7, 4)
#define LTC2672_16BIT_MAX			65535
// #define LTC2672_TOGGLE_SEL_MASK			GENMASK(4, 0)


enum {
	LTC2672_SPAN_3_125,
	LTC2672_SPAN_6_25,
	LTC2672_SPAN_12_5,
	LTC2672_SPAN_25,
	LTC2672_SPAN_50,
	LTC2672_SPAN_100,
	LTC2672_SPAN_200,
	LTC2672_SPAN_300,
};

enum {
	LTC2672_INPUT_A,
	LTC2672_INPUT_B,
};

struct ltc2672_chan {
	bool toggle;
	bool powerdown;
	u16 code;
	u16 raw[2];
};

struct ltc2672_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct ltc2672_chan channels[LTC2672_DAC_CHANNELS];
	struct mutex lock;
	struct regulator *vref;
	struct iio_chan_spec *iio_chan;
	int vref_mv;
	u32 span_code[5];
};

static const int ltc2672_span_helper[] = {
	3125, 6250, 12500, 25000, 50000, 100000, 200000, 300000,
};

static ssize_t ltc2672_dac_input_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	u16 val, toggle[5];
	int ret;

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC2672_16BIT_MAX || val < 0)
		return -EINVAL;

	if (private == LTC2672_INPUT_A)
		toggle[chan->channel] &= ~BIT(chan->channel);
	else
		toggle[chan->channel] |= BIT(chan->channel);

	ret = regmap_write(st->regmap, LTC2672_CMD_TOGGLE_SEL,
			   toggle[chan->channel]);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, LTC2672_CMD_CH_CODE(chan->channel), val);
	if (ret)
		return ret;

	return ret ?: len;
}

static ssize_t ltc2672_dac_input_read(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	// struct ltc2672_state *st = iio_priv(indio_dev);

	// return sysfs_emit(buf, "%d\n", st->toggle_chan[chan->channel]);
	return 0;
}

static ssize_t ltc2672_toggle_set(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, private, en);
	if (ret)
		return ret;

	return ret ?: len;
}

static ssize_t ltc2672_toggle_get(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	// struct ltc2672_state *st = iio_priv(indio_dev);

	// return sysfs_emit(buf, "%d\n", st->toggle_glbl);
	return 0;
}

static ssize_t ltc2672_powerdown_set(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, LTC2672_CMD_PWRDWN_CH(chan->channel), en);
	if (ret)
		return ret;

	return len;
}

static int ltc2672_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ltc2672_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > LTC2672_16BIT_MAX || val < 0)
			return -EINVAL;

		return regmap_write(st->regmap,
			            LTC2672_CMD_CODE_PWRUP_UPD_CH(chan->channel),
				    val);
	default:
		return -EINVAL;
	}
}

static const struct regmap_config ltc2672_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.zero_flag_mask = 1,
};

static const struct iio_info ltc2672_info = {
	.write_raw = ltc2672_write_raw,
	// .read_raw = ltc2672_read_raw,
	// .debugfs_reg_access = &ltc2672_reg_access,
};

#define LTC2672_CHAN_EXT_INFO(_name, _what, _shared, _read, _write) {	\
	.name = _name,							\
	.private = (_what),						\
	.read = (_read),						\
	.write = (_write),						\
	.shared = (_shared),						\
}

static const struct iio_chan_spec_ext_info toggle_mode_ext_info[] = {
	LTC2672_CHAN_EXT_INFO("raw0", LTC2672_INPUT_A, IIO_SEPARATE,
			      ltc2672_dac_input_read, ltc2672_dac_input_write),
	LTC2672_CHAN_EXT_INFO("raw1", LTC2672_INPUT_B, IIO_SEPARATE,
			      ltc2672_dac_input_read, ltc2672_dac_input_write),
	LTC2672_CHAN_EXT_INFO("toggle", LTC2672_CMD_TOGGLE_GLBL, IIO_SEPARATE,
			      ltc2672_toggle_get, ltc2672_toggle_set),
	LTC2672_CHAN_EXT_INFO("powerdown", 0, IIO_SEPARATE,
			      NULL, ltc2672_powerdown_set),
	{ }
};

static const struct iio_chan_spec_ext_info ltc2672_ext_info[] = {
	LTC2672_CHAN_EXT_INFO("powerdown", 0, IIO_SEPARATE,
			      NULL, ltc2672_powerdown_set),
	{ }
};

#define LTC2672_CHAN(_chan) {						\
	.type = IIO_CURRENT,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.ext_info = ltc2672_ext_info,					\
}

static const struct iio_chan_spec ltc2672_channels[] = {
	LTC2672_CHAN(0),
	LTC2672_CHAN(1),
	LTC2672_CHAN(2),
	LTC2672_CHAN(3),
	LTC2672_CHAN(4),
};

static int ltc2672_chan_config(struct ltc2672_state *st)
{
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *child;
	u32 reg, span_range, span_code[5];
	int ret, span;

	// ret = device_property_read_u32(dev, "adi,rfsadj-ohms", &st->rfsadj);
	// if (ret)
	// 	return dev_err_probe(dev, ret,
	// 			     "adi,rfsadj property not found\n");

	device_for_each_child_node(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, ret,
					     "reg property not found\n");
		}

		if (reg > LTC2672_DAC_CHANNELS) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "reg property out of range\n");
		}

		ret = fwnode_property_read_u32(child, "adi,output-range-microamp",
					       &span_range);
		if (!ret) {
			for (span = 0 ; span < ARRAY_SIZE(ltc2672_span_helper); span++) {
				if (ltc2672_span_helper[span] == span_range / 1000)
					span_code[reg] = span + 1;
			}
		}

		if (span_code[reg] < 0) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "adi,output-range-microvolt property out of range\n");
		}

		ret = regmap_write(st->regmap, LTC2672_CMD_SPAN_TO_CH(reg),
				   span_code[reg]);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}

		if (fwnode_property_present(child, "adi,toggle-mode")) {
			st->iio_chan[reg].ext_info = toggle_mode_ext_info;
			/*
			 * Clear IIO_CHAN_INFO_RAW bit as toggle channels expose
			 * out_voltage_raw{0|1} files.
			 */
			__clear_bit(IIO_CHAN_INFO_RAW, &st->iio_chan[reg].info_mask_separate);
		}
	}

	return 0;
}

static void ltc2672_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static int ltc2672_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ltc2672_state *st;
	struct gpio_desc *reset;
	struct regulator *vref;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->regmap = devm_regmap_init_spi(spi, &ltc2672_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->iio_chan = devm_kmemdup(dev, ltc2672_channels,
				    sizeof(ltc2672_channels), GFP_KERNEL);
	if (!st->iio_chan)
		return -ENOMEM;

	indio_dev->name = "ltc2672";
	indio_dev->info = &ltc2672_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->iio_chan;
	indio_dev->num_channels = ARRAY_SIZE(ltc2672_channels);

	vref = devm_regulator_get_optional(&spi->dev, "vref");
	if (IS_ERR(vref)) {
		if (PTR_ERR(vref) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(vref),
					     "Failed to get vref regulator");

		/* internal reference */
		st->vref_mv = 1250;
	} else {
		ret = regulator_enable(vref);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					"Failed to enable vref regulators\n");

		ret = devm_add_action_or_reset(&spi->dev,
					       ltc2672_regulator_disable,
					       vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to get vref\n");

		st->vref_mv = ret / 1000;
	}

	mutex_init(&st->lock);

	reset = devm_gpiod_get_optional(dev, "adi,reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to get reset gpio\n");
	if (reset) {
		ndelay(30);
		gpiod_set_value_cansleep(reset, 1);
	}

	ret = ltc2672_chan_config(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2672_of_match[] = {
	{ .compatible = "adi,ltc2672" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2672_of_match);

static const struct spi_device_id ltc2672_id[] = {
	{ "ltc2672" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ltc2672_id);

static struct spi_driver ltc2672_driver = {
	.driver = {
		.name	= "ltc2672",
		.of_match_table = ltc2672_of_match,
	},
	.probe		= ltc2672_probe,
	.id_table	= ltc2672_id,
};
module_spi_driver(ltc2672_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("LTC2672 DAC driver");
MODULE_LICENSE("GPL");
