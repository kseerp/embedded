// SPDX-License-Identifier: GPL-2.0
/*
 * LTC2672 12-/16 Bit Softspan DAC driver
 *
 * Copyright 2023 Analog Devices Inc.
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

#define LTC2672_16BIT_FULL_SCALE		65535

#define LTC2672_ADDR_MASK                       GENMASK(3, 0)
#define LTC2672_CMD_MASK                        GENMASK(7, 4)
// #define LTC2672_TOGGLE_SEL_MASK			GENMASK(4, 0)

/*
 * Definition used because the latter is not available
 * in project's source tree.
 */
#define IIO_DMA_MINALIGN	ARCH_KMALLOC_MINALIGN

enum ltc2672_cmd {
	LTC2672_CODE_TO_CHANNEL_X,
	LTC2672_PWRUP_UPD_CHANNEL_X,
	LTC2672_CODE_TO_CHANNEL_X_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_CODE_PWRUP_UPD_CHANNEL_X,
	LTC2672_PWRDWN_CHANNEL_X,
	LTC2672_PWRDWN_DEV,
	LTC2672_SPAN_TO_CHANNEL_X,
	LTC2672_CNFG_CMD,
	LTC2672_CODE_TO_CHANNEL_ALL,
	LTC2672_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_CODE_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_MON_MUX,
	LTC2672_TOGGLE_SEL,
	LTC2672_TOGGLE_GLBL,
	LTC2672_SPAN_TO_CHANNEL_ALL,
	LTC2672_NO_OP
};

/* Device Family */
enum ltc2672_device_id {
	LTC2672_12,
	LTC2672_16
};

enum ltc2672_output_range {
	LTC2672_OFF,
	LTC2672_50VREF,
	LTC2672_100VREF,
	LTC2672_200VREF,
	LTC2672_400VREF,
	LTC2672_800VREF,
	LTC2672_1600VREF,
	LTC2672_3200VREF,
	LTC2672_VMINUS_VREF,
	LTC2672_4800VREF = 0XF
};

enum {
	LTC2672_INPUT_A,
	LTC2672_INPUT_B,
};

enum {
	LTC2672_TOGGLE_LOW,
	LTC2672_TOGGLE_HIGH,
};

/* Multiplexer cmdand Codes */
enum ltc2672_mux_cmd {
	LTC2672_MUX_DISABLED,
	LTC2672_MUX_IOUT0,
	LTC2672_MUX_IOUT1,
	LTC2672_MUX_IOUT2,
	LTC2672_MUX_IOUT3,
	LTC2672_MUX_IOUT4,
	LTC2672_MUC_VCC,
	LTC2672_MUX_VREF = 0x08,
	LTC2672_MUX_VREF_LO,
	LTC2672_MUX_DIE_TEMP,
	LTC2672_MUX_VDD0 = 0x10,
	LTC2672_MUX_VDD1,
	LTC2672_MUX_VDD2,
	LTC2672_MUX_VDD3,
	LTC2672_MUX_VDD4,
	LTC2672_MUX_VMINUS = 0X16,
	LTC2672_MUX_GND,
	LTC2672_MUX_VOUT0,
	LTC2672_MUX_VOUT1,
	LTC2672_MUX_VOUT2,
	LTC2672_MUX_VOUT3,
	LTC2672_MUX_VOUT4
};

/* Faults */
enum ltc2672_faults {
	LTC2672_OPEN_OUT0, // Open circuit CH0
	LTC2672_OPEN_OUT1, // Open circuit CH0
	LTC2672_OPEN_OUT2, // Open circuit CH0
	LTC2672_OPEN_OUT3, // Open circuit CH0
	LTC2672_OPEN_OUT4, // Open circuit CH0
	LTC2672_OVER_TEMP, // Over-temperature (T > 175 deg C)
	LTC2672_UNUSED, // Unused fault register bit
	LTC2672_INV_LENGTH, // Invalid SPI Length (len != 24 or 32 * n)
};

struct ltc2672_state {
	struct spi_device       *spi;
        struct regmap           *regmap;
	struct mutex            lock;
	struct regulator        *vref;
	struct iio_chan_spec	*iio_chan;
	int			vref_mv;
	int			rfsadj;
	u32			span_code[5];
	u8			toggle_chan[5];
	u8			toggle_glbl;
};

static int ltc2672_set_current(struct ltc2672_state *st, u8 addr, u16 val)
{
	u16 span, code;
	int ret;

	span = FIELD_PREP(LTC2672_ADDR_MASK, addr) |
	       FIELD_PREP(LTC2672_CMD_MASK, LTC2672_SPAN_TO_CHANNEL_X);

	code = FIELD_PREP(LTC2672_ADDR_MASK, addr) |
	       FIELD_PREP(LTC2672_CMD_MASK, LTC2672_CODE_PWRUP_UPD_CHANNEL_X);

	ret = regmap_write(st->regmap, span, st->span_code[addr]);
	if (ret)
		return ret;

	return regmap_write(st->regmap, code, val);
}

static ssize_t ltc2672_dac_input_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	u16 val, code, toggle, span;
	int ret;

	span = FIELD_PREP(LTC2672_ADDR_MASK, chan->channel) |
	       FIELD_PREP(LTC2672_CMD_MASK, LTC2672_SPAN_TO_CHANNEL_X);

	code = FIELD_PREP(LTC2672_ADDR_MASK, chan->channel) |
	       FIELD_PREP(LTC2672_CMD_MASK, LTC2672_CODE_TO_CHANNEL_X);

	toggle = FIELD_PREP(LTC2672_CMD_MASK, LTC2672_TOGGLE_SEL);

	if (private == LTC2672_INPUT_A)
		st->toggle_chan[chan->channel] &= ~BIT(chan->channel);
	else if (private == LTC2672_INPUT_B)
		st->toggle_chan[chan->channel] |= BIT(chan->channel);

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	if (val > LTC2672_16BIT_FULL_SCALE)
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = regmap_write(st->regmap, span, st->span_code[chan->channel]);
	if (ret)
		goto out_unlock;

	ret = regmap_write(st->regmap, toggle, st->toggle_chan[chan->channel]);
	if (ret)
		goto out_unlock;

	ret = regmap_write(st->regmap, code, val);
	if (ret)
		goto out_unlock;

out_unlock:
	mutex_unlock(&st->lock);

	return len;
}

static ssize_t ltc2672_powerdown_set(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	u16 code;
	bool en;
	int ret;

	code = FIELD_PREP(LTC2672_ADDR_MASK, chan->channel) |
	       FIELD_PREP(LTC2672_CMD_MASK, LTC2672_PWRDWN_CHANNEL_X);

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = regmap_write(st->regmap, code, en);
	if (ret)
		goto out_unlock;

out_unlock:
	mutex_unlock(&st->lock);

	return len;
}

static ssize_t ltc2672_dac_input_read(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct ltc2672_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->toggle_chan[chan->channel]);
}

static ssize_t ltc2672_toggle_set(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct ltc2672_state *st = iio_priv(indio_dev);
	u16 toggle;
	bool en;
	int ret;

	toggle = FIELD_PREP(LTC2672_CMD_MASK, LTC2672_TOGGLE_GLBL);

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	st->toggle_glbl = en;

	ret = regmap_write(st->regmap, toggle, en);
	if (ret)
		goto out_unlock;

out_unlock:
	mutex_unlock(&st->lock);

	return ret ?: len;
}

static ssize_t ltc2672_toggle_get(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  char *buf)
{
	struct ltc2672_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->toggle_glbl);
}

static int ltc2672_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ltc2672_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return ltc2672_set_current(st, chan->channel, val);
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
	LTC2672_CHAN_EXT_INFO("toggle", 0, IIO_SEPARATE,
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
	u32 reg;
	int ret;

	ret = device_property_read_u32(dev, "adi,rfsadj-ohms", &st->rfsadj);
	if (ret)
		return dev_err_probe(dev, ret,
				     "adi,rfsadj property not found\n");

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

		if (fwnode_property_read_bool(child, "adi,toggle-mode"))
			st->iio_chan[reg].ext_info = toggle_mode_ext_info;

		st->span_code[reg] = LTC2672_50VREF;
		ret = fwnode_property_read_u32(child, "adi,current-span-code", &st->span_code[reg]);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, ret,
					     "adi,current-span-code property not found\n");
		}

		if (st->span_code[reg] > LTC2672_VMINUS_VREF &&
		    st->span_code[reg] != LTC2672_4800VREF) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "adi,current-span-code property out of range\n");
		}
		
		printk(KERN_INFO "span_code[%d] = %d\n", reg, st->span_code[reg]);
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
	struct iio_dev *indio_dev;
	struct ltc2672_state *st;
	struct device *dev = &spi->dev;
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
