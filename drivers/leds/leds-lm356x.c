/*
* General driver for Texas Instruments LM356x LED Flash driver chip
* Copyright (C) 2013 Texas Instruments
*
* Author: Daniel Jeong <daniel.jeong@ti.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/platform_data/leds-lm356x.h>
#include <cust_i2c.h>


enum lm356x_type {
	CHIP_LM3560 = 0,
	CHIP_LM3561,
};

enum lm356x_mode {
	MODE_SHDN = 0,
	MODE_INDI,
	MODE_TORCH,
	MODE_FLASH,
	MODE_MAX
};

enum lm356x_common_dev {
	DEV_FLASH1 = 0,
	DEV_FLASH2,
	DEV_TORCH1,
	DEV_TORCH2,
	DEV_INDI,
	DEV_MAX,
};

enum lm356x_cmds {
	CMD_ENABLE = 0,
	CMD_LED1_ENABLE,
	CMD_LED1_FLASH_I,
	CMD_LED1_TORCH_I,
	CMD_LED2_ENABLE,
	CMD_LED2_FLASH_I,
	CMD_LED2_TORCH_I,
	CMD_FLASH_TOUT,
	CMD_INDI_I,
	CMD_INDI_LED1,
	CMD_INDI_LED2,
	CMD_INDI_TPULSE,
	CMD_INDI_TBLANK,
	CMD_INDI_PERIOD,
	CMD_PIN_TORCH,
	CMD_PIN_STROBE,
	CMD_MAX,
};

struct lm356x_cmd_data {
	u8 reg;
	u8 mask;
	u8 shift;
};

struct lm356x_chip_data {
	struct device *dev;
	struct i2c_client *client;
	enum lm356x_type type;

	struct led_classdev cdev_flash1;
	struct led_classdev cdev_flash2;
	struct led_classdev cdev_torch1;
	struct led_classdev cdev_torch2;
	struct led_classdev cdev_indicator;

	struct work_struct work_flash1;
	struct work_struct work_flash2;
	struct work_struct work_torch1;
	struct work_struct work_torch2;
	struct work_struct work_indicator;

	u8 br_flash1;
	u8 br_flash2;
	u8 br_torch1;
	u8 br_torch2;
	u8 br_indicator;

	struct mutex lock;
	struct regmap *regmap;
	struct lm356x_cmd_data *cmd;
	struct lm356x_platform_data *pdata;
};

static struct lm356x_cmd_data lm3560_cmds[CMD_MAX] = {
	[CMD_ENABLE] = {0x10, 0x03, 0},
	[CMD_LED1_ENABLE] = {0x10, 0x01, 3},
	[CMD_LED1_FLASH_I] = {0xB0, 0x0F, 0},
	[CMD_LED1_TORCH_I] = {0xA0, 0x07, 0},
	[CMD_LED2_ENABLE] = {0x10, 0x01, 4},
	[CMD_LED2_FLASH_I] = {0xB0, 0x0F, 4},
	[CMD_LED2_TORCH_I] = {0xA0, 0x07, 4},
	[CMD_FLASH_TOUT] = {0xC0, 0x3F, 0},
	[CMD_INDI_I] = {0x12, 0x07, 0},
	[CMD_INDI_TPULSE] = {0x13, 0x0F, 0},
	[CMD_INDI_TBLANK] = {0x13, 0x07, 4},
	[CMD_INDI_PERIOD] = {0x12, 0x07, 3},
	[CMD_INDI_LED1] = {0x11, 0x01, 4},
	[CMD_INDI_LED2] = {0x11, 0x01, 5},
	[CMD_PIN_TORCH] = {0xE0, 0x01, 7},
	[CMD_PIN_STROBE] = {0xE0, 0x01, 2},
};

static struct lm356x_cmd_data lm3561_cmds[CMD_MAX] = {
	[CMD_ENABLE] = {0x10, 0x03, 0},
	[CMD_LED1_ENABLE] = {0x10, 0x01, 3},
	[CMD_LED1_FLASH_I] = {0xB0, 0x0F, 0},
	[CMD_LED1_TORCH_I] = {0xA0, 0x07, 0},
	[CMD_FLASH_TOUT] = {0xC0, 0x1F, 0},
	[CMD_INDI_I] = {0x12, 0x07, 0},
	[CMD_PIN_TORCH] = {0xE0, 0x01, 7},
	[CMD_PIN_STROBE] = {0xE0, 0x01, 2},
};

static int lm356x_reg_update(struct lm356x_chip_data *pchip,
			     u8 reg, u8 mask, u8 data)
{
	return regmap_update_bits(pchip->regmap, reg, mask, data);
}

static int lm356x_reg_read(struct lm356x_chip_data *pchip, u8 reg)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(pchip->regmap, reg, &reg_val);
	if (ret < 0)
		return ret;
	return reg_val & 0xFF;
}

static int lm356x_cmd_write(struct lm356x_chip_data *pchip,
			    enum lm356x_cmds cmd, u8 data)
{
	int ret;
	struct lm356x_cmd_data *pcmd = pchip->cmd;
	u8 reg = pcmd[cmd].reg;
	u8 mask = pcmd[cmd].mask << pcmd[cmd].shift;

	data = data << pcmd[cmd].shift;
	ret = lm356x_reg_update(pchip, reg, mask, data);
	if (ret < 0)
		return ret;
	return 0;
}

static int lm356x_chip_init(struct lm356x_chip_data *pchip)
{
	int rval;

	rval = lm356x_cmd_write(pchip, CMD_PIN_TORCH, pchip->pdata->torch);
	if (rval < 0)
		return rval;

	rval = lm356x_cmd_write(pchip, CMD_PIN_STROBE, pchip->pdata->strobe);
	if (rval < 0)
		return rval;

	rval =
	    lm356x_cmd_write(pchip, CMD_FLASH_TOUT, pchip->pdata->flash_time);
	if (rval < 0)
		return rval;

	if (pchip->type != CHIP_LM3560)
		goto out;

	rval = lm356x_cmd_write(pchip, CMD_INDI_LED1, pchip->pdata->indi_led1);
	if (rval < 0)
		return rval;

	rval = lm356x_cmd_write(pchip, CMD_INDI_LED2, pchip->pdata->indi_led2);
	if (rval < 0)
		return rval;
out:
	return 0;
}

static int lm356x_pdata_init(struct lm356x_chip_data *pchip)
{
	int rval;
	struct lm356x_cmd_data *pcmd = pchip->cmd;

	pchip->pdata = devm_kzalloc(pchip->dev,
				    sizeof(struct lm356x_platform_data),
				    GFP_KERNEL);
	if (!pchip->pdata)
		return -ENOMEM;
	rval = lm356x_reg_read(pchip, pcmd[CMD_PIN_STROBE].reg);
	if (rval < 0)
		return rval;
	pchip->pdata->torch = (rval >> pcmd[CMD_PIN_TORCH].shift)
	    & pcmd[CMD_PIN_TORCH].mask;
	pchip->pdata->strobe = (rval >> pcmd[CMD_PIN_STROBE].shift)
	    & pcmd[CMD_PIN_STROBE].mask;

	if (pchip->type != CHIP_LM3560)
		goto out;

	rval = lm356x_reg_read(pchip, pcmd[CMD_INDI_LED1].reg);
	if (rval < 0)
		return rval;
	pchip->pdata->indi_led1 = (rval >> pcmd[CMD_INDI_LED1].shift)
	    & pcmd[CMD_INDI_LED1].mask;
	pchip->pdata->indi_led2 = (rval >> pcmd[CMD_INDI_LED2].shift)
	    & pcmd[CMD_INDI_LED2].mask;
out:
	return 0;
}

/* mode control */
static void lm356x_ctrl_mode(struct lm356x_chip_data *pchip,
			     const char *buf, enum lm356x_mode mode)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out;
	if (state != 0)
		ret = lm356x_cmd_write(pchip, CMD_ENABLE, mode);
	else
		ret = lm356x_cmd_write(pchip, CMD_ENABLE, MODE_SHDN);
	if (ret < 0)
		goto out;
	return;
out:
	dev_err(pchip->dev, "%s: fail to set mode %d\n", __func__, mode);
	return;
}

/* input control */
static void lm356x_ctrl_input(struct lm356x_chip_data *pchip,
			      const char *buf, enum lm356x_cmds cmd)
{
	ssize_t ret;
	unsigned int state;

	ret = kstrtouint(buf, 10, &state);
	if (ret)
		goto out;
	ret = lm356x_cmd_write(pchip, cmd, state);
	if (ret < 0)
		goto out;
	return;
out:
	dev_err(pchip->dev, "%s: fail to set onoff\n", __func__);
	return;
}

#define pchip_lm356x(_cdev)\
	container_of(led_cdev, struct lm356x_chip_data, _cdev)

/* flash1 control */
static ssize_t lm356x_flash1_ctrl_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_flash1);

	if (lm356x_cmd_write(pchip, CMD_LED1_ENABLE, 1) < 0)
		return size;
	lm356x_ctrl_mode(pchip, buf, MODE_FLASH);
	return size;
}

static ssize_t lm356x_flash1_enable_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_flash1);

	lm356x_ctrl_input(pchip, buf, CMD_LED1_ENABLE);
	return size;
}

static void lm356x_deferred_flash1_brightness_set(struct work_struct *work)
{
	struct lm356x_chip_data *pchip =
	    container_of(work, struct lm356x_chip_data, work_flash1);

	mutex_lock(&pchip->lock);
	if (lm356x_cmd_write(pchip, CMD_LED1_FLASH_I, pchip->br_flash1) < 0)
		dev_err(pchip->dev,
			"%s: fail to set flash brightness\n", __func__);
	mutex_unlock(&pchip->lock);
}

static void lm356x_flash1_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm356x_chip_data *pchip =
	    container_of(cdev, struct lm356x_chip_data, cdev_flash1);

	pchip->br_flash1 = brightness;
	schedule_work(&pchip->work_flash1);
}

/* torch1 control */
static ssize_t lm356x_torch1_ctrl_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_torch1);

	if (lm356x_cmd_write(pchip, CMD_LED1_ENABLE, 1) < 0)
		return size;
	lm356x_ctrl_mode(pchip, buf, MODE_TORCH);
	return size;
}

static ssize_t lm356x_torch1_enable_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_torch1);

	lm356x_ctrl_input(pchip, buf, CMD_LED1_ENABLE);
	return size;
}

static void lm356x_deferred_torch1_brightness_set(struct work_struct *work)
{
	struct lm356x_chip_data *pchip =
	    container_of(work, struct lm356x_chip_data, work_torch1);

	mutex_lock(&pchip->lock);
	if (lm356x_cmd_write(pchip, CMD_LED1_TORCH_I, pchip->br_torch1) < 0)
		dev_err(pchip->dev,
			"%s: fail to set torch brightness\n", __func__);
	mutex_unlock(&pchip->lock);
}

static void lm356x_torch1_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm356x_chip_data *pchip =
	    container_of(cdev, struct lm356x_chip_data, cdev_torch1);
	pchip->br_torch1 = brightness;
	schedule_work(&pchip->work_torch1);
}

/* flash2 control */
static ssize_t lm356x_flash2_ctrl_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_flash2);

	if (lm356x_cmd_write(pchip, CMD_LED2_ENABLE, 1) < 0)
		return size;
	lm356x_ctrl_mode(pchip, buf, MODE_FLASH);
	return size;
}

static ssize_t lm356x_flash2_enable_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_flash2);

	lm356x_ctrl_input(pchip, buf, CMD_LED2_ENABLE);
	return size;
}

static void lm356x_deferred_flash2_brightness_set(struct work_struct *work)
{
	struct lm356x_chip_data *pchip =
	    container_of(work, struct lm356x_chip_data, work_flash2);

	mutex_lock(&pchip->lock);
	if (lm356x_cmd_write(pchip, CMD_LED2_FLASH_I, pchip->br_flash2) < 0)
		dev_err(pchip->dev,
			"%s: fail to set flash brightness\n", __func__);
	mutex_unlock(&pchip->lock);
}

static void lm356x_flash2_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm356x_chip_data *pchip =
	    container_of(cdev, struct lm356x_chip_data, cdev_flash2);

	pchip->br_flash2 = brightness;
	schedule_work(&pchip->work_flash2);
}

/* torch2 control */
static ssize_t lm356x_torch2_ctrl_store(struct device *dev,
					struct device_attribute *devAttr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_torch2);

	if (lm356x_cmd_write(pchip, CMD_LED2_ENABLE, 1) < 0)
		return size;
	lm356x_ctrl_mode(pchip, buf, MODE_TORCH);
	return size;
}

static ssize_t lm356x_torch2_enable_store(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_torch2);

	lm356x_ctrl_input(pchip, buf, CMD_LED2_ENABLE);
	return size;
}

static void lm356x_deferred_torch2_brightness_set(struct work_struct *work)
{
	struct lm356x_chip_data *pchip =
	    container_of(work, struct lm356x_chip_data, work_torch2);

	mutex_lock(&pchip->lock);
	if (lm356x_cmd_write(pchip, CMD_LED2_TORCH_I, pchip->br_torch2) < 0)
		dev_err(pchip->dev,
			"%s: fail to set torch brightness\n", __func__);
	mutex_unlock(&pchip->lock);
}

static void lm356x_torch2_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm356x_chip_data *pchip =
	    container_of(cdev, struct lm356x_chip_data, cdev_torch2);
	pchip->br_torch2 = brightness;
	schedule_work(&pchip->work_torch2);
}

/* indicator control */
static ssize_t lm356x_indicator_ctrl_store(struct device *dev,
					   struct device_attribute *devAttr,
					   const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_indicator);

	if (pchip->type == CHIP_LM3560) {
		if (lm356x_cmd_write
		    (pchip, CMD_INDI_LED1, pchip->pdata->indi_led1) < 0)
			goto err_out;
		if (lm356x_cmd_write
		    (pchip, CMD_INDI_LED2, pchip->pdata->indi_led2) < 0)
			goto err_out;
	}
	lm356x_ctrl_mode(pchip, buf, MODE_INDI);
	return size;
err_out:
	dev_err(pchip->dev, "fail to set indicator control\n");
	return size;
}

static ssize_t lm356x_indicator_blank_store(struct device *dev,
					    struct device_attribute *devAttr,
					    const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_indicator);

	lm356x_ctrl_input(pchip, buf, CMD_INDI_TBLANK);
	return size;
}

static ssize_t lm356x_indicator_pulse_store(struct device *dev,
					    struct device_attribute *devAttr,
					    const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_indicator);

	lm356x_ctrl_input(pchip, buf, CMD_INDI_TPULSE);
	return size;
}

static ssize_t lm356x_indicator_period_store(struct device *dev,
					     struct device_attribute *devAttr,
					     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm356x_chip_data *pchip = pchip_lm356x(cdev_indicator);

	lm356x_ctrl_input(pchip, buf, CMD_INDI_PERIOD);
	return size;
}

static void lm356x_deferred_indicator_brightness_set(struct work_struct *work)
{
	struct lm356x_chip_data *pchip =
	    container_of(work, struct lm356x_chip_data, work_indicator);

	mutex_lock(&pchip->lock);
	if (lm356x_cmd_write(pchip, CMD_INDI_I, pchip->br_indicator) < 0)
		dev_err(pchip->dev,
			"%s: fail to set indicator brightness\n", __func__);
	mutex_unlock(&pchip->lock);
}

static void lm356x_indicator_brightness_set(struct led_classdev *cdev,
					    enum led_brightness brightness)
{
	struct lm356x_chip_data *pchip =
	    container_of(cdev, struct lm356x_chip_data, cdev_indicator);

	pchip->br_indicator = brightness;
	schedule_work(&pchip->work_indicator);
}

#define lm356x_attr(_show, _store, _name)\
{\
	.attr = {\
		.name = _name,\
		.mode = S_IWUSR,\
	},\
	.show = _show,\
	.store = _store,\
}

static struct device_attribute dev_attr_ctrl[DEV_MAX] = {
	[DEV_FLASH1] = lm356x_attr(NULL, lm356x_flash1_ctrl_store, "ctrl"),
	[DEV_FLASH2] = lm356x_attr(NULL, lm356x_flash2_ctrl_store, "ctrl"),
	[DEV_TORCH1] = lm356x_attr(NULL, lm356x_torch1_ctrl_store, "ctrl"),
	[DEV_TORCH2] = lm356x_attr(NULL, lm356x_torch2_ctrl_store, "ctrl"),
	[DEV_INDI] = lm356x_attr(NULL, lm356x_indicator_ctrl_store, "ctrl"),
};

static struct device_attribute dev_attr_enable[DEV_MAX] = {
	[DEV_FLASH1] = lm356x_attr(NULL, lm356x_flash1_enable_store, "enable"),
	[DEV_FLASH2] = lm356x_attr(NULL, lm356x_flash2_enable_store, "enable"),
	[DEV_TORCH1] = lm356x_attr(NULL, lm356x_torch1_enable_store, "enable"),
	[DEV_TORCH2] = lm356x_attr(NULL, lm356x_torch2_enable_store, "enable"),
};

static DEVICE_ATTR(tblank, S_IWUSR, NULL, lm356x_indicator_blank_store);
static DEVICE_ATTR(tpulse, S_IWUSR, NULL, lm356x_indicator_pulse_store);
static DEVICE_ATTR(period, S_IWUSR, NULL, lm356x_indicator_period_store);

/* module initialize */
static const struct regmap_config lm356x_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int lm356x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lm356x_platform_data *pdata = client->dev.platform_data;
	struct lm356x_chip_data *pchip;
	int err;

	printk("%s\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev,
			     sizeof(struct lm356x_chip_data), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->client = client;
	pchip->dev = &client->dev;
	pchip->type = id->driver_data;

	printk("%s type is %d\n",__func__, pchip->type);
	switch (pchip->type) {
	case CHIP_LM3560:
		pchip->cmd = lm3560_cmds;
		break;
	case CHIP_LM3561:
		pchip->cmd = lm3561_cmds;
		break;
	default:
		return -ENOSYS;
	}
	/* regmap */
	pchip->regmap = devm_regmap_init_i2c(client, &lm356x_regmap);
	if (IS_ERR(pchip->regmap)) {
		err = PTR_ERR(pchip->regmap);
		return err;
	}
	/* platform data */
	if (pchip->pdata == NULL) {
		err = lm356x_pdata_init(pchip);
		if (err < 0)
			return err;
	} else {
		pchip->pdata = pdata;
		err = lm356x_chip_init(pchip);
		if (err < 0)
			return err;
	}
	mutex_init(&pchip->lock);
	i2c_set_clientdata(client, pchip);

	/* flash1 initialize */
	INIT_WORK(&pchip->work_flash1, lm356x_deferred_flash1_brightness_set);
	pchip->cdev_flash1.name = "flash1";
	pchip->cdev_flash1.max_brightness = 15;
	pchip->cdev_flash1.brightness_set = lm356x_flash1_brightness_set;
	pchip->cdev_flash1.default_trigger = "flash1";
	err = led_classdev_register((struct device *)
				    &client->dev, &pchip->cdev_flash1);
	if (err < 0)
		goto err_out_flash1;
	err =
	    device_create_file(pchip->cdev_flash1.dev,
			       &dev_attr_ctrl[DEV_FLASH1]);
	if (err < 0)
		goto err_out_flash1_ctrl;

	/* torch1 initialize */
	INIT_WORK(&pchip->work_torch1, lm356x_deferred_torch1_brightness_set);
	pchip->cdev_torch1.name = "torch1";
	pchip->cdev_torch1.max_brightness = 7;
	pchip->cdev_torch1.brightness_set = lm356x_torch1_brightness_set;
	pchip->cdev_torch1.default_trigger = "torch1";
	err = led_classdev_register((struct device *)
				    &client->dev, &pchip->cdev_torch1);
	if (err < 0)
		goto err_out_torch1;
	err =
	    device_create_file(pchip->cdev_torch1.dev,
			       &dev_attr_ctrl[DEV_TORCH1]);
	if (err < 0)
		goto err_out_torch1_ctrl;

	/* indicator initialize */
	INIT_WORK(&pchip->work_indicator,
		  lm356x_deferred_indicator_brightness_set);
	pchip->cdev_indicator.name = "indicator";
	pchip->cdev_indicator.max_brightness = 7;
	pchip->cdev_indicator.brightness_set = lm356x_indicator_brightness_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &pchip->cdev_indicator);
	if (err < 0)
		goto err_out_indicator;

	err = device_create_file(pchip->cdev_indicator.dev,
				 &dev_attr_ctrl[DEV_INDI]);
	if (err < 0)
		goto err_out_indicator_ctrl;

	if (pchip->type == CHIP_LM3560) {
		/* flash2 initialize */
		INIT_WORK(&pchip->work_flash2,
			  lm356x_deferred_flash2_brightness_set);
		pchip->cdev_flash2.name = "flash2";
		pchip->cdev_flash2.max_brightness = 15;
		pchip->cdev_flash2.brightness_set =
		    lm356x_flash2_brightness_set;
		pchip->cdev_flash2.default_trigger = "flash2";
		err = led_classdev_register((struct device *)
					    &client->dev, &pchip->cdev_flash2);
		if (err < 0)
			goto err_out_flash2;
		err = device_create_file(pchip->cdev_flash2.dev,
					 &dev_attr_ctrl[DEV_FLASH2]);
		if (err < 0)
			goto err_out_flash2_ctrl;
		err = device_create_file(pchip->cdev_flash2.dev,
					 &dev_attr_enable[DEV_FLASH2]);
		if (err < 0)
			goto err_out_flash2_on;

		/* torch2 initialize */
		INIT_WORK(&pchip->work_torch2,
			  lm356x_deferred_torch2_brightness_set);
		pchip->cdev_torch2.name = "torch2";
		pchip->cdev_torch2.max_brightness = 7;
		pchip->cdev_torch2.brightness_set =
		    lm356x_torch2_brightness_set;
		pchip->cdev_torch2.default_trigger = "torch2";
		err = led_classdev_register((struct device *)
					    &client->dev, &pchip->cdev_torch2);
		if (err < 0)
			goto err_out_torch2;
		err = device_create_file(pchip->cdev_torch2.dev,
					 &dev_attr_ctrl[DEV_TORCH2]);
		if (err < 0)
			goto err_out_torch2_ctrl;
		err = device_create_file(pchip->cdev_torch2.dev,
					 &dev_attr_enable[DEV_TORCH2]);
		if (err < 0)
			goto err_out_torch2_enable;
		err = device_create_file(pchip->cdev_flash1.dev,
					 &dev_attr_enable[DEV_FLASH1]);
		if (err < 0)
			goto err_out_flash1_enable;
		err = device_create_file(pchip->cdev_torch1.dev,
					 &dev_attr_enable[DEV_TORCH1]);
		if (err < 0)
			goto err_out_torch1_enable;
		err = device_create_file(pchip->cdev_indicator.dev,
					 &dev_attr_tpulse);
		if (err < 0)
			goto err_out_indicator_tpulse;
		err = device_create_file(pchip->cdev_indicator.dev,
					 &dev_attr_tblank);
		if (err < 0)
			goto err_out_indicator_tblank;
		err = device_create_file(pchip->cdev_indicator.dev,
					 &dev_attr_period);
		if (err < 0)
			goto err_out_indicator_period;
		dev_info(&client->dev, "LM3560 is initialized\n");
	} else {
		dev_info(&client->dev, "LM3561 is initialized\n");
	}

	return 0;

err_out_indicator_period:
	device_remove_file(pchip->cdev_flash1.dev, &dev_attr_tblank);
err_out_indicator_tblank:
	device_remove_file(pchip->cdev_flash1.dev, &dev_attr_tpulse);
err_out_indicator_tpulse:
	device_remove_file(pchip->cdev_flash1.dev,
			   &dev_attr_enable[DEV_TORCH1]);
err_out_torch1_enable:
	device_remove_file(pchip->cdev_flash1.dev,
			   &dev_attr_enable[DEV_FLASH1]);
err_out_flash1_enable:
	device_remove_file(pchip->cdev_torch1.dev,
			   &dev_attr_enable[DEV_TORCH2]);
err_out_torch2_enable:
	device_remove_file(pchip->cdev_torch2.dev, &dev_attr_ctrl[DEV_TORCH2]);
err_out_torch2_ctrl:
	led_classdev_unregister(&pchip->cdev_torch2);
	flush_work(&pchip->work_torch2);
err_out_torch2:
	device_remove_file(pchip->cdev_flash2.dev,
			   &dev_attr_enable[DEV_FLASH2]);
err_out_flash2_on:
	device_remove_file(pchip->cdev_flash2.dev, &dev_attr_ctrl[DEV_FLASH2]);
err_out_flash2_ctrl:
	led_classdev_unregister(&pchip->cdev_flash2);
	flush_work(&pchip->work_flash2);
err_out_flash2:
	device_remove_file(pchip->cdev_indicator.dev, &dev_attr_ctrl[DEV_INDI]);
err_out_indicator_ctrl:
	led_classdev_unregister(&pchip->cdev_indicator);
	flush_work(&pchip->work_indicator);
err_out_indicator:
	device_remove_file(pchip->cdev_torch1.dev, &dev_attr_ctrl[DEV_TORCH1]);
err_out_torch1_ctrl:
	led_classdev_unregister(&pchip->cdev_torch1);
	flush_work(&pchip->work_torch1);
err_out_torch1:
	device_remove_file(pchip->cdev_flash1.dev, &dev_attr_ctrl[DEV_FLASH1]);
err_out_flash1_ctrl:
	led_classdev_unregister(&pchip->cdev_flash1);
	flush_work(&pchip->work_flash1);
err_out_flash1:
	return err;
}

static int lm356x_remove(struct i2c_client *client)
{
	struct lm356x_chip_data *pchip = i2c_get_clientdata(client);

	if (pchip->type == CHIP_LM3560) {
		device_remove_file(pchip->cdev_flash1.dev, &dev_attr_period);
		device_remove_file(pchip->cdev_flash1.dev, &dev_attr_tpulse);
		device_remove_file(pchip->cdev_flash1.dev, &dev_attr_tblank);
		device_remove_file(pchip->cdev_torch1.dev,
				   &dev_attr_enable[DEV_TORCH1]);
		device_remove_file(pchip->cdev_flash1.dev,
				   &dev_attr_enable[DEV_FLASH1]);
		device_remove_file(pchip->cdev_torch2.dev,
				   &dev_attr_enable[DEV_TORCH2]);
		device_remove_file(pchip->cdev_torch2.dev,
				   &dev_attr_ctrl[DEV_TORCH2]);
		led_classdev_unregister(&pchip->cdev_torch2);
		flush_work(&pchip->work_torch2);
		device_remove_file(pchip->cdev_flash2.dev,
				   &dev_attr_enable[DEV_FLASH2]);
		device_remove_file(pchip->cdev_flash2.dev,
				   &dev_attr_ctrl[DEV_FLASH2]);
		led_classdev_unregister(&pchip->cdev_flash2);
		flush_work(&pchip->work_flash2);
	}
	device_remove_file(pchip->cdev_indicator.dev, &dev_attr_ctrl[DEV_INDI]);
	led_classdev_unregister(&pchip->cdev_indicator);
	flush_work(&pchip->work_indicator);
	device_remove_file(pchip->cdev_torch1.dev, &dev_attr_ctrl[DEV_TORCH1]);
	led_classdev_unregister(&pchip->cdev_torch1);
	flush_work(&pchip->work_torch1);
	device_remove_file(pchip->cdev_flash1.dev, &dev_attr_ctrl[DEV_FLASH1]);
	led_classdev_unregister(&pchip->cdev_flash1);
	flush_work(&pchip->work_flash1);
	return 0;
}

static const struct i2c_device_id lm356x_id[] = {
	{LM3560_NAME, CHIP_LM3560},
	{LM3561_NAME, CHIP_LM3561},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm356x_id);

static struct i2c_driver lm356x_i2c_driver = {
	.driver = {
		   .name = LM356x_NAME,
		   .owner = THIS_MODULE,
		   .pm = NULL,
		   },
	.probe = lm356x_probe,
	.remove = lm356x_remove,
	.id_table = lm356x_id,
};

module_i2c_driver(lm356x_i2c_driver);



struct lm356x_platform_data lm356x_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3560={ I2C_BOARD_INFO(LM3560_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), \
													.platform_data = &lm356x_pdata,};

static int __init lm3560_init(void)
{
	printk("%s",__func__);
	i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_LM3560, 1);

}
static void __exit lm3560_exit(void)
{}

module_init(lm3560_init);
module_exit(lm3560_exit);


MODULE_DESCRIPTION("Texas Instruments Flash Lighting driver for LM356x");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
