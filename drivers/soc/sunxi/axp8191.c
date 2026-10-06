// SPDX-License-Identifier: GPL-2.0
/*
 * X-Powers AXP8191 PMIC helper driver for Allwinner A733
 *
 * Ensures BLDO2 (1.8V for VCC-PJ and Audio ES8388) and BLDO1 (1.8V) are enabled
 * on Orange Pi 4 Pro.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#define AXP8191_DCDC_CTL2	0x11
#define AXP8191_LDO_ON_OFF1	0x20
#define AXP8191_LDO_ON_OFF2	0x21
#define AXP8191_BLDO1_VOL	0x2a
#define AXP8191_BLDO2_VOL	0x2b
#define AXP8191_BLDO4_VOL	0x2d

static int axp8191_probe(struct i2c_client *client)
{
	int ret, val;

	/* Ensure DC1SW1 (bit 3, VCC-EPHY) and DC1SW2 (bit 4, BLDO-INPUT) are enabled */
	val = i2c_smbus_read_byte_data(client, AXP8191_DCDC_CTL2);
	if (val >= 0) {
		val |= BIT(4) | BIT(3);
		ret = i2c_smbus_write_byte_data(client, AXP8191_DCDC_CTL2, val);
		if (ret < 0)
			dev_warn(&client->dev, "failed to set DCDC_CTL2: %d\n", ret);
	}

	/* Configure BLDO2 to 1.8V (500mV base + 13 * 100mV = 1800mV, 0x0d for VCC-PJ) */
	ret = i2c_smbus_write_byte_data(client, AXP8191_BLDO2_VOL, 0x0d);
	if (ret < 0)
		dev_warn(&client->dev, "failed to set BLDO2 voltage: %d\n", ret);

	/* Configure BLDO1 to 1.8V (VCC-PK) */
	ret = i2c_smbus_write_byte_data(client, AXP8191_BLDO1_VOL, 0x0d);
	if (ret < 0)
		dev_warn(&client->dev, "failed to set BLDO1 voltage: %d\n", ret);

	/* Configure BLDO4 to 1.8V (VCC-1V8-AUDIO / ES8388 DVDD/PVDD) */
	ret = i2c_smbus_write_byte_data(client, AXP8191_BLDO4_VOL, 0x0d);
	if (ret < 0)
		dev_warn(&client->dev, "failed to set BLDO4 voltage: %d\n", ret);

	/* Read LDO on/off control register 1 */
	val = i2c_smbus_read_byte_data(client, AXP8191_LDO_ON_OFF1);
	if (val < 0) {
		dev_err(&client->dev, "failed to read LDO_ON_OFF1: %d\n", val);
		return val;
	}

	/* Enable BLDO2 (bit 7) and BLDO1 (bit 6) */
	val |= BIT(7) | BIT(6);
	ret = i2c_smbus_write_byte_data(client, AXP8191_LDO_ON_OFF1, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write LDO_ON_OFF1: %d\n", ret);
		return ret;
	}

	/* Read LDO on/off control register 2 */
	val = i2c_smbus_read_byte_data(client, AXP8191_LDO_ON_OFF2);
	if (val >= 0) {
		/* Enable BLDO4 (bit 1) */
		val |= BIT(1);
		ret = i2c_smbus_write_byte_data(client, AXP8191_LDO_ON_OFF2, val);
		if (ret < 0)
			dev_warn(&client->dev, "failed to write LDO_ON_OFF2: %d\n", ret);
	}

	dev_info(&client->dev, "AXP8191 PMIC: BLDO1/2/4 enabled at 1.8V, DC1SW1/2 enabled\n");
	return 0;
}

static const struct of_device_id axp8191_of_match[] = {
	{ .compatible = "x-powers,axp8191" },
	{ }
};
MODULE_DEVICE_TABLE(of, axp8191_of_match);

static struct i2c_driver axp8191_driver = {
	.driver = {
		.name = "axp8191",
		.of_match_table = axp8191_of_match,
	},
	.probe = axp8191_probe,
};
module_i2c_driver(axp8191_driver);

MODULE_DESCRIPTION("X-Powers AXP8191 PMIC Helper Driver");
MODULE_LICENSE("GPL");
