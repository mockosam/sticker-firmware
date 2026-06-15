/*
 * Copyright (c) 2025 HARDWARIO a.s.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Flood probe: an ADS122C04 24-bit ADC behind a DS28E17 1-Wire-to-I2C bridge
 * (1-Wire family 0x19, the HARDWARIO "weight probe" board) wired for a resistive
 * flood/water sensor. A 1500 uA current source (IDAC1) is driven into AIN2 and
 * the voltage on AIN2 is measured single-ended against AVSS; the sensor sits
 * between AIN2 (red, JP4) and GND (black, JP5 -> AIN0). A wet (low-resistance)
 * sensor pulls the voltage down, a dry/open sensor lets it rise toward the rail.
 *
 * Ported from the CHESTER ctr_weight_probe subsystem (same hardware, different
 * ADC configuration). Returns the raw signed 24-bit count; the slot layer
 * converts it to millivolts and the alarm layer thresholds it. */

#include "app_flood_probe.h"
#include "app_w1.h"

/* STICKER includes */
#include <sticker/drivers/w1/ds28e17.h>

/* Zephyr includes */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* Standard includes */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

LOG_MODULE_REGISTER(app_flood_probe, LOG_LEVEL_DBG);

#define ADS122C04_I2C_ADDR  0x40
#define ADS122C04_IDLE_TIME K_MSEC(100)

#define CMD_RESET      0x06
#define CMD_START_SYNC 0x08
#define CMD_POWERDOWN  0x02
#define CMD_RDATA      0x10
#define CMD_RREG       0x20
#define CMD_WREG       0x40

#define REG_CONFIG_0 0x00
#define REG_CONFIG_1 0x01
#define REG_CONFIG_2 0x02
#define REG_CONFIG_3 0x03

#define DEF_CONFIG_0 0x00
#define DEF_CONFIG_1 0x00
#define DEF_CONFIG_2 0x00
#define DEF_CONFIG_3 0x00

#define POS_CONFIG_0_PGA_BYPASS 0
#define POS_CONFIG_0_GAIN	1
#define POS_CONFIG_0_MUX	4
#define POS_CONFIG_1_TS		0
#define POS_CONFIG_1_VREF	1
#define POS_CONFIG_1_CM		3
#define POS_CONFIG_1_MODE	4
#define POS_CONFIG_1_DR		5
#define POS_CONFIG_2_IDAC	0
#define POS_CONFIG_2_BCS	3
#define POS_CONFIG_2_CRC	4
#define POS_CONFIG_2_DCNT	6
#define POS_CONFIG_2_DRDY	7
#define POS_CONFIG_3_I2MUX	2
#define POS_CONFIG_3_I1MUX	5

#define MSK_CONFIG_2_DRDY BIT(7)

#define READ_MAX_RETRIES    100
#define READ_RETRY_DELAY_MS 10

struct sensor {
	uint64_t serial_number;
	const struct device *dev;
};

static K_MUTEX_DEFINE(m_lock);

static struct app_w1 m_w1;

/* One DS28E17 driver instance per discoverable flood probe (ROM is bound at scan
 * time via ds28e17_set_w1_config). Grow together with the devicetree flood_probe_N
 * nodes and APP_W1_SLOT_COUNT if more simultaneous probes are needed. */
static struct sensor m_sensors[] = {
	{.dev = DEVICE_DT_GET(DT_NODELABEL(flood_probe_0))},
	{.dev = DEVICE_DT_GET(DT_NODELABEL(flood_probe_1))},
};

static int m_count;

static int ads122c04_send_cmd(const struct device *dev, uint8_t cmd)
{
	int ret;

	uint8_t write_buf[1];

	write_buf[0] = cmd;

	ret = ds28e17_i2c_write(dev, ADS122C04_I2C_ADDR, write_buf, 1);
	if (ret) {
		LOG_ERR("Call `ds28e17_i2c_write` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int ads122c04_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	int ret;

	uint8_t write_buf[2];

	write_buf[0] = CMD_WREG | (reg & 0x03) << 2;
	write_buf[1] = val;

	ret = ds28e17_i2c_write(dev, ADS122C04_I2C_ADDR, write_buf, 2);
	if (ret) {
		LOG_ERR("Call `ds28e17_i2c_write` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int ads122c04_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	int ret;

	uint8_t write_buf[1];

	write_buf[0] = CMD_RREG | (reg & 0x03) << 2;

	ret = ds28e17_i2c_write_read(dev, ADS122C04_I2C_ADDR, write_buf, 1, val, 1);
	if (ret) {
		LOG_ERR("Call `ds28e17_i2c_write_read` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int ads122c04_read_data(const struct device *dev, uint8_t data[3])
{
	int ret;

	uint8_t write_buf[1];

	write_buf[0] = CMD_RDATA;

	ret = ds28e17_i2c_write_read(dev, ADS122C04_I2C_ADDR, write_buf, 1, data, 3);
	if (ret) {
		LOG_ERR("Call `ds28e17_i2c_write_read` failed: %d", ret);
		return ret;
	}

	return 0;
}

static int ads122c04_init(const struct device *dev, bool keep_powered)
{
	int ret;

	ret = ads122c04_send_cmd(dev, CMD_RESET);
	if (ret) {
		LOG_ERR("Call `ads122c04_send_cmd` (CMD_RESET) failed: %d", ret);
		return ret;
	}

	uint8_t reg_config_0 = DEF_CONFIG_0;
	uint8_t reg_config_1 = DEF_CONFIG_1;
	uint8_t reg_config_2 = DEF_CONFIG_2;
	uint8_t reg_config_3 = DEF_CONFIG_3;

	/* Flood-sensor front end: measure the voltage on AIN2 (single-ended vs AVSS)
	 * while sourcing 1500 uA into AIN2. Gain 1 with the PGA bypassed — the input
	 * is volts (I * R_sensor), not the millivolts a load-cell bridge produces, so
	 * the gain-128 weight config would saturate immediately. PGA bypass is also
	 * required by the ADS122C04 for single-ended (AINn = AVSS) measurements. */
	reg_config_0 |= 0xA << POS_CONFIG_0_MUX;      /* AINp = AIN2, AINn = AVSS */
	reg_config_0 |= 1 << POS_CONFIG_0_PGA_BYPASS; /* PGA disabled -> gain 1 */
	reg_config_1 |= 4 << POS_CONFIG_1_DR;         /* data rate 20 SPS */
	reg_config_1 |= 1 << POS_CONFIG_1_VREF;       /* internal 2048 mV reference */
	if (keep_powered) {
		reg_config_1 |= 1 << POS_CONFIG_1_CM; /* continuous conversion (IDAC stays on) */
	}
	reg_config_2 |= 7 << POS_CONFIG_2_IDAC;       /* IDAC 1500 uA */
	reg_config_3 |= 3 << POS_CONFIG_3_I1MUX;      /* IDAC1 -> AIN2 (sensor excitation) */

	ret = ads122c04_write_reg(dev, REG_CONFIG_0, reg_config_0);
	if (ret) {
		LOG_ERR("Call `ads122c04_write_reg` (REG_CONFIG_0) failed: %d", ret);
		return ret;
	}

	ret = ads122c04_write_reg(dev, REG_CONFIG_1, reg_config_1);
	if (ret) {
		LOG_ERR("Call `ads122c04_write_reg` (REG_CONFIG_1) failed: %d", ret);
		return ret;
	}

	ret = ads122c04_write_reg(dev, REG_CONFIG_2, reg_config_2);
	if (ret) {
		LOG_ERR("Call `ads122c04_write_reg` (REG_CONFIG_2) failed: %d", ret);
		return ret;
	}

	ret = ads122c04_write_reg(dev, REG_CONFIG_3, reg_config_3);
	if (ret) {
		LOG_ERR("Call `ads122c04_write_reg` (REG_CONFIG_3) failed: %d", ret);
		return ret;
	}

	if (!keep_powered) {
		ret = ads122c04_send_cmd(dev, CMD_POWERDOWN);
		if (ret) {
			LOG_ERR("Call `ads122c04_send_cmd` (CMD_POWERDOWN) failed: %d", ret);
			return ret;
		}
	}

	return 0;
}

static int ads122c04_read(const struct device *dev, uint8_t data[3])
{
	int ret;

	ret = ads122c04_send_cmd(dev, CMD_START_SYNC);
	if (ret) {
		LOG_ERR("Call `ads122c04_send_cmd` (CMD_START_SYNC) failed: %d", ret);
		goto error;
	}

	k_sleep(ADS122C04_IDLE_TIME);

	ret = ads122c04_send_cmd(dev, CMD_START_SYNC);
	if (ret) {
		LOG_ERR("Call `ads122c04_send_cmd` (CMD_START_SYNC) failed: %d", ret);
		goto error;
	}

	int retries = 0;

	for (;;) {
		uint8_t val;
		ret = ads122c04_read_reg(dev, REG_CONFIG_2, &val);
		if (ret) {
			LOG_ERR("Call `ads122c04_read_reg` (REG_CONFIG_2) failed: %d", ret);
			goto error;
		}

		if (val & MSK_CONFIG_2_DRDY) {
			break;
		}

		if (++retries >= READ_MAX_RETRIES) {
			LOG_ERR("Reached maximum DRDY poll attempts");
			ret = -EIO;
			goto error;
		}

		k_msleep(READ_RETRY_DELAY_MS);
	}

	ret = ads122c04_read_data(dev, data);
	if (ret) {
		LOG_ERR("Call `ads122c04_read_data` failed: %d", ret);
		goto error;
	}

	ret = ads122c04_send_cmd(dev, CMD_POWERDOWN);
	if (ret) {
		LOG_ERR("Call `ads122c04_send_cmd` (CMD_POWERDOWN) failed: %d", ret);
		goto error;
	}

	return 0;

error:
	ads122c04_send_cmd(dev, CMD_POWERDOWN);

	return ret;
}

static int scan_callback(struct w1_rom rom, void *user_data)
{
	int ret;

	if (rom.family != 0x19) {
		return 0;
	}

	uint64_t serial_number = sys_get_le48(rom.serial);

	if (m_count >= ARRAY_SIZE(m_sensors)) {
		LOG_WRN("No more space for additional device: %llu", serial_number);
		return 0;
	}

	if (!device_is_ready(m_sensors[m_count].dev)) {
		LOG_ERR("Device not ready");
		return -ENODEV;
	}

	struct w1_slave_config config = {.rom = rom};
	ret = ds28e17_set_w1_config(m_sensors[m_count].dev, config);
	if (ret) {
		LOG_ERR("Call `ds28e17_set_w1_config` failed: %d", ret);
		return ret;
	}

	ret = ds28e17_write_config(m_sensors[m_count].dev, DS28E17_I2C_SPEED_100_KHZ);
	if (ret) {
		LOG_ERR("Call `ds28e17_write_config` failed: %d", ret);
		return ret;
	}

	/* Discriminate flood probes from machine probes — both are DS28E17 (family
	 * 0x19). A flood probe answers at the ADS122C04 I2C address; a machine probe
	 * does not, so it is skipped here and registered by app_machine_probe_scan()
	 * (which conversely requires its LIS2DH12 to answer). */
	if (ads122c04_send_cmd(m_sensors[m_count].dev, CMD_POWERDOWN)) {
		LOG_DBG("Skipping serial number: %llu", serial_number);
		return 0;
	}

	m_sensors[m_count++].serial_number = serial_number;

	LOG_DBG("Registered serial number: %llu", serial_number);

	return 0;
}

int app_flood_probe_scan(void)
{
	int ret;
	int res = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&m_lock, K_FOREVER);

	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ds2484));

	if (!device_is_ready(dev)) {
		LOG_ERR("Device not ready");
		k_mutex_unlock(&m_lock);
		return -ENODEV;
	}

	ret = app_w1_acquire(&m_w1, dev);
	if (ret) {
		LOG_ERR("Call `app_w1_acquire` failed: %d", ret);
		res = ret;
		goto error;
	}

	m_count = 0;

	ret = app_w1_scan(&m_w1, dev, scan_callback, NULL);
	if (ret < 0) {
		LOG_ERR("Call `app_w1_scan` failed: %d", ret);
		res = ret;
		goto error;
	}

error:
	ret = app_w1_release(&m_w1, dev);
	if (ret) {
		LOG_ERR("Call `app_w1_release` failed: %d", ret);
		res = res ? res : ret;
	}

	k_mutex_unlock(&m_lock);

	return res;
}

int app_flood_probe_get_count(void)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&m_lock, K_FOREVER);
	int count = m_count;
	k_mutex_unlock(&m_lock);

	return count;
}

int app_flood_probe_read(int index, uint64_t *serial_number, int32_t *raw)
{
	int ret;
	int res = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&m_lock, K_FOREVER);

	if (index < 0 || index >= m_count || !m_count) {
		k_mutex_unlock(&m_lock);
		return -ERANGE;
	}

	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ds2484));

	if (!device_is_ready(dev)) {
		LOG_ERR("Device not ready");
		k_mutex_unlock(&m_lock);
		return -ENODEV;
	}

	if (serial_number) {
		*serial_number = m_sensors[index].serial_number;
	}

	ret = app_w1_acquire(&m_w1, dev);
	if (ret) {
		LOG_ERR("Call `app_w1_acquire` failed: %d", ret);
		res = ret;
		goto error;
	}

	if (!device_is_ready(m_sensors[index].dev)) {
		LOG_ERR("Device not ready");
		res = -ENODEV;
		goto error;
	}

	ret = ds28e17_write_config(m_sensors[index].dev, DS28E17_I2C_SPEED_100_KHZ);
	if (ret) {
		LOG_ERR("Call `ds28e17_write_config` failed: %d", ret);
		res = ret;
		goto error;
	}

	ret = ads122c04_init(m_sensors[index].dev, false);
	if (ret) {
		LOG_ERR("Call `ads122c04_init` failed: %d", ret);
		res = ret;
		goto error;
	}

	uint8_t data[3];
	ret = ads122c04_read(m_sensors[index].dev, data);
	if (ret) {
		LOG_ERR("Call `ads122c04_read` failed: %d", ret);
		res = ret;
		goto error;
	}

	/* 24-bit two's-complement sample, MSB first. Sign-extend into int32. */
	*raw = sys_get_be24(data);
	*raw <<= 8;
	*raw >>= 8;

error:
	ret = app_w1_release(&m_w1, dev);
	if (ret) {
		LOG_ERR("Call `app_w1_release` failed: %d", ret);
		res = res ? res : ret;
	}

	k_mutex_unlock(&m_lock);

	return res;
}

#if defined(CONFIG_SHELL)

#include <zephyr/shell/shell.h>

/* Standard includes */
#include <stdlib.h>

/* Bring-up: hold the 1.5 mA IDAC on AIN2 continuously (no power-down between
 * conversions) and stream the AIN2 voltage once a second, so the LD-81
 * flood/no-flood dynamics can be verified live (multimeter + short / water test).
 * Self-contained: re-scans the bus first, so no prior enroll is needed. Holds the
 * 1-Wire bus for the whole run — a bring-up tool, not for production. */
static int cmd_flood_test(const struct shell *shell, size_t argc, char **argv)
{
	int seconds = (argc >= 2) ? atoi(argv[1]) : 30;

	if (seconds < 1) {
		seconds = 1;
	} else if (seconds > 600) {
		seconds = 600;
	}

	int ret = app_flood_probe_scan();
	if (ret) {
		shell_error(shell, "scan failed: %d", ret);
		return ret;
	}
	if (app_flood_probe_get_count() <= 0) {
		shell_error(shell, "no flood probe on bus (check the 1-Wire connection)");
		return -ENODEV;
	}

	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ds2484));
	int res = 0;

	k_mutex_lock(&m_lock, K_FOREVER);

	const struct device *probe = m_sensors[0].dev;

	if (!device_is_ready(dev) || !device_is_ready(probe)) {
		k_mutex_unlock(&m_lock);
		shell_error(shell, "device not ready");
		return -ENODEV;
	}

	ret = app_w1_acquire(&m_w1, dev);
	if (ret) {
		k_mutex_unlock(&m_lock);
		shell_error(shell, "w1 acquire failed: %d", ret);
		return ret;
	}

	ret = ds28e17_write_config(probe, DS28E17_I2C_SPEED_100_KHZ);
	if (ret) {
		res = ret;
		goto out;
	}

	/* keep_powered = true: continuous conversion, IDAC sources 1.5 mA the whole run. */
	ret = ads122c04_init(probe, true);
	if (ret) {
		res = ret;
		goto out;
	}

	ret = ads122c04_send_cmd(probe, CMD_START_SYNC);
	if (ret) {
		res = ret;
		goto out;
	}

	shell_print(shell, "flood test: IDAC 1500 uA on AIN2, %d s", seconds);
	shell_print(shell, "  t[s]        raw     mV");

	for (int s = 0; s < seconds; s++) {
		int retries = 0;
		uint8_t cfg2;

		for (;;) {
			ret = ads122c04_read_reg(probe, REG_CONFIG_2, &cfg2);
			if (ret) {
				res = ret;
				goto out;
			}
			if (cfg2 & MSK_CONFIG_2_DRDY) {
				break;
			}
			if (++retries >= READ_MAX_RETRIES) {
				res = -EIO;
				goto out;
			}
			k_msleep(READ_RETRY_DELAY_MS);
		}

		uint8_t data[3];
		ret = ads122c04_read_data(probe, data);
		if (ret) {
			res = ret;
			goto out;
		}

		int32_t raw = sys_get_be24(data);
		raw <<= 8;
		raw >>= 8;

		shell_print(shell, "  %4d  %10ld  %5ld", s, (long)raw, (long)(raw / 4096));
		k_sleep(K_SECONDS(1));
	}

out:
	ads122c04_send_cmd(probe, CMD_POWERDOWN);
	ret = app_w1_release(&m_w1, dev);
	if (ret && !res) {
		res = ret;
	}
	k_mutex_unlock(&m_lock);

	if (res) {
		shell_error(shell, "flood test error: %d", res);
	}
	return res;
}

/* Bring-up: power the 1-Wire bus and hold it on, so the bus / probe VDD can be
 * measured with a meter (the bus is suspended at idle, so an idle measurement is
 * meaningless). Reports the reset presence pulse — a definitive check of whether
 * ANY 1-Wire device answers, independent of family code. */
static int cmd_flood_power(const struct shell *shell, size_t argc, char **argv)
{
	int seconds = (argc >= 2) ? atoi(argv[1]) : 20;

	if (seconds < 1) {
		seconds = 1;
	} else if (seconds > 120) {
		seconds = 120;
	}

	static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ds2484));

	if (!device_is_ready(dev)) {
		shell_error(shell, "ds2484 not ready (is cap-w1-sensors enabled?)");
		return -ENODEV;
	}

	k_mutex_lock(&m_lock, K_FOREVER);

	int ret = app_w1_acquire(&m_w1, dev);
	if (ret) {
		k_mutex_unlock(&m_lock);
		shell_error(shell, "w1 acquire failed: %d", ret);
		return ret;
	}

	int presence = w1_reset_bus(dev);

	shell_print(shell, "1-Wire reset presence pulse: %s (ret %d)",
		    presence == 1   ? "DETECTED (a device answers)"
		    : presence == 0 ? "NONE (no device answers)"
				    : "error",
		    presence);
	shell_print(shell, "bus held powered for %d s — measure VPU / probe VDD now", seconds);

	k_sleep(K_SECONDS(seconds));

	(void)app_w1_release(&m_w1, dev);
	k_mutex_unlock(&m_lock);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_flood,
	SHELL_CMD_ARG(test, NULL,
		      "Hold 1.5 mA IDAC on AIN2 and stream the voltage. Usage: flood test [seconds]",
		      cmd_flood_test, 1, 1),
	SHELL_CMD_ARG(power, NULL,
		      "Power + hold the 1-Wire bus and report the reset presence pulse. "
		      "Usage: flood power [seconds]",
		      cmd_flood_power, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(flood, &sub_flood, "Flood probe bring-up (ADS122C04 current-source test).", NULL);

#endif /* defined(CONFIG_SHELL) */
