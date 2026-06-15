/*
 * Copyright (c) 2025 HARDWARIO a.s.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_FLOOD_PROBE_H_
#define APP_FLOOD_PROBE_H_

/* Standard includes */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Flood probe: the HARDWARIO "weight probe" board (ADS122C04 ADC behind a DS28E17
 * 1-Wire-to-I2C bridge, family 0x19) repurposed for a resistive flood/water
 * sensor. The ADS122C04 drives a 1500 uA current source into AIN2 and measures
 * the voltage on AIN2 (single-ended vs AVSS); a wet sensor (low resistance) pulls
 * the voltage down, a dry/open sensor lets it rise toward the rail. */

/* Scan the 1-Wire bus for flood probes. Mirrors app_machine_probe_scan(): a
 * family 0x19 device whose ADS122C04 does not answer is skipped here and left to
 * the machine-probe driver, so the two coexist on one bus. Returns 0 or errno. */
int app_flood_probe_scan(void);

/* Number of flood probes registered by the last scan. */
int app_flood_probe_get_count(void);

/* Read one flood probe's raw ADC sample. index is 0-based (0..count-1). *raw
 * receives the signed 24-bit ADS122C04 count for the AIN2 voltage (sign-extended
 * to int32); the conversion to millivolts happens in the slot layer. *serial_number
 * (may be NULL) receives the 48-bit 1-Wire ROM serial. Returns 0 or errno. */
int app_flood_probe_read(int index, uint64_t *serial_number, int32_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* APP_FLOOD_PROBE_H_ */
