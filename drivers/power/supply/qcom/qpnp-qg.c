/* Copyright (c) 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"QG-K: %s: " fmt, __func__

#include <linux/alarmtimer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_batterydata.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/pmic-voter.h>
#include <linux/qpnp/qpnp-adc.h>
#include <uapi/linux/qg.h>
#include <uapi/linux/qg-profile.h>
#include "fg-alg.h"
#include "qg-sdam.h"
#include "qg-core.h"
#include "qg-reg.h"
#include "qg-util.h"
#include "qg-soc.h"
#include "qg-battery-profile.h"
#include "qg-defs.h"

#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-04-25  Add for OPPO_CHARGE */
#include "../../oppo/oppo_gauge.h"
#include "../../oppo/oppo_charger.h"
#include <soc/oppo/boot_mode.h>
#include <soc/oppo/device_info.h>
#include <linux/delay.h>

static bool use_qpnp_qg = true;
static struct qpnp_qg *the_chip = NULL;
#endif

static int qg_debug_mask;
module_param_named(
	debug_mask, qg_debug_mask, int, 0600
);

#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
static bool healthd_ready = false;

static bool qg_batt_valid_ocv = false;
module_param_named(batt_valid_ocv, qg_batt_valid_ocv, bool, S_IRUSR | S_IWUSR);

static int qg_batt_range_pct = 20;
module_param_named(batt_range_pct, qg_batt_range_pct, int, S_IRUSR | S_IWUSR);

enum batt_info_params {
	BATT_INFO_NOTIFY = 0,
	BATT_INFO_SOC,
	BATT_INFO_RES_ID,
	BATT_INFO_VOLTAGE,
	BATT_INFO_TEMP,
	BATT_INFO_FCC,
	BATT_INFO_MAX,
};
#endif

static bool is_battery_present(struct qpnp_qg *chip)
{
	u8 reg = 0;
	int rc;

	rc = qg_read(chip, chip->qg_base + QG_STATUS1_REG, &reg, 1);
	if (rc < 0)
		pr_err("Failed to read battery presence, rc=%d\n", rc);

	return !!(reg & BATTERY_PRESENT_BIT);
}

#define DEBUG_BATT_ID_LOW	6000
#define DEBUG_BATT_ID_HIGH	8500
static bool is_debug_batt_id(struct qpnp_qg *chip)
{
#ifndef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-04-24  use oppo BAT_ID */
	if (is_between(DEBUG_BATT_ID_LOW, DEBUG_BATT_ID_HIGH,
					chip->batt_id_ohm))
		return true;
#endif

	return false;
}

static int qg_read_ocv(struct qpnp_qg *chip, u32 *ocv_uv, u32 *ocv_raw, u8 type)
{
	int rc, addr;
	u64 temp = 0;
	char ocv_name[20];

	switch (type) {
	case S3_GOOD_OCV:
		addr = QG_S3_GOOD_OCV_V_DATA0_REG;
		strlcpy(ocv_name, "S3_GOOD_OCV", 20);
		break;
	case S7_PON_OCV:
		addr = QG_S7_PON_OCV_V_DATA0_REG;
		strlcpy(ocv_name, "S7_PON_OCV", 20);
		break;
	case S3_LAST_OCV:
		addr = QG_LAST_S3_SLEEP_V_DATA0_REG;
		strlcpy(ocv_name, "S3_LAST_OCV", 20);
		break;
	case SDAM_PON_OCV:
		addr = QG_SDAM_PON_OCV_OFFSET;
		strlcpy(ocv_name, "SDAM_PON_OCV", 20);
		break;
	default:
		pr_err("Invalid OCV type %d\n", type);
		return -EINVAL;
	}

	if (type == SDAM_PON_OCV) {
		rc = qg_sdam_read(SDAM_PON_OCV_UV, ocv_raw);
		if (rc < 0) {
			pr_err("Failed to read SDAM PON OCV rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = qg_read(chip, chip->qg_base + addr, (u8 *)ocv_raw, 2);
		if (rc < 0) {
			pr_err("Failed to read ocv, rc=%d\n", rc);
			return rc;
		}
	}

	temp = *ocv_raw;
	*ocv_uv = V_RAW_TO_UV(temp);

	pr_debug("%s: OCV_RAW=%x OCV=%duV\n", ocv_name, *ocv_raw, *ocv_uv);

	return rc;
}

#define DEFAULT_S3_FIFO_LENGTH		3
static int qg_update_fifo_length(struct qpnp_qg *chip, u8 length)
{
	int rc;
	u8 s3_entry_fifo_length = 0;

	if (!length || length > 8) {
		pr_err("Invalid FIFO length %d\n", length);
		return -EINVAL;
	}

	rc = qg_masked_write(chip, chip->qg_base + QG_S2_NORMAL_MEAS_CTL2_REG,
			FIFO_LENGTH_MASK, (length - 1) << FIFO_LENGTH_SHIFT);
	if (rc < 0)
		pr_err("Failed to write S2 FIFO length, rc=%d\n", rc);

	/* update the S3 FIFO length, when S2 length is updated */
	if (length > 3)
		s3_entry_fifo_length = (chip->dt.s3_entry_fifo_length > 0) ?
			chip->dt.s3_entry_fifo_length : DEFAULT_S3_FIFO_LENGTH;
	else	/* Use S3 length as 1 for any S2 length <= 3 */
		s3_entry_fifo_length = 1;

	rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK,
			s3_entry_fifo_length - 1);
	if (rc < 0)
		pr_err("Failed to write S3-entry fifo-length, rc=%d\n",
						rc);

	return rc;
}

static int qg_master_hold(struct qpnp_qg *chip, bool hold)
{
	int rc;

	/* clear the master */
	rc = qg_masked_write(chip, chip->qg_base + QG_DATA_CTL1_REG,
					MASTER_HOLD_OR_CLR_BIT, 0);
	if (rc < 0)
		return rc;

	if (hold) {
		/* 0 -> 1, hold the master */
		rc = qg_masked_write(chip, chip->qg_base + QG_DATA_CTL1_REG,
					MASTER_HOLD_OR_CLR_BIT,
					MASTER_HOLD_OR_CLR_BIT);
		if (rc < 0)
			return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS, "Master hold = %d\n", hold);

	return rc;
}

static void qg_notify_charger(struct qpnp_qg *chip)
{
	union power_supply_propval prop = {0, };
	int rc;

	if (!chip->batt_psy)
		return;

	if (is_debug_batt_id(chip)) {
		prop.intval = 1;
		power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &prop);
		return;
	}

	if (!chip->profile_loaded)
		return;

	prop.intval = chip->bp.float_volt_uv;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX, &prop);
	if (rc < 0) {
		pr_err("Failed to set voltage_max property on batt_psy, rc=%d\n",
			rc);
		return;
	}

	prop.intval = chip->bp.fastchg_curr_ma * 1000;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &prop);
	if (rc < 0) {
		pr_err("Failed to set constant_charge_current_max property on batt_psy, rc=%d\n",
			rc);
		return;
	}

	pr_debug("Notified charger on float voltage and FCC\n");
}

static bool is_batt_available(struct qpnp_qg *chip)
{
	if (chip->batt_psy)
		return true;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy)
		return false;

	/* batt_psy is initialized, set the fcc and fv */
	qg_notify_charger(chip);

	return true;
}

static int qg_update_sdam_params(struct qpnp_qg *chip)
{
	int rc, batt_temp = 0, i;
	unsigned long rtc_sec = 0;

	rc = get_rtc_time(&rtc_sec);
	if (rc < 0)
		pr_err("Failed to get RTC time, rc=%d\n", rc);
	else
		chip->sdam_data[SDAM_TIME_SEC] = rtc_sec;

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc < 0)
		pr_err("Failed to get battery-temp, rc = %d\n", rc);
	else
		chip->sdam_data[SDAM_TEMP] = (u32)batt_temp;

	rc = qg_sdam_write_all(chip->sdam_data);
	if (rc < 0)
		pr_err("Failed to write to SDAM rc=%d\n", rc);

	for (i = 0; i < SDAM_MAX; i++)
		qg_dbg(chip, QG_DEBUG_STATUS, "SDAM write param %d value=%d\n",
					i, chip->sdam_data[i]);

	return rc;
}

static int qg_process_fifo(struct qpnp_qg *chip, u32 fifo_length)
{
	int rc = 0, i, j = 0, temp;
	u8 v_fifo[MAX_FIFO_LENGTH * 2], i_fifo[MAX_FIFO_LENGTH * 2];
	u32 sample_interval = 0, sample_count = 0, fifo_v = 0, fifo_i = 0;

	chip->kdata.fifo_time = (u32)ktime_get_seconds();

	if (!fifo_length) {
		pr_debug("No FIFO data\n");
		return 0;
	}

	qg_dbg(chip, QG_DEBUG_FIFO, "FIFO length=%d\n", fifo_length);

	rc = get_sample_interval(chip, &sample_interval);
	if (rc < 0) {
		pr_err("Failed to get FIFO sample interval, rc=%d\n", rc);
		return rc;
	}

	rc = get_sample_count(chip, &sample_count);
	if (rc < 0) {
		pr_err("Failed to get FIFO sample count, rc=%d\n", rc);
		return rc;
	}

	/*
	 * If there is pending data from suspend, append the new FIFO
	 * data to it.
	 */
	if (chip->suspend_data) {
		j = chip->kdata.fifo_length; /* append the data */
		chip->suspend_data = false;
		qg_dbg(chip, QG_DEBUG_FIFO,
			"Pending suspend-data FIFO length=%d\n", j);
	} else {
		/* clear any old pending data */
		chip->kdata.fifo_length = 0;
	}

	for (i = 0; i < fifo_length * 2; i = i + 2, j++) {
		rc = qg_read(chip, chip->qg_base + QG_V_FIFO0_DATA0_REG + i,
					&v_fifo[i], 2);
		if (rc < 0) {
			pr_err("Failed to read QG_V_FIFO, rc=%d\n", rc);
			return rc;
		}
		rc = qg_read(chip, chip->qg_base + QG_I_FIFO0_DATA0_REG + i,
					&i_fifo[i], 2);
		if (rc < 0) {
			pr_err("Failed to read QG_I_FIFO, rc=%d\n", rc);
			return rc;
		}

		fifo_v = v_fifo[i] | (v_fifo[i + 1] << 8);
		fifo_i = i_fifo[i] | (i_fifo[i + 1] << 8);

		if (fifo_v == FIFO_V_RESET_VAL || fifo_i == FIFO_I_RESET_VAL) {
			pr_err("Invalid FIFO data V_RAW=%x I_RAW=%x - FIFO rejected\n",
						fifo_v, fifo_i);
			return -EINVAL;
		}

		temp = sign_extend32(fifo_i, 15);

		chip->kdata.fifo[j].v = V_RAW_TO_UV(fifo_v);
		chip->kdata.fifo[j].i = I_RAW_TO_UA(temp);
		chip->kdata.fifo[j].interval = sample_interval;
		chip->kdata.fifo[j].count = sample_count;

		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO %d raw_v=%d uV=%d raw_i=%d uA=%d interval=%d count=%d\n",
					j, fifo_v,
					chip->kdata.fifo[j].v,
					fifo_i,
					(int)chip->kdata.fifo[j].i,
					chip->kdata.fifo[j].interval,
					chip->kdata.fifo[j].count);
	}

	chip->kdata.fifo_length += fifo_length;
	chip->kdata.seq_no = chip->seq_no++ % U32_MAX;

	return rc;
}

static int qg_process_accumulator(struct qpnp_qg *chip)
{
	int rc, sample_interval = 0;
	u8 count, index = chip->kdata.fifo_length;
	u64 acc_v = 0, acc_i = 0;
	s64 temp = 0;

	rc = qg_read(chip, chip->qg_base + QG_ACCUM_CNT_RT_REG,
			&count, 1);
	if (rc < 0) {
		pr_err("Failed to read ACC count, rc=%d\n", rc);
		return rc;
	}

	if (!count) {
		pr_debug("No ACCUMULATOR data!\n");
		return 0;
	}

	rc = get_sample_interval(chip, &sample_interval);
	if (rc < 0) {
		pr_err("Failed to get ACC sample interval, rc=%d\n", rc);
		return 0;
	}

	rc = qg_read(chip, chip->qg_base + QG_V_ACCUM_DATA0_RT_REG,
			(u8 *)&acc_v, 3);
	if (rc < 0) {
		pr_err("Failed to read ACC RT V data, rc=%d\n", rc);
		return rc;
	}

	rc = qg_read(chip, chip->qg_base + QG_I_ACCUM_DATA0_RT_REG,
			(u8 *)&acc_i, 3);
	if (rc < 0) {
		pr_err("Failed to read ACC RT I data, rc=%d\n", rc);
		return rc;
	}

	temp = sign_extend64(acc_i, 23);

	chip->kdata.fifo[index].v = V_RAW_TO_UV(div_u64(acc_v, count));
	chip->kdata.fifo[index].i = I_RAW_TO_UA(div_s64(temp, count));
	chip->kdata.fifo[index].interval = sample_interval;
	chip->kdata.fifo[index].count = count;
	chip->kdata.fifo_length++;

	if (chip->kdata.fifo_length == 1)	/* Only accumulator data */
		chip->kdata.seq_no = chip->seq_no++ % U32_MAX;

	qg_dbg(chip, QG_DEBUG_FIFO, "ACC v_avg=%duV i_avg=%duA interval=%d count=%d\n",
			chip->kdata.fifo[index].v,
			(int)chip->kdata.fifo[index].i,
			chip->kdata.fifo[index].interval,
			chip->kdata.fifo[index].count);

	return rc;
}

static int qg_process_rt_fifo(struct qpnp_qg *chip)
{
	int rc;
	u32 fifo_length = 0;

	/* Get the real-time FIFO length */
	rc = get_fifo_length(chip, &fifo_length, true);
	if (rc < 0) {
		pr_err("Failed to read RT FIFO length, rc=%d\n", rc);
		return rc;
	}

	rc = qg_process_fifo(chip, fifo_length);
	if (rc < 0) {
		pr_err("Failed to process FIFO data, rc=%d\n", rc);
		return rc;
	}

	rc = qg_process_accumulator(chip);
	if (rc < 0) {
		pr_err("Failed to process ACC data, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define VBAT_LOW_HYST_UV		50000 /* 50mV */
static int qg_vbat_low_wa(struct qpnp_qg *chip)
{
	int rc, i, temp = 0;
	u32 vbat_low_uv = 0;

	rc = qg_get_battery_temp(chip, &temp);
	if (rc < 0) {
		pr_err("Failed to read batt_temp rc=%d\n", rc);
		temp = 250;
	}

	vbat_low_uv = 1000 * ((temp < chip->dt.cold_temp_threshold) ?
				chip->dt.vbatt_low_cold_mv :
				chip->dt.vbatt_low_mv);
	vbat_low_uv += VBAT_LOW_HYST_UV;

	if (!(chip->wa_flags & QG_VBAT_LOW_WA) || !chip->vbat_low)
		return 0;

	/*
	 * PMI632 1.0 does not generate a falling VBAT_LOW IRQ.
	 * To exit from VBAT_LOW config, check if any of the FIFO
	 * averages is > vbat_low threshold and reconfigure the
	 * FIFO length to normal.
	 */
	for (i = 0; i < chip->kdata.fifo_length; i++) {
		if (chip->kdata.fifo[i].v > vbat_low_uv) {
			rc = qg_master_hold(chip, true);
			if (rc < 0) {
				pr_err("Failed to hold master, rc=%d\n", rc);
				goto done;
			}
			rc = qg_update_fifo_length(chip,
					chip->dt.s2_fifo_length);
			if (rc < 0)
				goto done;

			rc = qg_master_hold(chip, false);
			if (rc < 0) {
				pr_err("Failed to release master, rc=%d\n", rc);
				goto done;
			}
			/* FIFOs restarted */
			chip->last_fifo_update_time = ktime_get();

			chip->vbat_low = false;
			pr_info("Exit VBAT_LOW vbat_avg=%duV vbat_low=%duV updated fifo_length=%d\n",
					chip->kdata.fifo[i].v, vbat_low_uv,
					chip->dt.s2_fifo_length);
			break;
		}
	}

	return 0;

done:
	qg_master_hold(chip, false);
	return rc;
}

static int qg_vbat_thresholds_config(struct qpnp_qg *chip)
{
	int rc, temp = 0, vbat_mv;
	u8 reg;

	rc = qg_get_battery_temp(chip, &temp);
	if (rc < 0) {
		pr_err("Failed to read batt_temp rc=%d\n", rc);
		return rc;
	}

	vbat_mv = (temp < chip->dt.cold_temp_threshold) ?
			chip->dt.vbatt_empty_cold_mv :
			chip->dt.vbatt_empty_mv;

	rc = qg_read(chip, chip->qg_base + QG_VBAT_EMPTY_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to read vbat-empty, rc=%d\n", rc);
		return rc;
	}

	if (vbat_mv == (reg * 50))	/* No change */
		goto config_vbat_low;

	reg = vbat_mv / 50;
	rc = qg_write(chip, chip->qg_base + QG_VBAT_EMPTY_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to write vbat-empty, rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS,
		"VBAT EMPTY threshold updated to %dmV temp=%d\n",
						vbat_mv, temp);

config_vbat_low:
	vbat_mv = (temp < chip->dt.cold_temp_threshold) ?
			chip->dt.vbatt_low_cold_mv :
			chip->dt.vbatt_low_mv;

	rc = qg_read(chip, chip->qg_base + QG_VBAT_LOW_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to read vbat-low, rc=%d\n", rc);
		return rc;
	}

	if (vbat_mv == (reg * 50))	/* No change */
		return 0;

	reg = vbat_mv / 50;
	rc = qg_write(chip, chip->qg_base + QG_VBAT_LOW_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to write vbat-low, rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS,
		"VBAT LOW threshold updated to %dmV temp=%d\n",
						vbat_mv, temp);

	return rc;
}

#define MIN_FIFO_FULL_TIME_MS			12000
static int process_rt_fifo_data(struct qpnp_qg *chip,
			bool update_vbat_low, bool update_smb)
{
	int rc = 0;
	ktime_t now = ktime_get();
	s64 time_delta;
	u8 fifo_length;

	/*
	 * Reject the FIFO read event if there are back-to-back requests
	 * This is done to gaurantee that there is always a minimum FIFO
	 * data to be processed, ignore this if vbat_low is set.
	 */
	time_delta = ktime_ms_delta(now, chip->last_user_update_time);

	qg_dbg(chip, QG_DEBUG_FIFO, "time_delta=%lld ms update_vbat_low=%d update_smb=%d\n",
				time_delta, update_vbat_low, update_smb);

	if (time_delta > MIN_FIFO_FULL_TIME_MS || update_vbat_low
						|| update_smb) {
		rc = qg_master_hold(chip, true);
		if (rc < 0) {
			pr_err("Failed to hold master, rc=%d\n", rc);
			goto done;
		}

		rc = qg_process_rt_fifo(chip);
		if (rc < 0) {
			pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
			goto done;
		}

		if (update_vbat_low) {
			/* change FIFO length */
			fifo_length = chip->vbat_low ?
					chip->dt.s2_vbat_low_fifo_length :
					chip->dt.s2_fifo_length;
			rc = qg_update_fifo_length(chip, fifo_length);
			if (rc < 0)
				goto done;

			qg_dbg(chip, QG_DEBUG_STATUS,
				"FIFO length updated to %d vbat_low=%d\n",
					fifo_length, chip->vbat_low);
		}

		if (update_smb) {
			rc = qg_masked_write(chip, chip->qg_base +
				QG_MODE_CTL1_REG, PARALLEL_IBAT_SENSE_EN_BIT,
				chip->parallel_enabled ?
					PARALLEL_IBAT_SENSE_EN_BIT : 0);
			if (rc < 0) {
				pr_err("Failed to update SMB_EN, rc=%d\n", rc);
				goto done;
			}
			qg_dbg(chip, QG_DEBUG_STATUS, "Parallel SENSE %d\n",
						chip->parallel_enabled);
		}

		rc = qg_master_hold(chip, false);
		if (rc < 0) {
			pr_err("Failed to release master, rc=%d\n", rc);
			goto done;
		}
		/* FIFOs restarted */
		chip->last_fifo_update_time = ktime_get();

		/* signal the read thread */
		chip->data_ready = true;
		wake_up_interruptible(&chip->qg_wait_q);
		chip->last_user_update_time = now;

		/* vote to stay awake until userspace reads data */
		vote(chip->awake_votable, FIFO_RT_DONE_VOTER, true, 0);
	} else {
		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO processing too early time_delta=%lld\n",
							time_delta);
	}
done:
	qg_master_hold(chip, false);
	return rc;
}

static void process_udata_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work,
			struct qpnp_qg, udata_work);
	int rc;

	if (chip->udata.param[QG_CC_SOC].valid)
		chip->cc_soc = chip->udata.param[QG_CC_SOC].data;

	if (chip->udata.param[QG_BATT_SOC].valid)
		chip->batt_soc = chip->udata.param[QG_BATT_SOC].data;

	if (chip->udata.param[QG_SOC].valid) {
		qg_dbg(chip, QG_DEBUG_SOC, "udata SOC=%d last SOC=%d\n",
			chip->udata.param[QG_SOC].data, chip->catch_up_soc);

		chip->catch_up_soc = chip->udata.param[QG_SOC].data;
		qg_scale_soc(chip, false);

		/* update parameters to SDAM */
		chip->sdam_data[SDAM_SOC] = chip->msoc;
		chip->sdam_data[SDAM_OCV_UV] =
				chip->udata.param[QG_OCV_UV].data;
		chip->sdam_data[SDAM_RBAT_MOHM] =
				chip->udata.param[QG_RBAT_MOHM].data;
		chip->sdam_data[SDAM_VALID] = 1;

		rc = qg_update_sdam_params(chip);
		if (rc < 0)
			pr_err("Failed to update SDAM params, rc=%d\n", rc);
	}

	if (chip->udata.param[QG_CHARGE_COUNTER].valid)
		chip->charge_counter_uah =
			chip->udata.param[QG_CHARGE_COUNTER].data;

	vote(chip->awake_votable, UDATA_READY_VOTER, false, 0);
}

static irqreturn_t qg_default_irq_handler(int irq, void *data)
{
	struct qpnp_qg *chip = data;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");

	return IRQ_HANDLED;
}

#define MAX_FIFO_DELTA_PERCENT		10
static irqreturn_t qg_fifo_update_done_handler(int irq, void *data)
{
	ktime_t now = ktime_get();
	int rc, hw_delta_ms = 0, margin_ms = 0;
	u32 fifo_length = 0;
	s64 time_delta_ms = 0;
	struct qpnp_qg *chip = data;

	time_delta_ms = ktime_ms_delta(now, chip->last_fifo_update_time);
	chip->last_fifo_update_time = now;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");
	mutex_lock(&chip->data_lock);

	rc = get_fifo_length(chip, &fifo_length, false);
	if (rc < 0) {
		pr_err("Failed to get FIFO length, rc=%d\n", rc);
		goto done;
	}

	rc = qg_process_fifo(chip, fifo_length);
	if (rc < 0) {
		pr_err("Failed to process QG FIFO, rc=%d\n", rc);
		goto done;
	}

	rc = qg_vbat_thresholds_config(chip);
	if (rc < 0)
		pr_err("Failed to apply VBAT EMPTY config rc=%d\n", rc);

	rc = qg_vbat_low_wa(chip);
	if (rc < 0) {
		pr_err("Failed to apply VBAT LOW WA, rc=%d\n", rc);
		goto done;
	}

	rc = get_fifo_done_time(chip, false, &hw_delta_ms);
	if (rc < 0)
		hw_delta_ms = 0;
	else
		margin_ms = (hw_delta_ms * MAX_FIFO_DELTA_PERCENT) / 100;

	if (abs(hw_delta_ms - time_delta_ms) < margin_ms) {
		chip->kdata.param[QG_FIFO_TIME_DELTA].data = time_delta_ms;
		chip->kdata.param[QG_FIFO_TIME_DELTA].valid = true;
		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO_done time_delta_ms=%lld\n",
							time_delta_ms);
	}

	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

	/* vote to stay awake until userspace reads data */
	vote(chip->awake_votable, FIFO_DONE_VOTER, true, 0);

done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static irqreturn_t qg_vbat_low_handler(int irq, void *data)
{
	int rc;
	struct qpnp_qg *chip = data;
	u8 status = 0;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");
	mutex_lock(&chip->data_lock);

	rc = qg_read(chip, chip->qg_base + QG_INT_RT_STS_REG, &status, 1);
	if (rc < 0) {
		pr_err("Failed to read RT status, rc=%d\n", rc);
		goto done;
	}
	chip->vbat_low = !!(status & VBAT_LOW_INT_RT_STS_BIT);

	rc = process_rt_fifo_data(chip, true, false);
	if (rc < 0)
		pr_err("Failed to process RT FIFO data, rc=%d\n", rc);

	qg_dbg(chip, QG_DEBUG_IRQ, "VBAT_LOW = %d\n", chip->vbat_low);
done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static irqreturn_t qg_vbat_empty_handler(int irq, void *data)
{
	struct qpnp_qg *chip = data;
	u32 ocv_uv = 0;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");
	pr_warn("VBATT EMPTY SOC = 0\n");

	chip->catch_up_soc = 0;
	qg_scale_soc(chip, true);

	qg_sdam_read(SDAM_OCV_UV, &ocv_uv);
	chip->sdam_data[SDAM_SOC] = 0;
	chip->sdam_data[SDAM_OCV_UV] = ocv_uv;
	chip->sdam_data[SDAM_VALID] = 1;

	qg_update_sdam_params(chip);

	if (chip->qg_psy)
		power_supply_changed(chip->qg_psy);

	return IRQ_HANDLED;
}

static irqreturn_t qg_good_ocv_handler(int irq, void *data)
{
	int rc;
	u8 status = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
	struct qpnp_qg *chip = data;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");

	mutex_lock(&chip->data_lock);

	rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status, 1);
	if (rc < 0) {
		pr_err("Failed to read status2 register rc=%d\n", rc);
		goto done;
	}

	if (!(status & GOOD_OCV_BIT))
		goto done;

	rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S3_GOOD_OCV);
	if (rc < 0) {
		pr_err("Failed to read good_ocv, rc=%d\n", rc);
		goto done;
	}

	chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_GOOD_OCV_UV].valid = true;

	vote(chip->awake_votable, GOOD_OCV_VOTER, true, 0);

	/* signal the readd thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);
done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static struct qg_irq_info qg_irqs[] = {
	[QG_BATT_MISSING_IRQ] = {
		.name		= "qg-batt-missing",
		.handler	= qg_default_irq_handler,
	},
	[QG_VBATT_LOW_IRQ] = {
		.name		= "qg-vbat-low",
		.handler	= qg_vbat_low_handler,
		.wake		= true,
	},
	[QG_VBATT_EMPTY_IRQ] = {
		.name		= "qg-vbat-empty",
		.handler	= qg_vbat_empty_handler,
		.wake		= true,
	},
	[QG_FIFO_UPDATE_DONE_IRQ] = {
		.name		= "qg-fifo-done",
		.handler	= qg_fifo_update_done_handler,
		.wake		= true,
	},
	[QG_GOOD_OCV_IRQ] = {
		.name		= "qg-good-ocv",
		.handler	= qg_good_ocv_handler,
		.wake		= true,
	},
	[QG_FSM_STAT_CHG_IRQ] = {
		.name		= "qg-fsm-state-chg",
		.handler	= qg_default_irq_handler,
	},
	[QG_EVENT_IRQ] = {
		.name		= "qg-event",
		.handler	= qg_default_irq_handler,
	},
};

static int qg_awake_cb(struct votable *votable, void *data, int awake,
			const char *client)
{
	struct qpnp_qg *chip = data;

	/* ignore if the QG device is not open */
	if (!chip->qg_device_open)
		return 0;

	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);

	pr_debug("client: %s awake: %d\n", client, awake);
	return 0;
}

static int qg_fifo_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].wake)
			disable_irq_wake(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq)
			disable_irq_nosync(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
	} else {
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq)
			enable_irq(qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].wake)
			enable_irq_wake(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
	}

	return 0;
}

static int qg_vbatt_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_VBATT_LOW_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
		if (qg_irqs[QG_VBATT_LOW_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
	} else {
		if (qg_irqs[QG_VBATT_LOW_IRQ].irq)
			enable_irq(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].irq)
			enable_irq(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
		if (qg_irqs[QG_VBATT_LOW_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
	}

	return 0;
}

static int qg_good_ocv_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_GOOD_OCV_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_GOOD_OCV_IRQ].irq);
		if (qg_irqs[QG_GOOD_OCV_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_GOOD_OCV_IRQ].irq);
	} else {
		if (qg_irqs[QG_GOOD_OCV_IRQ].irq)
			enable_irq(qg_irqs[QG_GOOD_OCV_IRQ].irq);
		if (qg_irqs[QG_GOOD_OCV_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_GOOD_OCV_IRQ].irq);
	}

	return 0;
}

/* ALG callback functions below */

static int qg_get_learned_capacity(void *data, int64_t *learned_cap_uah)
{
	struct qpnp_qg *chip = data;
	int16_t cc_mah;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	rc = qg_sdam_multibyte_read(QG_SDAM_LEARNED_CAPACITY_OFFSET,
					(u8 *)&cc_mah, 2);
	if (rc < 0) {
		pr_err("Error in reading learned_capacity, rc=%d\n", rc);
		return rc;
	}
	*learned_cap_uah = cc_mah * 1000;

	qg_dbg(chip, QG_DEBUG_ALG_CL, "Retrieved learned capacity %llduah\n",
					*learned_cap_uah);
	return 0;
}

static int qg_store_learned_capacity(void *data, int64_t learned_cap_uah)
{
	struct qpnp_qg *chip = data;
	int16_t cc_mah;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !learned_cap_uah)
		return -ENODEV;

	cc_mah = div64_s64(learned_cap_uah, 1000);
	rc = qg_sdam_multibyte_write(QG_SDAM_LEARNED_CAPACITY_OFFSET,
					 (u8 *)&cc_mah, 2);
	if (rc < 0) {
		pr_err("Error in writing learned_capacity, rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_ALG_CL, "Stored learned capacity %llduah\n",
					learned_cap_uah);
	return 0;
}

static int qg_get_cc_soc(void *data, int *cc_soc)
{
	struct qpnp_qg *chip = data;

	if (!chip)
		return -ENODEV;

	if (chip->cc_soc == INT_MIN)
		return -EINVAL;

	*cc_soc = chip->cc_soc;

	return 0;
}

static int qg_restore_cycle_count(void *data, u16 *buf, int length)
{
	struct qpnp_qg *chip = data;
	int id, rc = 0;
	u8 tmp[2];

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	if (!buf || length > BUCKET_COUNT)
		return -EINVAL;

	for (id = 0; id < length; id++) {
		rc = qg_sdam_multibyte_read(
				QG_SDAM_CYCLE_COUNT_OFFSET + (id * 2),
				(u8 *)tmp, 2);
		if (rc < 0) {
			pr_err("failed to read bucket %d rc=%d\n", id, rc);
			return rc;
		}
		*buf++ = tmp[0] | tmp[1] << 8;
	}

	return rc;
}

static int qg_store_cycle_count(void *data, u16 *buf, int id, int length)
{
	struct qpnp_qg *chip = data;
	int rc = 0;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	if (!buf || length > BUCKET_COUNT * 2 || id < 0 ||
		id > BUCKET_COUNT - 1 ||
		(((id * 2) + length) > BUCKET_COUNT * 2))
		return -EINVAL;

	rc = qg_sdam_multibyte_write(
			QG_SDAM_CYCLE_COUNT_OFFSET + (id * 2),
			(u8 *)buf, length);
	if (rc < 0)
		pr_err("failed to write bucket %d rc=%d\n", id, rc);

	return rc;
}

#define DEFAULT_BATT_TYPE	"Unknown Battery"
#define MISSING_BATT_TYPE	"Missing Battery"
#define DEBUG_BATT_TYPE		"Debug Board"
static const char *qg_get_battery_type(struct qpnp_qg *chip)
{
	if (chip->battery_missing)
		return MISSING_BATT_TYPE;

	if (is_debug_batt_id(chip))
		return DEBUG_BATT_TYPE;

	if (chip->bp.batt_type_str) {
		if (chip->profile_loaded)
			return chip->bp.batt_type_str;
	}

	return DEFAULT_BATT_TYPE;
}

static int qg_get_battery_current(struct qpnp_qg *chip, int *ibat_ua)
{
	int rc = 0, last_ibat = 0;

	if (chip->battery_missing) {
		*ibat_ua = 0;
		return 0;
	}

	rc = qg_read(chip, chip->qg_base + QG_LAST_ADC_I_DATA0_REG,
				(u8 *)&last_ibat, 2);
	if (rc < 0) {
		pr_err("Failed to read LAST_ADV_I reg, rc=%d\n", rc);
		return rc;
	}

	last_ibat = sign_extend32(last_ibat, 15);
	*ibat_ua = I_RAW_TO_UA(last_ibat);

	return rc;
}

static int qg_get_battery_voltage(struct qpnp_qg *chip, int *vbat_uv)
{
	int rc = 0;
	u64 last_vbat = 0;

	if (chip->battery_missing) {
		*vbat_uv = 3700000;
		return 0;
	}

	rc = qg_read(chip, chip->qg_base + QG_LAST_ADC_V_DATA0_REG,
				(u8 *)&last_vbat, 2);
	if (rc < 0) {
		pr_err("Failed to read LAST_ADV_V reg, rc=%d\n", rc);
		return rc;
	}

	*vbat_uv = V_RAW_TO_UV(last_vbat);

	return rc;
}

#define DEBUG_BATT_SOC		67
#define BATT_MISSING_SOC	50
#define EMPTY_SOC		0
#define FULL_SOC		100
static int qg_get_battery_capacity(struct qpnp_qg *chip, int *soc)
{
	if (is_debug_batt_id(chip)) {
		*soc = DEBUG_BATT_SOC;
		return 0;
	}

	if (chip->battery_missing || !chip->profile_loaded) {
		*soc = BATT_MISSING_SOC;
		return 0;
	}

	if (chip->charge_full) {
		*soc = FULL_SOC;
		return 0;
	}

	mutex_lock(&chip->soc_lock);

	if (chip->dt.linearize_soc && chip->maint_soc > 0)
		*soc = chip->maint_soc;
	else
		*soc = chip->msoc;

	mutex_unlock(&chip->soc_lock);

	return 0;
}

#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
static void qg_restore_battery_info(struct qpnp_qg *chip)
{
	int rc, batt_temp = 0;
        u32 ocv_uv = 0;

	chip->catch_up_soc = chip->batt_info[BATT_INFO_SOC];
	qg_scale_soc(chip, true);

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc) {
		pr_err("Failed to read BATT_TEMP rc=%d\n", rc);
		goto out;
	}
	rc = lookup_ocv_soc(&ocv_uv, chip->catch_up_soc, batt_temp, false);
	if (rc < 0) {
		pr_err("Failed to lookup ocv, rc=%d\n", rc);
		goto out;
	}

	mutex_lock(&chip->data_lock);

	chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_GOOD_OCV_UV].valid = true;

	vote(chip->awake_votable, GOOD_OCV_VOTER, true, 0);

	/* signal the readd thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);
	mutex_unlock(&chip->data_lock);

	if (chip->qg_psy)
		power_supply_changed(chip->qg_psy);

	qg_dbg(chip, QG_DEBUG_STATUS, "Restored battery info!\n");

out:
	return;
}

#define DELTA_BATT_TEMP		30
static bool qg_validate_battery_info(struct qpnp_qg *chip)
{
	int i, delta_pct, batt_id_kohm, batt_temp, batt_volt_mv, batt_soc;
	int rc = 0;

	for (i = 1; i < BATT_INFO_MAX; i++) {
		qg_dbg(chip, QG_DEBUG_STATUS, "batt_info[%d]: %d\n", i, chip->batt_info[i]);
	}

	batt_id_kohm = chip->batt_id_ohm / 1000;
	if (batt_id_kohm != chip->batt_info[BATT_INFO_RES_ID]) {
		qg_dbg(chip, QG_DEBUG_STATUS, "batt_id(%dK) does not match the stored batt_id(%dK)\n",
				batt_id_kohm, chip->batt_info[BATT_INFO_RES_ID]);
		return false;
	}

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc) {
		pr_err("Failed reading batt_temp, rc=%d\n", rc);
		return rc;
	}
	if (abs(chip->batt_info[BATT_INFO_TEMP] - batt_temp) >
			DELTA_BATT_TEMP) {
		qg_dbg(chip, QG_DEBUG_STATUS, "batt_temp(%d) is higher/lower than stored batt_temp(%d)\n",
				batt_temp, chip->batt_info[BATT_INFO_TEMP]);
		return false;
	}

	if (chip->batt_info[BATT_INFO_FCC] < 0) {
		qg_dbg(chip, QG_DEBUG_STATUS, "batt_fcc cannot be %d\n",
				chip->batt_info[BATT_INFO_FCC]);
		return false;
	}

	rc = qg_get_battery_voltage(chip, &batt_volt_mv);
	if (rc) {
		pr_err("Failed reading batt_volt, rc=%d\n", rc);
		return rc;
	}
	batt_volt_mv = batt_volt_mv / 1000;

	rc = qg_get_battery_capacity(chip, &batt_soc);
	if (rc) {
		pr_err("Failed reading capacity, rc=%d\n", rc);
		return rc;
	}

	if (*chip->batt_range_ocv && chip->bp.float_volt_uv > 1000)
		delta_pct =  DIV_ROUND_CLOSEST(abs(batt_volt_mv -
				chip->batt_info[BATT_INFO_VOLTAGE]) * 100,
				chip->bp.float_volt_uv / 1000);
	else
		delta_pct = abs(batt_soc - chip->batt_info[BATT_INFO_SOC]);

	qg_dbg(chip, QG_DEBUG_STATUS, "Validating by %s batt_voltage:%d capacity:%d delta_pct:%d\n",
			*chip->batt_range_ocv ? "OCV" : "SOC", batt_volt_mv, batt_soc, delta_pct);

	if (*chip->batt_range_pct && delta_pct > *chip->batt_range_pct) {
		qg_dbg(chip, QG_DEBUG_STATUS, "delta_pct(%d) is higher than batt_range_pct(%d)\n",
				delta_pct, *chip->batt_range_pct);
		return false;
	}

	return true;
}

static int qg_set_battery_info(struct qpnp_qg *chip, int val)
{
	if (chip->batt_info_id < 0 ||
			chip->batt_info_id >= BATT_INFO_MAX) {
		pr_err("Invalid batt_info_id %d\n", chip->batt_info_id);
		chip->batt_info_id = 0;
		return -EINVAL;
	}

	if (chip->batt_info_id == BATT_INFO_NOTIFY && val == INT_MAX - 1) {
		qg_dbg(chip, QG_DEBUG_STATUS, "Notified from userspace\n");
		if (chip->batt_info_restore) {
			if (!qg_validate_battery_info(chip)) {
				qg_dbg(chip, QG_DEBUG_STATUS, "Validating battery info failed\n");
			} else {
				qg_restore_battery_info(chip);
			}
		}
	}

	chip->batt_info[chip->batt_info_id] = val;
	return 0;
}
#endif

static const char *qg_get_cycle_counts(struct qpnp_qg *chip)
{
	int i, rc, len = 0;
	char *buf;

	buf = chip->counter_buf;
	for (i = 1; i <= BUCKET_COUNT; i++) {
		chip->counter->id = i;
		rc = get_cycle_count(chip->counter);
		if (rc < 0) {
			pr_err("Couldn't get cycle count rc=%d\n", rc);
			return NULL;
		}

		if (sizeof(chip->counter_buf) - len < 8) {
			pr_err("Invalid length %d\n", len);
			return NULL;
		}

		len += snprintf(buf+len, 8, "%d ", rc);
	}

	buf[len] = '\0';
	return buf;
}

static int qg_psy_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *pval)
{
	struct qpnp_qg *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
	case POWER_SUPPLY_PROP_BATTERY_INFO:
		rc = qg_set_battery_info(chip, pval->intval);
		break;
	case POWER_SUPPLY_PROP_BATTERY_INFO_ID:
		chip->batt_info_id = pval->intval;
		break;
        case POWER_SUPPLY_PROP_SOC_NOTIFY_READY:
                healthd_ready = pval->intval;
                qg_debug("healthd running, reset 5s_thread\n");
                oppo_chg_wake_update_work();
        	break;
#endif

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (chip->dt.cl_disable) {
			pr_warn("Capacity learning disabled!\n");
			return 0;
		}
		if (chip->cl->active) {
			pr_warn("Capacity learning active!\n");
			return 0;
		}
		if (pval->intval <= 0 || pval->intval > chip->cl->nom_cap_uah) {
			pr_err("charge_full is out of bounds\n");
			return -EINVAL;
		}
		mutex_lock(&chip->cl->lock);
		rc = qg_store_learned_capacity(chip, pval->intval);
		if (!rc)
			chip->cl->learned_cap_uah = pval->intval;
		mutex_unlock(&chip->cl->lock);
		break;
	default:
		break;
	}
	return 0;
}

static int qg_psy_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *pval)
{
	struct qpnp_qg *chip = power_supply_get_drvdata(psy);
	int rc = 0;
	int64_t temp = 0;

	pval->intval = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = qg_get_battery_capacity(chip, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = qg_get_battery_voltage(chip, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = qg_get_battery_current(chip, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		rc = qg_sdam_read(SDAM_OCV_UV, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = qg_get_battery_temp(chip, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		pval->intval = chip->batt_id_ohm;
		break;
	case POWER_SUPPLY_PROP_DEBUG_BATTERY:
		pval->intval = is_debug_batt_id(chip);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		rc = qg_sdam_read(SDAM_RBAT_MOHM, &pval->intval);
		if (!rc)
			pval->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_CAPACITIVE:
		pval->intval = chip->dt.rbat_conn_mohm;
		break;
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		pval->strval = qg_get_battery_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		pval->intval = chip->dt.vbatt_cutoff_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = chip->bp.float_volt_uv;
		break;
	case POWER_SUPPLY_PROP_BATT_FULL_CURRENT:
		pval->intval = chip->dt.iterm_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_BATT_PROFILE_VERSION:
		pval->intval = chip->bp.qg_profile_version;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		pval->intval = chip->charge_counter_uah;
		break;
#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
	case POWER_SUPPLY_PROP_SOC_REPORTING_READY:
		pval->intval = !!chip->soc_reporting_ready;
		break;
	case POWER_SUPPLY_PROP_BATTERY_INFO:
		if (chip->batt_info_id < 0 || chip->batt_info_id >= BATT_INFO_MAX) {
			return -EINVAL;
                }
		pval->intval = chip->batt_info[chip->batt_info_id];
		break;
	case POWER_SUPPLY_PROP_BATTERY_INFO_ID:
		pval->intval = chip->batt_info_id;
		break;
        case POWER_SUPPLY_PROP_SOC_NOTIFY_READY:
                pval->intval = healthd_ready;
        	break;
        case POWER_SUPPLY_PROP_RESTORE_SOC:
                if (healthd_ready) {
			pval->intval = chip->batt_info[BATT_INFO_SOC];
		} else {
			pval->intval = -1;
                }
		break;
#endif
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (!chip->dt.cl_disable && chip->dt.cl_feedback_on)
			rc = qg_get_learned_capacity(chip, &temp);
		else
			rc = qg_get_nominal_capacity((int *)&temp, 250, true);
		if (!rc)
			pval->intval = (int)temp;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = qg_get_nominal_capacity((int *)&temp, 250, true);
		if (!rc)
			pval->intval = (int)temp;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNTS:
		pval->strval = qg_get_cycle_counts(chip);
		break;
	default:
		pr_debug("Unsupported property %d\n", psp);
		break;
	}

	return rc;
}

static int qg_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp)
{
	switch (psp) {
#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
	case POWER_SUPPLY_PROP_BATTERY_INFO:
	case POWER_SUPPLY_PROP_BATTERY_INFO_ID:
        case POWER_SUPPLY_PROP_SOC_NOTIFY_READY:
		return 1;
#endif
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return 1;
	default:
		break;
	}
	return 0;
}

static enum power_supply_property qg_psy_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_RESISTANCE_CAPACITIVE,
	POWER_SUPPLY_PROP_DEBUG_BATTERY,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_BATT_FULL_CURRENT,
	POWER_SUPPLY_PROP_BATT_PROFILE_VERSION,
#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
	POWER_SUPPLY_PROP_SOC_REPORTING_READY,
	POWER_SUPPLY_PROP_SOC_NOTIFY_READY,
	POWER_SUPPLY_PROP_BATTERY_INFO,
	POWER_SUPPLY_PROP_BATTERY_INFO_ID,
	POWER_SUPPLY_PROP_RESTORE_SOC,
#endif
	POWER_SUPPLY_PROP_CYCLE_COUNTS,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static const struct power_supply_desc qg_psy_desc = {
	.name = "bms",
	.type = POWER_SUPPLY_TYPE_BMS,
	.properties = qg_psy_props,
	.num_properties = ARRAY_SIZE(qg_psy_props),
	.get_property = qg_psy_get_property,
	.set_property = qg_psy_set_property,
	.property_is_writeable = qg_property_is_writeable,
};

#define DEFAULT_RECHARGE_SOC 95
static int qg_charge_full_update(struct qpnp_qg *chip)
{
	union power_supply_propval prop = {0, };
	int rc, recharge_soc, health;

	if (!chip->dt.hold_soc_while_full)
		goto out;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_HEALTH, &prop);
	if (rc < 0) {
		pr_err("Failed to get battery health, rc=%d\n", rc);
		goto out;
	}
	health = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_RECHARGE_SOC, &prop);
	if (rc < 0 || prop.intval < 0) {
		pr_debug("Failed to get recharge-soc\n");
		recharge_soc = DEFAULT_RECHARGE_SOC;
	}
	recharge_soc = prop.intval;

	qg_dbg(chip, QG_DEBUG_STATUS, "msoc=%d health=%d charge_full=%d\n",
				chip->msoc, health, chip->charge_full);
	if (chip->charge_done && !chip->charge_full) {
		if (chip->msoc >= 99 && health == POWER_SUPPLY_HEALTH_GOOD) {
			chip->charge_full = true;
			qg_dbg(chip, QG_DEBUG_STATUS, "Setting charge_full (0->1) @ msoc=%d\n",
					chip->msoc);
		} else if (health != POWER_SUPPLY_HEALTH_GOOD) {
			/* terminated in JEITA */
			qg_dbg(chip, QG_DEBUG_STATUS, "Terminated charging @ msoc=%d\n",
					chip->msoc);
		}
	} else if ((!chip->charge_done || chip->msoc < recharge_soc)
				&& chip->charge_full) {

		if (chip->wa_flags & QG_RECHARGE_SOC_WA) {
			/* Force recharge */
			prop.intval = 0;
			rc = power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_RECHARGE_SOC, &prop);
			if (rc < 0)
				pr_err("Failed to force recharge rc=%d\n", rc);
			else
				qg_dbg(chip, QG_DEBUG_STATUS,
					"Forced recharge\n");
		}

		/*
		 * If recharge or discharge has started and
		 * if linearize soc dtsi property defined
		 * scale msoc from 100% for better UX.
		 */
		if (chip->dt.linearize_soc && chip->msoc < 99) {
			chip->maint_soc = FULL_SOC;
			qg_scale_soc(chip, false);
		}

		qg_dbg(chip, QG_DEBUG_STATUS, "msoc=%d recharge_soc=%d charge_full (1->0)\n",
					chip->msoc, recharge_soc);
		chip->charge_full = false;
	}
out:
	return 0;
}

static int qg_parallel_status_update(struct qpnp_qg *chip)
{
	int rc;
	bool parallel_enabled = is_parallel_enabled(chip);
	bool update_smb = false;

	if (parallel_enabled == chip->parallel_enabled)
		return 0;

	chip->parallel_enabled = parallel_enabled;
	qg_dbg(chip, QG_DEBUG_STATUS,
		"Parallel status changed Enabled=%d\n", parallel_enabled);

	mutex_lock(&chip->data_lock);

	/*
	 * Parallel charger uses the same external sense, hence do not
	 * enable SMB sensing if PMI632 is configured for external sense.
	 */
	if (!chip->dt.qg_ext_sense)
		update_smb = true;

	rc = process_rt_fifo_data(chip, false, update_smb);
	if (rc < 0)
		pr_err("Failed to process RT FIFO data, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qg_usb_status_update(struct qpnp_qg *chip)
{
	bool usb_present = is_usb_present(chip);

	if (chip->usb_present != usb_present) {
		qg_dbg(chip, QG_DEBUG_STATUS,
			"USB status changed Present=%d\n",
							usb_present);
		qg_scale_soc(chip, false);
	}

	chip->usb_present = usb_present;

	return 0;
}

static void qg_status_change_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work,
			struct qpnp_qg, qg_status_change_work);
	union power_supply_propval prop = {0, };
	int rc = 0, batt_temp = 0, batt_soc_32b = 0;

	if (!is_batt_available(chip)) {
		pr_debug("batt-psy not available\n");
		goto out;
	}

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS, &prop);
	if (rc < 0)
		pr_err("Failed to get charger status, rc=%d\n", rc);
	else
		chip->charge_status = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_DONE, &prop);
	if (rc < 0)
		pr_err("Failed to get charge done status, rc=%d\n", rc);
	else
		chip->charge_done = prop.intval;

	qg_dbg(chip, QG_DEBUG_STATUS, "charge_status=%d charge_done=%d\n",
			chip->charge_status, chip->charge_done);

	rc = qg_parallel_status_update(chip);
	if (rc < 0)
		pr_err("Failed to update parallel-status, rc=%d\n", rc);

	rc = qg_usb_status_update(chip);
	if (rc < 0)
		pr_err("Failed to update usb status, rc=%d\n", rc);

	cycle_count_update(chip->counter,
			DIV_ROUND_CLOSEST(chip->msoc * 255, 100),
			chip->charge_status, chip->charge_done,
			chip->usb_present);

	if (!chip->dt.cl_disable) {
		rc = qg_get_battery_temp(chip, &batt_temp);
		if (rc < 0) {
			pr_err("Failed to read BATT_TEMP at PON rc=%d\n", rc);
		} else {
			batt_soc_32b = div64_u64(
					chip->batt_soc * BATT_SOC_32BIT,
					QG_SOC_FULL);
			cap_learning_update(chip->cl, batt_temp, batt_soc_32b,
				chip->charge_status, chip->charge_done,
				chip->usb_present, false);
		}
	}
	rc = qg_charge_full_update(chip);
	if (rc < 0)
		pr_err("Failed in charge_full_update, rc=%d\n", rc);
out:
	pm_relax(chip->dev);
}

static int qg_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct qpnp_qg *chip = container_of(nb, struct qpnp_qg, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (work_pending(&chip->qg_status_change_work))
		return NOTIFY_OK;

	if ((strcmp(psy->desc->name, "battery") == 0)
		|| (strcmp(psy->desc->name, "parallel") == 0)
		|| (strcmp(psy->desc->name, "usb") == 0)) {
		/*
		 * We cannot vote for awake votable here as that takes
		 * a mutex lock and this is executed in an atomic context.
		 */
		pm_stay_awake(chip->dev);
		schedule_work(&chip->qg_status_change_work);
	}

	return NOTIFY_OK;
}

static int qg_init_psy(struct qpnp_qg *chip)
{
	struct power_supply_config qg_psy_cfg;
	int rc;

	qg_psy_cfg.drv_data = chip;
	qg_psy_cfg.of_node = NULL;
	qg_psy_cfg.supplied_to = NULL;
	qg_psy_cfg.num_supplicants = 0;
	chip->qg_psy = devm_power_supply_register(chip->dev,
				&qg_psy_desc, &qg_psy_cfg);
	if (IS_ERR_OR_NULL(chip->qg_psy)) {
		pr_err("Failed to register qg_psy rc = %ld\n",
				PTR_ERR(chip->qg_psy));
		return -ENODEV;
	}

	chip->nb.notifier_call = qg_notifier_cb;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0)
		pr_err("Failed register psy notifier rc = %d\n", rc);

	return rc;
}

static ssize_t qg_device_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	int rc;
	struct qpnp_qg *chip = file->private_data;
	unsigned long data_size = sizeof(chip->kdata);

	/* non-blocking access, return */
	if (!chip->data_ready && (file->f_flags & O_NONBLOCK))
		return 0;

	/* blocking access wait on data_ready */
	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(chip->qg_wait_q,
					chip->data_ready);
		if (rc < 0) {
			pr_debug("Failed wait! rc=%d\n", rc);
			return rc;
		}
	}

	mutex_lock(&chip->data_lock);

	if (!chip->data_ready) {
		pr_debug("No Data, false wakeup\n");
		rc = -EFAULT;
		goto fail_read;
	}


	if (copy_to_user(buf, &chip->kdata, data_size)) {
		pr_err("Failed in copy_to_user\n");
		rc = -EFAULT;
		goto fail_read;
	}
	chip->data_ready = false;

	/* release all wake sources */
	vote(chip->awake_votable, GOOD_OCV_VOTER, false, 0);
	vote(chip->awake_votable, FIFO_DONE_VOTER, false, 0);
	vote(chip->awake_votable, FIFO_RT_DONE_VOTER, false, 0);
	vote(chip->awake_votable, SUSPEND_DATA_VOTER, false, 0);

	qg_dbg(chip, QG_DEBUG_DEVICE,
		"QG device read complete Seq_no=%u Size=%ld\n",
				chip->kdata.seq_no, data_size);

	/* clear data */
	memset(&chip->kdata, 0, sizeof(chip->kdata));

	mutex_unlock(&chip->data_lock);

	return data_size;

fail_read:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static ssize_t qg_device_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	int rc = -EINVAL;
	struct qpnp_qg *chip = file->private_data;
	unsigned long data_size = sizeof(chip->udata);

	mutex_lock(&chip->data_lock);
	if (count == 0) {
		pr_err("No data!\n");
		goto fail;
	}

	if (count != 0 && count < data_size) {
		pr_err("Invalid datasize %zu expected %lu\n", count, data_size);
		goto fail;
	}

	if (copy_from_user(&chip->udata, buf, data_size)) {
		pr_err("Failed in copy_from_user\n");
		rc = -EFAULT;
		goto fail;
	}

	rc = data_size;
	vote(chip->awake_votable, UDATA_READY_VOTER, true, 0);
	schedule_work(&chip->udata_work);
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG write complete size=%d\n", rc);
fail:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static unsigned int qg_device_poll(struct file *file, poll_table *wait)
{
	struct qpnp_qg *chip = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &chip->qg_wait_q, wait);

	if (chip->data_ready)
		mask = POLLIN | POLLRDNORM;

	return mask;
}

static int qg_device_open(struct inode *inode, struct file *file)
{
	struct qpnp_qg *chip = container_of(inode->i_cdev,
				struct qpnp_qg, qg_cdev);

	file->private_data = chip;
	chip->qg_device_open = true;
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG device opened!\n");

	return 0;
}

static int qg_device_release(struct inode *inode, struct file *file)
{
	struct qpnp_qg *chip = container_of(inode->i_cdev,
				struct qpnp_qg, qg_cdev);

	file->private_data = chip;
	chip->qg_device_open = false;
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG device closed!\n");

	return 0;
}

static const struct file_operations qg_fops = {
	.owner		= THIS_MODULE,
	.open		= qg_device_open,
	.release	= qg_device_release,
	.read		= qg_device_read,
	.write		= qg_device_write,
	.poll		= qg_device_poll,
};

static int qg_register_device(struct qpnp_qg *chip)
{
	int rc;

	rc = alloc_chrdev_region(&chip->dev_no, 0, 1, "qg");
	if (rc < 0) {
		pr_err("Failed to allocate chardev rc=%d\n", rc);
		return rc;
	}

	cdev_init(&chip->qg_cdev, &qg_fops);
	rc = cdev_add(&chip->qg_cdev, chip->dev_no, 1);
	if (rc < 0) {
		pr_err("Failed to cdev_add rc=%d\n", rc);
		goto unregister_chrdev;
	}

	chip->qg_class = class_create(THIS_MODULE, "qg");
	if (IS_ERR_OR_NULL(chip->qg_class)) {
		pr_err("Failed to create qg class\n");
		rc = -EINVAL;
		goto delete_cdev;
	}
	chip->qg_device = device_create(chip->qg_class, NULL, chip->dev_no,
					NULL, "qg");
	if (IS_ERR(chip->qg_device)) {
		pr_err("Failed to create qg_device\n");
		rc = -EINVAL;
		goto destroy_class;
	}

	qg_dbg(chip, QG_DEBUG_DEVICE, "'/dev/qg' successfully created\n");

	return 0;

destroy_class:
	class_destroy(chip->qg_class);
delete_cdev:
	cdev_del(&chip->qg_cdev);
unregister_chrdev:
	unregister_chrdev_region(chip->dev_no, 1);
	return rc;
}

#define BID_RPULL_OHM		100000
#define BID_VREF_MV		1875
static int get_batt_id_ohm(struct qpnp_qg *chip, u32 *batt_id_ohm)
{
	int rc, batt_id_mv;
	int64_t denom;
	struct qpnp_vadc_result result;

	/* Read battery-id */
	rc = qpnp_vadc_read(chip->vadc_dev, VADC_BAT_ID_PU2, &result);
	if (rc) {
		pr_err("Failed to read BATT_ID over vadc, rc=%d\n", rc);
#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-09-13  default ATL battery if failed to read BATT_ID */
		qg_debug("Failed to read BATT_ID, default ATL\n");
		chip->oppo_battery_type = OPPO_ATL_BATT;
		*batt_id_ohm = 150000;
		return 0;
#else
		return rc;
#endif
	}

	batt_id_mv = div_s64(result.physical, 1000);
	if (batt_id_mv == 0) {
		pr_debug("batt_id_mv = 0 from ADC\n");
		return 0;
	}

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-05  authenticate battery */
	if (batt_id_mv >= chip->atl_battery_id_low && batt_id_mv <= chip->atl_battery_id_high) {
		chip->oppo_battery_type = OPPO_ATL_BATT;
	} else if (batt_id_mv >= chip->sdi_battery_id_low && batt_id_mv <= chip->sdi_battery_id_high) {
		chip->oppo_battery_type = OPPO_SDI_BATT;
	} else {
		chip->oppo_battery_type = NON_STD_BATT;
	}
#endif

	denom = div64_s64(BID_VREF_MV * 1000, batt_id_mv) - 1000;
	if (denom <= 0) {
		/* batt id connector might be open, return 0 kohms */
		return 0;
	}

	*batt_id_ohm = div64_u64(BID_RPULL_OHM * 1000 + denom / 2, denom);

	qg_dbg(chip, QG_DEBUG_PROFILE, "batt_id_mv=%d, batt_id_ohm=%d\n",
					batt_id_mv, *batt_id_ohm);

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-05  debug */
        qg_debug("batt_id_mv = %d, batt_id_ohm = %d\n", batt_id_mv, *batt_id_ohm);
#endif

	return 0;
}

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-10  use oppo battery */
#define SDI_BAT_ID_KOHM		27
#define ATL_BAT_ID_KOHM		150
#endif
static int qg_load_battery_profile(struct qpnp_qg *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *batt_node, *profile_node;
	int rc;

	batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!batt_node) {
		pr_err("Batterydata not available\n");
		return -ENXIO;
	}


#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-10  use oppo battery */
	if (chip->oppo_battery_type == OPPO_SDI_BATT) {
		profile_node = of_batterydata_get_best_profile(batt_node,
				SDI_BAT_ID_KOHM, NULL);
	} else {
		profile_node = of_batterydata_get_best_profile(batt_node,
				ATL_BAT_ID_KOHM, NULL);
	}
#else
	profile_node = of_batterydata_get_best_profile(batt_node,
				chip->batt_id_ohm / 1000, NULL);
#endif


	if (IS_ERR(profile_node)) {
		rc = PTR_ERR(profile_node);
		pr_err("Failed to detect valid QG battery profile %d\n", rc);
		return rc;
	}

	rc = of_property_read_string(profile_node, "qcom,battery-type",
				&chip->bp.batt_type_str);

	if (rc < 0) {
		pr_err("Failed to detect battery type rc:%d\n", rc);
		return rc;
	}

	rc = qg_batterydata_init(profile_node);
	if (rc < 0) {
		pr_err("Failed to initialize battery-profile rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
				&chip->bp.float_volt_uv);
	if (rc < 0) {
		pr_err("Failed to read battery float-voltage rc:%d\n", rc);
		chip->bp.float_volt_uv = -EINVAL;
	}

	rc = of_property_read_u32(profile_node, "qcom,fastchg-current-ma",
				&chip->bp.fastchg_curr_ma);
	if (rc < 0) {
		pr_err("Failed to read battery fastcharge current rc:%d\n", rc);
		chip->bp.fastchg_curr_ma = -EINVAL;
	}

	rc = of_property_read_u32(profile_node, "qcom,qg-batt-profile-ver",
				&chip->bp.qg_profile_version);
	if (rc < 0) {
		pr_err("Failed to read QG profile version rc:%d\n", rc);
		chip->bp.qg_profile_version = -EINVAL;
	}

	qg_dbg(chip, QG_DEBUG_PROFILE, "profile=%s FV=%duV FCC=%dma\n",
			chip->bp.batt_type_str, chip->bp.float_volt_uv,
			chip->bp.fastchg_curr_ma);

	return 0;
}

static int qg_setup_battery(struct qpnp_qg *chip)
{
	int rc;

	if (!is_battery_present(chip)) {
		qg_dbg(chip, QG_DEBUG_PROFILE, "Battery Missing!\n");
		chip->battery_missing = true;
		chip->profile_loaded = false;
	} else {
		/* battery present */
		rc = get_batt_id_ohm(chip, &chip->batt_id_ohm);
		if (rc < 0) {
			pr_err("Failed to detect batt_id rc=%d\n", rc);
			chip->profile_loaded = false;
		} else {
			rc = qg_load_battery_profile(chip);
			if (rc < 0)
				pr_err("Failed to load battery-profile rc=%d\n",
								rc);
			else
				chip->profile_loaded = true;
		}
	}

	qg_dbg(chip, QG_DEBUG_PROFILE, "battery_missing=%d batt_id_ohm=%d Ohm profile_loaded=%d profile=%s\n",
			chip->battery_missing, chip->batt_id_ohm,
			chip->profile_loaded, chip->bp.batt_type_str);

	return 0;
}


static struct ocv_all ocv[] = {
	[S7_PON_OCV] = { 0, 0, "S7_PON_OCV"},
	[S3_GOOD_OCV] = { 0, 0, "S3_GOOD_OCV"},
	[S3_LAST_OCV] = { 0, 0, "S3_LAST_OCV"},
	[SDAM_PON_OCV] = { 0, 0, "SDAM_PON_OCV"},
};

#define S7_ERROR_MARGIN_UV		20000
static int qg_determine_pon_soc(struct qpnp_qg *chip)
{
	int rc = 0, batt_temp = 0, i;
	bool use_pon_ocv = true;
	unsigned long rtc_sec = 0;
	u32 ocv_uv = 0, soc = 0, shutdown[SDAM_MAX] = {0};
	char ocv_type[20] = "NONE";

	if (!chip->profile_loaded) {
		qg_dbg(chip, QG_DEBUG_PON, "No Profile, skipping PON soc\n");
		return 0;
	}

	rc = get_rtc_time(&rtc_sec);
	if (rc < 0) {
		pr_err("Failed to read RTC time rc=%d\n", rc);
		goto use_pon_ocv;
	}

	rc = qg_sdam_read_all(shutdown);
	if (rc < 0) {
		pr_err("Failed to read shutdown params rc=%d\n", rc);
		goto use_pon_ocv;
	}

	qg_dbg(chip, QG_DEBUG_PON, "Shutdown: Valid=%d SOC=%d OCV=%duV time=%dsecs, time_now=%ldsecs\n",
			shutdown[SDAM_VALID],
			shutdown[SDAM_SOC],
			shutdown[SDAM_OCV_UV],
			shutdown[SDAM_TIME_SEC],
			rtc_sec);
	/*
	 * Use the shutdown SOC if
	 * 1. The device was powered off for < ignore_shutdown_time
	 * 2. SDAM read is a success & SDAM data is valid
	 */
	if (shutdown[SDAM_VALID] && is_between(0,
			chip->dt.ignore_shutdown_soc_secs,
			(rtc_sec - shutdown[SDAM_TIME_SEC]))) {
		use_pon_ocv = false;
		ocv_uv = shutdown[SDAM_OCV_UV];
		soc = shutdown[SDAM_SOC];
		strlcpy(ocv_type, "SHUTDOWN_SOC", 20);
		qg_dbg(chip, QG_DEBUG_PON, "Using SHUTDOWN_SOC @ PON\n");
	}

use_pon_ocv:
	if (use_pon_ocv == true) {
		rc = qg_get_battery_temp(chip, &batt_temp);
		if (rc) {
			pr_err("Failed to read BATT_TEMP at PON rc=%d\n", rc);
			goto done;
		}

		/* read all OCVs */
		for (i = S7_PON_OCV; i < PON_OCV_MAX; i++) {
			rc = qg_read_ocv(chip, &ocv[i].ocv_uv,
						&ocv[i].ocv_raw, i);
			if (rc < 0)
				pr_err("Failed to read %s OCV rc=%d\n",
						ocv[i].ocv_type, rc);
			else
				qg_dbg(chip, QG_DEBUG_PON, "%s OCV=%d\n",
					ocv[i].ocv_type, ocv[i].ocv_uv);
		}

		if (ocv[S3_LAST_OCV].ocv_raw == FIFO_V_RESET_VAL) {
			if (!ocv[SDAM_PON_OCV].ocv_uv) {
				strlcpy(ocv_type, "S7_PON_SOC", 20);
				ocv_uv = ocv[S7_PON_OCV].ocv_uv;
			} else if (ocv[SDAM_PON_OCV].ocv_uv <=
					ocv[S7_PON_OCV].ocv_uv) {
				strlcpy(ocv_type, "S7_PON_SOC", 20);
				ocv_uv = ocv[S7_PON_OCV].ocv_uv;
			} else if (!shutdown[SDAM_VALID] &&
				((ocv[SDAM_PON_OCV].ocv_uv -
					ocv[S7_PON_OCV].ocv_uv) >
					S7_ERROR_MARGIN_UV)) {
				strlcpy(ocv_type, "S7_PON_SOC", 20);
				ocv_uv = ocv[S7_PON_OCV].ocv_uv;
			} else {
				strlcpy(ocv_type, "SDAM_PON_SOC", 20);
				ocv_uv = ocv[SDAM_PON_OCV].ocv_uv;
			}
		} else {
			if (ocv[S3_LAST_OCV].ocv_uv >= ocv[S7_PON_OCV].ocv_uv) {
				strlcpy(ocv_type, "S3_LAST_SOC", 20);
				ocv_uv = ocv[S3_LAST_OCV].ocv_uv;
			} else {
				strlcpy(ocv_type, "S7_PON_SOC", 20);
				ocv_uv = ocv[S7_PON_OCV].ocv_uv;
			}
		}

		ocv_uv = CAP(QG_MIN_OCV_UV, QG_MAX_OCV_UV, ocv_uv);
		rc = lookup_soc_ocv(&soc, ocv_uv, batt_temp, false);
		if (rc < 0) {
			pr_err("Failed to lookup SOC@PON rc=%d\n", rc);
			goto done;
		}
	}
done:
	if (rc < 0) {
		pr_err("Failed to get %s @ PON, rc=%d\n", ocv_type, rc);
		return rc;
	}

	chip->pon_soc = chip->catch_up_soc = chip->msoc = soc;
	chip->kdata.param[QG_PON_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_PON_OCV_UV].valid = true;

	/* write back to SDAM */
	chip->sdam_data[SDAM_SOC] = soc;
	chip->sdam_data[SDAM_OCV_UV] = ocv_uv;
	chip->sdam_data[SDAM_VALID] = 1;

	rc = qg_write_monotonic_soc(chip, chip->msoc);
	if (rc < 0)
		pr_err("Failed to update MSOC register rc=%d\n", rc);

	rc = qg_update_sdam_params(chip);
	if (rc < 0)
		pr_err("Failed to update sdam params rc=%d\n", rc);

	pr_info("using %s @ PON ocv_uv=%duV soc=%d\n",
			ocv_type, ocv_uv, chip->msoc);

	return 0;
}

static int qg_set_wa_flags(struct qpnp_qg *chip)
{
	switch (chip->pmic_rev_id->pmic_subtype) {
	case PMI632_SUBTYPE:
		chip->wa_flags |= QG_RECHARGE_SOC_WA;
		if (chip->pmic_rev_id->rev4 == PMI632_V1P0_REV4)
			chip->wa_flags |= QG_VBAT_LOW_WA;
		break;
	default:
		pr_err("Unsupported PMIC subtype %d\n",
			chip->pmic_rev_id->pmic_subtype);
		return -EINVAL;
	}

	qg_dbg(chip, QG_DEBUG_PON, "wa_flags = %x\n", chip->wa_flags);

	return 0;
}

static int qg_hw_init(struct qpnp_qg *chip)
{
	int rc, temp;
	u8 reg;

	rc = qg_set_wa_flags(chip);
	if (rc < 0) {
		pr_err("Failed to update PMIC type flags, rc=%d\n", rc);
		return rc;
	}

	rc = qg_master_hold(chip, true);
	if (rc < 0) {
		pr_err("Failed to hold master, rc=%d\n", rc);
		goto done_fifo;
	}

	rc = qg_process_rt_fifo(chip);
	if (rc < 0) {
		pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
		goto done_fifo;
	}

	/* update the changed S2 fifo DT parameters */
	if (chip->dt.s2_fifo_length > 0) {
		rc = qg_update_fifo_length(chip, chip->dt.s2_fifo_length);
		if (rc < 0)
			goto done_fifo;
	}

	if (chip->dt.s2_acc_length > 0) {
		reg = ilog2(chip->dt.s2_acc_length) - 1;
		rc = qg_masked_write(chip, chip->qg_base +
				QG_S2_NORMAL_MEAS_CTL2_REG,
				NUM_OF_ACCUM_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write S2 ACC length, rc=%d\n", rc);
			goto done_fifo;
		}
	}

	if (chip->dt.s2_acc_intvl_ms > 0) {
		reg = chip->dt.s2_acc_intvl_ms / 10;
		rc = qg_write(chip, chip->qg_base +
				QG_S2_NORMAL_MEAS_CTL3_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S2 ACC intrvl, rc=%d\n", rc);
			goto done_fifo;
		}
	}

	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

done_fifo:
	rc = qg_master_hold(chip, false);
	if (rc < 0) {
		pr_err("Failed to release master, rc=%d\n", rc);
		return rc;
	}
	chip->last_fifo_update_time = ktime_get();

	if (chip->dt.ocv_timer_expiry_min != -EINVAL) {
		if (chip->dt.ocv_timer_expiry_min < 2)
			chip->dt.ocv_timer_expiry_min = 2;
		else if (chip->dt.ocv_timer_expiry_min > 30)
			chip->dt.ocv_timer_expiry_min = 30;

		reg = (chip->dt.ocv_timer_expiry_min - 2) / 4;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_MEAS_CTL4_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write OCV timer, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.ocv_tol_threshold_uv != -EINVAL) {
		if (chip->dt.ocv_tol_threshold_uv < 0)
			chip->dt.ocv_tol_threshold_uv = 0;
		else if (chip->dt.ocv_tol_threshold_uv > 12262)
			chip->dt.ocv_tol_threshold_uv = 12262;

		reg = chip->dt.ocv_tol_threshold_uv / 195;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_TREND_CTL2_REG,
			TREND_TOL_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write OCV tol-thresh, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.s3_entry_fifo_length != -EINVAL) {
		if (chip->dt.s3_entry_fifo_length < 1)
			chip->dt.s3_entry_fifo_length = 1;
		else if (chip->dt.s3_entry_fifo_length > 8)
			chip->dt.s3_entry_fifo_length = 8;

		reg = chip->dt.s3_entry_fifo_length - 1;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write S3-entry fifo-length, rc=%d\n",
							rc);
			return rc;
		}
	}

	if (chip->dt.s3_entry_ibat_ua != -EINVAL) {
		if (chip->dt.s3_entry_ibat_ua < 0)
			chip->dt.s3_entry_ibat_ua = 0;
		else if (chip->dt.s3_entry_ibat_ua > 155550)
			chip->dt.s3_entry_ibat_ua = 155550;

		reg = chip->dt.s3_entry_ibat_ua / 610;
		rc = qg_write(chip, chip->qg_base +
				QG_S3_ENTRY_IBAT_THRESHOLD_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S3-entry ibat-uA, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.s3_exit_ibat_ua != -EINVAL) {
		if (chip->dt.s3_exit_ibat_ua < 0)
			chip->dt.s3_exit_ibat_ua = 0;
		else if (chip->dt.s3_exit_ibat_ua > 155550)
			chip->dt.s3_exit_ibat_ua = 155550;

		rc = qg_read(chip, chip->qg_base +
				QG_S3_ENTRY_IBAT_THRESHOLD_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to read S3-entry ibat-uA, rc=%d", rc);
			return rc;
		}
		temp = reg * 610;
		if (chip->dt.s3_exit_ibat_ua < temp)
			chip->dt.s3_exit_ibat_ua = temp;
		else
			chip->dt.s3_exit_ibat_ua -= temp;

		reg = chip->dt.s3_exit_ibat_ua / 610;
		rc = qg_write(chip,
			chip->qg_base + QG_S3_EXIT_IBAT_THRESHOLD_REG,
			&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S3-entry ibat-uA, rc=%d\n", rc);
			return rc;
		}
	}

	/* vbat based configs */
	if (chip->dt.vbatt_low_mv < 0)
		chip->dt.vbatt_low_mv = 0;
	else if (chip->dt.vbatt_low_mv > 12750)
		chip->dt.vbatt_low_mv = 12750;

	if (chip->dt.vbatt_empty_mv < 0)
		chip->dt.vbatt_empty_mv = 0;
	else if (chip->dt.vbatt_empty_mv > 12750)
		chip->dt.vbatt_empty_mv = 12750;

	if (chip->dt.vbatt_empty_cold_mv < 0)
		chip->dt.vbatt_empty_cold_mv = 0;
	else if (chip->dt.vbatt_empty_cold_mv > 12750)
		chip->dt.vbatt_empty_cold_mv = 12750;

	rc = qg_vbat_thresholds_config(chip);
	if (rc < 0) {
		pr_err("Failed to configure VBAT empty/low rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int qg_post_init(struct qpnp_qg *chip)
{
	/* disable all IRQs if profile is not loaded */
	if (!chip->profile_loaded) {
		vote(chip->vbatt_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
		vote(chip->fifo_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
		vote(chip->good_ocv_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
	} else {
		/* disable GOOD_OCV IRQ at init */
		vote(chip->good_ocv_irq_disable_votable,
				QG_INIT_STATE_IRQ_DISABLE, true, 0);
	}

	return 0;
}

static int qg_get_irq_index_byname(const char *irq_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qg_irqs); i++) {
		if (strcmp(qg_irqs[i].name, irq_name) == 0)
			return i;
	}

	return -ENOENT;
}

static int qg_request_interrupt(struct qpnp_qg *chip,
		struct device_node *node, const char *irq_name)
{
	int rc, irq, irq_index;

	irq = of_irq_get_byname(node, irq_name);
	if (irq < 0) {
		pr_err("Failed to get irq %s byname\n", irq_name);
		return irq;
	}

	irq_index = qg_get_irq_index_byname(irq_name);
	if (irq_index < 0) {
		pr_err("%s is not a defined irq\n", irq_name);
		return irq_index;
	}

	if (!qg_irqs[irq_index].handler)
		return 0;

	rc = devm_request_threaded_irq(chip->dev, irq, NULL,
				qg_irqs[irq_index].handler,
				IRQF_ONESHOT, irq_name, chip);
	if (rc < 0) {
		pr_err("Failed to request irq %d\n", irq);
		return rc;
	}

	qg_irqs[irq_index].irq = irq;
	if (qg_irqs[irq_index].wake)
		enable_irq_wake(irq);

	qg_dbg(chip, QG_DEBUG_PON, "IRQ %s registered wakeable=%d\n",
			qg_irqs[irq_index].name, qg_irqs[irq_index].wake);

	return 0;
}

static int qg_request_irqs(struct qpnp_qg *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *child;
	const char *name;
	struct property *prop;
	int rc = 0;

	for_each_available_child_of_node(node, child) {
		of_property_for_each_string(child, "interrupt-names",
					    prop, name) {
			rc = qg_request_interrupt(chip, child, name);
			if (rc < 0)
				return rc;
		}
	}


	return 0;
}

static int qg_alg_init(struct qpnp_qg *chip)
{
	struct cycle_counter *counter;
	struct cap_learning *cl;
	struct device_node *node = chip->dev->of_node;
	int rc;

	counter = devm_kzalloc(chip->dev, sizeof(*counter), GFP_KERNEL);
	if (!counter)
		return -ENOMEM;

	counter->restore_count = qg_restore_cycle_count;
	counter->store_count = qg_store_cycle_count;
	counter->data = chip;

	rc = cycle_count_init(counter);
	if (rc < 0) {
		dev_err(chip->dev, "Error in initializing cycle counter, rc:%d\n",
			rc);
		counter->data = NULL;
		devm_kfree(chip->dev, counter);
		return rc;
	}

	chip->counter = counter;

	chip->dt.cl_disable = of_property_read_bool(node,
					"qcom,cl-disable");

	/*Return if capacity learning is disabled*/
	if (chip->dt.cl_disable)
		return 0;

	cl = devm_kzalloc(chip->dev, sizeof(*cl), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;

	cl->cc_soc_max = QG_SOC_FULL;
	cl->get_cc_soc = qg_get_cc_soc;
	cl->get_learned_capacity = qg_get_learned_capacity;
	cl->store_learned_capacity = qg_store_learned_capacity;
	cl->data = chip;

	rc = cap_learning_init(cl);
	if (rc < 0) {
		dev_err(chip->dev, "Error in initializing capacity learning, rc:%d\n",
			rc);
		counter->data = NULL;
		cl->data = NULL;
		devm_kfree(chip->dev, counter);
		devm_kfree(chip->dev, cl);
		return rc;
	}

	chip->cl = cl;
	return 0;
}

#define DEFAULT_VBATT_EMPTY_MV		3200
#define DEFAULT_VBATT_EMPTY_COLD_MV	3000
#define DEFAULT_VBATT_CUTOFF_MV		3400
#define DEFAULT_VBATT_LOW_MV		3500
#define DEFAULT_VBATT_LOW_COLD_MV	3800
#define DEFAULT_ITERM_MA		100
#define DEFAULT_S2_FIFO_LENGTH		5
#define DEFAULT_S2_VBAT_LOW_LENGTH	2
#define DEFAULT_S2_ACC_LENGTH		128
#define DEFAULT_S2_ACC_INTVL_MS		100
#define DEFAULT_DELTA_SOC		1
#define DEFAULT_SHUTDOWN_SOC_SECS	360
#define DEFAULT_COLD_TEMP_THRESHOLD	0
#define DEFAULT_CL_MIN_START_SOC	10
#define DEFAULT_CL_MAX_START_SOC	15
#define DEFAULT_CL_MIN_TEMP_DECIDEGC	150
#define DEFAULT_CL_MAX_TEMP_DECIDEGC	500
#define DEFAULT_CL_MAX_INC_DECIPERC	5
#define DEFAULT_CL_MAX_DEC_DECIPERC	100
#define DEFAULT_CL_MIN_LIM_DECIPERC	0
#define DEFAULT_CL_MAX_LIM_DECIPERC	0
static int qg_parse_dt(struct qpnp_qg *chip)
{
	int rc = 0;
	struct device_node *revid_node, *child, *node = chip->dev->of_node;
	u32 base, temp;
	u8 type;

	if (!node)  {
		pr_err("Failed to find device-tree node\n");
		return -ENXIO;
	}

	revid_node = of_parse_phandle(node, "qcom,pmic-revid", 0);
	if (!revid_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	chip->pmic_rev_id = get_revid_data(revid_node);
	of_node_put(revid_node);
	if (IS_ERR_OR_NULL(chip->pmic_rev_id)) {
		pr_err("Failed to get pmic_revid, rc=%ld\n",
			PTR_ERR(chip->pmic_rev_id));
		/*
		 * the revid peripheral must be registered, any failure
		 * here only indicates that the rev-id module has not
		 * probed yet.
		 */
		return -EPROBE_DEFER;
	}

	qg_dbg(chip, QG_DEBUG_PON, "PMIC subtype %d Digital major %d\n",
		chip->pmic_rev_id->pmic_subtype, chip->pmic_rev_id->rev4);

	for_each_available_child_of_node(node, child) {
		rc = of_property_read_u32(child, "reg", &base);
		if (rc < 0) {
			pr_err("Failed to read base address, rc=%d\n", rc);
			return rc;
		}

		rc = qg_read(chip, base + PERPH_TYPE_REG, &type, 1);
		if (rc < 0) {
			pr_err("Failed to read type, rc=%d\n", rc);
			return rc;
		}

		switch (type) {
		case QG_TYPE:
			chip->qg_base = base;
			break;
		default:
			break;
		}
	}

	if (!chip->qg_base) {
		pr_err("QG device node missing\n");
		return -EINVAL;
	}

	/* S2 state params */
	rc = of_property_read_u32(node, "qcom,s2-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s2_fifo_length = DEFAULT_S2_FIFO_LENGTH;
	else
		chip->dt.s2_fifo_length = temp;

	rc = of_property_read_u32(node, "qcom,s2-vbat-low-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s2_vbat_low_fifo_length = DEFAULT_S2_VBAT_LOW_LENGTH;
	else
		chip->dt.s2_vbat_low_fifo_length = temp;

	rc = of_property_read_u32(node, "qcom,s2-acc-length", &temp);
	if (rc < 0)
		chip->dt.s2_acc_length = DEFAULT_S2_ACC_LENGTH;
	else
		chip->dt.s2_acc_length = temp;

	rc = of_property_read_u32(node, "qcom,s2-acc-interval-ms", &temp);
	if (rc < 0)
		chip->dt.s2_acc_intvl_ms = DEFAULT_S2_ACC_INTVL_MS;
	else
		chip->dt.s2_acc_intvl_ms = temp;

	qg_dbg(chip, QG_DEBUG_PON, "DT: S2 FIFO length=%d low_vbat_length=%d acc_length=%d acc_interval=%d\n",
		chip->dt.s2_fifo_length, chip->dt.s2_vbat_low_fifo_length,
		chip->dt.s2_acc_length, chip->dt.s2_acc_intvl_ms);

	/* OCV params */
	rc = of_property_read_u32(node, "qcom,ocv-timer-expiry-min", &temp);
	if (rc < 0)
		chip->dt.ocv_timer_expiry_min = -EINVAL;
	else
		chip->dt.ocv_timer_expiry_min = temp;

	rc = of_property_read_u32(node, "qcom,ocv-tol-threshold-uv", &temp);
	if (rc < 0)
		chip->dt.ocv_tol_threshold_uv = -EINVAL;
	else
		chip->dt.ocv_tol_threshold_uv = temp;

	qg_dbg(chip, QG_DEBUG_PON, "DT: OCV timer_expiry =%dmin ocv_tol_threshold=%duV\n",
		chip->dt.ocv_timer_expiry_min, chip->dt.ocv_tol_threshold_uv);

	/* S3 sleep configuration */
	rc = of_property_read_u32(node, "qcom,s3-entry-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s3_entry_fifo_length = -EINVAL;
	else
		chip->dt.s3_entry_fifo_length = temp;

	rc = of_property_read_u32(node, "qcom,s3-entry-ibat-ua", &temp);
	if (rc < 0)
		chip->dt.s3_entry_ibat_ua = -EINVAL;
	else
		chip->dt.s3_entry_ibat_ua = temp;

	rc = of_property_read_u32(node, "qcom,s3-exit-ibat-ua", &temp);
	if (rc < 0)
		chip->dt.s3_exit_ibat_ua = -EINVAL;
	else
		chip->dt.s3_exit_ibat_ua = temp;

	/* VBAT thresholds */
	rc = of_property_read_u32(node, "qcom,vbatt-empty-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_empty_mv = DEFAULT_VBATT_EMPTY_MV;
	else
		chip->dt.vbatt_empty_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-empty-cold-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_empty_cold_mv = DEFAULT_VBATT_EMPTY_COLD_MV;
	else
		chip->dt.vbatt_empty_cold_mv = temp;

	rc = of_property_read_u32(node, "qcom,cold-temp-threshold", &temp);
	if (rc < 0)
		chip->dt.cold_temp_threshold = DEFAULT_COLD_TEMP_THRESHOLD;
	else
		chip->dt.cold_temp_threshold = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-low-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_low_mv = DEFAULT_VBATT_LOW_MV;
	else
		chip->dt.vbatt_low_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-low-cold-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_low_cold_mv = DEFAULT_VBATT_LOW_COLD_MV;
	else
		chip->dt.vbatt_low_cold_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-cutoff-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_cutoff_mv = DEFAULT_VBATT_CUTOFF_MV;
	else
		chip->dt.vbatt_cutoff_mv = temp;

	/* IBAT thresholds */
	rc = of_property_read_u32(node, "qcom,qg-iterm-ma", &temp);
	if (rc < 0)
		chip->dt.iterm_ma = DEFAULT_ITERM_MA;
	else
		chip->dt.iterm_ma = temp;

	rc = of_property_read_u32(node, "qcom,delta-soc", &temp);
	if (rc < 0)
		chip->dt.delta_soc = DEFAULT_DELTA_SOC;
	else
		chip->dt.delta_soc = temp;

	rc = of_property_read_u32(node, "qcom,ignore-shutdown-soc-secs", &temp);
	if (rc < 0)
		chip->dt.ignore_shutdown_soc_secs = DEFAULT_SHUTDOWN_SOC_SECS;
	else
		chip->dt.ignore_shutdown_soc_secs = temp;

	chip->dt.hold_soc_while_full = of_property_read_bool(node,
					"qcom,hold-soc-while-full");

	chip->dt.linearize_soc = of_property_read_bool(node,
					"qcom,linearize-soc");

	rc = of_property_read_u32(node, "qcom,rbat-conn-mohm", &temp);
	if (rc < 0)
		chip->dt.rbat_conn_mohm = 0;
	else
		chip->dt.rbat_conn_mohm = temp;

#ifndef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-02  save soc */
        chip->batt_info_restore = of_property_read_bool(node,
                                        "qcom,qg-restore-batt-info");
#else
        chip->batt_info_restore = true;
	qg_dbg(chip, QG_DEBUG_STATUS, "restore: %d validate_by_ocv: %d range_pct: %d\n",
					chip->batt_info_restore, qg_batt_valid_ocv, qg_batt_range_pct);
#endif

	chip->dt.qg_ext_sense = of_property_read_bool(node, "qcom,qg-ext-sns");

	/* Capacity learning params*/
	if (!chip->dt.cl_disable) {
		chip->dt.cl_feedback_on = of_property_read_bool(node,
						"qcom,cl-feedback-on");

		rc = of_property_read_u32(node, "qcom,cl-min-start-soc", &temp);
		if (rc < 0)
			chip->cl->dt.min_start_soc = DEFAULT_CL_MIN_START_SOC;
		else
			chip->cl->dt.min_start_soc = temp;

		rc = of_property_read_u32(node, "qcom,cl-max-start-soc", &temp);
		if (rc < 0)
			chip->cl->dt.max_start_soc = DEFAULT_CL_MAX_START_SOC;
		else
			chip->cl->dt.max_start_soc = temp;

		rc = of_property_read_u32(node, "qcom,cl-min-temp", &temp);
		if (rc < 0)
			chip->cl->dt.min_temp = DEFAULT_CL_MIN_TEMP_DECIDEGC;
		else
			chip->cl->dt.min_temp = temp;


		rc = of_property_read_u32(node, "qcom,cl-max-temp", &temp);
		if (rc < 0)
			chip->cl->dt.max_temp = DEFAULT_CL_MAX_TEMP_DECIDEGC;
		else
			chip->cl->dt.max_temp = temp;

		rc = of_property_read_u32(node, "qcom,cl-max-increment", &temp);
		if (rc < 0)
			chip->cl->dt.max_cap_inc = DEFAULT_CL_MAX_INC_DECIPERC;
		else
			chip->cl->dt.max_cap_inc = temp;

		rc = of_property_read_u32(node, "qcom,cl-max-decrement", &temp);
		if (rc < 0)
			chip->cl->dt.max_cap_dec = DEFAULT_CL_MAX_DEC_DECIPERC;
		else
			chip->cl->dt.max_cap_dec = temp;

		rc = of_property_read_u32(node, "qcom,cl-min-limit", &temp);
		if (rc < 0)
			chip->cl->dt.min_cap_limit =
						DEFAULT_CL_MIN_LIM_DECIPERC;
		else
			chip->cl->dt.min_cap_limit = temp;

		rc = of_property_read_u32(node, "qcom,cl-max-limit", &temp);
		if (rc < 0)
			chip->cl->dt.max_cap_limit =
						DEFAULT_CL_MAX_LIM_DECIPERC;
		else
			chip->cl->dt.max_cap_limit = temp;

		qg_dbg(chip, QG_DEBUG_PON, "DT: cl_min_start_soc=%d cl_max_start_soc=%d cl_min_temp=%d cl_max_temp=%d\n",
			chip->cl->dt.min_start_soc, chip->cl->dt.max_start_soc,
			chip->cl->dt.min_temp, chip->cl->dt.max_temp);
	}
	qg_dbg(chip, QG_DEBUG_PON, "DT: vbatt_empty_mv=%dmV vbatt_low_mv=%dmV delta_soc=%d ext-sns=%d\n",
			chip->dt.vbatt_empty_mv, chip->dt.vbatt_low_mv,
			chip->dt.delta_soc, chip->dt.qg_ext_sense);

#ifdef VENDOR_EDIT
/* Yichun.Chen	PSW.BSP.CHG  2018-08-23  recognize SDI\ATL battery */
	rc = of_property_read_u32(node, "atl_battery_id_low", &temp);
	if (rc < 0) {
		chip->atl_battery_id_low = 820;
	} else
		chip->atl_battery_id_low = temp;

	rc = of_property_read_u32(node, "atl_battery_id_high", &temp);
	if (rc < 0) {
		chip->atl_battery_id_high = 1250;
	} else
		chip->atl_battery_id_high = temp;

	rc = of_property_read_u32(node, "sdi_battery_id_low", &temp);
	if (rc < 0) {
		chip->sdi_battery_id_low = 300;
	} else
		chip->sdi_battery_id_low = temp;

	rc = of_property_read_u32(node, "sdi_battery_id_high", &temp);
	if (rc < 0) {
		chip->sdi_battery_id_high = 550;
	} else
		chip->sdi_battery_id_high = temp;

	qg_debug("parse bat id range SDI: %d - %d, ATL: %d - %d\n", chip->sdi_battery_id_low, chip->sdi_battery_id_high,
			chip->atl_battery_id_low, chip->atl_battery_id_high);
#endif

	return 0;
}

static int process_suspend(struct qpnp_qg *chip)
{
	u8 status = 0;
	int rc;
	u32 fifo_rt_length = 0, sleep_fifo_length = 0;

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	chip->suspend_data = false;

	/* ignore any suspend processing if we are charging */
	if (chip->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
		qg_dbg(chip, QG_DEBUG_PM, "Charging @ suspend - ignore processing\n");
		return 0;
	}

	rc = get_fifo_length(chip, &fifo_rt_length, true);
	if (rc < 0) {
		pr_err("Failed to read FIFO RT count, rc=%d\n", rc);
		return rc;
	}

	rc = qg_read(chip, chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			(u8 *)&sleep_fifo_length, 1);
	if (rc < 0) {
		pr_err("Failed to read sleep FIFO count, rc=%d\n", rc);
		return rc;
	}
	sleep_fifo_length &= SLEEP_IBAT_QUALIFIED_LENGTH_MASK;
	/*
	 * If the real-time FIFO count is greater than
	 * the the #fifo to enter sleep, save the FIFO data
	 * and reset the fifo count.
	 */
	if (fifo_rt_length >= (chip->dt.s2_fifo_length - sleep_fifo_length)) {
		rc = qg_master_hold(chip, true);
		if (rc < 0) {
			pr_err("Failed to hold master, rc=%d\n", rc);
			return rc;
		}

		rc = qg_process_rt_fifo(chip);
		if (rc < 0) {
			pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
			qg_master_hold(chip, false);
			return rc;
		}

		rc = qg_master_hold(chip, false);
		if (rc < 0) {
			pr_err("Failed to release master, rc=%d\n", rc);
			return rc;
		}
		/* FIFOs restarted */
		chip->last_fifo_update_time = ktime_get();

		chip->suspend_data = true;
	}

	/* read STATUS2 register to clear its last state */
	qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status, 1);

	qg_dbg(chip, QG_DEBUG_PM, "FIFO rt_length=%d sleep_fifo_length=%d default_s2_count=%d suspend_data=%d\n",
			fifo_rt_length, sleep_fifo_length,
			chip->dt.s2_fifo_length, chip->suspend_data);

	return rc;
}

static int process_resume(struct qpnp_qg *chip)
{
	u8 status2 = 0, rt_status = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
#ifndef VENDOR_EDIT
/* Yichun.Chen	PSW.BSP.CHG  2018-10-13  delete useless code */
	int rc, batt_temp = 0;
#else
	int rc;
#endif

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status2, 1);
	if (rc < 0) {
		pr_err("Failed to read status2 register, rc=%d\n", rc);
		return rc;
	}

	if (status2 & GOOD_OCV_BIT) {
		rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S3_GOOD_OCV);
		if (rc < 0) {
			pr_err("Failed to read good_ocv, rc=%d\n", rc);
			return rc;
		}

#ifndef VENDOR_EDIT
/* Yichun.Chen	PSW.BSP.CHG  2018-10-13  delete useless code */
		rc = qg_get_battery_temp(chip, &batt_temp);
		if (rc < 0) {
			pr_err("Failed to read BATT_TEMP, rc=%d\n", rc);
			return rc;
		}
#endif

		chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
		chip->kdata.param[QG_GOOD_OCV_UV].valid = true;
		 /* Clear suspend data as there has been a GOOD OCV */
		chip->suspend_data = false;
		qg_dbg(chip, QG_DEBUG_PM, "GOOD OCV @ resume good_ocv=%d uV\n",
				ocv_uv);
	}

	rc = qg_read(chip, chip->qg_base + QG_INT_LATCHED_STS_REG,
						&rt_status, 1);
	if (rc < 0) {
		pr_err("Failed to read latched status register, rc=%d\n", rc);
		return rc;
	}
	rt_status &= FIFO_UPDATE_DONE_INT_LAT_STS_BIT;

	qg_dbg(chip, QG_DEBUG_PM, "FIFO_DONE_STS=%d suspend_data=%d good_ocv=%d\n",
				!!rt_status, chip->suspend_data,
				chip->kdata.param[QG_GOOD_OCV_UV].valid);
	/*
	 * If this is not a wakeup from FIFO-done,
	 * process the data immediately if - we have data from
	 * suspend or there is a good OCV.
	 */
	if (!rt_status && (chip->suspend_data ||
			chip->kdata.param[QG_GOOD_OCV_UV].valid)) {
		vote(chip->awake_votable, SUSPEND_DATA_VOTER, true, 0);
		/* signal the read thread */
		chip->data_ready = true;
		wake_up_interruptible(&chip->qg_wait_q);
		chip->suspend_data = false;
	}

	return rc;
}

static int qpnp_qg_suspend_noirq(struct device *dev)
{
	int rc;
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->data_lock);

	rc = process_suspend(chip);
	if (rc < 0)
		pr_err("Failed to process QG suspend, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qpnp_qg_resume_noirq(struct device *dev)
{
	int rc;
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->data_lock);

	rc = process_resume(chip);
	if (rc < 0)
		pr_err("Failed to process QG resume, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qpnp_qg_suspend(struct device *dev)
{
       struct qpnp_qg *chip = dev_get_drvdata(dev);

       /* skip if profile is not loaded */
       if (!chip->profile_loaded)
               return 0;

       /* disable GOOD_OCV IRQ in sleep */
       vote(chip->good_ocv_irq_disable_votable,
                       QG_INIT_STATE_IRQ_DISABLE, true, 0);

       return 0;
}

static int qpnp_qg_resume(struct device *dev)
{
       struct qpnp_qg *chip = dev_get_drvdata(dev);

       /* skip if profile is not loaded */
       if (!chip->profile_loaded)
               return 0;

       /* enable GOOD_OCV IRQ when active */
       vote(chip->good_ocv_irq_disable_votable,
                       QG_INIT_STATE_IRQ_DISABLE, false, 0);

       return 0;
}

static const struct dev_pm_ops qpnp_qg_pm_ops = {
	.suspend_noirq	= qpnp_qg_suspend_noirq,
	.resume_noirq	= qpnp_qg_resume_noirq,
	.suspend	= qpnp_qg_suspend,
	.resume 	= qpnp_qg_resume,
};

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-04-24  OPPO_CHARGE */
#define DEFAULT_BATT_TEMP            250
#define DEFAULT_BATT_VOLT            3800
#define DEFAULT_BATT_SOC             50
#define WAIT_FOR_HEALTHD_SOC         -1
#define DEFAULT_BATT_CURRENT         500
#define MAX_WAIT_FOR_HEALTHD_COUNT   12
#define BATT_CAPACITY                4100
#define VBAT_HIGH_THRESHOLD          4500
#define TBAT_LOW_THRESHOLD           -190
#define TBAT_HIGH_THRESHOLD          530

static int qpnp_qg_get_battery_mvolts(void)
{
        int rc = 0, uv_bat = 0;

        if (!the_chip) {
                return DEFAULT_BATT_VOLT;
        }

        rc = qg_get_battery_voltage(the_chip, &uv_bat);
        if (rc < 0) {
                qg_debug("failed to get battery voltage, return 3800mV\n");
                return DEFAULT_BATT_VOLT;
        }

        /* if abnormal, read again */
        if (uv_bat > VBAT_HIGH_THRESHOLD * 1000) {
                msleep(80);
                qg_get_battery_voltage(the_chip, &uv_bat);
                if (rc < 0) {
                        qg_debug("failed to get battery voltage, return 3800mV\n");
                        return DEFAULT_BATT_VOLT;
                }
        }

        return uv_bat;
}

static int qpnp_qg_get_battery_temperature(void)
{
        int rc = 0, temp_bat = 0;

        if (!the_chip) {
                return DEFAULT_BATT_TEMP;
        }

        rc = qg_get_battery_temp(the_chip, &temp_bat);
        if (rc < 0) {
                qg_debug("failed to get battery temp, return 25C\n");
                return DEFAULT_BATT_TEMP;
        }

        /* if abnormal, read again */
        if (temp_bat < TBAT_LOW_THRESHOLD || temp_bat > TBAT_HIGH_THRESHOLD) {
                msleep(80);
                rc = qg_get_battery_temp(the_chip, &temp_bat);
                if (rc < 0) {
                        qg_debug("failed to get battery temp, return 25C\n");
                        return DEFAULT_BATT_TEMP;
                }
        }

        return temp_bat;
}

static int qpnp_qg_get_batt_remaining_capacity(void)
{
        return -1;
}

static int qpnp_qg_get_battery_soc(void)
{
        int soc_bat = 0;
        static int count = 0;

        if (!the_chip) {
                return DEFAULT_BATT_SOC;
        }

        qg_get_battery_capacity(the_chip, &soc_bat);

        if (get_boot_mode() == MSM_BOOT_MODE__RECOVERY) {
                return soc_bat;
        }

        if (healthd_ready == false && count < MAX_WAIT_FOR_HEALTHD_COUNT) {
                qg_debug("healthd not ready, count = %d\n", count);
                count ++;
                return WAIT_FOR_HEALTHD_SOC;
        }

        return soc_bat;
}

static int qpnp_qg_get_current(void)
{
        int ua_bat = 0;

        if (!the_chip) {
                return DEFAULT_BATT_CURRENT;
        }

        qg_get_battery_current(the_chip, &ua_bat);

        return ua_bat/1000;
}

static int qpnp_qg_get_battery_fcc(void)
{
        return BATT_CAPACITY;
}

static int qpnp_qg_get_battery_cc(void)
{
        return -1;
}

static int qpnp_qg_get_battery_soh(void)
{
        return -1;
}

static bool qpnp_qg_get_battery_authenticate(void)
{
	int rc, batt_id_mv;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		return false;
	}

	if (the_chip->oppo_battery_type != NON_STD_BATT) {
		return true;
	}

	rc = qpnp_vadc_read(the_chip->vadc_dev, VADC_BAT_ID_PU2, &result);
	if (rc) {
		qg_debug("Failed to read BATT_ID over vadc, rc=%d\n", rc);
		return false;
	}

	batt_id_mv = div_s64(result.physical, 1000);
	if (batt_id_mv == 0) {
		qg_debug("batt_id_mv = 0 from ADC\n");
		return false;
	}

	if (batt_id_mv >= the_chip->atl_battery_id_low && batt_id_mv <= the_chip->atl_battery_id_high) {
		the_chip->oppo_battery_type = OPPO_ATL_BATT;
		return true;
	} else if (batt_id_mv >= the_chip->sdi_battery_id_low && batt_id_mv <= the_chip->sdi_battery_id_high) {
		the_chip->oppo_battery_type = OPPO_SDI_BATT;
		return true;
	} else {
		the_chip->oppo_battery_type = NON_STD_BATT;
		return false;
	}
}

static void qpnp_qg_set_battery_full(bool enable)
{
}

static struct oppo_gauge_operations qpnp_qg_gauge = {
        .get_battery_mvolts             = qpnp_qg_get_battery_mvolts,
        .get_prev_battery_mvolts        = qpnp_qg_get_battery_mvolts,
        .get_battery_temperature        = qpnp_qg_get_battery_temperature,
        .get_batt_remaining_capacity    = qpnp_qg_get_batt_remaining_capacity,
        .get_battery_soc                = qpnp_qg_get_battery_soc,
        .get_prev_battery_soc           = qpnp_qg_get_battery_soc,
        .get_average_current            = qpnp_qg_get_current,
        .get_prev_average_current       = qpnp_qg_get_current,
        .get_battery_fcc                = qpnp_qg_get_battery_fcc,
        .get_battery_cc                 = qpnp_qg_get_battery_cc,
        .get_battery_soh                = qpnp_qg_get_battery_soh,
        .get_battery_authenticate       = qpnp_qg_get_battery_authenticate,
        .set_battery_full               = qpnp_qg_set_battery_full,
};
#endif

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-11  authenticate battery */
static void register_gauge_devinfo(struct qpnp_qg *chip)
{
        int rc = 0;
        char *version;
        char *manufacture;

	switch (chip->oppo_battery_type) {
	case OPPO_SDI_BATT:
		manufacture = "SDI";
		break;
	case OPPO_ATL_BATT:
		manufacture = "ATL";
		break;
	case OPPO_LG_BATT:
		manufacture = "LG";
		break;
	case NON_STD_BATT:
		manufacture = "UNKNOWN";
		break;
	default:
		manufacture = "UNKNOWN";
		break;
	}

	version = "4.4V NON_VOOC";

        rc = register_device_proc("battery", version, manufacture);
        if (rc) {
                pr_err("register_battery_devinfo fail\n");
        }
}
#endif

static int qpnp_qg_probe(struct platform_device *pdev)
{
	int rc = 0, soc = 0, nom_cap_uah;
	struct qpnp_qg *chip;

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-04-24  OPPO_CHARGE */
	struct oppo_gauge_chip *the_oppo_gauge_chip = NULL;
#endif

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		pr_err("Parent regmap is unavailable\n");
		return -ENXIO;
	}

	/* VADC for BID */
	chip->vadc_dev = qpnp_get_vadc(&pdev->dev, "qg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("Failed to find VADC node, rc=%d\n", rc);

		return rc;
	}

	chip->dev = &pdev->dev;
	chip->debug_mask = &qg_debug_mask;
	platform_set_drvdata(pdev, chip);
	INIT_WORK(&chip->udata_work, process_udata_work);
	INIT_WORK(&chip->qg_status_change_work, qg_status_change_work);
	mutex_init(&chip->bus_lock);
	mutex_init(&chip->soc_lock);
	mutex_init(&chip->data_lock);
	init_waitqueue_head(&chip->qg_wait_q);
	chip->maint_soc = -EINVAL;
	chip->batt_soc = INT_MIN;
	chip->cc_soc = INT_MIN;

	rc = qg_alg_init(chip);
	if (rc < 0) {
		pr_err("Error in alg_init, rc:%d\n", rc);
		return rc;
	}

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-04  authenticate battery */
        chip->oppo_battery_type = NON_STD_BATT;
#endif

	rc = qg_parse_dt(chip);
	if (rc < 0) {
		pr_err("Failed to parse DT, rc=%d\n", rc);
		return rc;
	}

	rc = qg_hw_init(chip);
	if (rc < 0) {
		pr_err("Failed to hw_init, rc=%d\n", rc);
		return rc;
	}

	rc = qg_setup_battery(chip);
	if (rc < 0) {
		pr_err("Failed to setup battery, rc=%d\n", rc);
		return rc;
	}

	rc = qg_register_device(chip);
	if (rc < 0) {
		pr_err("Failed to register QG char device, rc=%d\n", rc);
		return rc;
	}

	rc = qg_sdam_init(chip->dev);
	if (rc < 0) {
		pr_err("Failed to initialize QG SDAM, rc=%d\n", rc);
		return rc;
	}

	rc = qg_soc_init(chip);
	if (rc < 0) {
		pr_err("Failed to initialize SOC scaling init rc=%d\n", rc);
		return rc;
	}

	if (chip->profile_loaded) {
		if (!chip->dt.cl_disable) {
			/*
			 * Use FCC @ 25 C and charge-profile for
			 * Nominal Capacity
			 */
			rc = qg_get_nominal_capacity(&nom_cap_uah, 250, true);
			if (!rc) {
				rc = cap_learning_post_profile_init(chip->cl,
						nom_cap_uah);
				if (rc < 0) {
					pr_err("Error in cap_learning_post_profile_init rc=%d\n",
						rc);
					return rc;
				}
			}
		}
		rc = restore_cycle_count(chip->counter);
		if (rc < 0) {
			pr_err("Error in restoring cycle_count, rc=%d\n", rc);
			return rc;
		}
	}

	rc = qg_determine_pon_soc(chip);
	if (rc < 0) {
		pr_err("Failed to determine initial state, rc=%d\n", rc);
		goto fail_device;
	}

	chip->awake_votable = create_votable("QG_WS", VOTE_SET_ANY,
					 qg_awake_cb, chip);
	if (IS_ERR(chip->awake_votable)) {
		rc = PTR_ERR(chip->awake_votable);
		chip->awake_votable = NULL;
		goto fail_device;
	}

	chip->vbatt_irq_disable_votable = create_votable("QG_VBATT_IRQ_DISABLE",
				VOTE_SET_ANY, qg_vbatt_irq_disable_cb, chip);
	if (IS_ERR(chip->vbatt_irq_disable_votable)) {
		rc = PTR_ERR(chip->vbatt_irq_disable_votable);
		chip->vbatt_irq_disable_votable = NULL;
		goto fail_device;
	}

	chip->fifo_irq_disable_votable = create_votable("QG_FIFO_IRQ_DISABLE",
				VOTE_SET_ANY, qg_fifo_irq_disable_cb, chip);
	if (IS_ERR(chip->fifo_irq_disable_votable)) {
		rc = PTR_ERR(chip->fifo_irq_disable_votable);
		chip->fifo_irq_disable_votable = NULL;
		goto fail_device;
	}

	chip->good_ocv_irq_disable_votable =
			create_votable("QG_GOOD_IRQ_DISABLE",
			VOTE_SET_ANY, qg_good_ocv_irq_disable_cb, chip);
	if (IS_ERR(chip->good_ocv_irq_disable_votable)) {
		rc = PTR_ERR(chip->good_ocv_irq_disable_votable);
		chip->good_ocv_irq_disable_votable = NULL;
		goto fail_device;
	}

	rc = qg_init_psy(chip);
	if (rc < 0) {
		pr_err("Failed to initialize QG psy, rc=%d\n", rc);
		goto fail_votable;
	}

	rc = qg_request_irqs(chip);
	if (rc < 0) {
		pr_err("Failed to register QG interrupts, rc=%d\n", rc);
		goto fail_votable;
	}

	rc = qg_post_init(chip);
	if (rc < 0) {
		pr_err("Failed in qg_post_init rc=%d\n", rc);
		goto fail_votable;
	}

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-04-24  OPPO_CHARGE */
	the_chip = chip;
	if(use_qpnp_qg) {
		the_oppo_gauge_chip = devm_kzalloc(chip->dev,
			sizeof(struct oppo_gauge_chip), GFP_KERNEL);
		if (!the_oppo_gauge_chip) {
			pr_err("kzalloc() failed.\n");
			the_chip = NULL;
			return -ENOMEM;
		} else {
			the_oppo_gauge_chip->dev = chip->dev;
			the_oppo_gauge_chip->gauge_ops = &qpnp_qg_gauge;
			oppo_gauge_init(the_oppo_gauge_chip);
		}
	}
#endif

#ifdef VENDOR_EDIT
/* Yichun.Chen PSW.BSP.CHG  2018-05-11  authenticate battery */
        register_gauge_devinfo(chip);
#endif

#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-04  save soc */
        chip->batt_range_ocv = &qg_batt_valid_ocv;
        chip->batt_range_pct = &qg_batt_range_pct;
        chip->soc_reporting_ready = true;
        memset(chip->batt_info, 0, sizeof(chip->batt_info));
#endif

#ifdef VENDOR_EDIT
/* Yichun.Chen  PSW.BSP.CHG  2018-06-13  avoid when reboot soc reduce 1% */
        chip->skip_scale_soc_count = 0;
#endif

	qg_get_battery_capacity(chip, &soc);
	pr_info("QG initialized! battery_profile=%s SOC=%d\n",
				qg_get_battery_type(chip), soc);

	return rc;

fail_votable:
	destroy_votable(chip->awake_votable);
fail_device:
	device_destroy(chip->qg_class, chip->dev_no);
	cdev_del(&chip->qg_cdev);
	unregister_chrdev_region(chip->dev_no, 1);
	return rc;
}

static int qpnp_qg_remove(struct platform_device *pdev)
{
	struct qpnp_qg *chip = platform_get_drvdata(pdev);

	qg_batterydata_exit();
	qg_soc_exit(chip);

	cancel_work_sync(&chip->udata_work);
	cancel_work_sync(&chip->qg_status_change_work);
	device_destroy(chip->qg_class, chip->dev_no);
	cdev_del(&chip->qg_cdev);
	unregister_chrdev_region(chip->dev_no, 1);
	mutex_destroy(&chip->bus_lock);
	mutex_destroy(&chip->data_lock);
	mutex_destroy(&chip->soc_lock);
	if (chip->awake_votable)
		destroy_votable(chip->awake_votable);

	return 0;
}

static void qpnp_qg_shutdown(struct platform_device *pdev)
{
	struct qpnp_qg *chip = platform_get_drvdata(pdev);

	if (!is_usb_present(chip) || !chip->profile_loaded)
		return;
	/*
	 * Charging status doesn't matter when the device shuts down and we
	 * have to treat this as charge done. Hence pass charge_done as true.
	 */
	cycle_count_update(chip->counter,
			DIV_ROUND_CLOSEST(chip->msoc * 255, 100),
			POWER_SUPPLY_STATUS_NOT_CHARGING,
			true, chip->usb_present);
}

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,qpnp-qg", },
	{ },
};

static struct platform_driver qpnp_qg_driver = {
	.driver		= {
		.name		= "qcom,qpnp-qg",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
		.pm		= &qpnp_qg_pm_ops,
	},
	.probe		= qpnp_qg_probe,
	.remove		= qpnp_qg_remove,
	.shutdown	= qpnp_qg_shutdown,
};
module_platform_driver(qpnp_qg_driver);

MODULE_DESCRIPTION("QPNP QG Driver");
MODULE_LICENSE("GPL v2");
