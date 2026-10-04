// SPDX-License-Identifier: GPL-2.0-only
/*
 * huawei-gaokun-battery - A power supply driver for HUAWEI Matebook E Go
 *
 * Copyright (C) 2024 Pengyu Luo <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_data/huawei-gaokun-ec.h>
#include <linux/power_supply.h>
#include <linux/sprintf.h>

/* -------------------------------------------------------------------------- */
/* String Data Reg */

#define EC_BAT_VENDOR		0x01 /* from 0x01 to 0x0F, SUNWODA */
#define EC_BAT_MODEL		0x11 /* from 0x11 to 0x1F, HB30A8P9ECW-22T */

#define EC_ADP_STATUS		0x81
#define EC_AC_STATUS		BIT(0)
#define EC_BAT_PRESENT		BIT(1) /* BATC._STA */

#define EC_BAT_STATUS		0x82 /* _BST */
#define EC_BAT_DISCHARGING	BIT(0)
#define EC_BAT_CHARGING		BIT(1)
#define EC_BAT_CRITICAL		BIT(2) /* Low Battery Level */
#define EC_BAT_FULL		BIT(3)
#define EC_BAT_STATUS_KNOWN	(EC_BAT_DISCHARGING | EC_BAT_CHARGING | \
				 EC_BAT_CRITICAL | EC_BAT_FULL)

/* -------------------------------------------------------------------------- */
/* Word Data Reg */

/* 0x5A: ?
 * 0x5C: ?
 * 0x5E: ?
 * 0X60: ?
 * 0x84: ?
 */

#define EC_BAT_STATUS_START	0x90
#define EC_BAT_PERCENTAGE	0x90
#define EC_BAT_VOLTAGE		0x92
#define EC_BAT_CAPACITY		0x94
#define EC_BAT_FULL_CAPACITY	0x96
/* 0x98: ? */
#define EC_BAT_CURRENT		0x9A
/* 0x9C: ? */

#define EC_BAT_INFO_START	0xA0
/* 0xA0: POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT? */
#define EC_BAT_DESIGN_CAPACITY	0xA2
#define EC_BAT_DESIGN_VOLTAGE	0xA4
#define EC_BAT_SERIAL_NUMBER	0xA6
#define EC_BAT_CYCLE_COUNT	0xAA

/* -------------------------------------------------------------------------- */
/* Battery Event ID */

#define EC_EVENT_BAT_A0		0xA0
#define EC_EVENT_BAT_A1		0xA1
#define EC_EVENT_BAT_A2		0xA2
#define EC_EVENT_BAT_A3		0xA3
#define EC_EVENT_BAT_B1		0xB1
/* EVENT B1 A0 A1 repeat about every 1s 2s 3s respectively */

/* ACPI _BIX field, Min sampling time, the duration between two _BST */
#define CACHE_TIME		2000 /* cache time in milliseconds */

#define MILLI_TO_MICRO		1000

#define SMART_CHARGE_MODE	0
#define SMART_CHARGE_DELAY	1
#define SMART_CHARGE_START	2
#define SMART_CHARGE_END	3

#define NO_DELAY_MODE	1
#define DELAY_MODE	4

/*
 * EC 的 CRITICAL 位在多少电量置位，至今没有实测（1.0 计划 BATT-3）。如果它在
 * 5% 以上就置位，直接拿它报 Critical 会让 Android 过早关机，所以只在电量
 * 不高于这个值时才把它升级成 Critical；0 = 只在 0% 时才认它。
 */
static int critical_max_capacity = 2;
module_param(critical_max_capacity, int, 0644);
MODULE_PARM_DESC(critical_max_capacity,
		 "Report CAPACITY_LEVEL Critical on the EC critical bit only at or below this capacity (%)");

struct gaokun_psy_bat_status {
	__le16 percentage_now;	/* 0x90 */
	__le16 voltage_now;
	__le16 capacity_now;
	__le16 full_capacity;
	__le16 unknown1;
	__le16 rate_now;
	__le16 unknown2;	/* 0x9C */
} __packed;

struct gaokun_psy_bat_info {
	__le16 unknown3;	/* 0xA0 */
	__le16 design_capacity;
	__le16 design_voltage;
	__le16 serial_number;
	__le16 padding2;
	__le16 cycle_count;	/* 0xAA */
} __packed;

struct gaokun_psy {
	struct gaokun_ec *ec;
	struct device *dev;
	struct notifier_block nb;

	struct power_supply *bat_psy;
	struct power_supply *adp_psy;

	unsigned long update_time;
	struct gaokun_psy_bat_status status;
	struct gaokun_psy_bat_info info;
	struct gpio_desc *bat_sta;

	char battery_model[0x10]; /* HB30A8P9ECW-22T, the real one is XXX-22A */
	char battery_serial[0x10];
	char battery_vendor[0x10];

	int charge_now;
	u8 bat_status_raw;	/* 0x82 最近一次读成功的原始值 */
	int online;
	int bat_present;
};

/* -------------------------------------------------------------------------- */
/* Adapter */

static int gaokun_psy_get_adp_status(struct gaokun_psy *ecbat)
{
	/* _PSR */
	int ret;
	u8 online;

	ret = gaokun_ec_psy_read_byte(ecbat->ec, EC_ADP_STATUS, &online);
	if (ret)
		return ret;

	ecbat->online = !!(online & EC_AC_STATUS);

	return 0;
}

static int gaokun_psy_get_adp_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	int ret;

	ret = gaokun_psy_get_adp_status(ecbat);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ecbat->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_C;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property gaokun_psy_adp_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc gaokun_psy_adp_desc = {
	.name		= "gaokun-ec-adapter",
	.type		= POWER_SUPPLY_TYPE_USB,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_C),
	.get_property	= gaokun_psy_get_adp_property,
	.properties	= gaokun_psy_adp_props,
	.num_properties	= ARRAY_SIZE(gaokun_psy_adp_props),
};

/* -------------------------------------------------------------------------- */
/* Battery */

static void gaokun_psy_get_bat_present(struct gaokun_psy *ecbat)
{
	/* _STA */
	int ret, val;
	u8 present;

	/* in output mode, in and out bit are synced, so read the in bit */
	val = gpiod_get_value(ecbat->bat_sta);
	if (!val) {
		/* Some kind of initialization */
		gpiod_set_value(ecbat->bat_sta, 1);
		gaokun_ec_write(ecbat->ec, (u8 []){0x02, 0xB2, 1, 0x90});
	}

	ret = gaokun_ec_psy_read_byte(ecbat->ec, EC_ADP_STATUS, &present);

	ecbat->bat_present = ret ? false : !!(present & EC_BAT_PRESENT);
}

static inline int gaokun_psy_bat_present(struct gaokun_psy *ecbat)
{
	return ecbat->bat_present;
}

static int gaokun_psy_get_bat_info(struct gaokun_psy *ecbat)
{
	/* _BIX */
	if (!gaokun_psy_bat_present(ecbat)) /* TODO: ensure info is erased */
		return 0;

	return gaokun_ec_psy_multi_read(ecbat->ec, EC_BAT_INFO_START,
					sizeof(ecbat->info), (u8 *)&ecbat->info);
}

/*
 * 0x82 按位解码。它与 ACPI _BST 的 state 字段同构（bit0 放电、bit1 充电、
 * bit2 电量危急），bit3 是 EC 自己的"充满"。原实现按整个字节精确匹配，
 * 0x00（插着电但不充）和任何组合值（如 0x05 = 放电 + 危急）都只打一条警告、
 * status 停在上一次的值 —— 于是"充电中"可以在电量一路下降时一直挂着。
 *
 * 优先级：放电位压过其他位（0x05 / 0x09 / 矛盾的 0x03 都报放电 —— 宁可让
 * Android 的关机规则生效，也不要被一个过时的"充电中"挡住）；其次充满、充电；
 * 一位都没有时，插着电报 NOT_CHARGING，没插电报放电。
 */
static int gaokun_psy_decode_bat_status(struct gaokun_psy *ecbat, u8 raw)
{
	if (raw & EC_BAT_DISCHARGING)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (raw & EC_BAT_FULL)
		return POWER_SUPPLY_STATUS_FULL;
	if (raw & EC_BAT_CHARGING)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (gaokun_psy_get_adp_status(ecbat))
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return ecbat->online ? POWER_SUPPLY_STATUS_NOT_CHARGING :
			       POWER_SUPPLY_STATUS_DISCHARGING;
}

static void gaokun_psy_update_bat_charge(struct gaokun_psy *ecbat)
{
	u8 charge;

	/* 读失败时 charge 是未初始化的栈变量，原实现照样拿去解码 */
	if (gaokun_ec_psy_read_byte(ecbat->ec, EC_BAT_STATUS, &charge))
		return;

	if (charge & ~EC_BAT_STATUS_KNOWN)
		dev_warn_once(ecbat->dev, "unknown charge state bits 0x%02x\n", charge);

	ecbat->bat_status_raw = charge;
	ecbat->charge_now = gaokun_psy_decode_bat_status(ecbat, charge);
}

/*
 * Android 的 BatteryService.shouldShutdownLocked()（本机 services.jar 反汇编，
 * 1.0 计划 BATT-1）：capacity_level 节点存在时【只】在它等于 Critical 时关机；
 * 不存在时才退回"电量 <= 0 且 status 不是充电中"。所以一旦导出这个属性，
 * 0% 关机就完全靠这里报 Critical，必须覆盖原规则的全部情形：
 *
 *   * 0%，且不是"EC 说在充、电流也确实为正" ⇒ Critical。这比原规则多出的
 *     那一种 —— 插着电脑 USB 口这类弱电源，EC 报充电、电量照掉 —— 正是
 *     BATT-1（09-14 那次没关机画面的断电）。
 *   * EC 的 CRITICAL 位、电量不高于 critical_max_capacity ⇒ Critical
 *     （同样排除"确实在充"），给 EC 自己的低电判断留一点余量（BATT-2）。
 *
 * ⚠️ current_now 的正负约定没有带负载实测过：2026-10-05 插着 Mac 的 USB 口、
 *    status=Charging、电量 92% 时 7 次采样里 5 次为 +15…+106 mA、2 次为
 *    -19 / -57 mA，与"正 = 流入电池"一致，但只是旁证（实测步骤见 BATT-1）。
 *    约定若是反的：0% 接着真充电器会被报 Critical（多关一次机），弱电源下
 *    掉到 0% 则仍不报（退回修之前的行为）—— 验收时两种都要看。
 */
static int gaokun_psy_capacity_level(struct gaokun_psy *ecbat)
{
	int cap = le16_to_cpu(ecbat->status.percentage_now);
	int rate = (s16)le16_to_cpu(ecbat->status.rate_now);
	u8 raw = ecbat->bat_status_raw;
	bool gaining = (raw & EC_BAT_CHARGING) && !(raw & EC_BAT_DISCHARGING) &&
		       rate > 0;

	if (!gaining && (cap <= 0 ||
			 ((raw & EC_BAT_CRITICAL) && cap <= critical_max_capacity)))
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

	if (ecbat->charge_now == POWER_SUPPLY_STATUS_FULL || cap >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;

	if (raw & EC_BAT_CRITICAL)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;

	return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

static int gaokun_psy_get_bat_status(struct gaokun_psy *ecbat)
{
	/* _BST */
	int ret;

	if (time_before(jiffies, ecbat->update_time +
			msecs_to_jiffies(CACHE_TIME)))
		return 0;

	gaokun_psy_update_bat_charge(ecbat);
	ret = gaokun_ec_psy_multi_read(ecbat->ec, EC_BAT_STATUS_START,
				       sizeof(ecbat->status), (u8 *)&ecbat->status);

	ecbat->update_time = jiffies;

	return ret;
}

static void gaokun_psy_init(struct gaokun_psy *ecbat)
{
	gaokun_psy_get_bat_present(ecbat);
	if (!gaokun_psy_bat_present(ecbat))
		return;

	gaokun_psy_get_bat_info(ecbat);

	snprintf(ecbat->battery_serial, sizeof(ecbat->battery_serial),
		 "%d", le16_to_cpu(ecbat->info.serial_number));

	gaokun_ec_psy_multi_read(ecbat->ec, EC_BAT_VENDOR,
				 sizeof(ecbat->battery_vendor) - 1,
				 ecbat->battery_vendor);

	gaokun_ec_psy_multi_read(ecbat->ec, EC_BAT_MODEL,
				 sizeof(ecbat->battery_model) - 1,
				 ecbat->battery_model);

	ecbat->battery_model[14] = 'A'; /* FIX UP */
}

static int gaokun_psy_get_bat_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	u8 buf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	int ret;

	if (gaokun_psy_bat_present(ecbat))
		gaokun_psy_get_bat_status(ecbat);
	else if (psp != POWER_SUPPLY_PROP_PRESENT)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ecbat->charge_now;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ecbat->bat_present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = le16_to_cpu(ecbat->info.cycle_count);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = le16_to_cpu(ecbat->info.design_voltage) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = le16_to_cpu(ecbat->status.voltage_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (s16)le16_to_cpu(ecbat->status.rate_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(ecbat->info.design_capacity) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = le16_to_cpu(ecbat->status.full_capacity) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = le16_to_cpu(ecbat->status.capacity_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		ret = gaokun_ec_psy_get_smart_charge(ecbat->ec, buf);
		if (ret)
			return ret;

		if (psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD)
			val->intval = buf[SMART_CHARGE_START];
		else
			val->intval = buf[SMART_CHARGE_END];
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = le16_to_cpu(ecbat->status.percentage_now);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = gaokun_psy_capacity_level(ecbat);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = ecbat->battery_model;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = ecbat->battery_vendor;
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = ecbat->battery_serial;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int gaokun_psy_set_bat_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	u8 buf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	int ret;

	if (!gaokun_psy_bat_present(ecbat))
		return -ENODEV;

	ret = gaokun_ec_psy_get_smart_charge(ecbat->ec, buf);
	if (ret)
		return ret;

	switch (psp) {
	/*
	 * Resetting another thershold makes single thersold setting more likely
	 * to succeed. But setting start = end makes thing strange(failure).
	 */
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		buf[SMART_CHARGE_START] = val->intval;
		if (buf[SMART_CHARGE_START] > buf[SMART_CHARGE_END])
			buf[SMART_CHARGE_END] = buf[SMART_CHARGE_START] + 1;
		return gaokun_ec_psy_set_smart_charge(ecbat->ec, buf);

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		buf[SMART_CHARGE_END] = val->intval;
		if (buf[SMART_CHARGE_END] < buf[SMART_CHARGE_START])
			buf[SMART_CHARGE_START] = buf[SMART_CHARGE_END] - 1;
		return gaokun_ec_psy_set_smart_charge(ecbat->ec, buf);

	default:
		return -EINVAL;
	}

	return 0;
}

static int gaokun_psy_is_bat_property_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static enum power_supply_property gaokun_psy_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc gaokun_psy_bat_desc = {
	.name			= "gaokun-ec-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= gaokun_psy_get_bat_property,
	.set_property		= gaokun_psy_set_bat_property,
	.property_is_writeable	= gaokun_psy_is_bat_property_writeable,
	.properties		= gaokun_psy_bat_props,
	.num_properties		= ARRAY_SIZE(gaokun_psy_bat_props),
};

/* -------------------------------------------------------------------------- */
/* Sysfs */

/*
 * Note that, HUAWEI calls them SBAC/GBAC and SBCM/GBCM in DSDT, they are likely
 * Set/Get Battery Adaptive Charging and Set/Get Battery Charging Mode.
 */

/* battery adaptive charge */
static ssize_t battery_adaptive_charge_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = to_power_supply(dev);
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	int ret;
	bool on;

	ret = gaokun_ec_psy_get_smart_charge_enable(ecbat->ec, &on);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", on);
}

static ssize_t battery_adaptive_charge_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t size)
{
	struct power_supply *psy = to_power_supply(dev);
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	int ret;
	bool on;

	if (kstrtobool(buf, &on))
		return -EINVAL;

	ret = gaokun_ec_psy_set_smart_charge_enable(ecbat->ec, on);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(battery_adaptive_charge);

static inline int get_charge_delay(u8 buf[GAOKUN_SMART_CHARGE_DATA_SIZE])
{
	return buf[SMART_CHARGE_MODE] == NO_DELAY_MODE ? 0 : buf[SMART_CHARGE_DELAY];
}

static inline void
set_charge_delay(u8 buf[GAOKUN_SMART_CHARGE_DATA_SIZE], u8 delay)
{
	if (delay) {
		buf[SMART_CHARGE_DELAY] = delay;
		buf[SMART_CHARGE_MODE] = DELAY_MODE;
	} else {
		 /* No writing zero, there is a specific mode for it. */
		buf[SMART_CHARGE_MODE] = NO_DELAY_MODE;
	}
}

/* Smart charge */
static ssize_t smart_charge_delay_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = to_power_supply(dev);
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	u8 bf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	int ret;

	ret = gaokun_ec_psy_get_smart_charge(ecbat->ec, bf);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", get_charge_delay(bf));
}

static ssize_t smart_charge_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct power_supply *psy = to_power_supply(dev);
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	u8 bf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	u8 delay;
	int ret;

	if (kstrtou8(buf, 10, &delay))
		return -EINVAL;

	ret = gaokun_ec_psy_get_smart_charge(ecbat->ec, bf);
	if (ret)
		return ret;

	set_charge_delay(bf, delay);

	ret = gaokun_ec_psy_set_smart_charge(ecbat->ec, bf);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(smart_charge_delay);

/*
 * ec_raw：只读，给实测用（1.0 计划 BATT-1/2 的放电采样、电池温度 LIVE-11/PERF-11）。
 * 打出 0x82 的原始状态字节，以及三个含义未知的字：0x98、0x9C（状态块，随 _BST
 * 每 2 秒读一次）和 0xA0（_BIX 块，事件时刷新）—— 都是驱动本来就在读的，不增加
 * EC 事务。电池温度若在 EC 里，最可能就在这几个字中（例如以 0.1 K 为单位时约
 * 2980），但这只是猜测，所以【不】据此导出 POWER_SUPPLY_PROP_TEMP：
 * 报一个错的温度比报不出来更糟（温度高了会触发框架的过温关机）。
 */
static ssize_t ec_raw_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct power_supply *psy = to_power_supply(dev);
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);

	if (!gaokun_psy_bat_present(ecbat))
		return -ENODEV;

	gaokun_psy_get_bat_status(ecbat);

	return sysfs_emit(buf, "status=0x%02x w98=0x%04x w9c=0x%04x wa0=0x%04x\n",
			  ecbat->bat_status_raw,
			  le16_to_cpu(ecbat->status.unknown1),
			  le16_to_cpu(ecbat->status.unknown2),
			  le16_to_cpu(ecbat->info.unknown3));
}

static DEVICE_ATTR_RO(ec_raw);

static struct attribute *gaokun_psy_features_attrs[] = {
	&dev_attr_battery_adaptive_charge.attr,
	&dev_attr_smart_charge_delay.attr,
	&dev_attr_ec_raw.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gaokun_psy_features);

static int gaokun_psy_notify(struct notifier_block *nb,
			     unsigned long action, void *data)
{
	struct gaokun_psy *ecbat = container_of(nb, struct gaokun_psy, nb);

	switch (action) {
	case EC_EVENT_BAT_A2:
	case EC_EVENT_BAT_B1:
		gaokun_psy_get_bat_info(ecbat);
		return NOTIFY_OK;

	case EC_EVENT_BAT_A0:
		gaokun_psy_get_adp_status(ecbat);
		power_supply_changed(ecbat->adp_psy);
		msleep(10);
		fallthrough;

	case EC_EVENT_BAT_A1:
	case EC_EVENT_BAT_A3:
		if (action == EC_EVENT_BAT_A3) {
			gaokun_psy_get_bat_present(ecbat);
			gaokun_psy_get_bat_info(ecbat);
			msleep(100);
		}
		gaokun_psy_get_bat_status(ecbat);
		power_supply_changed(ecbat->bat_psy);
		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int gaokun_psy_probe(struct auxiliary_device *adev,
			    const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &adev->dev;
	struct gaokun_psy *ecbat;

	ecbat = devm_kzalloc(&adev->dev, sizeof(*ecbat), GFP_KERNEL);
	if (!ecbat)
		return -ENOMEM;

	ecbat->ec = ec;
	ecbat->dev = dev;
	ecbat->nb.notifier_call = gaokun_psy_notify;

	ecbat->bat_sta = devm_gpiod_get_optional(dev, "bat", GPIOD_OUT_LOW);
	if (IS_ERR(ecbat->bat_sta))
		dev_err_probe(dev, PTR_ERR(ecbat->bat_sta),
			      "Failed to get bat-gpios\n");

	auxiliary_set_drvdata(adev, ecbat);

	psy_cfg.drv_data = ecbat;
	psy_cfg.no_wakeup_source = true;
	ecbat->adp_psy = devm_power_supply_register(dev, &gaokun_psy_adp_desc,
						    &psy_cfg);
	if (IS_ERR(ecbat->adp_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->adp_psy),
				     "Failed to register AC power supply\n");

	psy_cfg.supplied_to = (char **)&gaokun_psy_bat_desc.name;
	psy_cfg.num_supplicants = 1;
	psy_cfg.attr_grp = gaokun_psy_features_groups;
	ecbat->bat_psy = devm_power_supply_register(dev, &gaokun_psy_bat_desc,
						    &psy_cfg);
	if (IS_ERR(ecbat->bat_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->bat_psy),
				     "Failed to register battery power supply\n");
	gaokun_psy_init(ecbat);

	return gaokun_ec_register_notify(ec, &ecbat->nb);
}

static void gaokun_psy_remove(struct auxiliary_device *adev)
{
	struct gaokun_psy *ecbat = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(ecbat->ec, &ecbat->nb);
}

static const struct auxiliary_device_id gaokun_psy_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_PSY, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, gaokun_psy_id_table);

static struct auxiliary_driver gaokun_psy_driver = {
	.name = GAOKUN_DEV_PSY,
	.id_table = gaokun_psy_id_table,
	.probe = gaokun_psy_probe,
	.remove = gaokun_psy_remove,
};

module_auxiliary_driver(gaokun_psy_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go psy driver");
MODULE_LICENSE("GPL");
