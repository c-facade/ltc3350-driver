/*
 * Driver for Linear Technology LTC3350 High Current Supercapacitor Backup Controller and System Monitor
 *
 * Datasheet: https://www.analog.com/en/products/ltc3350.html?doc=LTC3350.pdf
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/string.h>

// READ/WRITE

#define CLR_ALARMS 0x00
#define MSK_ALARMS 0x01
#define MSK_MON_STATUS 0x02
#define CAP_ESR_PER 0x04
#define VCAPFB_DAC 0x05
#define VSHUNT 0x06
#define CAP_UV_LVL 0x07
#define CAP_OV_LVL 0x08
#define GPI_UV_LVL 0x09
#define GPI_OV_LVL 0x0A
#define VIN_UV_LVL 0x0B
#define VIN_OV_LVL 0x0C
#define VCAP_UV_LVL 0x0D
#define VCAP_OV_LVL 0x0E
#define VOUT_UV_LVL 0x0F
#define VOUT_OV_LVL 0x10
#define IIN_OC_LVL 0x11
#define ICHG_UC_LVL 0x12
#define DTEMP_COLD_LVL 0x13
#define DTEMP_HOT_LVL 0x14
#define ESR_HI_LVL 0x15
#define CAP_LO_LVL 0x16
#define CTL_REG 0x17

// READONLY

#define NUM_CAPS 0x1A
#define CHRG_STATUS 0x1B
#define MON_STATUS 0x1C
#define ALARM_REG 0x1D
#define MEAS_CAP 0x1E
#define MEAS_ESR 0x1F
#define MEAS_VCAP1 0x20
#define MEAS_VCAP2 0x21
#define MEAS_VCAP3 0x22
#define MEAS_VCAP4 0x23
#define MEAS_GPI 0x24
#define MEAS_VIN 0x25
#define MEAS_VCAP 0x26
#define MEAS_VOUT 0x27
#define MEAS_IIN 0x28
#define MEAS_ICHG 0x29
#define MEAS_DTEMP 0x2A

// alarm_reg bits
#define ALARM_CAP_UV BIT(0) //capacitor undervoltage alarm
#define ALARM_CAP_OV BIT(1) //capacitor overvoltage alarm
#define ALARM_GPI_UV BIT(2) //gpi undervoltage alarm
#define ALARM_GPI_OV BIT(3) //gpi overvoltage alarm
#define ALARM_VIN_UV BIT(4) //vin undervoltage alarm
#define ALARM_VIN_OV BIT(5) //vin overvoltage alarm
#define ALARM_VCAP_UV BIT(6) //vcap undervoltage alarm
#define ALARM_VCAP_OV BIT(7) //vcap overvoltage alarm
#define ALARM_VOUT_UV BIT(8) //vout undervoltage alarm
#define ALARM_VOUT_OV BIT(9) //vout overvoltage alarm
#define ALARM_IIN_OC BIT(10) //input overcurrent alarm
#define ALARM_ICHG_UC BIT(11) //charge undercurrent alarm
#define ALARM_DTEMP_COLD BIT(12) //die temperature cold alarm
#define ALARM_DTEMP_HOT BIT(13) //die temperature hot alarm
#define ALARM_ESR_HI BIT(14) //esr high alarm
#define ALARM_CAP_LO BIT(15) //capacitance low alarm

// mon status bits
#define MON_CAPSR_ACTIVE BIT(0)
#define MON_CAPESR_SCHEDULED BIT(1)
#define MON_CAPESR_PENDING BIT(2)
#define MON_CAP_DONE BIT(3)
#define MON_ESR_DONE BIT(4)
#define MON_CAP_FAILED BIT(5)
#define MON_ESR_FAILED BIT(6)
#define MON_POWER_FAILED BIT(8)
#define MON_POWER_RETURNED BIT(9)

#define log_alarm(alarm_name, reg1, reg2) \
	if (alarm_value & alarm_name) { \
		dev_info(dev, #alarm_name "\n"); \
	}

// declared for conveniency
static void ltc3350_show_alarms(unsigned int alarm_value, unsigned int monitor_value, struct i2c_client *client);
static void ltc3350_handle_initial_measurements(struct i2c_client *client, int monitor_value);

/**
 * struct ltc3350_hwmon_data
 * @client - reference to i2c client device
 *
 * This data will be associated to the hwmon_device that will
 * be registered in the probe function.
 * This will be available in the show/store functions
*/
struct ltc3350_hwmon_data {
	struct i2c_client *client;
};

/*
 * struct ltc3350_data
 * @hwmon_dev - hwmon device
 * @cap_esr_num - number of capacitance/esr measurements done
 * @initial_meas - cap_esr measurements to be done on first power up
 * @cap_esr_per - period between cap_esr_measurements, after initial measurements.
 *
 * This data will be associated to the i2c client, so that it
 * will be available in the alert function.
*/
struct ltc3350_data {
	struct device *hwmon_dev;
	int cap_esr_num; // number of capacitance/esr measurement
	int initial_meas; // cap_esr measurements to be done on first power up
	int cap_esr_per;
};


// READ/WRITE on BUS -----------------------------------------------

/**
 * ltc3350_get_value - Read directly from register
 * @i2c - i2c client
 * @index - the index of the attribute to read
 * @result - value read
 * Return: 0 on success, propagates error otherwise.
*/
static int ltc3350_get_value(struct i2c_client *i2c, int index, int *result)
{
	u8 reg = index;
	// Read the 32-bit value from the I2C device
  int32_t raw_val = i2c_smbus_read_word_data(i2c, reg);

  // Extract the lower 16 bits
  uint16_t u16_val = (uint16_t)(raw_val & 0xFFFF);

  // Convert to signed 16-bit integer
  int16_t signed_val = (int16_t)u16_val;
	/*
	int val = i2c_smbus_read_word_data(i2c, reg);
	if (unlikely(val < 0))
		return val;
	*/
	*result = signed_val;
	return 0;
}

/**
 * ltc3350_set_value - write directly into register
 * @i2c - i2c client
 * @index - index of the attribute to write on
 * @buf - the content to write
*/
static int ltc3350_set_value(struct i2c_client *i2c,
		int index, const char *buf)
{
	u8 reg = index;
	long val;
	uint16_t u16_val;
	int ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;

	u16_val = (uint16_t)(val & 0xFFFF);

	ret = i2c_smbus_write_word_data(i2c, reg, u16_val);
	if (unlikely(ret < 0))
		return ret;
	return 0;
}

//---------------------------------------------------------------------------

/**
 * ltc3350_value_show() - expose attribute value
 * @dev - hwmon device
 * @da - device attribute to expose
 * @buf - allocated buffer for sysfs_emit
*/
static ssize_t ltc3350_value_show(struct device *dev,
								struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ltc3350_hwmon_data *data = dev_get_drvdata(dev);
	int value;
	int ret;

	if (IS_ERR(data))
		return PTR_ERR(data);

	ret = ltc3350_get_value(data->client, attr->index, &value);

	if (unlikely(ret < 0)) {
		return ret;
	}

	return sysfs_emit(buf, "%d\n", value);
}

// STORE ATTRIBUTE VALUE THROUGH SYSFS
/**
 * ltc3350_value_store() - store value on register
 * @dev - hwmon device
 * @da - attribute to write on
 * @buf - buffer containing sysfs data
 * @count - size of contents
 * Return: count on success, an error code otherwise.
*/
static ssize_t ltc3350_value_store(struct device *dev,
		struct device_attribute *da, const char *buf,
		size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ltc3350_hwmon_data *data = dev_get_drvdata(dev);
	int ret;

	if (IS_ERR(data))
		return PTR_ERR(data);

	ret = ltc3350_set_value(data->client, attr->index, buf);

	if (unlikely(ret < 0)) {
		return ret;
	}

	return count;
}

// SMBUS ALERT ----------------------------------------------------------

/**
 * ltc3350_alert() - handle smbus alert
 * @client - i2c client
 * @i2c_alert_protocol - unused
 * @data - eventual associated data
 *
 * This function handles SMBUS ALERTS.
 * Since the alert interrupt handler is present in the device
 * tree, the ARA client is initialized automatically.
 * Logs information on active alarms.
 * The sysfs_notify call allows a userspace application to poll for new alerts.
*/
static void ltc3350_alert(struct i2c_client *client, enum i2c_alert_protocol, unsigned int data){
	int alarm_value;
	int monitor_value;
	int ret1, ret2;
	struct kobject *kobj;
	struct device *hwmon_dev;
	struct ltc3350_data *client_data = dev_get_drvdata(&client->dev);
	hwmon_dev = client_data->hwmon_dev;

	dev_dbg(&client->dev, "Alert from device at address 0x%02x\n", client->addr);
	ret1 = ltc3350_get_value(client, ALARM_REG, &alarm_value);
	ret2 = ltc3350_get_value(client, MON_STATUS, &monitor_value);

	if (unlikely(ret1<0 || ret2 < 0))
		dev_err(&client->dev, "Error reading value of ALARM_REG or MON_STATUS\n");

	ltc3350_show_alarms(alarm_value, monitor_value, client);

	if (hwmon_dev == NULL) {
		dev_err(&client->dev, "Error getting hwmon_dev. Sysfs notifications disabled.\n");
	} else {
		kobj = &hwmon_dev->kobj;
		sysfs_notify(kobj, NULL, "alarm_reg");
		sysfs_notify(kobj, NULL, "mon_status");
	}
}



/*
 * On receiving an alert, this function logs which alarm has
 * been received.
*/
static void ltc3350_show_alarms(unsigned int alarm_value, unsigned int monitor_value, struct i2c_client *client){
	struct device *dev = &client->dev;
	struct ltc3350_data *devdata = dev_get_drvdata(dev);
	if (devdata->cap_esr_num <= devdata->initial_meas) {
		ltc3350_handle_initial_measurements(client, monitor_value);
	} else {
		if(monitor_value & MON_CAPSR_ACTIVE)
			dev_info(dev, "Capacitance/ESR measurement is in progress");
		if(monitor_value & MON_CAPESR_SCHEDULED)
			dev_info(dev, "Waiting programmed time to begin a capacitance/ESR measurement");
		if(monitor_value & MON_CAPESR_PENDING)
			dev_info(dev, "Waiting for satisfactory conditions to begin a capacitance/ESR measurement");
		if(monitor_value & MON_CAP_DONE)
			dev_info(dev, "Capacitance measurement has completed");
		if(monitor_value & MON_ESR_DONE)
			dev_info(dev, "ESR Measurement has completed");
		if(monitor_value & MON_CAP_FAILED)
			dev_info(dev, "The last attempted capacitance measurement was unable to complete");
		if(monitor_value & MON_ESR_FAILED)
			dev_info(dev, "The last attempted ESR measurement was unable to complete");
		if(monitor_value & MON_POWER_FAILED)
			dev_info(dev, "The device is no longer charging");
		if(monitor_value & MON_POWER_RETURNED)
			dev_info(dev, "The device is charging");
	}

	log_alarm(ALARM_CAP_UV, MEAS_CAP, CAP_UV_LVL)
	log_alarm(ALARM_CAP_OV, MEAS_CAP, CAP_OV_LVL)
	log_alarm(ALARM_GPI_UV, MEAS_GPI, GPI_UV_LVL)
	log_alarm(ALARM_GPI_OV, MEAS_GPI, GPI_OV_LVL)
	log_alarm(ALARM_VIN_UV, MEAS_VIN, VIN_UV_LVL)
	log_alarm(ALARM_VIN_OV, MEAS_VIN, VIN_OV_LVL)
	log_alarm(ALARM_VCAP_UV, MEAS_VCAP, VCAP_UV_LVL)
	log_alarm(ALARM_VCAP_OV, MEAS_VCAP, VCAP_OV_LVL)
	log_alarm(ALARM_VOUT_UV, MEAS_VOUT, VOUT_UV_LVL)
	log_alarm(ALARM_VOUT_OV, MEAS_VOUT, VOUT_OV_LVL)
	log_alarm(ALARM_IIN_OC, MEAS_IIN, IIN_OC_LVL)
	log_alarm(ALARM_ICHG_UC, MEAS_ICHG, ICHG_UC_LVL)
	log_alarm(ALARM_DTEMP_COLD, MEAS_DTEMP, DTEMP_COLD_LVL)
	log_alarm(ALARM_DTEMP_HOT, MEAS_DTEMP, DTEMP_HOT_LVL)
	log_alarm(ALARM_ESR_HI, MEAS_ESR, ESR_HI_LVL)
	log_alarm(ALARM_CAP_LO, MEAS_CAP, CAP_LO_LVL)
}

// READ/WRITE UTILITY FUNCTIONS -------------------------------------

// write a number to a register
static int ltc3350_write_num(struct i2c_client *client, u8 reg, int value)
{
	char buf[8];
	snprintf(buf, sizeof(buf), "%d", value);
	return ltc3350_set_value(client, reg, buf);
}


// Set the alarm in the msk_alarm register
static int ltc3350_set_alarm(struct i2c_client *client, int alarm)
{
	int ret;
	int msk_alarms;

	ret = ltc3350_get_value(client, MSK_ALARMS, &msk_alarms);
	if (ret)
		return ret;

	msk_alarms = msk_alarms | alarm;
	ret = ltc3350_write_num(client, MSK_ALARMS, msk_alarms);
	if (ret)
		return ret;

	return 0;
}

// set register and corresponding alarm
static int set_alarm_and_level(struct i2c_client *client, u8 reg, int value, int alarm)
{
	int res;
	res = ltc3350_write_num(client, reg, value);
	if (res) {
		dev_err(&client->dev, "Error %d writing in %d register\n", res, reg);
		return res;
	}
	res = ltc3350_set_alarm(client, alarm);
	if (res) {
		dev_err(&client->dev, "Error %d setting %d alarm\n", res, alarm);
		return res;
	}
	return 0;
}




// Enable/disable monitor alert in the msk_mon_status register
static int set_monitor_alert(struct i2c_client *client, int alert, int set)
{
	int ret;
	int msk_mon_status;

	ret = ltc3350_get_value(client, MSK_MON_STATUS, &msk_mon_status);
	if (ret)
		return ret;
	if (set)
		msk_mon_status = msk_mon_status | alert;
	else
		msk_mon_status = msk_mon_status & !alert;
	ret = ltc3350_write_num(client, MSK_MON_STATUS, msk_mon_status);
	if (ret)
		return ret;

	return 0;
}

/**
 * Schedules a Capacitance and ESR test, sets the test frequency to one hour
*/
static int start_capesr_test(struct i2c_client *client)
{
	int ret, value;
	ret = ltc3350_get_value(client, CTL_REG, &value);
	if (ret)
		return ret;
	value = value | 1;
	ret = ltc3350_write_num(client, CTL_REG, value);
	if (ret)
		return ret;
	ret = ltc3350_write_num(client, CAP_ESR_PER, 360);
	return 0;
}

// CONFIGURATION ------------------------------------------------------------------

/*
 * If initial measurements are active and a measurement has just completed
 * warns the user about potential inaccurate values
 * Eventually disables initial measurements.
*/
static void ltc3350_handle_initial_measurements(struct i2c_client *client, int monitor_value)
{
	struct device *dev = &client->dev;
	struct ltc3350_data *devdata = dev_get_drvdata(dev);
	if ((monitor_value & MON_ESR_DONE) && devdata->initial_meas) {
		if (devdata->cap_esr_num < devdata->initial_meas) {
			dev_warn(dev, "Initial capacitance and ESR measurements may be inaccurate. Measurement number %d.", devdata->cap_esr_num);
		}
		if (devdata->cap_esr_num == devdata->initial_meas) {
			dev_warn(dev, "First %d capacitance and ESR tests done. Measurements will resume at rate specified in the device tree. MON_ESR_DONE alert will be turned off.", devdata->cap_esr_num);
			set_monitor_alert(client, MON_ESR_DONE, 0);
			ltc3350_write_num(client, CAP_ESR_PER, devdata->cap_esr_per);
		}
		devdata->cap_esr_num++;
	}
}


/*
 * This function reads default values from the device tree.
 * and sets alarms and levels.
*/
static int ltc3350_configure(struct i2c_client *client){
	struct device_node *np = client->dev.of_node;
	struct ltc3350_data *data;
	int value;
	int ret;

	data = dev_get_drvdata(&client->dev);
	if (!data)
		return -EINVAL;

	ret = of_property_read_u32(np, "capacitor-overvoltage-level", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: capacitor-overvoltage-level is %d\n", value);
		ret = set_alarm_and_level(client, CAP_OV_LVL, value, ALARM_CAP_OV);
		if (ret)
			return ret;
	} else {
		dev_dbg(&client->dev, "No capacitor-overvoltage-level code: %d\n", ret);
	}

	ret = of_property_read_u32(np, "maximum-temperature", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: maximum-temperature is %d\n", value);
		ret = set_alarm_and_level(client, DTEMP_HOT_LVL, value, ALARM_DTEMP_HOT);
		if (ret)
			return ret;
	} else{
		dev_dbg(&client->dev, "No maximum temperature %d\n", ret);
	}

	ret = of_property_read_u32(np, "esr-high-level", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: esr-high-level is %d\n", value);
		ret = set_alarm_and_level(client, ESR_HI_LVL, value, ALARM_ESR_HI);
		if (ret)
			return ret;
	} else {
		dev_dbg(&client->dev, "No esr high level %d\n", ret);
	}

	ret = of_property_read_u32(np, "capacitance-low-level", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: capacitance-low-level is %d\n", value);
		ret = set_alarm_and_level(client, CAP_LO_LVL, value, ALARM_CAP_LO);
		if (ret)
			return ret;
	} else {
		dev_dbg(&client->dev, "No capacitance-low-level %d\n", ret);
	}

	ret = of_property_read_u32(np, "cap-esr-measurement-period", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: cap-esr-measurement-period is %d\n", value);
		ret = ltc3350_write_num(client, CAP_ESR_PER, value);
		if (ret) {
			return ret;
		}
	}
	else{
		dev_dbg(&client->dev, "No cap-esr-measurement-period %d\n", ret);
	}

	ret = of_property_read_u32(np, "cap-esr-initial-measurements", &value);
	if (ret == 0) {
		dev_dbg(&client->dev, "DTS: cap-esr-initial-measurements is %d\n", value);
		data->initial_meas = value;
		if (value) {
			ret = set_monitor_alert(client, MON_ESR_DONE, 1);
			if (ret)
				return ret;
			ret = start_capesr_test(client);
			if (ret)
				return ret;
		}
	}
	else {
		dev_dbg(&client->dev, "No cap-esr-initial-measurements %d. Defaulting to 72.\n", ret);
		data->initial_meas = 72;
		ret = start_capesr_test(client);
		if (ret)
			return ret;
	}
	return 0;
}




// READ/WRITE

static SENSOR_DEVICE_ATTR_RW(clr_alarms, ltc3350_value, CLR_ALARMS);
static SENSOR_DEVICE_ATTR_RW(msk_alarms, ltc3350_value, MSK_ALARMS);
static SENSOR_DEVICE_ATTR_RW(msk_mon_status, ltc3350_value, MSK_MON_STATUS);
static SENSOR_DEVICE_ATTR_RW(cap_esr_per, ltc3350_value, CAP_ESR_PER);
static SENSOR_DEVICE_ATTR_RW(vcapfb_dac, ltc3350_value, VCAPFB_DAC);
static SENSOR_DEVICE_ATTR_RW(vshunt, ltc3350_value, VSHUNT);
static SENSOR_DEVICE_ATTR_RW(cap_uv_lvl, ltc3350_value, CAP_UV_LVL);
static SENSOR_DEVICE_ATTR_RW(cap_ov_lvl, ltc3350_value, CAP_OV_LVL);
static SENSOR_DEVICE_ATTR_RW(gpi_uv_lvl, ltc3350_value, GPI_UV_LVL);
static SENSOR_DEVICE_ATTR_RW(gpi_ov_lvl, ltc3350_value, GPI_OV_LVL);
static SENSOR_DEVICE_ATTR_RW(vin_uv_lvl, ltc3350_value, VIN_UV_LVL);
static SENSOR_DEVICE_ATTR_RW(vin_ov_lvl, ltc3350_value, VIN_OV_LVL);
static SENSOR_DEVICE_ATTR_RW(vcap_uv_lvl, ltc3350_value, VCAP_UV_LVL);
static SENSOR_DEVICE_ATTR_RW(vcap_ov_lvl, ltc3350_value, VCAP_OV_LVL);
static SENSOR_DEVICE_ATTR_RW(vout_uv_lvl, ltc3350_value, VOUT_UV_LVL);
static SENSOR_DEVICE_ATTR_RW(vout_ov_lvl, ltc3350_value, VOUT_OV_LVL);
static SENSOR_DEVICE_ATTR_RW(iin_oc_lvl, ltc3350_value, IIN_OC_LVL);
static SENSOR_DEVICE_ATTR_RW(ichg_uc_lvl, ltc3350_value, ICHG_UC_LVL);
static SENSOR_DEVICE_ATTR_RW(dtemp_cold_lvl, ltc3350_value, DTEMP_COLD_LVL);
static SENSOR_DEVICE_ATTR_RW(dtemp_hot_lvl, ltc3350_value, DTEMP_HOT_LVL);
static SENSOR_DEVICE_ATTR_RW(esr_hi_lvl, ltc3350_value, ESR_HI_LVL);
static SENSOR_DEVICE_ATTR_RW(cap_lo_lvl, ltc3350_value, CAP_LO_LVL);
static SENSOR_DEVICE_ATTR_RW(ctl_reg, ltc3350_value, CTL_REG);



// READONLY

static SENSOR_DEVICE_ATTR_RO(num_caps, ltc3350_value, NUM_CAPS);
static SENSOR_DEVICE_ATTR_RO(chrg_status, ltc3350_value, CHRG_STATUS);
static SENSOR_DEVICE_ATTR_RO(mon_status, ltc3350_value, MON_STATUS);
static SENSOR_DEVICE_ATTR_RO(alarm_reg, ltc3350_value, ALARM_REG);
static SENSOR_DEVICE_ATTR_RO(meas_cap, ltc3350_value, MEAS_CAP);
static SENSOR_DEVICE_ATTR_RO(meas_esr, ltc3350_value, MEAS_ESR);
static SENSOR_DEVICE_ATTR_RO(meas_vcap1, ltc3350_value, MEAS_VCAP1);
static SENSOR_DEVICE_ATTR_RO(meas_vcap2, ltc3350_value, MEAS_VCAP2);
static SENSOR_DEVICE_ATTR_RO(meas_vcap3, ltc3350_value, MEAS_VCAP3);
static SENSOR_DEVICE_ATTR_RO(meas_vcap4, ltc3350_value, MEAS_VCAP4);
static SENSOR_DEVICE_ATTR_RO(meas_gpi, ltc3350_value, MEAS_GPI);
static SENSOR_DEVICE_ATTR_RO(meas_vin, ltc3350_value, MEAS_VIN);
static SENSOR_DEVICE_ATTR_RO(meas_vcap, ltc3350_value, MEAS_VCAP);
static SENSOR_DEVICE_ATTR_RO(meas_vout, ltc3350_value, MEAS_VOUT);
static SENSOR_DEVICE_ATTR_RO(meas_iin, ltc3350_value, MEAS_IIN);
static SENSOR_DEVICE_ATTR_RO(meas_ichg, ltc3350_value, MEAS_ICHG);
static SENSOR_DEVICE_ATTR_RO(meas_dtemp, ltc3350_value, MEAS_DTEMP);


static struct attribute *ltc3350_attrs[] = {
	&sensor_dev_attr_clr_alarms.dev_attr.attr,
	&sensor_dev_attr_msk_alarms.dev_attr.attr,
	&sensor_dev_attr_msk_mon_status.dev_attr.attr,
	&sensor_dev_attr_cap_esr_per.dev_attr.attr,
	&sensor_dev_attr_vcapfb_dac.dev_attr.attr,
	&sensor_dev_attr_vshunt.dev_attr.attr,
	&sensor_dev_attr_cap_uv_lvl.dev_attr.attr,
	&sensor_dev_attr_cap_ov_lvl.dev_attr.attr,
	&sensor_dev_attr_gpi_uv_lvl.dev_attr.attr,
	&sensor_dev_attr_gpi_ov_lvl.dev_attr.attr,
	&sensor_dev_attr_vin_uv_lvl.dev_attr.attr,
	&sensor_dev_attr_vin_ov_lvl.dev_attr.attr,
	&sensor_dev_attr_vcap_uv_lvl.dev_attr.attr,
	&sensor_dev_attr_vcap_ov_lvl.dev_attr.attr,
	&sensor_dev_attr_vout_uv_lvl.dev_attr.attr,
	&sensor_dev_attr_vout_ov_lvl.dev_attr.attr,
	&sensor_dev_attr_iin_oc_lvl.dev_attr.attr,
	&sensor_dev_attr_ichg_uc_lvl.dev_attr.attr,
	&sensor_dev_attr_dtemp_cold_lvl.dev_attr.attr,
	&sensor_dev_attr_dtemp_hot_lvl.dev_attr.attr,
	&sensor_dev_attr_esr_hi_lvl.dev_attr.attr,
	&sensor_dev_attr_cap_lo_lvl.dev_attr.attr,
	&sensor_dev_attr_ctl_reg.dev_attr.attr,

	&sensor_dev_attr_num_caps.dev_attr.attr,
	&sensor_dev_attr_chrg_status.dev_attr.attr,
	&sensor_dev_attr_mon_status.dev_attr.attr,
	&sensor_dev_attr_alarm_reg.dev_attr.attr,
	&sensor_dev_attr_meas_cap.dev_attr.attr,
	&sensor_dev_attr_meas_esr.dev_attr.attr,
	&sensor_dev_attr_meas_vcap1.dev_attr.attr,
	&sensor_dev_attr_meas_vcap2.dev_attr.attr,
	&sensor_dev_attr_meas_vcap3.dev_attr.attr,
	&sensor_dev_attr_meas_vcap4.dev_attr.attr,
	&sensor_dev_attr_meas_gpi.dev_attr.attr,
	&sensor_dev_attr_meas_vin.dev_attr.attr,
	&sensor_dev_attr_meas_vcap.dev_attr.attr,
	&sensor_dev_attr_meas_vout.dev_attr.attr,
	&sensor_dev_attr_meas_iin.dev_attr.attr,
	&sensor_dev_attr_meas_ichg.dev_attr.attr,
	&sensor_dev_attr_meas_dtemp.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ltc3350);


/**
 * ltc3350_probe() - handle device registration and initialize data.
 * @client the i2c_client that will handle read/write operation
 *
 * Initialize the data to be associated to the i2c client, which will be available in alert functions,
 * and the one associated to the hwmon device, which will be available in read/write operations.
 * Read configuration from the device tree and set corresponding
 * registers.
 * Return: 0 on success, error number otherwise.
*/
static int ltc3350_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = &client->dev;
	struct ltc3350_hwmon_data *hwmon_data;
	struct ltc3350_data *data;
	struct device *hwmon_dev;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "no functionality\n");
		return -ENODEV;
	}

	hwmon_data = devm_kzalloc(dev, sizeof(*hwmon_data), GFP_KERNEL);
	if (!hwmon_data) {
		dev_err(&client->dev, "no hwmon data\n");
		return -ENOMEM;
	}

	hwmon_data->client = client;
	hwmon_dev = devm_hwmon_device_register_with_groups(
		dev,
		client->name,
		hwmon_data,
		ltc3350_groups);

	dev_info(&client->dev, "Registered LTC3350 power monitor\n");
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "no data\n");
		return PTR_ERR_OR_ZERO(hwmon_dev);
	}

	data->hwmon_dev = hwmon_dev;
	data->cap_esr_num = 0;
	data->initial_meas = 10;
	data->cap_esr_per = 0;
	dev_set_drvdata(&client->dev, data);

	ret = ltc3350_configure(client);
	if (ret)
		dev_err(&client->dev, "Error in reading configuration from device tree %d.\n", ret);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}


static const struct i2c_device_id ltc3350_id[] ={
	{ "ltc3350"},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc3350_id);

static const struct of_device_id ltc3350_match[] = {
	{ .compatible = "ltc3350" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc3350_match);


static struct i2c_driver ltc3350_driver = {
	.driver = {
		.name = "ltc3350",
		.of_match_table = of_match_ptr(ltc3350_match),
	},
	.probe_new	= ltc3350_probe,
	.id_table = ltc3350_id,
	.alert = ltc3350_alert,
};

module_i2c_driver(ltc3350_driver);

MODULE_AUTHOR("Francesca Pinna <francecipinna@gmail.com>");
MODULE_DESCRIPTION("LTC3350 driver");
MODULE_LICENSE("GPL");
