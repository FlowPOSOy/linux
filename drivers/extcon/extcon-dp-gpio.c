// SPDX-License-Identifier: GPL-2.0
/*
 * extcon-dp-gpio: report a hard-wired DisplayPort connector as an extcon
 * EXTCON_DISP_DP cable, driven by a hot-plug-detect GPIO.
 *
 * RK3399 cdn-dp and phy-rockchip-typec (as of v7.4) only learn about a DP sink
 * through extcon. Boards that route a Type-C PHY to a plain DP/HDMI connector
 * (no Type-C controller) need something that turns HPD into EXTCON_DISP_DP with
 * the Type-C properties the PHY expects. Clean-room replacement for Rockchip's
 * downstream "linux,extcon-pd-virtual" (as used on Posbank ACT 1560, mode 3).
 *
 * Reported state:
 *   HPD asserted:   EXTCON_DISP_DP = 1, POLARITY = flip, SS = 0 -> PHY MODE_DFP_DP (4 lanes)
 *   HPD deasserted: everything 0                              -> PHY MODE_DFP_USB
 * EXTCON_USB is never set: mainline tcphy_get_mode() prefers UFP over DP.
 *
 * Device tree:
 *   dp_extcon: dp-extcon {
 *           compatible = "linux,extcon-dp-gpio";
 *           hpd-gpios = <&gpio4 RK_PD1 GPIO_ACTIVE_HIGH>;
 *           vdd-supply = <&vcc5v0_dp>;           // optional
 *           // typec-flip;                       // optional, swap lane polarity
 *   };
 */
#include <linux/device.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define DP_HPD_DEBOUNCE_MS	10	/* vendor driver: 3 jiffies @ HZ=300 */

struct dp_gpio_extcon {
	struct device *dev;
	struct extcon_dev *edev;
	struct gpio_desc *hpd;
	struct delayed_work work;
	bool flip;
	int connected;		/* -1 = unknown */
};

static const unsigned int dp_gpio_cables[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

static void dp_gpio_extcon_report(struct dp_gpio_extcon *data, bool on)
{
	union extcon_property_value pol = { .intval = data->flip };
	union extcon_property_value ss = { .intval = 0 };

	/* Properties first, then state: consumers read them from the notifier. */
	extcon_set_property(data->edev, EXTCON_DISP_DP,
			    EXTCON_PROP_USB_TYPEC_POLARITY, pol);
	extcon_set_property(data->edev, EXTCON_DISP_DP, EXTCON_PROP_USB_SS, ss);
	extcon_set_property(data->edev, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_TYPEC_POLARITY, pol);
	extcon_set_property(data->edev, EXTCON_USB_HOST, EXTCON_PROP_USB_SS, ss);

	extcon_set_state(data->edev, EXTCON_USB, false);
	extcon_set_state(data->edev, EXTCON_USB_HOST, false);
	extcon_set_state(data->edev, EXTCON_DISP_DP, on);
	extcon_sync(data->edev, EXTCON_DISP_DP);
}

static void dp_gpio_extcon_work(struct work_struct *work)
{
	struct dp_gpio_extcon *data =
		container_of(to_delayed_work(work), struct dp_gpio_extcon, work);
	int val = gpiod_get_value_cansleep(data->hpd);

	if (val < 0 || val == data->connected)
		return;

	data->connected = val;
	dev_dbg(data->dev, "DP sink %s\n", val ? "connected" : "disconnected");
	dp_gpio_extcon_report(data, val);
}

static irqreturn_t dp_gpio_extcon_irq(int irq, void *dev_id)
{
	struct dp_gpio_extcon *data = dev_id;

	mod_delayed_work(system_wq, &data->work,
			 msecs_to_jiffies(DP_HPD_DEBOUNCE_MS));
	return IRQ_HANDLED;
}

static void dp_gpio_extcon_cancel(void *arg)
{
	cancel_delayed_work_sync(arg);
}

static int dp_gpio_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dp_gpio_extcon *data;
	int irq, ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->connected = -1;
	data->flip = device_property_read_bool(dev, "typec-flip");

	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable vdd\n");

	data->hpd = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	if (IS_ERR(data->hpd))
		return dev_err_probe(dev, PTR_ERR(data->hpd), "failed to get hpd gpio\n");

	data->edev = devm_extcon_dev_allocate(dev, dp_gpio_cables);
	if (IS_ERR(data->edev))
		return PTR_ERR(data->edev);

	ret = devm_extcon_dev_register(dev, data->edev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register extcon\n");

	ret = extcon_set_property_capability(data->edev, EXTCON_DISP_DP,
					     EXTCON_PROP_USB_TYPEC_POLARITY);
	ret = ret ?: extcon_set_property_capability(data->edev, EXTCON_DISP_DP,
						    EXTCON_PROP_USB_SS);
	ret = ret ?: extcon_set_property_capability(data->edev, EXTCON_USB_HOST,
						    EXTCON_PROP_USB_TYPEC_POLARITY);
	ret = ret ?: extcon_set_property_capability(data->edev, EXTCON_USB_HOST,
						    EXTCON_PROP_USB_SS);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set extcon capabilities\n");

	INIT_DELAYED_WORK(&data->work, dp_gpio_extcon_work);
	ret = devm_add_action_or_reset(dev, dp_gpio_extcon_cancel, &data->work);
	if (ret)
		return ret;

	irq = gpiod_to_irq(data->hpd);
	if (irq < 0)
		return dev_err_probe(dev, irq, "hpd gpio has no irq\n");

	ret = devm_request_irq(dev, irq, dp_gpio_extcon_irq,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       dev_name(dev), data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request hpd irq\n");

	platform_set_drvdata(pdev, data);
	/* initial state, so a monitor attached at boot is picked up */
	dp_gpio_extcon_work(&data->work.work);
	return 0;
}

static int dp_gpio_extcon_resume(struct device *dev)
{
	struct dp_gpio_extcon *data = dev_get_drvdata(dev);

	mod_delayed_work(system_wq, &data->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(dp_gpio_extcon_pm, NULL, dp_gpio_extcon_resume);

static const struct of_device_id dp_gpio_extcon_of_match[] = {
	{ .compatible = "linux,extcon-dp-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, dp_gpio_extcon_of_match);

static struct platform_driver dp_gpio_extcon_driver = {
	.probe = dp_gpio_extcon_probe,
	.driver = {
		.name = "extcon-dp-gpio",
		.of_match_table = dp_gpio_extcon_of_match,
		.pm = pm_sleep_ptr(&dp_gpio_extcon_pm),
	},
};
module_platform_driver(dp_gpio_extcon_driver);

MODULE_DESCRIPTION("GPIO hot-plug-detect DisplayPort extcon");
MODULE_LICENSE("GPL");
