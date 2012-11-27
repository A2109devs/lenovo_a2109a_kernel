#ifndef WAKEUP_SOURCE_H
#define WAKEUP_SOURCE_H

#include <linux/suspend.h>

#define PMIC_TOPIRQ_ONOFF_MASK	(1 << 1)
#define PMIC_TOPIRQ_RTC_MASK		(1 << 3)
#define PMIC_TOPIRQ_GPIO_MASK		(1 << 4)

#define PMIC_LV2IRQ_ACOK_MASK		(3 << 6)
#define PMIC_LV2IRQ_EN0_MASK		(3 << 2)

#define PMIC_LV2IRQ_GPIO1_MASK	(1 << 1)
#define PMIC_LV2IRQ_GPIO6_MASK	(1 << 6)

enum wkup_reason{
	WKUP_PWR_KEY,
	WKUP_RTC,
	WKUP_CABLE,
	WKUP_SD,
	WKUP_WIFI,
	WKUP_BT,
	WKUP_HDMI,
	WKUP_BATT_LOW,
	WKUP_KEYPAD,	
	WKUP_SIM_CARD,	
	WKUP_CHG_FULL,
	WKUP_UNKNOWN,	
};

struct wkup_source {
	const char *name;
	enum wkup_reason reason;
};

static struct wkup_source cl2n_wkup_source[] = {
		{.name="wkup_pwr_key",	.reason=WKUP_PWR_KEY},		// PMU_INT, EN0
		{.name="wkup_rtc",		.reason=WKUP_RTC},			// PMU_INT, RTC
		{.name="wkup_cable",		.reason=WKUP_CABLE},		// PMU_INT, ACOK
		{.name="wkup_sd",		.reason=WKUP_SD},			// GPIO_PI2
		{.name="wkup_wifi",		.reason=WKUP_WIFI},			// GPIO_PV1
		{.name="wkup_bt",		.reason=WKUP_BT},			// GPIO_PS6		
		{.name="wkup_hdmi",		.reason=WKUP_HDMI},			// GPIO_PN7		
		{.name="wkup_batt_low",	.reason=WKUP_BATT_LOW},		// PMU_INT, GPIO1	
		{.name="wkup_keypad",	.reason=WKUP_KEYPAD},		// Tegra KBC
		{.name="wkup_sim_card",	.reason=WKUP_SIM_CARD},		// 	
		{.name="wkup_chg_full",	.reason=WKUP_CHG_FULL},		// PMU_INT, GPIO1	
		{.name="wkup_unknown",  	.reason=WKUP_UNKNOWN},
};

// record for the reason from PMIC
extern u8 g_pmic_wk_top_irq;
extern u8 g_pmic_wk_lv2_irq;

// record for the reason from Trgra
extern u64 g_tegra_wk_status;

//&*&*&*HC1_20120609
// record for cable wake event
extern u8 g_cable_wk_irq;
//&*&*&*HC2_20120609

extern int tegra3_get_wakeup_reason(void);
extern void SendPowerbuttonEvent(void);
#ifdef CONFIG_EARLYSUSPEND
extern suspend_state_t get_suspend_state(void);
#endif
#endif // WAKEUP_SOURCE_H
