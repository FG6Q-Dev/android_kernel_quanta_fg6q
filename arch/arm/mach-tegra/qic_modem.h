/*
 * Definitions for qic_modem.
 */
#ifndef _LINUX_QIC_MODEM_H
#define _LINUX_QIC_MODEM_H

#define QIC_MODEM_SIM_DETECTION_ENHANCEMENT

/* MDM_FATAL GPIO_100*/
/* WAKE_WWAN# GPIO_50 no safe_mode*/
/* MDM_WAKE_UP# GPIO_wk0 */
/* MDM_DISABLE# GPIO_wk30 */
/* MDM_RST#_1 GPIO_1 */
/* MDM_ONKEY GPIO_157 */
/* 3G_DET GPIO_56 no safe_mode*/
#define QIC_MODEM_GPIO_NULL -1

struct qic_modem_platform_data {
	int gpio_power_enable;		/* [O] power gate */
	int gpio_host_wakeup_wwan;	/* [O] host wake up wwan */			
	int gpio_radio_off;			/* [O] disable modem's radio */
	int gpio_reset;				/* [O] to reset modem */
	int gpio_onkey;				/* [O] to enable modem */
	int gpio_fatal;				/* [I] modem notify ACPU that modem is failed. */
	int gpio_modem_wakeup_host;	/* [I] modem wake up host */
	int gpio_sim_detection;		/* [I] sim card detection from machinical */
#if 0
	int gpio_radio_off_dummy;		/* [I] redundant radio off pin, should be floating. */
#endif
};
#endif