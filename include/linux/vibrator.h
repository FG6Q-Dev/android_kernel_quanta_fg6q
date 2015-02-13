#ifndef __ARCH_VIBRATOR_H
#define __ARCH_VIBRATOR_H

struct omap_vibrator{
	int (*init)(void);
	int (*enable)(int);
	int (*time)(int);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __ARCH_VIBRATOR_H */
