/* drivers/staging/android/vibrator.h
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef _DRIVERS_STAGING_ANDROID_VIBRATION_H
#define _DRIVERS_STAGING_ANDROID_VIBRATION_H

#define VIBRATION_MAX_ENTRIES		1
#define VIBRATION_MAX_DUTYCYCLE		127	

enum {
	VIBRATION_START,
	VIBRATION_SUSTAIN,
	VIBRATION_STOP,
	VIBRATION_DONE,
};

struct waveform {
 	u32 dutycycle;
 	unsigned long duration;
};

struct vibration_dev {
	const char	*name;

	/* enable the output*/
	void	(*enable)(struct vibration_dev *vdev, unsigned long timeout);

	/* private data */
	struct device	*dev;
	int	index;
	int	state;
        
    struct waveform pattern[VIBRATION_DONE][VIBRATION_MAX_ENTRIES];	
};

extern int vibration_dev_register(struct vibration_dev *dev);
extern void vibration_dev_unregister(struct vibration_dev *dev);

#endif
