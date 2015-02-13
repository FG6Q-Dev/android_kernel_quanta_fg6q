/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Android EXTRA_DOCK_STATE */
enum mic_location {
        DOWN_TOP = 1,
        DOWN_BACK,
        TOP_ONLY,
        TOP_BACK,
};

/* Android EXTRA_DOCK_STATE */
enum mic_state {
        NORMAL= 0,
        DOCK,
        MONO,
		RESERVE,
};

struct mic_switch_platform_data {
	unsigned dmic_sw1;
	unsigned dmic_sw2;
	unsigned dmic_lr;	
};


