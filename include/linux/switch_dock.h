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

struct gen2_dock_data {
        struct switch_dev sdev;
        int irq;
        int debounce;
        struct delayed_work dwork;
};

/* Android EXTRA_DOCK_STATE */
enum dock_state {
        UNDOCKED = 0,
        DESK_DOCK,
        CAR_DOCK,
        LE_DESK_DOCK, /* low end analog dock */
        HE_DESK_DOCK, /* high end digital dock */
};

struct switch_dock_device {
        const char * name;
        int debounce_interval; /* debounce time in msecs */
};   

void dock_switch_work(int state);
static int dock_num;
