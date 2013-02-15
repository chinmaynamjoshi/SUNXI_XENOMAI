/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <rtdm/rtdm_driver.h>

#include <rtcan_dev.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>

#ifdef CONFIG_XENO_DRIVERS_CAN_DEBUG

static int rtcan_sja_proc_regs(char *buf, char **start, off_t offset,
				 int count, int *eof, void *data)
{
    struct rtcan_device *dev = (struct rtcan_device *)data;
    struct rtcan_sja1000 *chip = (struct rtcan_sja1000 *)dev->priv;
    int i;
    RTCAN_PROC_PRINT_VARS(80);

    if (!RTCAN_PROC_PRINT("SJA1000 registers"))
	goto done;
    for (i = 0; i < 0x20; i++) {
	if ((i % 0x10) == 0) {
	    if (!RTCAN_PROC_PRINT("\n%02x:", i))
		goto done;
	}
	if (!RTCAN_PROC_PRINT(" %02x", chip->read_reg(dev, i)))
	    goto done;
    }
    if (!RTCAN_PROC_PRINT("\n"))
	goto done;

 done:
    RTCAN_PROC_PRINT_DONE;
}

int rtcan_sja_create_proc(struct rtcan_device* dev)
{
    struct proc_dir_entry *proc_entry;

    if (!dev->proc_root)
	return -EINVAL;

    proc_entry = create_proc_entry("registers", S_IFREG | S_IRUGO | S_IWUSR,
				   dev->proc_root);
    if (!proc_entry)
	goto error;
    proc_entry->read_proc = rtcan_sja_proc_regs;
    proc_entry->data = dev;

    return 0;

  error:
    printk("%s: unable to create /proc entries for SJA\n", dev->name);
    return -1;
}

void rtcan_sja_remove_proc(struct rtcan_device* dev)
{
    if (!dev->proc_root)
	return;

    remove_proc_entry("registers", dev->proc_root);
}

#else /* !CONFIG_XENO_DRIVERS_CAN_DEBUG */

void rtcan_sja_remove_proc(struct rtcan_device* dev)
{
}

int rtcan_sja_create_proc(struct rtcan_device* dev)
{
    return 0;
}
#endif	/* CONFIG_XENO_DRIVERS_CAN_DEBUG */
