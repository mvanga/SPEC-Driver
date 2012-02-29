/*
 * This file is part of the Simple PCI Express Carrier (SPEC) Linux driver.
 * More information available at: http://www.ohwr.org/projects/spec
 *
 * Copyright (c) 2011 Alessandro Rubini <rubini@gnudd.com>
 * Copyright (c) 2011 Manohar Vanga <manohar.vanga@cern.ch>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#ifndef GENNUM_H
#define GENNUM_H

#include <linux/init.h>

enum {
	GN412X_GPIO_BASE = 0xA00,
	GN412X_GPIO_DIRECTION_MODE = GN412X_GPIO_BASE + 0x4,
	GN412X_GPIO_OUTPUT_ENABLE = GN412X_GPIO_BASE + 0x8,
	GN412X_GPIO_OUTPUT_VALUE = GN412X_GPIO_BASE + 0xC,
	GN412X_GPIO_INPUT_VALUE = GN412X_GPIO_BASE + 0x10,

        GN412X_FCL_BASE = 0xB00,
        GN412X_FCL_CTRL = GN412X_FCL_BASE,
        GN412X_FCL_STATUS = GN412X_FCL_BASE + 0x4,
        GN412X_FCL_IODATA_IN = GN412X_FCL_BASE + 0x8,
        GN412X_FCL_IODATA_OUT = GN412X_FCL_BASE + 0xC,
        GN412X_FCL_EN = GN412X_FCL_BASE + 0x10,
        GN412X_FCL_TIMER_0 = GN412X_FCL_BASE + 0x14,
        GN412X_FCL_TIMER_1 = GN412X_FCL_BASE + 0x18,
        GN412X_FCL_CLK_DIV = GN412X_FCL_BASE + 0x1C,
        GN412X_FCL_IRQ = GN412X_FCL_BASE + 0x20,
        GN412X_FCL_TIMER_CTRL = GN412X_FCL_BASE + 0x24,
        GN412X_FCL_IM = GN412X_FCL_BASE + 0x28,
        GN412X_FCL_TIMER2_0 = GN412X_FCL_BASE + 0x2C,
        GN412X_FCL_TIMER2_1 = GN412X_FCL_BASE + 0x30,
        GN412X_FCL_DBG_STS = GN412X_FCL_BASE + 0x34,
        GN412X_FCL_FIFO = 0xE00,
        PCI_SYS_CFG_SYSTEM = 0x800
};

int gennum_loader(void __iomem *, const void *, int);

#endif
