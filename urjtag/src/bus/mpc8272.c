/*
 * $Id: mpc8272.c $
 *
 * Freescale MPC8272 compatible bus driver via BSR
 * Copyright (C) 2007 Techno Trade S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by Laurent Pinchart <laurent.pinchart@technotrade.biz>, 2007.
 * Based on work by Marcel Telka <marcel@telka.sk>, 2003.
 *
 * Documentation:
 * [1] Freescale, Inc., "MPC8272 PowerQUICK II Family Reference Manual",
 *     MPC8272RM, 10/2005, Rev. 2
 *
 */

#include <sysdep.h>

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <urjtag/part.h>
#include <urjtag/chain.h>
#include <urjtag/bus.h>
#include <urjtag/bssignal.h>
#include <urjtag/tap_state.h>

#include "buses.h"
#include "generic_bus.h"

typedef struct {
	uint32_t last_addr;
	urj_part_signal_t *nwe[4];
	urj_part_signal_t *npoe;
	urj_part_signal_t *nbctl0;
	urj_part_signal_t *nbctl1;
	urj_part_signal_t *ncs[8];
	urj_part_signal_t *a[32];
	urj_part_signal_t *d[32];
} bus_params_t;

#define	LAST_ADDR	((bus_params_t *) bus->params)->last_addr
#define	nCS		((bus_params_t *) bus->params)->ncs
#define	nWE		((bus_params_t *) bus->params)->nwe
#define	nPOE		((bus_params_t *) bus->params)->npoe
#define	nBCTL0		((bus_params_t *) bus->params)->nbctl0
#define	nBCTL1		((bus_params_t *) bus->params)->nbctl1
#define	A		((bus_params_t *) bus->params)->a
#define	D		((bus_params_t *) bus->params)->d

typedef struct {
	urj_bus_area_t area;
	unsigned int cs;
	unsigned int lines;
} mpc8272_area_t;

static const mpc8272_area_t mpc8272_areas[] = {
	{ { N_("SDRAM"), 0x00000000, 0x10000000, 0  }, 0, 0 },
	{ { N_("None"),  0x10000000, 0x30000000, 0  }, 0, 0 },
	{ { N_("Flash"), 0x40000000, 0x10000000, 16 }, 0, 27 },
	{ { N_("None"),  0x50000000, 0xA7000000, 0  }, 0, 0 },
	{ { N_("Debug"), 0xF7000000, 0x01000000, 8  }, 7, 8 },
	{ { N_("None"),  0xF8000000, 0x08000000, 0  }, 0, 0 },
};

static const mpc8272_area_t *
find_area(urj_bus_t *bus, uint32_t addr)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mpc8272_areas); ++i)
	{
		const mpc8272_area_t *area = &mpc8272_areas[i];
		if (addr >= area->area.start &&
		    addr < area->area.start + area->area.length)
			return area;
	}

	return NULL;
}

static void
setup_address(urj_bus_t *bus, const mpc8272_area_t *area, uint32_t addr)
{
	unsigned int i;

	for (i = 0; i < area->lines; i++)
		urj_part_set_signal (bus->part, A[31-i], 1, (addr >> i) & 1);
}

static void
set_data_in(urj_bus_t *bus, const mpc8272_area_t *area)
{
	unsigned int i;

	for (i = 0; i < 32; i++)
		urj_part_set_signal (bus->part, D[i], 0, 0);
}

static void
setup_data(urj_bus_t *bus, const mpc8272_area_t *area, uint32_t d)
{
	unsigned int i;

	for (i = 0; i < area->area.width; i++)
		urj_part_set_signal (bus->part, D[i], 1,
				     (d >> (area->area.width - i - 1)) & 1);
}

static uint32_t
get_data(urj_bus_t *bus, const mpc8272_area_t *area)
{
	unsigned int i;
	uint32_t d = 0;

	for (i = 0; i < area->area.width; i++)
	{
		d <<= 1;
		d |= urj_part_get_signal (bus->part, D[i]) ? 1 : 0;
	}

	return d;
}

static urj_bus_t *
mpc8272_bus_new(urj_chain_t *chain, const urj_bus_driver_t *driver,
		const urj_param_t *cmd_params[])
{
	urj_bus_t *bus;
	urj_part_t *part;
	int failed = 0;
	char buff[10];
	unsigned int i;

	bus = urj_bus_generic_new (chain, driver, sizeof (bus_params_t));
	if (bus == NULL)
		return NULL;
	part = bus->part;

	/* Control signals */
	for (i = 0; i < 4; ++i) {
		sprintf (buff, "WE_DQM_BS_B%d", i);
		failed |= urj_bus_generic_attach_sig (part, &nWE[i], buff);
	}

	failed |= urj_bus_generic_attach_sig (part, &nPOE, "OE_B_SDRAS_B_GPL2");
	failed |= urj_bus_generic_attach_sig (part, &nBCTL0, "BCTL0_B");
	failed |= urj_bus_generic_attach_sig (part, &nBCTL1, "CS6_BCTL1_SMI_B");

	/* Chip select */
	for (i = 0; i < 6; ++i) {
		sprintf (buff, "CS_B%d", i);
		failed |= urj_bus_generic_attach_sig (part, &nCS[i], buff);
	}

	failed |= urj_bus_generic_attach_sig (part, &nCS[6], "CS6_BCTL1_SMI_B");
	failed |= urj_bus_generic_attach_sig (part, &nCS[7], "CS7_TLBISYNC_B");

	/* Address */
	for (i = 0; i < 32; i++) {
		sprintf (buff, "A%d", i);
		failed |= urj_bus_generic_attach_sig (part, &A[i], buff);
	}

	/* Data */
	for (i = 0; i < 32; i++) {
		sprintf (buff, "D%d", i);
		failed |= urj_bus_generic_attach_sig (part, &D[i], buff);
	}

	if (failed)
	{
		urj_bus_generic_free (bus);
		return NULL;
	}

	return bus;
}

static void
mpc8272_bus_printinfo(urj_log_level_t ll, urj_bus_t *bus)
{
	unsigned int i;

	for (i = 0; i < bus->chain->parts->len; i++)
		if (bus->part == bus->chain->parts->parts[i])
			break;

	urj_log (ll, _("Freescale MPC8272 compatible bus driver via BSR (JTAG "
		 "part No. %d)\n"), i);
}

static int
mpc8272_bus_init(urj_bus_t *bus)
{
	unsigned int i;

	if (urj_tap_state (bus->chain) != URJ_TAP_STATE_RUN_TEST_IDLE)
	{
		/* silently skip initialization if TAP isn't in RUNTEST/IDLE state
		   this is required to avoid interfering with detect when initbus
		   is contained in the part description file
		   URJ_BUS_INIT() will be called latest by URJ_BUS_PREPARE() */
		return URJ_STATUS_OK;
	}

	for (i = 0; i < 8; ++i)
		urj_part_set_signal (bus->part, nCS[i], 1, 1);

	for (i = 0; i < 4; ++i)
		urj_part_set_signal (bus->part, nWE[i], 1, 1);

	urj_part_set_signal (bus->part, nBCTL0, 1, 1);
	urj_part_set_signal (bus->part, nBCTL1, 1, 1);
	urj_part_set_signal (bus->part, nPOE, 1, 1);

	urj_part_set_instruction (bus->part, "SAMPLE/PRELOAD");
	urj_tap_chain_shift_instructions (bus->chain);
	urj_tap_chain_shift_data_registers (bus->chain, 0);

	bus->initialized = 1;
	return URJ_STATUS_OK;
}

static int
mpc8272_bus_area(urj_bus_t *bus, uint32_t addr, urj_bus_area_t *area)
{
	const mpc8272_area_t *mpc_area;

	mpc_area = find_area (bus, addr);
	if (mpc_area == NULL)
		return URJ_STATUS_FAIL;

	*area = mpc_area->area;
	return URJ_STATUS_OK;
}

static int
mpc8272_bus_read_start(urj_bus_t *bus, uint32_t addr)
{
	const mpc8272_area_t *area;

	area = find_area (bus, addr);

	LAST_ADDR = addr;

	/* see Figure 6-45 in [1] */
	urj_part_set_signal (bus->part, nCS[area->cs], 1, 0);
	urj_part_set_signal (bus->part, nBCTL0,  1, 0);
	urj_part_set_signal (bus->part, nBCTL1,  1, 0);
	urj_part_set_signal (bus->part, nPOE, 1, 0);

	setup_address (bus, area, addr);
	set_data_in (bus, area);

	urj_tap_chain_shift_data_registers (bus->chain, 0);

	return URJ_STATUS_OK;
}

static uint32_t
mpc8272_bus_read_next(urj_bus_t *bus, uint32_t addr)
{
	const mpc8272_area_t *area;
	uint32_t d;

	area = find_area (bus, addr);

	setup_address (bus, area, addr);
	urj_tap_chain_shift_data_registers (bus->chain, 1);

	d = get_data (bus, area);
	LAST_ADDR = addr;
	return d;
}

static uint32_t
mpc8272_bus_read_end(urj_bus_t *bus)
{
	const mpc8272_area_t *area;

	area = find_area (bus, LAST_ADDR);

	urj_part_set_signal (bus->part, nCS[area->cs], 1, 1);
	urj_part_set_signal (bus->part, nBCTL0, 1, 1);
	urj_part_set_signal (bus->part, nBCTL1,  1, 1);
	urj_part_set_signal (bus->part, nPOE, 1, 1);

	urj_tap_chain_shift_data_registers(bus->chain, 1);

	return get_data(bus, area);
}

static void
mpc8272_bus_write(urj_bus_t *bus, uint32_t addr, uint32_t data)
{
	const mpc8272_area_t *area;

	area = find_area (bus, addr);

	urj_part_set_signal (bus->part, nCS[area->cs], 1, 0);
	urj_part_set_signal (bus->part, nBCTL0, 1, 1);
	urj_part_set_signal (bus->part, nBCTL1,  1, 0);
	setup_address (bus, area, addr);
	setup_data (bus, area, data);
	urj_tap_chain_shift_data_registers (bus->chain, 0);

	urj_part_set_signal (bus->part, nWE[0], 1, 0);
	urj_part_set_signal (bus->part, nWE[1], 1, 0);
	urj_tap_chain_shift_data_registers (bus->chain, 0);

	urj_part_set_signal (bus->part, nCS[area->cs], 1, 1);
	urj_part_set_signal (bus->part, nBCTL0, 1, 1);
	urj_part_set_signal (bus->part, nBCTL1, 1, 1);
	urj_part_set_signal (bus->part, nWE[0], 1, 1);
	urj_part_set_signal (bus->part, nWE[1], 1, 1);
	urj_tap_chain_shift_data_registers (bus->chain, 0);
}

const urj_bus_driver_t urj_bus_mpc8272_bus = {
	"mpc8272",
	N_("Freescale MPC8272 compatible bus driver via BSR"),
	mpc8272_bus_new,
	urj_bus_generic_free,
	mpc8272_bus_printinfo,
	urj_bus_generic_prepare_extest,
	mpc8272_bus_area,
	mpc8272_bus_read_start,
	mpc8272_bus_read_next,
	mpc8272_bus_read_end,
	urj_bus_generic_read,
	urj_bus_generic_write_start,
	mpc8272_bus_write,
	mpc8272_bus_init,
	urj_bus_generic_no_enable,
	urj_bus_generic_no_disable,
	URJ_BUS_TYPE_PARALLEL,
};
