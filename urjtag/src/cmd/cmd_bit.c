/*
 * $Id$
 *
 * Copyright (C) 2003 ETC s.r.o.
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
 * Written by Marcel Telka <marcel@telka.sk>, 2003.
 *
 */

#include "sysdep.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "jtag.h"

#include "cmd.h"

static void
cmd_bit_print_params (char *params[], unsigned int parameters, char *command,
                      size_t command_size)
{
    unsigned int i;

    command[0] = '\0';
    strncat (command, params[0], command_size);
    for (i = 1; i < parameters; i++)
    {
        strncat (command, " ", command_size);
        strncat (command, params[i], command_size);
    }
}

static int
cmd_bit_run (urj_chain_t *chain, char *params[])
{
    urj_part_t *part;
    urj_data_register_t *bsr;
    unsigned int bit;
    int type;
    int safe;
    unsigned int control;
    unsigned int parameters = urj_cmd_params (params);
    char command[1024];

    cmd_bit_print_params (params, parameters, command, sizeof command);

    if ((parameters != 5) && (parameters != 8))
    {
        printf (_("%s: invalid number of parameters (%d) for command '%s'\n"),
                "bit", parameters, command);
        return -1;
    }

    if (!urj_cmd_test_cable (chain))
    {
        printf (_("%s: cable test failed for command '%s'\n"), "bit",
                command);
        return 1;
    }

    if (!chain->parts)
    {
        printf (_("Run \"detect\" first.\n"));
        return 1;
    }

    if (chain->active_part >= chain->parts->len)
    {
        printf (_("%s: no active part\n"), "bit");
        return 1;
    }

    part = chain->parts->parts[chain->active_part];
    bsr = urj_part_find_data_register (part, "BSR");
    if (bsr == NULL)
    {
        printf (_
                ("%s: missing Boundary Scan Register (BSR) for command '%s'\n"),
                "bit", command);
        return 1;
    }

    /* bit number */
    if (urj_cmd_get_number (params[1], &bit))
    {
        printf (_("%s: unable to get boundary bit number for command '%s'\n"),
                "bit", command);
        return -1;
    }

    if (bit >= bsr->in->len)
    {
        printf (_("%s: invalid boundary bit number for command '%s'\n"),
                "bit", command);
        return 1;
    }
    if (part->bsbits[bit] != NULL)
    {
        printf (_("%s: duplicate bit declaration for command '%s'\n"), "bit",
                command);
        return 1;
    }

    /* bit type */
    if (strlen (params[2]) != 1)
    {
        printf (_("%s: invalid bit type length for command '%s'\n"), "bit",
                command);
        return -1;
    }
    switch (params[2][0])
    {
    case 'I':
    case 'i':
        type = URJ_BSBIT_INPUT;
        break;
    case 'O':
    case 'o':
        type = URJ_BSBIT_OUTPUT;
        break;
    case 'B':
    case 'b':
        type = URJ_BSBIT_BIDIR;
        break;
    case 'C':
    case 'c':
        type = URJ_BSBIT_CONTROL;
        break;
    case 'X':
    case 'x':
        type = URJ_BSBIT_INTERNAL;
        break;
    default:
        printf (_("%s: invalid bit type for command '%s'\n"), "bit", command);
        return -1;
    }

    /* default (safe) value */
    if (strlen (params[3]) != 1)
    {
        printf (_("%s: invalid default value length for command '%s'\n"),
                "bit", command);
        return -1;
    }

    safe = (params[3][0] == '1');
    bsr->in->data[bit] = safe;

    /* allocate bsbit */
    part->bsbits[bit] =
        urj_part_bsbit_alloc (bit, params[4], type,
                              urj_part_find_signal (part, params[4]), safe);
    if (part->bsbits[bit] == NULL)
    {
        printf (_("%s: out of memory for command '%s'\n"), "bit", command);
        return 1;
    }

    /* test for control bit */
    if (urj_cmd_params (params) == 5)
        return 1;

    /* control bit number */
    if (urj_cmd_get_number (params[5], &control))
    {
        printf (_("%s: unable to get control bit number for command '%s'\n"),
                "bit", command);
        return -1;
    }
    if (control >= bsr->in->len)
    {
        printf (_("%s: invalid control bit number for command '%s'\n"), "bit",
                command);
        return 1;
    }
    part->bsbits[bit]->control = control;

    /* control value */
    if (strlen (params[6]) != 1)
    {
        printf (_("%s: invalid control value length for command '%s'\n"),
                "bit", command);
        return -1;
    }
    part->bsbits[bit]->control_value = (params[6][0] == '1') ? 1 : 0;

    /* control state */
    if (strcasecmp (params[7], "Z"))
        return -1;

    part->bsbits[bit]->control_state = URJ_BSBIT_STATE_Z;

    return 1;
}

static void
cmd_bit_help (void)
{
    printf (_("Usage: %s NUMBER TYPE DEFAULT SIGNAL [CBIT CVAL CSTATE]\n"
              "Define new BSR (Boundary Scan Register) bit for SIGNAL, with\n"
              "DEFAULT value.\n"
              "\n"
              "NUMBER        Bit number in the BSR\n"
              "TYPE          Bit type, valid values are I, O, B, C, and X\n"
              "DEFAULT       Default (safe) bit value, valid values are 1, 0, ?\n"
              "SIGNAL        Associated signal name\n"
              "CBIT          Control bit number\n"
              "CVAL          Control value\n"
              "CSTATE        Control state, valid state is only Z\n"), "bit");
}

urj_cmd_t cmd_bit = {
    "bit",
    N_("define new BSR bit"),
    cmd_bit_help,
    cmd_bit_run
};
