/*
 * $Id$
 *
 * Copyright (C) 2002, 2003 ETC s.r.o.
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
 * Written by Marcel Telka <marcel@telka.sk>, 2002, 2003.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "gettext.h"
#define	_(s)		gettext(s)
#define	N_(s)		gettext_noop(s)
#define	P_(s,p,n)	ngettext(s,p,n)

#include <stdio.h>
#include <string.h>

#include "cable.h"
#include "flash.h"

void
help( const char *cmd )
{
	if (!cmd)
		printf(
			_("Command list:\n"
			"\n"
			"quit          exit from %s\n"
			"help          display this help\n"
			"frequency     setup JTAG frequency\n"
			"cable         select JTAG cable\n"
			"detect        detect parts on the JTAG chain\n"
			"discovery     discovery unknown parts in the JTAG chain\n"
			"print         display JTAG chain list/status\n"
			"instruction   change active instruction for a part\n"
			"shift         shift data/instruction register through JTAG chain\n"
			"dr            display active data register for a part\n"
			"detectflash   detect parameters of flash chip attached to a part\n"
			"readmem       read content of the memory and write it to file\n"
			"flashmem      burn flash memory with data from a file\n"
			"set           set external signal value\n"
			"get           get external signal value\n"
			"script        run command sequence from external file\n"
			"\n"
			"Type \"help COMMAND\" for details about particular command.\n"), PACKAGE
		);
	else if (strcmp( cmd, "quit" ) == 0)
		printf(
			_("Usage: %s\n"
			"Exit from %s.\n"), "quit", PACKAGE
		);
	else if (strcmp( cmd, "help" ) == 0)
		printf(
			_("Usage: %s [COMMAND]\n"
			"Print short help for COMMAND, or list of available commands.\n"), "help"
		);
	else if (strcmp( cmd, "frequency" ) == 0)
		printf(
			_("Usage: %s FREQ\n"
			"Change TCK frequency to FREQ.\n"
			"\n"
			"FREQ is in hertz. It's a maximum TCK frequency for JTAG interface.\n"
			"In some cases the TCK frequency is less than FREQ, but the frequency\n"
			"is never more than FREQ. Maximum supported frequency depends on JTAG\n"
			"adapter.\n"
			"\n"
			"FREQ must be an unsigned integer. Minimum allowed frequency is 1 Hz.\n"
			"Use 0 for FREQ to disable frequency limit.\n"), "frequency"
		);
	else if (strcmp( cmd, "cable" ) == 0) {
		int i;

		printf(
			_("Usage: %s PORTADDR CABLE\n"
			"Select JTAG cable connected to parallel port.\n"
			"\n"
			"PORTADDR   parallel port address (e.g. 0x378)\n"
			"CABLE      cable type\n"
			"\n"
			"List of supported cables:\n"
			"none          No cable connected\n"), "cable parallel"
		);

		for (i = 0; cable_drivers[i]; i++)
			printf( "%-14s%s\n", cable_drivers[i]->name, cable_drivers[i]->description );
	} else if (strcmp( cmd, "detect" ) == 0)
		printf(
			_("Usage: %s\n"
			"Detect parts on the JTAG chain.\n"
			"\n"
			"Output from this command is a list of the detected parts.\n"
			"If no parts are detected other commands may not work properly.\n"), "detect"
		);
	else if (strcmp( cmd, "discovery" ) == 0)
		printf(
			_("Usage: %s FILENAME\n"
			"Discovery unknown parts in the JTAG chain.\n"
			"\n"
			"Detail output (report) is directed to the FILENAME.\n"
			"'discovery' attempt to detect these parameters of an unknown JTAG\n"
			"chain:\n"
			" 1. JTAG chain size (number of parts in the chain)\n"
			" 2. IR (instruction register) length\n"
			" 3. DR (data register) length for all possible instructions\n"
			"\n"
			"Warning: This may be dangerous for some parts (especially, if the\n"
			"part doesn't have TRST signal).\n"), "discovery"
		);
	else if (strcmp( cmd, "print" ) == 0)
		printf(
			_("Usage: %s\n"
			"Display JTAG chain status.\n"
			"\n"
			"Display list of the parts connected to the JTAG chain including\n"
			"part number and current (active) instruction and data register.\n"), "print"
		);
	else if (strcmp( cmd, "instruction" ) == 0)
		printf(
			_("Usage: %s PART INSTRUCTION\n"
			"Change active INSTRUCTION for a PART.\n"
			"\n"
			"PART          part number (see print command)\n"
			"INSTRUCTION   instruction name (e.g. BYPASS)\n"), "instruction"
		);
	else if (strcmp( cmd, "shift" ) == 0)
		printf(
			_("Usage: %s\n"
			"Usage: %s\n"
			"Shift instruction or data register through JTAG chain.\n"), "shift ir", "shift dr"
		);
	else if (strcmp( cmd, "dr" ) == 0)
		printf(
			_("Usage: %s PART [DIR]\n"
			"Display input or output data register content.\n"
			"\n"
			"PART          part number (see print command)\n"
			"DIR           requested data register; possible values: 'in' for\n"
			"                input and 'out' for output; default is 'out'\n"), "dr"
		);
	else if (strcmp( cmd, "detectflash" ) == 0)
		printf(
			_("Usage: %s\n"
			"Detect flash memory type connected to part.\n"
			"\n"
			"Only detects flash connected to part 0. Part 0 must support\n"
			"bus operations.\n"), "detectflash"
		);
	else if (strcmp( cmd, "readmem" ) == 0)
		printf(
			_("Usage: %s ADDR LEN FILENAME\n"
			"Copy device memory content starting with ADDR to FILENAME file.\n"
			"\n"
			"ADDR       start address of the copied memory area\n"
			"LEN        copied memory length\n"
			"FILENAME   name of the output file\n"
			"\n"
			"ADDR and LEN could be in decimal or hexadecimal (prefixed with 0x) form.\n"
			"\n"
			"`%s' command works only with part 0. Part 0 must support bus operations.\n"),
			"readmem", "readmem"
		);
	else if (strcmp( cmd, "flashmem" ) == 0) {
 	        int i;
		printf(
			_("Usage: %s ADDR FILENAME\n"
			"Usage: flashmem msbin FILENAME\n"
			"Program FILENAME content to flash memory.\n"
			"\n"
			"ADDR       target addres for raw binary image\n"
			"FILENAME   name of the input file\n"
			"msbin      FILENAME is in MS .bin format (for WinCE)\n"
			"\n"
			"ADDR could be in decimal or hexadecimal (prefixed with 0x) form.\n"
			"\n"
			"`%s' command works only with part 0. Part 0 must support bus operations.\n"
			"Supported Flash Memories\n"), "flashmem", "flashmem"
		);
		for (i = 0; flash_drivers[i]; i++)
			printf( "%s\n     %s\n", flash_drivers[i]->name, flash_drivers[i]->description );
	} else if (strcmp( cmd, "get" ) == 0)
		printf(
			_("Usage: %s PART SIGNAL\n"
			"Get signal state from output BSR (Boundary Scan Register).\n"
			"\n"
			"PART          part number (see print command)\n"
			"SIGNAL        signal name (from JTAG declaration file)\n"), "get signal"
		);
	else if (strcmp( cmd, "set" ) == 0)
		printf(
			_("Usage: %s PART SIGNAL DIR [DATA]\n"
			"Set signal state in input BSR (Boundary Scan Register).\n"
			"\n"
			"PART          part number (see print command)\n"
			"SIGNAL        signal name (from JTAG declaration file)\n"
			"DIR           requested signal direction; possible values: 'in' or 'out'\n"
			"DATA          desired output signal value ('0' or '1');  used only if DIR\n"
			"                is 'out'\n"), "set signal"
		);
	else if (strcmp( cmd, "script" ) == 0)
		printf(
			_("Usage: %s FILENAME\n"
			"Run command sequence from external FILENAME.\n"
			"\n"
			"FILENAME      Name of the file with commands\n"), "script"
		);
	else
		printf( _("Invalid command.\n") );
}
