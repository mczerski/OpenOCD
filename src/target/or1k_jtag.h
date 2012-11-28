/***************************************************************************
 *   Copyright (C) 2011 Julius Baxter                                      *
 *   julius@opencores.org                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef OR1K_JTAG
#define OR1K_JTAG

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/types.h"

struct or1k_jtag
{
	struct jtag_tap *tap;
};

int or1k_jtag_init(struct or1k_jtag *jtag_info);

/* Currently hard set in functions to 32-bits */
int or1k_jtag_read_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, int count, uint32_t *value);
int or1k_jtag_write_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, int count, const uint32_t * value);

int or1k_jtag_read_cpu_cr(struct or1k_jtag *jtag_info,
		uint32_t *value);

int or1k_jtag_write_cpu_cr(struct or1k_jtag *jtag_info,
			   uint32_t stall, uint32_t reset);


int or1k_jtag_read_memory32(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint32_t *buffer);
int or1k_jtag_read_memory16(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint16_t *buffer);
int or1k_jtag_read_memory8(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint8_t *buffer);

int or1k_jtag_write_memory32(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint32_t *buffer);
int or1k_jtag_write_memory16(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint16_t *buffer);
int or1k_jtag_write_memory8(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint8_t *buffer);


#endif /* OR1K_JTAG */

