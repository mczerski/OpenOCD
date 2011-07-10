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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "helper/types.h"
#include "jtag/jtag.h"
#include "or1k_jtag.h"
#include "or1k.h"

/* Currently hard set in functions to 32-bits */
int or1k_jtag_read_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, uint32_t *value)
{
  /* TODO - this function! */
  return ERROR_OK;
}
int or1k_jtag_write_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, uint32_t value)
{
  /* TODO - this function! */
  return ERROR_OK;
}


int or1k_jtag_read_cpu_cr(struct or1k_jtag *jtag_info,
		uint32_t *value)
{
  /* TODO - this function! */
  return ERROR_OK;
}

int or1k_jtag_write_cpu_cr(struct or1k_jtag *jtag_info,
		uint32_t value)
{
  /* TODO - this function! */
  return ERROR_OK;
}




int or1k_jtag_read_memory32(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint32_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}
int or1k_jtag_read_memory16(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint16_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}
int or1k_jtag_read_memory8(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, uint8_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}

int or1k_jtag_write_memory32(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint32_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}
int or1k_jtag_write_memory16(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint16_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}
int or1k_jtag_write_memory8(struct or1k_jtag *jtag_info, 
		uint32_t addr, int count, const uint8_t *buffer)
{
  /* TODO - this function! */
  return ERROR_OK;
}


int or1k_jtag_read_regs(struct or1k_jtag *jtag_info, uint32_t *regs)
{
	int i;

	/* read core registers */
	for (i = 0; i < OR1KNUMCOREREGS - 1; i++) 
		or1k_jtag_read_cpu(jtag_info, i, regs + i);

	/* read status register */
	/*
	retval = or1k_jtag_exec(jtag_info, MFSR(0, 0));
	if (retval != ERROR_OK)
		return retval;
	*/
	/*
	retval = or1k_jtag_read_reg(jtag_info, 0, regs + AVR32_REG_SR);

	return retval;
	*/
	return ERROR_OK;
}

int or1k_jtag_write_regs(struct or1k_jtag *jtag_info, uint32_t *regs)
{
	int i;
	
	/*
	retval = or1k_jtag_write_reg(jtag_info, 0, regs[OR1K_REG_SR]);
	*/
	/* Restore Status reg */
        /*
	retval = or1k_jtag_exec(jtag_info, MTSR(0, 0));
	if (retval != ERROR_OK)
		return retval;
	*/
	/*
	 * And now the rest of registers
	 */
	for (i = 0; i < OR1KNUMCOREREGS - 1; i++) 
		or1k_jtag_write_cpu(jtag_info, i, regs[i]);

	return ERROR_OK;
}

