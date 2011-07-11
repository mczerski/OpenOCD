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


static int or1k_jtag_inited = 0;
static int or1k_jtag_module_selected = -1;

int or1k_jtag_init(struct or1k_jtag *jtag_info)
{

  LOG_DEBUG("%s: Initialising OpenCores JTAG TAP for Mohor Debug Interface",
	    __func__);

  /* Put TAP into state where it can talk to the debug interface
     by shifting in correct value to IR */
  struct jtag_tap *tap;

  tap = jtag_info->tap;
  if (tap == NULL)
    return ERROR_FAIL;

  /* tap->ir_length should be set to 4 already, or we can hard code it */
  if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != 
      (uint32_t)OR1K_TAP_INST_DEBUG) /* OpenCores Mohor JTAG TAP-specific */
    {
      struct scan_field field;
      uint8_t t[4];
      uint8_t ret[4];
      
      field.num_bits = tap->ir_length;
      field.out_value = t;
      /* OpenCores Mohor JTAG TAP-specific */
      buf_set_u32(t, 0, field.num_bits, OR1K_TAP_INST_DEBUG);
      field.in_value = ret;

      /* Ensure TAP is reset - maybe not necessary*/
      jtag_add_tlr();
      
      jtag_add_ir_scan(tap, &field, TAP_IDLE);
      if (jtag_execute_queue() != ERROR_OK)
	{
	  LOG_ERROR("%s: setting TAP's IR to DEBUG failed", __func__);
	  return ERROR_FAIL;
	}
    }

  /* Tap should now be configured to communicate with debug interface */
  or1k_jtag_inited = 1;

  return ERROR_OK;

}

static uint32_t or1k_jtag_mohor_debug_crc_calc(uint32_t crc, 
					       uint32_t input_bit) 
{
  uint32_t d = (input_bit) ? 0xfffffff : 0x0000000;
  uint32_t crc_32 = ((crc >> 31)&1) ? 0xfffffff : 0x0000000;
  crc <<= 1;
#define OR1K_JTAG_MOHOR_DBG_CRC_POLY      0x04c11db7
  return crc ^ ((d ^ crc_32) & OR1K_JTAG_MOHOR_DBG_CRC_POLY);
}


int or1k_jtag_mohor_debug_select_module(struct or1k_jtag *jtag_info, 
					uint32_t module)
{
  int i;
  uint32_t out_module_select_bit, out_module,
    out_crc, in_crc, expected_in_crc, in_status;

  struct jtag_tap *tap;
  
  tap = jtag_info->tap;
  if (tap == NULL)
    return ERROR_FAIL;   

  if (module > 15)
    {
      LOG_ERROR("%s: setting debug interface module failed (%d)", __func__, 
		module);
      return ERROR_FAIL;
    }


  /*
   * CPU control register write
   * Send:
   * {1,4'moduleID,32'CRC,36'x           }
   * Receive:
   * {37'x               ,4'status,32'CRC}
   */  
  struct scan_field fields[5];
  
  /* 1st bit is module select, set to '1' */
  out_module_select_bit = 1;

  fields[0].num_bits = 1;
  fields[0].out_value = (uint8_t*) &out_module_select_bit;
  fields[0].in_value = NULL;

  /* Module number */
  out_module = flip_u32(module,4);
  fields[1].num_bits = 4;
  fields[1].out_value = (uint8_t*) &out_module;
  fields[1].in_value = NULL;

  /* CRC calculations */
  out_crc = 0xffffffff;
  out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, out_module_select_bit);
  for(i=0;i<4;i++)
    out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, ((out_module>>i)&0x1));
  out_crc = flip_u32(out_crc,32);

  /* CRC going out */
  fields[2].num_bits = 32;
  fields[2].out_value = (uint8_t*) &out_crc;
  fields[2].in_value = NULL;

  /* Status coming in */
  fields[3].num_bits = 4;
  fields[3].out_value = NULL;
  fields[3].in_value = (uint8_t*) &in_status;

  /* CRC coming in */
  fields[4].num_bits = 32;
  fields[4].out_value = NULL;
  fields[4].in_value = (uint8_t*) &in_crc;
  
  LOG_DEBUG("%s: setting mohor debug IF module: %d",__func__, module);

  jtag_add_dr_scan(tap, 5, fields, TAP_IDLE);

  if (jtag_execute_queue() != ERROR_OK)
    {
      LOG_ERROR("%s: performing module change failed", __func__);
      return ERROR_FAIL;
    }

  /* Calculate expected CRC for status */
  expected_in_crc = 0xffffffff;
  for(i=0;i<4;i++)
    expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, 
						     ((in_status>>i)&0x1));
  /* Check CRCs now */
  /* Bit reverse received CRC */
  expected_in_crc = flip_u32(expected_in_crc,32);

  if (in_crc != expected_in_crc)
    {
      LOG_ERROR("%s: received CRC (0x%08x) not same as calculated CRC (0x%08x)",
		__func__, in_crc, expected_in_crc);
      return ERROR_FAIL;
    }
  
  if (in_status & OR1K_MOHORDBGIF_MODULE_SELECT_CRC_ERROR)
    {
      LOG_ERROR("%s: debug IF module select status: CRC error",__func__);
      return ERROR_FAIL;
    }
  else if (in_status & OR1K_MOHORDBGIF_MODULE_SELECT_MODULE_NOT_EXIST)
    {
      LOG_ERROR("%s: debug IF module select status: Invalid module (%d)",
		__func__, module);
      return ERROR_FAIL;
    }
  else if ((in_status & 0xf) == OR1K_MOHORDBGIF_MODULE_SELECT_OK)
    {
      LOG_DEBUG("%s: setting mohor debug IF OK",__func__);
      or1k_jtag_module_selected = module;
    }
  else
    {
      LOG_ERROR("%s: debug IF module select status: Unknown status (%x)",
		__func__, in_status & 0xf);
      return ERROR_FAIL;
    }


  return ERROR_OK;

}

/* Currently hard set in functions to 32-bits */
int or1k_jtag_read_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, uint32_t *value)
{
  
  if (!or1k_jtag_inited)
    or1k_jtag_init(jtag_info);

  if (or1k_jtag_module_selected != OR1K_MOHORDBGIF_MODULE_CPU0)
    or1k_jtag_mohor_debug_select_module(jtag_info, OR1K_MOHORDBGIF_MODULE_CPU0);

  


  /* TODO - this function! */

  return ERROR_OK;
}
int or1k_jtag_write_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, uint32_t value)
{
  LOG_DEBUG("%s: writing CPU reg: 0x%x",__func__, value);

  if (!or1k_jtag_inited)
    or1k_jtag_init(jtag_info);

  if (or1k_jtag_module_selected != OR1K_MOHORDBGIF_MODULE_CPU0)
    or1k_jtag_mohor_debug_select_module(jtag_info, OR1K_MOHORDBGIF_MODULE_CPU0);

  return ERROR_OK;

}


int or1k_jtag_read_cpu_cr(struct or1k_jtag *jtag_info,
		uint32_t *value)
{
  LOG_DEBUG("%s: reading CPU control reg",__func__);

  if (!or1k_jtag_inited)
    or1k_jtag_init(jtag_info);
  
  if (or1k_jtag_module_selected != OR1K_MOHORDBGIF_MODULE_CPU0)
    or1k_jtag_mohor_debug_select_module(jtag_info, OR1K_MOHORDBGIF_MODULE_CPU0);

  struct jtag_tap *tap;
  
  tap = jtag_info->tap;
  if (tap == NULL)
    return ERROR_FAIL;   

  /*
   * CPU control register write
   * Send:
   * {1'0, 4'command, 32'CRC, (52+4+32)'x                             }
   * Receive:
   * {37'x                  , 1'reset, 1'stall, 50'x, 4'status, 32'CRC}
   */
  struct scan_field fields[9];
  uint32_t out_module_select_bit, out_cmd, out_crc;
  uint32_t in_status, in_crc, expected_in_crc, in_reset = 0, in_stall = 0,
    in_zeroes0, in_zeroes1;
  int i;

  /* 1st bit is module select, set to '0', we're not selecting a module */
  out_module_select_bit = 0;

  fields[0].num_bits = 1;
  fields[0].out_value = (uint8_t*) &out_module_select_bit;
  fields[0].in_value = NULL;

  /* Command, 4-bits */
  out_cmd = flip_u32(OR1K_MOHORDBGIF_CPU_MODULE_CMD_CTRL_READ,4);
  fields[1].num_bits = 4;
  fields[1].out_value = (uint8_t*) &out_cmd;
  fields[1].in_value = NULL;

  /* CRC calculations */
  out_crc = 0xffffffff;
  out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, out_module_select_bit);
  for(i=0;i<4;i++)
    out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, ((out_cmd>>i)&0x1));
  out_crc = flip_u32(out_crc,32);

  /* CRC going out */
  fields[2].num_bits = 32;
  fields[2].out_value = (uint8_t*) &out_crc;
  fields[2].in_value = NULL;

  
  /* 52-bit control register */
  fields[3].num_bits = 1;
  fields[3].out_value = NULL;
  fields[3].in_value = (uint8_t*) &in_reset;

  fields[4].num_bits = 1;
  fields[4].out_value = NULL;
  fields[4].in_value = (uint8_t*) &in_stall;

  /* Assuming the next 50 bits will always be 0 */
  fields[5].num_bits = 32;
  fields[5].out_value = NULL;
  fields[5].in_value = (uint8_t*) &in_zeroes0;

  fields[6].num_bits = 18;
  fields[6].out_value = NULL;
  fields[6].in_value = (uint8_t*) &in_zeroes1;
  
  /* Status coming in */
  fields[7].num_bits = 4;
  fields[7].out_value = NULL;
  fields[7].in_value = (uint8_t*) &in_status;

  /* CRC coming in */
  fields[8].num_bits = 32;
  fields[8].out_value = NULL;
  fields[8].in_value = (uint8_t*) &in_crc;
  
  jtag_add_dr_scan(tap, 9, fields, TAP_IDLE);

  if (jtag_execute_queue() != ERROR_OK)
    {
      LOG_ERROR("%s: performing CPU CR read failed", __func__);
      return ERROR_FAIL;
    }

  /*
  LOG_DEBUG("in_zeroes0: 0x%08x, in_zeroes1: 0x%08x",in_zeroes0,
	    in_zeroes1 & 0x3ffff);
  */

  /* Calculate expected CRC for status */
  expected_in_crc = 0xffffffff;
  expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, in_reset);
  expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, in_stall);
  /* Assuming next 50 bits are zero - we don't check, though!*/
  for(i=0;i<32;i++)
    expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, 
						     ((in_zeroes0>>i)&0x1));
  for(i=0;i<18;i++)
    expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, 
						     ((in_zeroes1>>i)&0x1));
  for(i=0;i<4;i++)
    expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, 
						     ((in_status>>i)&0x1));
  /* Check CRCs now */
  /* Bit reverse received CRC */
  expected_in_crc = flip_u32(expected_in_crc,32);

  if (in_crc != expected_in_crc)
    {
      LOG_ERROR("%s: received CRC (0x%08x) not same as calculated CRC (0x%08x)",
		__func__, in_crc, expected_in_crc);
      return ERROR_FAIL;
    }
  
  if (in_status & OR1K_MOHORDBGIF_CMD_CRC_ERROR)
    {
      LOG_ERROR("%s: debug IF CPU CR read status: CRC error",__func__);
      return ERROR_FAIL;
    }
  else if ((in_status&0xf) == OR1K_MOHORDBGIF_CMD_OK)
    {
      LOG_DEBUG("%s: debug IF CPU CR read OK",__func__);
    }
  else
    {
      LOG_ERROR("%s: debug IF CPU CR read: Unknown status (%d)",
		__func__, in_status);
      return ERROR_FAIL;
    }

  /* Convey status of control register */
  *value = 0;
  
  LOG_DEBUG("CPU CR reset bit: %0x",in_reset & 0x1);
  LOG_DEBUG("CPU CR stall bit: %0x",in_stall & 0x1);
  
  if (in_reset & 0x1)
    *value |= OR1K_MOHORDBGIF_CPU_CR_RESET;
    
  
  if (in_stall & 0x1)
    {
      *value |= OR1K_MOHORDBGIF_CPU_CR_STALL;
    }

  return ERROR_OK;
}

int or1k_jtag_write_cpu_cr(struct or1k_jtag *jtag_info,
			   uint32_t stall, uint32_t reset)
{

  LOG_DEBUG("%s: writing CPU control reg, reset: %d, stall: %d",__func__, 
	    reset, stall);

  if (!or1k_jtag_inited)
    or1k_jtag_init(jtag_info);

  if (or1k_jtag_module_selected != OR1K_MOHORDBGIF_MODULE_CPU0)
    or1k_jtag_mohor_debug_select_module(jtag_info, OR1K_MOHORDBGIF_MODULE_CPU0);

      
  struct jtag_tap *tap;
  
  tap = jtag_info->tap;
  if (tap == NULL)
    return ERROR_FAIL;   

  /*
   * CPU control register write
   * Send:
   * {1'0, 4'command, 1'reset, 1'stall, 50'0, 32'CRC, 36'x            }
   * Receive:
   * {89'x                                          , 4'status, 32'CRC}
   */
  struct scan_field fields[9];
  uint32_t out_module_select_bit, out_cmd, out_crc;
  uint32_t in_status, in_crc, expected_in_crc;
  int i;
  
  /* 1st bit is module select, set to '0', we're not selecting a module */
  out_module_select_bit = 0;

  fields[0].num_bits = 1;
  fields[0].out_value = (uint8_t*) &out_module_select_bit;
  fields[0].in_value = NULL;

  /* Command, 4-bits */
  out_cmd = flip_u32(OR1K_MOHORDBGIF_CPU_MODULE_CMD_CTRL_WRITE,4);
  fields[1].num_bits = 4;
  fields[1].out_value = (uint8_t*) &out_cmd;
  fields[1].in_value = NULL;
  
  /* 52-bit control register */
  fields[2].num_bits = 1;
  fields[2].out_value = (uint8_t*) &reset;
  fields[2].in_value = NULL;

  fields[3].num_bits = 1;
  fields[3].out_value = (uint8_t*) &stall;
  fields[3].in_value = NULL;

  fields[4].num_bits = 32;
  fields[4].out_value = NULL;
  fields[4].in_value = NULL;

  fields[5].num_bits = 18;
  fields[5].out_value = NULL;
  fields[5].in_value = NULL;

  /* CRC calculations */
  out_crc = 0xffffffff;
  out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, out_module_select_bit);
  for(i=0;i<4;i++)
    out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, ((out_cmd>>i)&0x1));
  out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, reset);
  out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, stall);
  for(i=0;i<50;i++)
    out_crc = or1k_jtag_mohor_debug_crc_calc(out_crc, 0);
  out_crc = flip_u32(out_crc,32);

  /* CRC going out */
  fields[6].num_bits = 32;
  fields[6].out_value = (uint8_t*) &out_crc;
  fields[6].in_value = NULL;

  /* Status coming in */
  fields[7].num_bits = 4;
  fields[7].out_value = NULL;
  fields[7].in_value = (uint8_t*) &in_status;

  /* CRC coming in */
  fields[8].num_bits = 32;
  fields[8].out_value = NULL;
  fields[8].in_value = (uint8_t*) &in_crc;
  
  jtag_add_dr_scan(tap, 9, fields, TAP_IDLE);

  if (jtag_execute_queue() != ERROR_OK)
    {
      LOG_ERROR("%s: performing CPU CR write failed", __func__);
      return ERROR_FAIL;
    }

  /* Calculate expected CRC for status */
  expected_in_crc = 0xffffffff;
  for(i=0;i<4;i++)
    expected_in_crc = or1k_jtag_mohor_debug_crc_calc(expected_in_crc, 
						     ((in_status>>i)&0x1));
  /* Check CRCs now */
  /* Bit reverse received CRC */
  expected_in_crc = flip_u32(expected_in_crc,32);

  if (in_crc != expected_in_crc)
    {
      LOG_ERROR("%s: received CRC (0x%08x) not same as calculated CRC (0x%08x)",
		__func__, in_crc, expected_in_crc);
      return ERROR_FAIL;
    }
  
  if (in_status & OR1K_MOHORDBGIF_CMD_CRC_ERROR)
    {
      LOG_ERROR("%s: debug IF CPU CR write status: CRC error",__func__);
      return ERROR_FAIL;
    }
  else if (in_status == OR1K_MOHORDBGIF_CMD_OK)
    {
      LOG_DEBUG("%s: debug IF CPU CR write OK",__func__);
    }
  else
    {
      LOG_ERROR("%s: debug IF module select status: Unknown status (%d)",
		__func__, in_status);
      return ERROR_FAIL;
    }

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

