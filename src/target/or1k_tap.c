/*
 * or1k_tap.c
 *
 *  Created on: 19-11-2012
 *      Author: Marek Czerski
 */

#include "or1k_tap.h"
#include "or1k_jtag.h"

#include "jtag/jtag.h"

// Contains constants relevant to the Altera Virtual JTAG
// device, which are not included in the BSDL.
// As of this writing, these are constant across every
// device which supports virtual JTAG.

// These are commands for the FPGA's IR
#define ALTERA_CYCLONE_CMD_VIR     0x0E
#define ALTERA_CYCLONE_CMD_VDR     0x0C

// These defines are for the virtual IR (not the FPGA's)
// The virtual TAP was defined in hardware to match the OpenCores native
// TAP in both IR size and DEBUG command.
#define ALT_VJTAG_IR_SIZE    4
#define ALT_VJTAG_CMD_DEBUG  0x8

// SLD node ID
#define JTAG_TO_AVALON_NODE_ID	0x84
#define VJTAG_NODE_ID		0x08
#define SIGNAL_TAP_NODE_ID	0x00

union hub_info {
	struct {
		unsigned m_width :8;
		unsigned manufacturer_id :11;
		unsigned nb_of_node :8;
		unsigned version :5;
	};
	uint32_t dword;
};

union node_info {
	struct {
		unsigned instance_id :8;
		unsigned manufacturer_id :11;
		unsigned node_id :8;
		unsigned version :5;
	};
	uint32_t dword;
};

/* tap instructions - Mohor JTAG TAP */
#define OR1K_TAP_INST_IDCODE 0x2
#define OR1K_TAP_INST_DEBUG 0x8

#if (ALTERA_VJTAG == 1)
#ifdef VERBOSE_SLD_NODE
static char * id_to_string(unsigned char id)
{
	switch(id) {
		case VJTAG_NODE_ID          : return "Virtual JTAG";
		case JTAG_TO_AVALON_NODE_ID : return "JTAG to avalon bridge";
		case SIGNAL_TAP_NODE_ID     : return "Signal TAP";
	}
	return "unknown";
}
#endif
static unsigned char guess_addr_width(unsigned char number_of_nodes)
{
	unsigned char width = 0;

	while (number_of_nodes) {
		number_of_nodes >>= 1;
		width ++;
	}

	return width;
}
#endif //(ALTERA_VJTAG == 1)

int or1k_tap_init(struct or1k_jtag *jtag_info)
{
#if (ALTERA_VJTAG == 1)
	union hub_info hub;
	union node_info node;
	struct scan_field field;
	struct jtag_tap *tap;
	int i;
	int node_index;
	int vjtag_node_address = 0;

	uint8_t t[8];
	uint8_t ret;

	tap = jtag_info->tap;
	if (tap == NULL)
		return ERROR_FAIL;

	hub.dword = 0;
	node.dword = 0;

	LOG_DEBUG("Initialising Altera Virtual JTAG TAP");

	/* Ensure TAP is reset - maybe not necessary*/
	jtag_add_tlr();

	/* Select VIR */
	field.num_bits = tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, ALTERA_CYCLONE_CMD_VIR);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	/* The SLD hub contains the HUB IP Configuration Register and SLD_NODE_INFO
	 * register for each SLD node in the design. The HUB IP configuration register provides
	 * information needed to determine the dimensions of the USER1 DR chain. The
	 * SLD_NODE_INFO register is used to determine the address mapping for Virtual
	 * JTAG instance in your design. This register set is shifted out by issuing the
	 * HUB_INFO instruction. Both the ADDR bits for the SLD hub and the HUB_INFO
	 * instruction is 0 × 0.
	 * Because m and n are unknown at this point, the DR register
	 * (ADDR bits + VIR_VALUE) must be filled with zeros. Shifting a sequence of 64 zeroes
	 * into the USER1 DR is sufficient to cover the most conservative case for m and n.
	 */

	field.num_bits= 64;
	field.out_value = t;
	field.in_value = NULL;
	memset(t, 0, 8);
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* Select VDR */
	field.num_bits = tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, ALTERA_CYCLONE_CMD_VDR);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	jtag_execute_queue();

	for(i = 0;i < 8;i++) {
		field.num_bits= 4;
		field.out_value = NULL;
		field.in_value = &ret;
		jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
		jtag_execute_queue();
		hub.dword = ((hub.dword >> 4) | ((ret & 0xf) << 28));
	}

#ifdef VERBOSE_SLD_NODE
	LOG_DEBUG("\nSLD HUB Configuration register");
	LOG_DEBUG("------------------------------");
	LOG_DEBUG("m_width         = %d", hub.m_width);
	LOG_DEBUG("manufacturer_id = 0x%02x", hub.manufacturer_id);
	LOG_DEBUG("nb_of_node      = %d", hub.nb_of_node);
	LOG_DEBUG("version         = %d\n", hub.version);
	LOG_DEBUG("VIR length      = %d", guess_addr_width(hub.nb_of_node) + hub.m_width);
#endif

	/* Because the number of SLD nodes is now known, the Nodes on the hub can be
	 * enumerated by repeating the 8 four-bit nibble scans, once for each Node,
	 * to yield the SLD_NODE_INFO register of each Node. The DR nibble shifts
	 * are a continuation of the HUB_INFO DR shift used to shift out the Hub IP
	 * Configuration register.
	 */

	for(node_index = 0;node_index < hub.nb_of_node;node_index++) {

		for(i = 0;i < 8;i++) {
			field.num_bits= 4;
			field.out_value = NULL;
			field.in_value = &ret;
			jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
			jtag_execute_queue();
			node.dword = ((node.dword >> 4) | ((ret & 0xf) << 28));
		}
#ifdef VERBOSE_SLD_NODE
		LOG_DEBUG("\nNode info register");
		LOG_DEBUG("--------------------");
		LOG_DEBUG("instance_id     = %d",node.instance_id);
		LOG_DEBUG("manufacturer_id = 0x%02x", node.manufacturer_id);
		LOG_DEBUG("node_id         = %d (%s)", node.node_id, id_to_string(node.node_id));
		LOG_DEBUG("version         = %d\n", node.version);
#endif
		if(node.node_id == VJTAG_NODE_ID) vjtag_node_address = node_index + 1;
	}

	/* Select VIR */
	field.num_bits = tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, ALTERA_CYCLONE_CMD_VIR);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	/* Send the DEBUG command */
	field.num_bits = guess_addr_width(hub.nb_of_node) + hub.m_width;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, (vjtag_node_address << hub.m_width) | ALT_VJTAG_CMD_DEBUG);
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* Select VDR */
	field.num_bits = tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, ALTERA_CYCLONE_CMD_VDR);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	jtag_execute_queue();
#else //(ALTERA_VJTAG == 1)

	LOG_DEBUG(" Initialising OpenCores JTAG TAP");

	/* Put TAP into state where it can talk to the debug interface
	   by shifting in correct value to IR */
	struct jtag_tap *tap;

	tap = jtag_info->tap;
	if (tap == NULL)
		return ERROR_FAIL;


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
		LOG_ERROR(" setting TAP's IR to DEBUG failed");
		return ERROR_FAIL;
	}

#endif //(ALTERA_VJTAG == 1)
	return ERROR_OK;

}

