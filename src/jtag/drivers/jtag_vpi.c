/*
 * JTAG to VPI driver
 *
 * Copyright (C) 2012 Franck JULLIEN, <elec4fun@gmail.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <arpa/inet.h>

#define NO_TAP_SHIFT	0
#define TAP_SHIFT	1

#define SERVER_ADDRESS	"127.0.0.1"

#define CMD_RESET		0
#define CMD_TMS_SEQ		1
#define CMD_SCAN_CHAIN		2
#define CMD_SCAN_CHAIN_FLIP_TMS	3

int server_port = 500020;

int sockfd = 0;
struct sockaddr_in serv_addr;

struct vpi_cmd {
	int cmd;
	unsigned char buffer_out[512];
	unsigned char buffer_in[512];
	int length;
	int nb_bits;
};

static int jtag_vpi_send_cmd(struct vpi_cmd * vpi)
{
	return write(sockfd, vpi, sizeof(struct vpi_cmd));
}

static int jtag_vpi_receive_cmd(struct vpi_cmd * vpi)
{
	return read(sockfd, vpi, sizeof(struct vpi_cmd));
}

static int jtag_vpi_speed(int speed)
{
	return ERROR_OK;
}

static int jtag_vpi_speed_div(int speed, int* khz)
{
	return ERROR_OK;
}

static int jtag_vpi_khz(int khz, int* jtag_speed)
{
	return ERROR_OK;
}

/**
 * jtag_vpi_reset - ask to reset the JTAG device
 * @trst: 1 if TRST is to be asserted
 * @srst: 1 if SRST is to be asserted
 */
static void jtag_vpi_reset(int trst, int srst)
{
	struct vpi_cmd vpi;

	vpi.cmd = CMD_RESET;
	vpi.length = 0;
	jtag_vpi_send_cmd(&vpi);
}

/**
 * jtag_vpi_tms_seq - ask a TMS sequence transition to JTAG
 * @bits: TMS bits to be written (bit0, bit1 .. bitN)
 * @nb_bits: number of TMS bits (between 1 and 8)
 *
 * Write a serie of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=<new_state>, TDI=<???>
 *  - writing out TCK=1, TMS=<new_state>, TDI=<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
static void jtag_vpi_tms_seq(const uint8_t *bits, int nb_bits)
{
	struct vpi_cmd vpi;
	int nb_bytes;

	nb_bytes = (nb_bits / 8) + !!(nb_bits % 8);

	vpi.cmd = CMD_TMS_SEQ;
	memcpy(vpi.buffer_out, bits, nb_bytes);
	vpi.length = nb_bytes;
	vpi.nb_bits = nb_bits;
	LOG_DEBUG("jtag_vpi_tms_seq: (bits=%02x..., nb_bits=%d)", bits[0], nb_bits);
	jtag_vpi_send_cmd(&vpi);
}

/**
 * jtag_vpi_path_move - ask a TMS sequence transition to JTAG
 * @cmd: path transition
 *
 * Write a serie of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=<new_state>, TDI=<???>
 *  - writing out TCK=1, TMS=<new_state>, TDI=<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
static void jtag_vpi_path_move(struct pathmove_command *cmd)
{
	int i;
	const uint8_t tms_0 = 0;
	const uint8_t tms_1 = 1;

	LOG_DEBUG("jtag_vpi_path_move: (num_states=%d, last_state=%d)",
		  cmd->num_states, cmd->path[cmd->num_states - 1]);

	for (i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
			jtag_vpi_tms_seq(&tms_0, 1);
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			jtag_vpi_tms_seq(&tms_1, 1);
		tap_set_state(cmd->path[i]);
	}
}

/**
 * jtag_vpi_tms - ask a tms command
 * @cmd: tms command
 */
static void jtag_vpi_tms(struct tms_command *cmd)
{
	LOG_DEBUG("jtag_vpi_tms: (num_bits=%d)", cmd->num_bits);
	jtag_vpi_tms_seq(cmd->bits, cmd->num_bits);
}

static void jtag_vpi_state_move(tap_state_t state)
{
	uint8_t tms_scan;
	int tms_len;

	LOG_DEBUG("jtag_vpi_state_move: (from %s to %s)", tap_state_name(tap_get_state()),
		  tap_state_name(state));

	if (tap_get_state() == state)
		return;

	tms_scan = tap_get_tms_path(tap_get_state(), state);
	tms_len = tap_get_tms_path_len(tap_get_state(), state);
	jtag_vpi_tms_seq(&tms_scan, tms_len);
	tap_set_state(state);
}

/**
 * jtag_vpi_queue_tdi - short description
 * @bits: bits to be queued on TDI (or NULL if 0 are to be queued)
 * @nb_bits: number of bits
 * @scan: scan type (ie. if TDO read back is required or not)
 *
 * Outputs a serie of TDI bits on TDI.
 * As a side effect, the last TDI bit is sent along a TMS=1, and triggers a JTAG
 * TAP state shift if input bits were non NULL.
 *
 * In order to not saturate the USB Blaster queues, this method reads back TDO
 * if the scan type requests it, and stores them back in bits.
 *
 * As a side note, the state of TCK when entering this function *must* be
 * low. This is because byteshift mode outputs TDI on rising TCK and reads TDO
 * on falling TCK if and only if TCK is low before queuing byteshift mode bytes.
 * If TCK was high, the USB blaster will queue TDI on falling edge, and read TDO
 * on rising edge !!!
 */
static void jtag_vpi_queue_tdi(uint8_t *bits, int nb_bits, enum scan_type scan, int tap_shift)
{
	struct vpi_cmd vpi;
	int nb_bytes;

	nb_bytes = (nb_bits / 8) + !!(nb_bits % 8);

	vpi.cmd = tap_shift ? CMD_SCAN_CHAIN_FLIP_TMS : CMD_SCAN_CHAIN;

	if (bits)
		memcpy(vpi.buffer_out, bits, nb_bytes);
	else
		memset(vpi.buffer_out, 0xff, nb_bytes);

	vpi.length = nb_bytes;
	vpi.nb_bits = nb_bits;

	LOG_DEBUG("jtag_vpi_queue_tdi: (bits=%02x..., nb_bits=%d)", bits[0], nb_bits);
	jtag_vpi_send_cmd(&vpi);
	jtag_vpi_receive_cmd(&vpi);

	if (bits)
		memcpy(bits, vpi.buffer_in, nb_bytes);
}

/**
 * jtag_vpi_clock_tms - clock a TMS transition
 * @tms: the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
static void jtag_vpi_clock_tms(int tms)
{
	const uint8_t tms_0 = 0;
	const uint8_t tms_1 = 1;

	jtag_vpi_tms_seq(tms ? &tms_1 : &tms_0, 1);
}

/**
 * jtag_vpi_scan - launches a DR-scan or IR-scan
 * @cmd: the command to launch
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occured.
 */
static int jtag_vpi_scan(struct scan_command *cmd)
{
	int scan_bits;
	uint8_t *buf = NULL;
	enum scan_type type;
	int ret = ERROR_OK;

	type = jtag_scan_type(cmd);
	scan_bits = jtag_build_buffer(cmd, &buf);

	if (cmd->ir_scan)
		jtag_vpi_state_move(TAP_IRSHIFT);
	else
		jtag_vpi_state_move(TAP_DRSHIFT);

	if(cmd->end_state == TAP_DRSHIFT)
		jtag_vpi_queue_tdi(buf, scan_bits, type, NO_TAP_SHIFT);
	else
		jtag_vpi_queue_tdi(buf, scan_bits, type, TAP_SHIFT);

	if(cmd->end_state != TAP_DRSHIFT) {
		/*
		 * As our JTAG is in an unstable state (IREXIT1 or DREXIT1), move it
		 * forward to a stable IRPAUSE or DRPAUSE.
		 */
		jtag_vpi_clock_tms(0);
		if (cmd->ir_scan)
			tap_set_state(TAP_IRPAUSE);
		else
			tap_set_state(TAP_DRPAUSE);
	}

	ret = jtag_read_buffer(buf, cmd);
	if (buf)
		free(buf);

	if(cmd->end_state != TAP_DRSHIFT)
		jtag_vpi_state_move(cmd->end_state);

	return ret;
}

static void jtag_vpi_runtest(int cycles, tap_state_t state)
{
	LOG_DEBUG("jtag_vpi_runtest: (cycles=%i, end_state=%d)", cycles, state);

	jtag_vpi_state_move(TAP_IDLE);
	jtag_vpi_queue_tdi(NULL, cycles, SCAN_OUT, TAP_SHIFT);
	jtag_vpi_state_move(state);
}

static void jtag_vpi_stableclocks(int cycles)
{
	LOG_DEBUG("jtag_vpi_stableclocks: (cycles=%i)", cycles);
	jtag_vpi_queue_tdi(NULL, cycles, SCAN_OUT, TAP_SHIFT);
}

static int jtag_vpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG("--> JTAG_RESET");
			jtag_vpi_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			LOG_DEBUG("--> JTAG_RUNTEST");
			jtag_vpi_runtest(cmd->cmd.runtest->num_cycles,
				         cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			LOG_DEBUG("--> JTAG_STABLECLOCKS");
			jtag_vpi_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			LOG_DEBUG("--> JTAG_TLR_RESET");
			jtag_vpi_state_move(cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			LOG_DEBUG("--> JTAG_PATHMOVE");
			jtag_vpi_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			LOG_DEBUG("--> JTAG_TMS");
			jtag_vpi_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			LOG_DEBUG("--> JTAG_SLEEP");
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			LOG_DEBUG("--> JTAG_SCAN");
			ret = jtag_vpi_scan(cmd->cmd.scan);
			break;
		}
	}

	return ret;
}

static int jtag_vpi_init(void)
{
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Error : Could not create socket \n");
		return ERROR_FAIL;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(server_port);

	if (inet_pton(AF_INET, SERVER_ADDRESS, &serv_addr.sin_addr) <= 0) {
		printf("\n inet_pton error occured\n");
		return ERROR_FAIL;
	}

	printf("Connection to %s : %u ", SERVER_ADDRESS, server_port);
	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		printf("failed\n");
		return ERROR_COMMAND_CLOSE_CONNECTION;
	}

	printf("succeed\n");

	return ERROR_OK;
}


static int jtag_vpi_quit(void)
{
	LOG_DEBUG("--> jtag_vpi_quit");
	close(sockfd);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_set_port)
{
	if (CMD_ARGC == 0) {
		LOG_WARNING("You need to set a port number");
	} else {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], server_port);
	}

	printf("Set server port to %u\n", server_port);

	return ERROR_OK;
}


static const struct command_registration jtag_vpi_command_handlers[] = {
	{
		.name = "jtag_vpi_set_port",
		.handler = &jtag_vpi_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the VPi server",
		.usage = "description_string",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface jtag_vpi_interface = {
	.name = "jtag_vpi",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = jtag_vpi_command_handlers,
	.transports = jtag_only,

	.init = jtag_vpi_init,
	.quit = jtag_vpi_quit,
	.speed = jtag_vpi_speed,
	.speed_div = jtag_vpi_speed_div,
	.khz = jtag_vpi_khz,
	.execute_queue = jtag_vpi_execute_queue,
};
