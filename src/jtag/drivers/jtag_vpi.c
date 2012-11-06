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

#define SERVER_PORT	50020
#define SERVER_ADDRESS	"127.0.0.1"

#define CMD_RESET	0
#define CMD_TMS_SEQ	1
#define CMD_SCAN_CHAIN	2

int sockfd = 0;
struct sockaddr_in serv_addr;

struct vpi_cmd {
	int cmd;
	unsigned char buffer_out[128];
	unsigned char buffer_in[128];
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
	printf("jtag_vpi_tms_seq: (bits=%02x..., nb_bits=%d)\n", bits[0], nb_bits);
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

	printf("jtag_vpi_path_move: (num_states=%d, last_state=%d)\n",
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
	printf("jtag_vpi_tms: (num_bits=%d)\n", cmd->num_bits);
	jtag_vpi_tms_seq(cmd->bits, cmd->num_bits);
}

static void jtag_vpi_state_move(tap_state_t state)
{
	uint8_t tms_scan;
	int tms_len;

	printf("jtag_vpi_state_move: (from %s to %s)\n", tap_state_name(tap_get_state()),
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
static void jtag_vpi_queue_tdi(uint8_t *bits, int nb_bits, enum scan_type scan)
{
	struct vpi_cmd vpi;
	int nb_bytes;

	nb_bytes = (nb_bits / 8) + !!(nb_bits % 8);

	vpi.cmd = CMD_SCAN_CHAIN;
	memcpy(vpi.buffer_out, bits, nb_bytes);
	vpi.length = nb_bytes;
	vpi.nb_bits = nb_bits;

	printf("jtag_vpi_queue_tdi: (bits=%02x..., nb_bits=%d)\n", bits[0], nb_bits);
	jtag_vpi_send_cmd(&vpi);
	jtag_vpi_receive_cmd(&vpi);
	memcpy(bits, vpi.buffer_in, nb_bytes);
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

	jtag_vpi_queue_tdi(buf, scan_bits, type);

	if(cmd->end_state != TAP_DRSHIFT) {
		/*
		 * As our JTAG is in an unstable state (IREXIT1 or DREXIT1), move it
		 * forward to a stable IRPAUSE or DRPAUSE.
		 */
		// ???? ublast_clock_tms(0);
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
	DEBUG_JTAG_IO("%s(cycles=%i, end_state=%d)", __func__, cycles, state);

	jtag_vpi_state_move(TAP_IDLE);
	jtag_vpi_queue_tdi(NULL, cycles, SCAN_OUT);
	jtag_vpi_state_move(state);
}

static void jtag_vpi_stableclocks(int cycles)
{
	DEBUG_JTAG_IO("%s(cycles=%i)", __func__, cycles);
	jtag_vpi_queue_tdi(NULL, cycles, SCAN_OUT);
}

static int jtag_vpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			printf("--> JTAG_RESET\n");
			jtag_vpi_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			printf("--> JTAG_RUNTEST\n");
			jtag_vpi_runtest(cmd->cmd.runtest->num_cycles,
				         cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			printf("--> JTAG_STABLECLOCKS\n");
			jtag_vpi_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			printf("--> JTAG_TLR_RESET\n");
			jtag_vpi_state_move(cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			printf("--> JTAG_PATHMOVE\n");
			jtag_vpi_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			printf("--> JTAG_TMS\n");
			jtag_vpi_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			printf("--> JTAG_SLEEP\n");
			break;
		case JTAG_SCAN:
			printf("--> JTAG_SCAN\n");
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
	serv_addr.sin_port = htons(SERVER_PORT);

	if (inet_pton(AF_INET, SERVER_ADDRESS, &serv_addr.sin_addr) <= 0) {
		printf("\n inet_pton error occured\n");
		return ERROR_FAIL;
	}

	printf("Connection to %s : %u ", SERVER_ADDRESS, SERVER_PORT);
	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		printf("failed\n");
		return ERROR_FAIL;
	}

	printf("succeed\n");

	return ERROR_OK;
}


static int jtag_vpi_quit(void)
{
	printf("--> jtag_vpi_quit\n");
	close(sockfd);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_handle_test)
{
	printf("--> jtag_vpi_handle_test\n");

	return ERROR_OK;
}


static const struct command_registration jtag_vpi_command_handlers[] = {
	{
		.name = "jtag_vpi_handle_test",
		.handler = &jtag_vpi_handle_test,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the FTDI FT2232 device",
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
