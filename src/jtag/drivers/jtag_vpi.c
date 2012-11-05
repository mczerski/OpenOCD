#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

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
/*
static int ft2232_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type)
	{
	case JTAG_RESET:	retval = ft2232_execute_reset(cmd); break;
	case JTAG_RUNTEST:	retval = ft2232_execute_runtest(cmd); break;
	case JTAG_TLR_RESET: retval = ft2232_execute_statemove(cmd); break;
	case JTAG_PATHMOVE:	retval = ft2232_execute_pathmove(cmd); break;
	case JTAG_SCAN:		retval = ft2232_execute_scan(cmd); break;
	case JTAG_SLEEP:	retval = ft2232_execute_sleep(cmd); break;
	case JTAG_STABLECLOCKS:	retval = ft2232_execute_stableclocks(cmd); break;
	case JTAG_TMS:
		retval = ft2232_execute_tms(cmd);
		break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		retval = ERROR_JTAG_QUEUE_FAILED;
		break;
	}
	return retval;
}
*/
static int jtag_vpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			printf("--> JTAG_RESET\n");
			break;
		case JTAG_RUNTEST:
			printf("--> JTAG_RUNTEST\n");
			break;
		case JTAG_STABLECLOCKS:
			printf("--> JTAG_STABLECLOCKS\n");
			break;
		case JTAG_TLR_RESET:
			printf("--> JTAG_TLR_RESET\n");
			break;
		case JTAG_PATHMOVE:
			printf("--> JTAG_PATHMOVE\n");
			break;
		case JTAG_TMS:
			printf("--> JTAG_TMS\n");
			break;
		case JTAG_SLEEP:
			printf("--> JTAG_SLEEP\n");
			break;
		case JTAG_SCAN:
			printf("--> JTAG_SCAN\n");
			break;
		}
	}

	return ret;
}

static int jtag_vpi_init(void)
{
	return ERROR_OK;
}


static int jtag_vpi_quit(void)
{
	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_handle_test)
{
	if (CMD_ARGC == 1)
	{
		LOG_ERROR("expected exactly one argument to ft2232_device_desc <description>");
	}
	else
	{
		LOG_ERROR("expected exactly one argument to ft2232_device_desc <description>");
	}

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
