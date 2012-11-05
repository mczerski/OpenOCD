#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <arpa/inet.h>

#define SERVER_PORT	50010
#define SERVER_ADDRESS	"127.0.0.1"

int sockfd = 0;
struct sockaddr_in serv_addr;

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
