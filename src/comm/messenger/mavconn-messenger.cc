/*=====================================================================

 MAVCONN Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

 This file is part of the MAVCONN project

 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
 * @file
 *   @brief MAVLINK Messenger
 *
 *   Send Messages from command line or script
 *
 *   @author Laurens Mackay
 *
 */
#include <iostream>
#include <glib.h>
#include "mavconn.h"

lcm_t * lcm; ///< LCM interprocess reference

void send_reset(void) {
	char param[15] = "SYS_IMU_RESET";//{'I','M','U','_','R','E','S','E','T'};
	mavlink_message_t msg;
	mavlink_param_set_t p;
	p.param_value = 1;
	p.target_system = (uint8_t) getSystemID();
	p.target_component = (uint8_t) MAV_COMP_ID_IMU;
	strncpy((char*) p.param_id, param, 15);
	p.param_id[14] = 0;
	mavlink_msg_param_set_encode(getSystemID(), PX_COMP_ID_MESSENGER, &msg, &p);

	sendMAVLinkMessage(lcm, &msg);
}

int main(int argc, char* argv[]) {
	// Handling Program options

	bool reset = false;
	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "reset", 'r', 0, G_OPTION_ARG_NONE, &reset, "Reset board", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- send a notification to the system");
	g_option_context_add_main_entries (context, entries, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		printf("Option parsing failed: %s\n", error->message);
		exit (1);
	}

	lcm = lcm_create("udpm://");
	if (!lcm) {
		printf("\nCouldn't start LCM link, aborting\n");
		return 1;
	}
	if (reset) {

		if (reset) {
			send_reset();
			printf("\nRESET IMU\n");
		} else {

			printf("\nNO reset\n");
		}
		return 1;
	}

}
