/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

(c) 2009, 2010 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief
 *
 *   @author Samuel Zihlmann <samuezih@ee.ethz.ch>, Sebastian Wendland <wendlans@ee.ethz.ch>
 *   @author Alex Trofimov <talex@student.ethz.ch>
 */

#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include <mavconn.h>
#include <math.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

struct timeval tv;	///< timer for benchmarking

lcm_t * lcm;

mavlink_local_position_t currentpos;	///< current position
mavlink_attitude_t currentatt;	///< current attitude
mavlink_waypoint_t* wpl = NULL;	///< current waypoint list

uint16_t wn=0;	///< number of waypoints in the waypoint list
uint16_t cseq=0;	///< current sequence number (of the actual waypoint in the list)
bool rwpl=true;	///< true if requesting new waypointlist

uint8_t systemid = getSystemID();
uint8_t compid = MAV_COMP_ID_IMU;

typedef struct _mav_destination
{
	uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	float x; //local: x position, global: longitude
	float y; //local: y position, global: latitude
	float z; //local: z position, global: altitude
	float yaw; //Yaw orientation in degrees, [0..360] 0 = NORTH
} mav_destination;


mav_destination cur_dest;				 ///< current flight destination




/** @brief request the actual waypointlist from waypointplanner
 *
 */
void requestWaypointlist ()
{
	//printf("Request waypoints.\n");
	rwpl=true;
	mavlink_message_t message;
    mavlink_waypoint_request_list_t wprl;
    wprl.target_system = systemid;
    wprl.target_component = MAV_COMP_ID_WAYPOINTPLANNER;
    mavlink_msg_waypoint_request_list_encode(systemid, compid, &message, &wprl);
    mavlink_message_t_publish(lcm, "MAVLINK", &message);
}

/** @brief handle incoming MAVLINKs
 *
 * handle 'Local Position', 'Attitude', 'Waypoint', 'Waypoint count', 'Waypoint Current', 'Waypoint Ack'
 */
static void
mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel,
		const mavlink_message_t* msg, void * user)
{
	//printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

	switch(msg->msgid)
	{
	uint64_t receiveTime;
	uint64_t sendTime;

	case MAVLINK_MSG_ID_LOCAL_POSITION:
		mavlink_msg_local_position_decode(msg, &currentpos);
		//printf("POS: x: %f, y: %f, z: %f\n", currentpos.x, currentpos.y, currentpos.z);
		break;
	case MAVLINK_MSG_ID_ATTITUDE:
		gettimeofday(&tv, NULL);
		receiveTime = (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
		sendTime = mavlink_msg_attitude_get_usec(msg);
		//printf("Received attitude message, transport took %d us\n", (receiveTime - sendTime));
		mavlink_msg_attitude_decode(msg, &currentatt);
		//printf("ATT: yaw: %f\n", currentatt.yaw);
		break;

	case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET:
		mavlink_local_position_setpoint_set_t sp;
		mavlink_msg_local_position_setpoint_set_decode(msg, &sp);
		cur_dest.x = sp.x;
		cur_dest.y = sp.y;
		cur_dest.z = sp.z;
		cur_dest.yaw = sp.yaw;
		break;
	default:
		printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
}

/** @brief thread waiting for incoming MAVLINKs
 *
 */
void* lcm_wait(void* lcm_ptr)
		{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1)
	{
		lcm_handle (lcm);
	}
	return NULL;
		}

/** @brief send the calculated position to MAVLINK
 *
 */
void sendPosition(mavlink_local_position_t* pos, mavlink_attitude_t* att)
{
	printf("Send Position: x: %f, y: %f, z: %f\n",pos->x,pos->y,pos->z);

	mavlink_message_t msg;

	//refresh time
	gettimeofday(&tv, NULL);
	pos->usec = (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;

	mavlink_msg_local_position_encode(systemid,compid,&msg,pos);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	att->usec= (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;

	mavlink_msg_attitude_encode(systemid, compid, &msg, att);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

}

/** @brief calculate the next position out of the current position
 *
 */
void calcPosition(mavlink_local_position_t* nextpos, mavlink_attitude_t* nextatt)
{
	//P-controller gains
	float K_p = 0.08;
	float K_t = 0.08;

	// velocity*dt
	float d_x=0;
	float d_y=0;
	float d_z=0;
	float d_theta = 0;

	d_x = K_p*(cur_dest.x - currentpos.x);
	d_y = K_p*(cur_dest.y - currentpos.y);
	d_z = K_p*(cur_dest.z - currentpos.z);
	d_theta = K_t*(cur_dest.yaw - currentatt.yaw);

	nextpos->x = currentpos.x + d_x;
	nextpos->y = currentpos.y + d_y;
	nextpos->z = currentpos.z + d_z;
	nextatt->yaw = currentatt.yaw + d_theta;


}

/** @brief initialize and run simulation
 *
 */
int main (int argc, char ** argv)
{

	//initialize variables
	currentpos.x=0; currentpos.y=0; currentpos.z=-0.5;
	currentpos.vx=0; currentpos.vy=0; currentpos.vz=0;
	currentatt.pitch=0; currentatt.roll=0; currentatt.yaw=0;
	currentatt.pitchspeed=0; currentatt.rollspeed=0; currentatt.yawspeed=0;

	cur_dest.frame = 1; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	cur_dest.x = 0; //local: x position, global: longitude
	cur_dest.y = 0; //local: y position, global: latitude
	cur_dest.z = 0; //local: z position, global: altitude
	cur_dest.yaw = 0; //Yaw orientation in degrees, [0..360] 0 = NORTH

	mavlink_local_position_t nextpos = currentpos;
	mavlink_attitude_t nextatt = currentatt;



	lcm = lcm_create ("udpm://");
	if (!lcm)
		return 1;

	mavlink_message_t_subscription_t * comm_sub =
	mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, NULL);

	// Thread
	GThread* lcm_thread;
	GError* err;

	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if( (lcm_thread = g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err)) == NULL)
	{
		printf("Thread create failed: %s!!\n", err->message );
		g_error_free ( err ) ;
	}

	printf("PX SWEEP SIMULATOR STARTED\n");

	requestWaypointlist();

	while (1)
	{
		//lcm_handle (lcm);
		/*
		uint16_t i=0;
	    while(rwpl)
		{
			i=i+1;
			if(i>100) {
				printf("ERROR: receiving waypointlist timeout\n");
				i=0;
				//exit(1);
			}
			//sendPosition(&currentpos,&currentatt);
			usleep(20000);
		}
		*/
	    calcPosition(&nextpos,&nextatt);

	    sendPosition(&nextpos,&nextatt);

		usleep(100000);
	}

	mavlink_message_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	g_thread_join(lcm_thread);
	delete [] wpl;
	return 0;
}

