/*======================================================================
========================================================================*/

/**
*   @file
*   @brief a program to manage waypoints and exchange them with the ground station
*
*   @author Benjamin Knecht <bknecht@student.ethz.ch>
*   @author Christian Schluchter <schluchc@ee.ethz.ch>
*   @author Petri Tanskanen <mavteam@student.ethz.ch>
*   @author Alex Trofimov <mavteam@student.ethz.ch>
*/

#include <boost/program_options.hpp>
#include <iostream>
#include <vector>

#include "PxVector3.h"

#include "mavconn.h"
#include <glib.h>

namespace config = boost::program_options;

lcm_t* lcm;
mavlink_message_t_subscription_t* comm_sub;


bool debug;             	///< boolean for debug output or behavior
bool verbose;           	///< boolean for verbose output
std::string configFile;		///< Configuration file for parameters

//=== struct for storing the current destination ===
typedef struct _mav_destination
{
	uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	float x; //local: x position, global: longitude
	float y; //local: y position, global: latitude
	float z; //local: z position, global: altitude
	float yaw; //Yaw orientation in degrees, [0..360] 0 = NORTH
	float rad; //Radius in which the destination counts as reached
} mav_destination;


//==== variables for the planner ====
bool idle = false;      				///< indicates if the system is following the waypoints or is waiting
uint16_t current_active_wp_id = -1;		///< id of current waypoint
uint16_t next_wp_id = -1;
bool ready_to_continue = false;
uint64_t timestamp_lastoutside_orbit = 0;///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
uint64_t timestamp_firstinside_orbit = 0;///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value
uint64_t timestamp_delay_started = 0;	 ///< timestamp when the current delay command was initiated

bool search_success = false; ///< variable for testing search waypoint commands
mavlink_local_position_t search_success_pos; ///< position of MAV when it succeeded in search
mavlink_attitude_t search_success_att;		 ///< attitude of MAV when it succeeded in search
uint16_t npic = 0;						     ///< number of times the picture has been detected
//static GString* SEARCH_PIC = g_string_new("media/sweep_images/mona.jpg");	///< relative path of the search image

mav_destination cur_dest;				 ///< current flight destination
mavlink_local_position_t last_known_pos; ///< latest received position of MAV
mavlink_attitude_t last_known_att;		 ///< latest received attitude of MAV

std::vector<mavlink_waypoint_t*> waypoints1;	///< vector1 that holds the waypoints
std::vector<mavlink_waypoint_t*> waypoints2;	///< vector2 that holds the waypoints

std::vector<mavlink_waypoint_t*>* waypoints = &waypoints1;					///< pointer to the currently active waypoint vector
std::vector<mavlink_waypoint_t*>* waypoints_receive_buffer = &waypoints2;	///< pointer to the receive buffer waypoint vector

GError *error = NULL;
GThread* waypoint_lcm_thread = NULL;
static GStaticMutex *main_mutex = new GStaticMutex;


//==== variables needed for communication protocol ====
uint8_t systemid = getSystemID();          		///< indicates the ID of the system
uint8_t compid = MAV_COMP_ID_WAYPOINTPLANNER;	///< indicates the component ID of the waypointplanner

PxParamClient* paramClient;

enum PX_WAYPOINTPLANNER_STATES
{
    PX_WPP_IDLE = 0,
    PX_WPP_SENDLIST,
    PX_WPP_SENDLIST_SENDWPS,
    PX_WPP_GETLIST,
    PX_WPP_GETLIST_GETWPS,
    PX_WPP_GETLIST_GOTALL
};


enum MAV_CMD_ID
{
	//MAV_CMD_NAV_WAYPOINT = 16,     //Navigate to waypoint
	//MAV_CMD_CONDITION_DELAY = 112,	//Delay mission state machine.
	//MAV_CMD_DO_JUMP	= 177, //Jump to the desired command in the mission list. Repeat this action only the specified number of times
	MAV_CMD_DO_START_SEARCH = 237,
	MAV_CMD_DO_FINISH_SEARCH = 238,
	MAV_CMD_DO_SEND_MESSAGE = 239
};


PX_WAYPOINTPLANNER_STATES current_state = PX_WPP_IDLE;
uint16_t protocol_current_wp_id = 0;
uint16_t protocol_current_count = 0;
uint8_t protocol_current_partner_systemid = 0;
uint8_t protocol_current_partner_compid = 0;

uint64_t protocol_timestamp_lastaction = 0;
uint64_t timestamp_last_send_setpoint = 0;
uint64_t timestamp_last_handle_waypoint = 0;

void send_waypoint_ack(uint8_t target_systemid, uint8_t target_compid, uint8_t type)
/*
*  @brief Sends an waypoint ack message
*/
{
    mavlink_message_t msg;
    mavlink_waypoint_ack_t wpa;

    wpa.target_system = target_systemid;
    wpa.target_component = target_compid;
    wpa.type = type;

    mavlink_msg_waypoint_ack_encode(systemid, compid, &msg, &wpa);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);

    usleep(paramClient->getParamValue("PROTDELAY"));

    if (verbose) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
}

void send_waypoint_current(uint16_t seq)
/*
*  @brief Broadcasts the new target waypoint and directs the MAV to fly there
*
*  This function broadcasts its new active waypoint sequence number
*
*  @param seq The sequence number of currently active waypoint.
*/
{
    if(seq < waypoints->size())
    {
        mavlink_waypoint_t *cur = waypoints->at(seq);

        mavlink_message_t msg;
        mavlink_waypoint_current_t wpc;

        wpc.seq = cur->seq;

        mavlink_msg_waypoint_current_encode(systemid, compid, &msg, &wpc);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);

        usleep(paramClient->getParamValue("PROTDELAY"));

        if (verbose) printf("Broadcasted new current waypoint %u\n", wpc.seq);
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds 1\n");
    }
}

void send_setpoint(void)
/*
*  @brief Directs the MAV to fly to a position
*
*  Sends a message to the controller, advising it to fly to the coordinates
*  of the waypoint with a given orientation
*
*  @param seq The waypoint sequence number the MAV should fly to.
*/
{
        mavlink_message_t msg;
        mavlink_local_position_setpoint_set_t PControlSetPoint;

        // send new set point to local IMU
        if (cur_dest.frame == 1)
        {
            PControlSetPoint.target_system = systemid;
            PControlSetPoint.target_component = MAV_COMP_ID_IMU;
            PControlSetPoint.x = cur_dest.x;
            PControlSetPoint.y = cur_dest.y;
            PControlSetPoint.z = cur_dest.z;
            PControlSetPoint.yaw = cur_dest.yaw;

            mavlink_msg_local_position_setpoint_set_encode(systemid, compid, &msg, &PControlSetPoint);
            mavlink_message_t_publish(lcm, "MAVLINK", &msg);

            if (verbose) printf("Send setpoint: x: %.2f | y: %.2f | z: %.2f | yaw: %.3f\n", cur_dest.x, cur_dest.y, cur_dest.z, cur_dest.yaw);
            usleep(paramClient->getParamValue("PROTDELAY"));
        }
        else
        {
            if (verbose) printf("No new set point sent to IMU because the new waypoint had no local coordinates\n");
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
        timestamp_last_send_setpoint = now;
}

void send_waypoint_count(uint8_t target_systemid, uint8_t target_compid, uint16_t count)
{
    mavlink_message_t msg;
    mavlink_waypoint_count_t wpc;

    wpc.target_system = target_systemid;
    wpc.target_component = target_compid;
    wpc.count = count;

    mavlink_msg_waypoint_count_encode(systemid, compid, &msg, &wpc);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);

    if (verbose) printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);

    usleep(paramClient->getParamValue("PROTDELAY"));
}

void send_waypoint(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
    if (seq < waypoints->size())
    {
        mavlink_message_t msg;
        mavlink_waypoint_t *wp = waypoints->at(seq);
        wp->target_system = target_systemid;
        wp->target_component = target_compid;
        mavlink_msg_waypoint_encode(systemid, compid, &msg, wp);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);
        if (verbose) printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);

        usleep(paramClient->getParamValue("PROTDELAY"));
    }
    else
    {
        if (verbose) printf("ERROR: index out of bounds 2\n");
    }
}

void send_waypoint_request(uint8_t target_systemid, uint8_t target_compid, uint16_t seq)
{
    if (seq < protocol_current_count) //changed this from if(seq < waypoints->size())
    {
        mavlink_message_t msg;
        mavlink_waypoint_request_t wpr;
        wpr.target_system = target_systemid;
        wpr.target_component = target_compid;
        wpr.seq = seq;
        mavlink_msg_waypoint_request_encode(systemid, compid, &msg, &wpr);
        mavlink_message_t_publish(lcm, "MAVLINK", &msg);
        if (verbose) printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);

        usleep(paramClient->getParamValue("PROTDELAY"));
    }

    else
    {
        if (verbose) printf("ERROR: index out of bounds. seq = %u ,size = %u\n",seq,(uint16_t) waypoints->size());
    }
}

void send_waypoint_reached(uint16_t seq)
/*
*  @brief emits a message that a waypoint reached
*
*  This function broadcasts a message that a waypoint is reached.
*
*  @param seq The waypoint sequence number the MAV has reached.
*/
{
    mavlink_message_t msg;
    mavlink_waypoint_reached_t wp_reached;

    wp_reached.seq = seq;

    mavlink_msg_waypoint_reached_encode(systemid, compid, &msg, &wp_reached);
    mavlink_message_t_publish(lcm, "MAVLINK", &msg);

    if (verbose) printf("Sent waypoint %u reached message\n", wp_reached.seq);

    usleep(paramClient->getParamValue("PROTDELAY"));
}

void set_destination(mavlink_waypoint_t* wp)
{
	cur_dest.frame = wp->frame;
	cur_dest.x = wp->x;
	cur_dest.y = wp->y;
	cur_dest.z = wp->z;
	cur_dest.yaw = wp->param4;
	cur_dest.rad = wp->param2;

	if(wp->command != MAV_CMD_NAV_WAYPOINT)
	{
		if (verbose) printf("Warning: New destination coordinates do not origin from MAV_CMD_NAV_WAYPOINT waypoint.\n");
	}
}

float distanceToSegment(float x, float y, float z , uint16_t next_NAV_wp_id)
{
    	const PxVector3 A(cur_dest.x, cur_dest.y, cur_dest.z);
        const PxVector3 C(x, y, z);

        // next_NAV_wp_id not the second last waypoint
        if ((uint16_t)(next_NAV_wp_id) < waypoints->size())
        {
            mavlink_waypoint_t *next = waypoints->at(next_NAV_wp_id);
            const PxVector3 B(next->x, next->y, next->z);
            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
            if (r >= 0 && r <= 1)
            {
                const PxVector3 P(A + r*(B-A));
                return (P-C).length();
            }
            else if (r < 0.f || next->command != MAV_CMD_NAV_WAYPOINT)
            {
                return (C-A).length();
            }
            else
            {
                return (C-B).length();
            }
        }
        else
        {

            return (C-A).length();

        }
}

float distanceToPoint(float x, float y, float z)
{
	const PxVector3 A(cur_dest.x, cur_dest.y, cur_dest.z);
    const PxVector3 C(x, y, z);

    return (C-A).length();
}

void* search_thread_func (gpointer lcm_ptr)
{

	while (1)
	{
		if (npic>=1)
		{
			search_success = true;
			search_success_pos = last_known_pos;
			search_success_att = last_known_att;
			break;
		}
		usleep(1000);
	}
	return NULL;
}



/*
void* search_thread_func (gpointer lcm_ptr)
{
	while (1)
	{
		if (cur_dest.x == 0)
		{
			search_success = true;
			search_success_pos = last_known_pos;
			search_success_att = last_known_att;
			printf("Found!\n");
			break;
		}
		usleep(paramClient->getParamValue("PROTDELAY")); //Reasonable length of this delay is yet to be found.
	}
	return NULL;
}
*/
void handle_waypoint (uint16_t seq, uint64_t now)
{
	//if (debug) printf("Started executing waypoint(%u)...\n",seq);

	if (seq < waypoints->size())
	{
		mavlink_waypoint_t *cur_wp = waypoints->at(seq);
		if (ready_to_continue == false){
	    switch(cur_wp->command)
	    {
	    case MAV_CMD_NAV_WAYPOINT:
	    {
	    	set_destination(cur_wp);

	    	bool yawReached = false;						///< boolean for yaw attitude reached
	    	bool posReached = false;						///< boolean for position reached

            // compare last known position with current destination
            float dist;
            next_wp_id = seq+1;
            if (waypoints->at(seq)->param1 == 0 && next_wp_id < waypoints->size() && waypoints->at(next_wp_id)->command == MAV_CMD_NAV_WAYPOINT)
            {
            	//if (debug) printf("Both current and next waypoint (%u) are MAV_CMD_NAV_WAYPOINT. Using distanceToSegment.\n", next_wp_id);
                dist = distanceToSegment(last_known_pos.x, last_known_pos.y, last_known_pos.z, next_wp_id);
            }
            else
            {
                dist = distanceToPoint(last_known_pos.x, last_known_pos.y, last_known_pos.z);
            }

            if (dist >= 0.f && dist <= cur_dest.rad)
            {
                posReached = true;
            }

	    	// yaw reached?
            float yaw_tolerance = paramClient->getParamValue("YAWTOLERANCE");
            //compare last known yaw with current desired yaw
            if (last_known_att.yaw - yaw_tolerance >= 0.0f && last_known_att.yaw + yaw_tolerance < 2.f*M_PI)
            {
                if (last_known_att.yaw - yaw_tolerance <= cur_dest.yaw*M_PI/180 && last_known_att.yaw + yaw_tolerance >= cur_dest.yaw*M_PI/180)
                    yawReached = true;
            }
            else if(last_known_att.yaw - yaw_tolerance < 0.0f)
            {
                float lowerBound = 2.f*M_PI + last_known_att.yaw - yaw_tolerance;
                if (lowerBound < cur_dest.yaw*M_PI/180 || cur_dest.yaw*M_PI/180 < last_known_att.yaw + yaw_tolerance)
                    yawReached = true;
            }
            else
            {
                float upperBound = last_known_att.yaw + yaw_tolerance - 2.f*M_PI;
                if (last_known_att.yaw - yaw_tolerance < cur_dest.yaw*M_PI/180 || cur_dest.yaw*M_PI/180 < upperBound)
                    yawReached = true;
            }
	    		//check if the current waypoint was reached
            if ((posReached && yawReached && !idle))
            {
            	if (timestamp_firstinside_orbit == 0)
            	{
            	     // Announce that last waypoint was reached
            		 if (verbose) printf("*** Reached waypoint %u ***\n", cur_wp->seq);
    	             send_waypoint_reached(cur_wp->seq);
         	         timestamp_firstinside_orbit = now;
         	    }
            	// check if the MAV was long enough inside the waypoint orbit
	            if(now-timestamp_firstinside_orbit >= cur_wp->param1*1000000)
	            {
	            	ready_to_continue = true;
	            	timestamp_firstinside_orbit = 0;
	            }
    	    }
    	    else
    	    {
    	        timestamp_lastoutside_orbit = now;
    	    }
    	    break;
	    }
	    case MAV_CMD_NAV_LOITER_UNLIM:
	    	break;
	    case MAV_CMD_NAV_LOITER_TURNS:
	    	break;
	    case MAV_CMD_NAV_LOITER_TIME:
	    	break;
	    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
	    	break;
	    case MAV_CMD_NAV_LAND:
	    	break;
	    case MAV_CMD_NAV_TAKEOFF:
	    	break;
	    case MAV_CMD_NAV_LAST:
	    	break;
	    case MAV_CMD_CONDITION_DELAY:
	    {
	    	if (timestamp_delay_started == 0)
	    	{
	    		timestamp_delay_started = now;
	    		if (verbose) printf("Delay initiated (%.2f sec)...\n", cur_wp->param1);
	    		if (verbose && paramClient->getParamValue("HANDLEWPDELAY")>cur_wp->param1)
	    			{
	    				printf("Warning: Delay shorter than HANDLEWPDELAY parameter (%.2f sec)!\n", paramClient->getParamValue("HANDLEWPDELAY"));
	    			}
	    	}
	    	if (now - timestamp_delay_started >= cur_wp->param1*1000000)
	    	{
	    		timestamp_delay_started = 0;
	    		next_wp_id = seq + 1;
				ready_to_continue = true;
				if (verbose) printf("... delay finished. Proceed to next waypoint.\n");
	    	}
	    	else
	    	{
	    		if (verbose) printf("... %.2f sec left...\n", cur_wp->param1-(float)(now - timestamp_delay_started)/1000000.0);
	    	}
	    	break;
	    }
	    case MAV_CMD_CONDITION_CHANGE_ALT:
	    	break;
	    case MAV_CMD_CONDITION_DISTANCE:
	    	break;
	    case MAV_CMD_CONDITION_LAST:
	    	break;
	    case MAV_CMD_DO_SET_MODE:
	    	break;
	    case MAV_CMD_DO_JUMP:
	    {
			if (cur_wp->param2 > 0)
			{
				cur_wp->param2 = cur_wp->param2 - 1;
				if (verbose) printf("Jump from waypoint %u to waypoint %u. %u jumps left\n", current_active_wp_id, (uint32_t) cur_wp->param1, (uint32_t) cur_wp->param2);
				next_wp_id = cur_wp->param1;
			}
			else
			{
				next_wp_id = seq + 1;
                if (verbose) printf("Jump command not performed: jump limit reached. Proceed to next waypoint\n");
			}
			ready_to_continue = true;
			break;
	    }
	    case MAV_CMD_DO_CHANGE_SPEED:
	    	break;
	    case MAV_CMD_DO_SET_HOME:
	    	break;
	    case MAV_CMD_DO_SET_PARAMETER:
	    	break;
	    case MAV_CMD_DO_SET_RELAY:
	    	break;
	    case MAV_CMD_DO_REPEAT_RELAY:
	    	break;
	    case MAV_CMD_DO_SET_SERVO:
	    	break;
	    case MAV_CMD_DO_REPEAT_SERVO:
	    {
	    	break;
	    }
	    case MAV_CMD_DO_START_SEARCH:
	    {
	    	if( !g_thread_supported() )
	    	{
	    		g_thread_init(NULL); // Only initialize g thread if not already done
	    	}
	    	gpointer ptr = NULL;
	    	GThread* search_thread = NULL;
	    	if( (search_thread = g_thread_create(search_thread_func, ptr, TRUE, &error)) == NULL)
	    	{
	    		printf("Thread creation failed: %s!!\n", error->message );
	    		g_error_free ( error ) ;
	    	}
	    	if (verbose) printf("Search thread created!\n");
	    	next_wp_id = seq + 1;
	    	ready_to_continue = true;
	    	break;
	    }
	    case MAV_CMD_DO_FINISH_SEARCH:
	    {
			if (cur_wp->param3 > 0)
			{
				cur_wp->param3 = cur_wp->param3 - 1;

		    	if (cur_wp->param1 < waypoints->size() && cur_wp->param2 < waypoints->size())
		    	{
			    	if (search_success == true){
			    		next_wp_id = cur_wp->param1;
			    		if (verbose) printf("The search was successful! Proceeding to waypoint %u\n",next_wp_id);
			    	}
			    	else
			    	{
			    		next_wp_id = cur_wp->param2;
			    		if (verbose) printf("The search was not successful. Proceeding to waypoint %u\n",next_wp_id);
			    	}
		    	}
		    	else
		    	{
		    		next_wp_id = seq + 1;
		    		cur_wp->autocontinue = false;
		    		printf("Invalid parameters for MAV_CMD_DO_FINISH_SEARCH waypoint. Next waypoint is set to %u. Autocontinue turned off. \n",next_wp_id);
		    	}
			}
			else
			{
	    		next_wp_id = seq + 1;
			}
			ready_to_continue = true;
	    	break;
	    }
	    case MAV_CMD_DO_SEND_MESSAGE:
	    {
	    	mavlink_message_t msg;
	    	mavlink_statustext_t stext;

	    	uint16_t some_nr = 13;

	    	stext.severity = 0;
	    	sprintf((char*)&stext.text,"Some very important message: %u",some_nr);
	    	mavlink_msg_statustext_encode(systemid, compid, &msg, &stext);
	    	mavlink_message_t_publish(lcm, "MAVLINK", &msg);
	    	next_wp_id = seq + 1;
	    	ready_to_continue = true;
	    	break;
	    }

	    }} //end switch, end if

		if (ready_to_continue == true)
		{
			if (cur_wp->autocontinue)
		    {
				//if (verbose) printf("check current (%u) next (%u) size (%u) \n", current_active_wp_id, next_wp_id, waypoints->size());
		        if (next_wp_id < waypoints->size())
		        {
			        cur_wp->current = false;
			        ready_to_continue = false;
		        	current_active_wp_id = next_wp_id;
		        	next_wp_id = -1;
		           	// Proceed to next waypoint
		            send_waypoint_current(current_active_wp_id);
		            waypoints->at(current_active_wp_id)->current = true;
		            if (verbose) printf("Set new waypoint (%u)\n", current_active_wp_id);
		            //Waypoint changed, execute next waypoint at once. Warning: recursion!!!
		            handle_waypoint(current_active_wp_id,now);
		        }
		        else //the end of waypoint list is reached.
		        {
		        	if (verbose) printf("Reached the end of the list.\n");
		        }
		    }
			else
			{
				if (verbose) printf("New waypoint (%u) has been set. Waiting for autocontinue...\n", next_wp_id);
			}
		}

	}
	else
	{
		if (verbose) printf("Waypoint %u is out of bounds. Currently have %u waypoints.\n", seq, (uint16_t)waypoints->size());
	}

	//if (debug) printf("Finished executing waypoint(%u)...\n",seq);
	timestamp_last_handle_waypoint = now;

}

static void handle_communication (const mavlink_message_t* msg, uint64_t now)
{
	switch(msg->msgid)
	{
		case MAVLINK_MSG_ID_WAYPOINT_ACK:
	        {
	            mavlink_waypoint_ack_t wpa;
	            mavlink_msg_waypoint_ack_decode(msg, &wpa);

	            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wpa.target_system == systemid && wpa.target_component == compid))
	            {
	                protocol_timestamp_lastaction = now;

	                if (current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)
	                {
	                    if (protocol_current_wp_id == waypoints->size()-1)
	                    {
	                        if (verbose) printf("Received Ack after having sent last waypoint, going to state PX_WPP_IDLE\n");
	                        current_state = PX_WPP_IDLE;
	                        protocol_current_wp_id = 0;
	                    }
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
	        {
	            mavlink_waypoint_set_current_t wpc;
	            mavlink_msg_waypoint_set_current_decode(msg, &wpc);

	            if(wpc.target_system == systemid && wpc.target_component == compid)
	            {
	                protocol_timestamp_lastaction = now;

	                if (current_state == PX_WPP_IDLE)
	                {
	                    if (wpc.seq < waypoints->size())
	                    {
	                        if (verbose) printf("Received MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT\n");
	                        current_active_wp_id = wpc.seq;
	                        uint32_t i;
	                        for(i = 0; i < waypoints->size(); i++)
	                        {
	                            if (i == current_active_wp_id)
	                            {
	                                waypoints->at(i)->current = true;
	                            }
	                            else
	                            {
	                                waypoints->at(i)->current = false;
	                            }
	                        }
	                        if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
	                        send_waypoint_current(current_active_wp_id);
	                        ready_to_continue = false;
	                        handle_waypoint(current_active_wp_id,now);
	                        send_setpoint();
	                        timestamp_firstinside_orbit = 0;
	                    }
	                    else
	                    {
	                        if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT: Index out of bounds\n");
	                    }
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
	        {
	            mavlink_waypoint_request_list_t wprl;
	            mavlink_msg_waypoint_request_list_decode(msg, &wprl);
	            if(wprl.target_system == systemid && wprl.target_component == compid)
	            {
	                protocol_timestamp_lastaction = now;

	                if (current_state == PX_WPP_IDLE || current_state == PX_WPP_SENDLIST)
	                {
	                    if (waypoints->size() > 0)
	                    {
	                        if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u changing state to PX_WPP_SENDLIST\n", msg->sysid);
	                        if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST again from %u staying in state PX_WPP_SENDLIST\n", msg->sysid);
	                        current_state = PX_WPP_SENDLIST;
	                        protocol_current_wp_id = 0;
	                        protocol_current_partner_systemid = msg->sysid;
	                        protocol_current_partner_compid = msg->compid;
	                    }
	                    else
	                    {
	                        if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
	                    }
	                    protocol_current_count = waypoints->size();
	                    send_waypoint_count(msg->sysid,msg->compid, protocol_current_count);
	                }
	                else
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST because i'm doing something else already (state=%i).\n", current_state);
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
	        {
	            mavlink_waypoint_request_t wpr;
	            mavlink_msg_waypoint_request_decode(msg, &wpr);
	            if(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid && wpr.target_system == systemid && wpr.target_component == compid)
	            {
	                protocol_timestamp_lastaction = now;

	                //ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
	                if ((current_state == PX_WPP_SENDLIST && wpr.seq == 0) || (current_state == PX_WPP_SENDLIST_SENDWPS && (wpr.seq == protocol_current_wp_id || wpr.seq == protocol_current_wp_id + 1) && wpr.seq < waypoints->size()))
	                {
	                    if (verbose && current_state == PX_WPP_SENDLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u changing state to PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
	                    if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id + 1) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
	                    if (verbose && current_state == PX_WPP_SENDLIST_SENDWPS && wpr.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT_REQUEST of waypoint %u (again) from %u staying in state PX_WPP_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

	                    current_state = PX_WPP_SENDLIST_SENDWPS;
	                    protocol_current_wp_id = wpr.seq;
	                    send_waypoint(protocol_current_partner_systemid, protocol_current_partner_compid, wpr.seq);
	                }
	                else
	                {
	                    if (verbose)
	                    {
	                        if (!(current_state == PX_WPP_SENDLIST || current_state == PX_WPP_SENDLIST_SENDWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because i'm doing something else already (state=%i).\n", current_state); break; }
	                        else if (current_state == PX_WPP_SENDLIST)
	                        {
	                            if (wpr.seq != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
	                        }
	                        else if (current_state == PX_WPP_SENDLIST_SENDWPS)
	                        {
	                            if (wpr.seq != protocol_current_wp_id && wpr.seq != protocol_current_wp_id + 1) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, protocol_current_wp_id, protocol_current_wp_id+1);
	                            else if (wpr.seq >= waypoints->size()) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
	                        }
	                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST - FIXME: missed error description\n");
	                    }
	                }
	            }
	            else
	            {
	                //we we're target but already communicating with someone else
	                if((wpr.target_system == systemid && wpr.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid))
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, protocol_current_partner_systemid);
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_WAYPOINT_COUNT:
	        {
	            mavlink_waypoint_count_t wpc;
	            mavlink_msg_waypoint_count_decode(msg, &wpc);
	            if(wpc.target_system == systemid && wpc.target_component == compid)
	            {
	                protocol_timestamp_lastaction = now;

	                if (current_state == PX_WPP_IDLE || (current_state == PX_WPP_GETLIST && protocol_current_wp_id == 0))
	                {
	                    if (wpc.count > 0)
	                    {
	                        if (verbose && current_state == PX_WPP_IDLE) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) from %u changing state to PX_WPP_GETLIST\n", wpc.count, msg->sysid);
	                        if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT_COUNT (%u) again from %u\n", wpc.count, msg->sysid);

	                        current_state = PX_WPP_GETLIST;
	                        protocol_current_wp_id = 0;
	                        protocol_current_partner_systemid = msg->sysid;
	                        protocol_current_partner_compid = msg->compid;
	                        protocol_current_count = wpc.count;

	                        printf("clearing receive buffer and readying for receiving waypoints\n");
	                        while(waypoints_receive_buffer->size() > 0)
	                        {
	                            delete waypoints_receive_buffer->back();
	                            waypoints_receive_buffer->pop_back();
	                        }

	                        send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
	                    }
	                    else if (wpc.count == 0)
	                    {
	                        printf("got waypoint count of 0, clearing waypoint list and staying in state PX_WPP_IDLE\n");
	                        while(waypoints_receive_buffer->size() > 0)
	                        {
	                            delete waypoints->back();
	                            waypoints->pop_back();
	                        }
	                        current_active_wp_id = -1;
	                        break;

	                    }
	                    else
	                    {
	                        if (verbose) printf("Ignoring MAVLINK_MSG_ID_WAYPOINT_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
	                    }
	                }
	                else
	                {
	                    if (verbose && !(current_state == PX_WPP_IDLE || current_state == PX_WPP_GETLIST)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm doing something else already (state=%i).\n", current_state);
	                    else if (verbose && current_state == PX_WPP_GETLIST && protocol_current_wp_id != 0) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT because i'm already receiving waypoint %u.\n", protocol_current_wp_id);
	                    else printf("Ignored MAVLINK_MSG_ID_WAYPOINT_COUNT - FIXME: missed error description\n");
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_WAYPOINT:
	        {
	            mavlink_waypoint_t wp;
	            mavlink_msg_waypoint_decode(msg, &wp);

	            if((msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && (wp.target_system == systemid && wp.target_component == compid))
	            {
	                protocol_timestamp_lastaction = now;
	                printf("Received WP %3u%s: Frame: %u\tCommand: %3u\tparam1: %6.2f\tparam2: %7.2f\tparam3: %6.2f\tparam4: %7.2f\tX: %7.2f\tY: %7.2f\tZ: %7.2f\tAuto-Cont: %u\t\n", wp.seq, (wp.current?"*":" "), wp.frame, wp.command, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, wp.autocontinue);

	                //ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
	                if ((current_state == PX_WPP_GETLIST && wp.seq == 0) || (current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id && wp.seq < protocol_current_count))
	                {
	                    if (verbose && current_state == PX_WPP_GETLIST) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u changing state to PX_WPP_GETLIST_GETWPS\n", wp.seq, msg->sysid);
	                    if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u from %u\n", wp.seq, msg->sysid);
	                    if (verbose && current_state == PX_WPP_GETLIST_GETWPS && wp.seq-1 == protocol_current_wp_id) printf("Got MAVLINK_MSG_ID_WAYPOINT %u (again) from %u\n", wp.seq, msg->sysid);

	                    current_state = PX_WPP_GETLIST_GETWPS;
	                    protocol_current_wp_id = wp.seq + 1;
	                    mavlink_waypoint_t* newwp = new mavlink_waypoint_t;
	                    memcpy(newwp, &wp, sizeof(mavlink_waypoint_t));
	                    waypoints_receive_buffer->push_back(newwp);

	                    if(protocol_current_wp_id == protocol_current_count && current_state == PX_WPP_GETLIST_GETWPS)
	                    {
	                        if (verbose) printf("Got all %u waypoints, changing state to PX_WPP_IDLE\n", protocol_current_count);

	                        send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);

	                        if (current_active_wp_id > waypoints_receive_buffer->size()-1)
	                        {
	                            current_active_wp_id = waypoints_receive_buffer->size() - 1;
	                        }

	                        // switch the waypoints list
	                        std::vector<mavlink_waypoint_t*>* waypoints_temp = waypoints;
	                        waypoints = waypoints_receive_buffer;
	                        waypoints_receive_buffer = waypoints_temp;

	                        //get the new current waypoint
	                        uint32_t i;
	                        for(i = 0; i < waypoints->size(); i++)
	                        {
	                            if (waypoints->at(i)->current == 1)
	                            {
	                                current_active_wp_id = i;
	                                //if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
	                                send_waypoint_current(current_active_wp_id);
	                                ready_to_continue = false;
	                                handle_waypoint(current_active_wp_id,now);
	                                send_setpoint();
	                                timestamp_firstinside_orbit = 0;
	                                break;
	                            }
	                        }

	                        if (i == waypoints->size())
	                        {
	                            current_active_wp_id = -1;
	                            timestamp_firstinside_orbit = 0;
	                        }

	                        current_state = PX_WPP_IDLE;
	                    }
	                    else
	                    {
	                        send_waypoint_request(protocol_current_partner_systemid, protocol_current_partner_compid, protocol_current_wp_id);
	                    }
	                }
	                else
	                {
	                    if (current_state == PX_WPP_IDLE)
	                    {
	                        //we're done receiving waypoints, answer with ack.
	                        send_waypoint_ack(protocol_current_partner_systemid, protocol_current_partner_compid, 0);
	                        printf("Received MAVLINK_MSG_ID_WAYPOINT while state=PX_WPP_IDLE, answered with WAYPOINT_ACK.\n");
	                    }
	                    if (verbose)
	                    {
	                        if (!(current_state == PX_WPP_GETLIST || current_state == PX_WPP_GETLIST_GETWPS)) { printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u because i'm doing something else already (state=%i).\n", wp.seq, current_state); break; }
	                        else if (current_state == PX_WPP_GETLIST)
	                        {
	                            if(!(wp.seq == 0)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the first waypoint ID (%u) was not 0.\n", wp.seq);
	                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
	                        }
	                        else if (current_state == PX_WPP_GETLIST_GETWPS)
	                        {
	                            if (!(wp.seq == protocol_current_wp_id)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was not the expected %u.\n", wp.seq, protocol_current_wp_id);
	                            else if (!(wp.seq < protocol_current_count)) printf("Ignored MAVLINK_MSG_ID_WAYPOINT because the waypoint ID (%u) was out of bounds.\n", wp.seq);
	                            else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
	                        }
	                        else printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u - FIXME: missed error description\n", wp.seq);
	                    }
	                }
	            }
	            else
	            {
	                //we we're target but already communicating with someone else
	                if((wp.target_system == systemid && wp.target_component == compid) && !(msg->sysid == protocol_current_partner_systemid && msg->compid == protocol_current_partner_compid) && current_state != PX_WPP_IDLE)
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, protocol_current_partner_systemid);
	                }
	                else if(wp.target_system == systemid && wp.target_component == compid)
	                {
	                    if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
	                }
	            }
	            break;
	        }

		case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
	        {
	            mavlink_waypoint_clear_all_t wpca;
	            mavlink_msg_waypoint_clear_all_decode(msg, &wpca);

	            if(wpca.target_system == systemid && wpca.target_component == compid && current_state == PX_WPP_IDLE)
	            {
	                protocol_timestamp_lastaction = now;

	                if (verbose) printf("Got MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
	                while(waypoints->size() > 0)
	                {
	                    delete waypoints->back();
	                    waypoints->pop_back();
	                }
	                current_active_wp_id = -1;
	            }
	            else if (wpca.target_system == systemid && wpca.target_component == compid && current_state != PX_WPP_IDLE)
	            {
	                if (verbose) printf("Ignored MAVLINK_MSG_ID_WAYPOINT_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, current_state);
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_ATTITUDE:
	        {
	            if(msg->sysid == systemid)
	            {
	                if(cur_dest.frame == 1)
	                {
	                    mavlink_msg_attitude_decode(msg, &last_known_att);
	                    struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
                        if(now-timestamp_last_handle_waypoint > paramClient->getParamValue("HANDLEWPDELAY")*1000000 && current_active_wp_id != (uint16_t)-1)
                        {
                        	handle_waypoint(current_active_wp_id,now);
                        }
	                }
	            }
	            break;
	        }

	    case MAVLINK_MSG_ID_LOCAL_POSITION:
	        {
	            if(msg->sysid == systemid)
	            {
	                if(cur_dest.frame == 1)
	                {
	                    mavlink_msg_local_position_decode(msg, &last_known_pos);
	                    if (debug) printf("Received new position: x: %f | y: %f | z: %f\n", last_known_pos.x, last_known_pos.y, last_known_pos.z);
	                    struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
                        if(now-timestamp_last_handle_waypoint > paramClient->getParamValue("HANDLEWPDELAY")*1000000 && current_active_wp_id != (uint16_t)-1)
                        {
                        	handle_waypoint(current_active_wp_id,now);
                        }
	                }
	            }
	            break;
	        }
	    case MAVLINK_MSG_ID_PATTERN_DETECTED:
			{
				GString* SEARCH_PIC = g_string_new("media/sweep_images/mona.jpg");
				mavlink_pattern_detected_t pd;
				mavlink_msg_pattern_detected_decode(msg, &pd);
				//printf("Pattern - conf: %f, detect: %i, file: %s, type: %i\n",pd.confidence,pd.detected,pd.file,pd.type);

				if(pd.detected==1 && strcmp((char*)pd.file,SEARCH_PIC->str) == 0) {
					++npic;
					if(debug) printf("Found it! - confidence: %f, detect: %i, file: %s, type: %i\n",pd.confidence,pd.detected,pd.file,pd.type);
				}
				break;
			}

	    case MAVLINK_MSG_ID_ACTION: // special action from ground station
	        {
	            mavlink_action_t action;
	            mavlink_msg_action_decode(msg, &action);
	            if(action.target == systemid)
	            {
	                if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
	                switch (action.action)
	                {
	                    //				case MAV_ACTION_LAUNCH:
	                    //					if (verbose) std::cerr << "Launch received" << std::endl;
	                    //					current_active_wp_id = 0;
	                    //					if (waypoints->size()>0)
	                    //					{
	                    //						setActive(waypoints[current_active_wp_id]);
	                    //					}
	                    //					else
	                    //						if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
	                    //					break;

	                    //				case MAV_ACTION_CONTINUE:
	                    //					if (verbose) std::c
	                    //					err << "Continue received" << std::endl;
	                    //					idle = false;
	                    //					setActive(waypoints[current_active_wp_id]);
	                    //					break;

	                    //				case MAV_ACTION_HALT:
	                    //					if (verbose) std::cerr << "Halt received" << std::endl;
	                    //					idle = true;
	                    //					break;

	                    //				default:
	                    //					if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
	                    //					break;
	                }
	            }
	            break;
	        }

		default:
        {
            if (debug) std::cerr << "Waypoint: received message of unknown type" << std::endl;
            break;
        }

	}
}

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavlink_message_t* msg, void * user)
{
	g_static_mutex_lock(main_mutex);

    // Handle param messages
    paramClient->handleMAVLinkPacket(msg);

    //check for timed-out operations
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;
    if (now-protocol_timestamp_lastaction > paramClient->getParamValue("PROTTIMEOUT")*1000000 && current_state != PX_WPP_IDLE)
    {
        if (verbose) printf("Last operation (state=%u) timed out, changing state to PX_WPP_IDLE\n", current_state);
        current_state = PX_WPP_IDLE;
        protocol_current_count = 0;
        protocol_current_partner_systemid = 0;
        protocol_current_partner_compid = 0;
        protocol_current_wp_id = -1;

        if(waypoints->size() == 0)
        {
            current_active_wp_id = -1;
        }
    }

    handle_communication(msg, now);

    g_static_mutex_unlock(main_mutex);
}

void* lcm_thread_func (gpointer lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	while (1)
	{
		lcm_handle(lcm);
	}
	return NULL;
}

int main(int argc, char* argv[])
/**
*  @brief main function of process (start here)
*
*  The function parses for program options, sets up some example waypoints and connects to IPC
*/
{
	std::string waypointfile;
    config::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("debug,d", config::bool_switch(&debug)->default_value(false), "Emit debug information")
            ("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
            ("config", config::value<std::string>(&configFile)->default_value("config/parameters_waypointplanner.cfg"), "Config file for system parameters")
            ("waypointfile", config::value<std::string>(&waypointfile)->default_value(""), "Config file for waypoint")
            ;
    config::variables_map vm;
    config::store(config::parse_command_line(argc, argv, desc), vm);
    config::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }


    //initialize current destination as (0,0,0) local coordinates
    cur_dest.frame = 1; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	cur_dest.x = 0; //local: x position, global: longitude
	cur_dest.y = 0; //local: y position, global: latitude
	cur_dest.z = 0; //local: z position, global: altitude
	cur_dest.yaw = 0; //Yaw orientation in degrees, [0..360] 0 = NORTH
	cur_dest.rad = 0; //Acceptance radius, in meter

    /**********************************
    * Run LCM and subscribe to MAVLINK
    **********************************/
    lcm = lcm_create ("udpm://");
    if (!lcm)
    {
    	printf("LCM failed.\n");
    	return NULL;
    }
    comm_sub = mavlink_message_t_subscribe (lcm, "MAVLINK", &mavlink_handler, NULL);


    /**********************************
    * Set default parameters and read parameters from file, if available.
    * Warning: Parameter name may not be longer than 14 characters!!!
    **********************************/
    paramClient = new PxParamClient(systemid, compid, lcm, configFile, verbose);
    paramClient->setParamValue("POSFILTER", 1.f);
    paramClient->setParamValue("SETPOINTDELAY", 1.0);
    paramClient->setParamValue("HANDLEWPDELAY",0.2);
    paramClient->setParamValue("PROTDELAY",40);	 //Attention: microseconds!!
    paramClient->setParamValue("PROTTIMEOUT", 2.0);
    paramClient->setParamValue("YAWTOLERANCE", 0.1745f);
    paramClient->readParamsFromFile(configFile);


    /**********************************
    * Run the LCM thread
    **********************************/
	if( !g_thread_supported() )
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	gpointer lcm_ptr = (gpointer) lcm;

	if( (waypoint_lcm_thread = g_thread_create(lcm_thread_func, lcm_ptr, TRUE, &error)) == NULL)
	{
		printf("Thread creation failed: %s!!\n", error->message );
		g_error_free ( error ) ;
	}

    /**********************************
    * Initialize mutex
    **********************************/
	if (!main_mutex)
	{
		g_static_mutex_init(main_mutex);
	}


    /**********************************
    * Read waypoints from file and
    * set the new current waypoint
    **********************************/
	g_static_mutex_lock(main_mutex);
    if (waypointfile.length())
    {
        std::ifstream wpfile;
        wpfile.open(waypointfile.c_str());
        if (!wpfile) {
            printf("Unable to open waypoint file\n");
            exit(1); // terminate with error
        }

        if (!wpfile.eof())
        {
            std::string check;
            //			int ver;
            //			bool good = false;
            //			wpfile >> check;
            //			if (!strcmp(check.c_str(),"QGC"))
            //			{
            //				wpfile >> check;
            //				if (!strcmp(check.c_str(),"WPL"))
            //				{
            //					wpfile >> ver;
            //					if (ver == 100)
            //					{
            //						char c = (char)wpfile.peek();
            //						if(c == '\r' || c == '\n')
            //						{
            //							good = true;
            //						}
            //					}
            //				}
            //			}

            printf("Loading waypoint file...\n");

            getline(wpfile,check);

            printf("Version line: %s\n", check.c_str());

            if(!strcmp(check.c_str(),"QGC WPL 120"))
            {
                printf("Waypoint file version mismatch\n");
                exit(1); // terminate with error
            }

            while (!wpfile.eof())
            {
                mavlink_waypoint_t *wp = new mavlink_waypoint_t();

                uint16_t temp;

                wpfile >> wp->seq; //waypoint id
                wpfile >> temp; wp->current = temp;
                wpfile >> temp; wp->frame = temp;
                wpfile >> temp; wp->command = temp;
                wpfile >> wp->param1;
                wpfile >> wp->param2;
                wpfile >> wp->param3; //old "orbit"
                wpfile >> wp->param4; //old "yaw"
                wpfile >> wp->x;
                wpfile >> wp->y;
                wpfile >> wp->z;
                wpfile >> temp; wp->autocontinue = temp;

                char c = (char)wpfile.peek();
                if(c != '\r' && c != '\n')
                {
                    delete wp;
                    break;
                }

                printf("WP %3u%s: Frame: %u\tCommand: %3u\tparam1: %6.2f\tparam2: %7.2f\tparam3: %6.2f\tparam4: %7.2f\tX: %7.2f\tY: %7.2f\tZ: %7.2f\tAuto-Cont: %u\t\n", wp->seq, (wp->current?"*":" "), wp->frame, wp->command, wp->param1, wp->param2, wp->param3, wp->param4, wp->x, wp->y, wp->z, wp->autocontinue);
                waypoints->push_back(wp);
            }
        }
        else
        {
            printf("Empty waypoint file!\n");
        }
        wpfile.close();

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;

        uint32_t i;
        for(i = 0; i < waypoints->size(); i++)
        {
            if (waypoints->at(i)->current == 1)
            {
        		current_active_wp_id = i;
        		if (verbose) printf("New current waypoint %u\n", current_active_wp_id);
        		handle_waypoint(current_active_wp_id,now);
        		send_setpoint();
        		break;
            }
        }
        if (i == waypoints->size())
        {
            current_active_wp_id = -1;
            timestamp_firstinside_orbit = 0;
        }

    }
    g_static_mutex_unlock(main_mutex);


    printf("WAYPOINTPLANNER INITIALIZATION DONE, RUNNING...\n");

    /**********************************
    * Main loop
    **********************************/
    while (1)//need some break condition, e.g. if LCM fails
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now = ((uint64_t)tv.tv_sec)*1000000 + tv.tv_usec;

        // resend current destination every "SETPOINTDELAY" seconds
        g_static_mutex_lock(main_mutex);
        if(now-timestamp_last_send_setpoint > paramClient->getParamValue("SETPOINTDELAY")*1000000) //&& current_active_wp_id != (uint16_t)-1)
        {
            send_setpoint();
        }
        g_static_mutex_unlock(main_mutex);
    }

    /**********************************
    * Terminate the LCM
    **********************************/
	mavlink_message_t_unsubscribe (lcm, comm_sub);
	lcm_destroy (lcm);
	printf("WAYPOINTPLANNER TERMINATED\n");

}
