/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
 *   @brief LCM/SYS-V Shared memory image client
 *
 *   Process for capturing image and GPS data.
 *
 *   @author Bernhard Zeisl
 *
 */

#include <inttypes.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <glib.h>
#include <time.h>
// BOOST includes
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
// OpenCV includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <interface/shared_mem/PxSHMImageClient.h>
#include "mavconn.h"

namespace config = boost::program_options;
namespace bfs = boost::filesystem;

int sysid = getSystemID();
int compid = 30;
bool silent, verbose, stereo_front;

#define CAPTURE_DIR			"dataset_capture/"
#define IMAGE_CAPTURE_FILE	"imagedata.txt"
#define PLAIN_CAPTURE_FILE	"imagedata_extended.txt"

using namespace std;

string captureDir;
string captureDirLeft;
string captureDirRight;
ofstream imageDataFile;
ofstream plainLogFile;
ofstream mavlinkFile;

cv::Mat img_left( 480, 640, CV_8UC1 );
cv::Mat img_right( 480, 640, CV_8UC1 );

uint64_t imgNum = 0;
bool useStereo;

bool recordData = false;
bool bPause = false;

static GMutex *image_grabbed_mutex = NULL;
GCond *image_grabbed_cond = NULL;
bool image_grabbed = false;

//image information to store
uint64_t timestamp;
double roll, pitch, yaw, lat, lon, alt, pres_alt, ground_dist, vdop, hdop, satcount, local_x, local_y, local_z, vx, vy, vz, gx, gy, gz, gvx, gvy, gvz;

bool gotFirstImage = false;
bool imageMetaContainsGPS = false;
bool imageMetaContainsAttitude = false;
bool imageMetaContainsGroundTruth = false;
bool imageMetaContainsLocalPosition = false;
bool imageMetaContainsLocalSpeed = false;
bool imageMetaContainsPressureAltitude = false;
bool imageMetaContainsGroundDistance = false;

static void image_writer (void)
{
	//get lock for condition
	g_mutex_lock(image_grabbed_mutex);

	while(1)
	{
		//unlock mutex and wait for condition
		while (!image_grabbed)
			g_cond_wait(image_grabbed_cond, image_grabbed_mutex);

		//now store the new image - this can take as long as it is necessary - but all incoming images during this time will get dropped

		char fileName[40];
		sprintf(fileName, "%llu.bmp", (long long unsigned)timestamp);

		if (useStereo)
		{
			cv::imwrite((captureDirLeft + fileName).c_str(), img_left);
			cv::imwrite((captureDirRight + fileName).c_str(), img_right);
		}
		else
		{
			cv::imwrite((captureDirLeft + fileName).c_str(), img_left);
		}

		imageDataFile.precision(32);
		imageDataFile << timestamp << ", " << roll << ", " << pitch << ", " << yaw << ", " << lat << ", " << lon << ", " << alt << ", " << ground_dist << ", " << gx << ", " << gy << ", " << gz << endl;
		plainLogFile.precision(32);
		plainLogFile << timestamp << ", " << roll << ", " << pitch << ", " << yaw << ", " << lat << ", " << lon << ", " << alt << ", " << pres_alt << ", " << ground_dist << ", " << vdop << ", " << hdop << ", " << satcount << ", " << local_x << ", " << local_y << ", " << local_z << ", " << vx << ", " << vy << ", " << vz << ", " << gx << ", " << gy << ", " << gz << ", " << gvx << ", " << gvy << ", " << gvz << endl;

		//fprintf(plainLogFile, "%llu, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f, %15.36f", timestamp, roll, pitch, yaw, lat, lon, alt, pres_alt, ground_dist, vdop, hdop, satcount, local_x, local_y, local_z, vx, vy, vz, gx, gy, gz, gvx, gvy, gvz);

		++imgNum;
		image_grabbed = false;
	}
}

/**
 * @brief Handle incoming MAVLink packets containing images
 */
static void image_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	if (recordData && !bPause)
	{
		//try to get the lock, if it is still locked by image writer, skip the current image (this is for very slow computers)
		if(g_mutex_trylock(image_grabbed_mutex))
		{
			const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

			// Pointer to shared memory data
			PxSHMImageClient* client = static_cast<PxSHMImageClient*>(user);

			if (verbose)
			{
				cout << ". " << flush;
			}

			// copy stereo images from shared buffer
			if (client->readStereoImage(msg, img_left, img_right))
			{
				useStereo = true;
			}
			// copy one image from shared buffer
			else if (client->readMonoImage(msg, img_left))
			{
				useStereo = false;
			}

			//cache the meta data
			timestamp = client->getTimestamp(msg);
			
			float camroll, campitch, camyaw, camlat, camlon, camalt, camground_dist, camgx, camgy, camgz;			
			client->getRollPitchYaw(msg, camroll, campitch, camyaw);
			client->getLocalHeight(msg, camground_dist);
			client->getGPS(msg, camlat, camlon, camalt);
			client->getGroundTruth(msg, camgx, camgy, camgz);
			
			// FIXME should be parametrized differently,
			// add a client->isGPSKnown() function call
			if (camlat != 0.0 || camlon != 0.0 || camalt != 0.0) 
			{
				lat = camlat;
				lon = camlon;
				alt = camalt;
				imageMetaContainsGPS = true;
			}
			
			if (camroll != 0.0 || campitch != 0.0 || camyaw != 0.0)
			{
				imageMetaContainsAttitude = true;
				roll = camroll;
				pitch = campitch;
				yaw = camyaw;
			}
			
			if (camgx != 0.0 || camgy != 0.0 || camgz != 0.0)
			{
				imageMetaContainsGroundTruth = true;
				gx = camgx;
				gy = camgy;
				gz = camgz;
			}
			
			if (camground_dist != 0.0)
			{
				imageMetaContainsGroundDistance = true;
				ground_dist = camground_dist;
			}
			
			gotFirstImage = true;

			//signal to image_writer thread that a new image can be stored
			image_grabbed = true;
			g_cond_signal(image_grabbed_cond);
			g_mutex_unlock(image_grabbed_mutex);
			g_thread_yield();
		}
		else
		{
			if (verbose)
			{
				cout << "x " << flush;
			}
		}
	}
}

/*
 * @brief Creates data directory based on current date and time, if not existent so far
 */
string createCaptureDirectory( string baseDir, struct tm* timeinfo )
{
	// create current data as string
	// date is used as name for a new folder where the data will be stored
	char dateBuf[80];
	strftime( dateBuf, 80, "%Y%m%d_%H%M%S\0", timeinfo );

	// create directory where to safe the images and data file
	string dir = baseDir + string(dateBuf) + "/";
	try
	{
		bfs::create_directories( bfs::path(dir) );
		bfs::create_directories( bfs::path(dir + std::string("left/")) );
		bfs::create_directories( bfs::path(dir + std::string("right/")) );
	} catch (...) {
		cout << "Directory " << dir << " for data capture couldn't be created. EXITING!!" << endl;
		exit(EXIT_FAILURE);
	}

	return dir;
}

/*
 * @brief Writes header into capture file
 */
void prepareCaptureFile( ofstream& file, const string& dir, const string& filename, const string& structure, const string& type, struct tm* timeinfo )
{
	cout << "Writing " << type << " capture data into " << dir << filename << endl;
	file.open((dir + filename).c_str(), ios::out);
	file << "# Data Capture for stereo head on Pixhawk helicopter." << endl;
	char dayBuf[20], timeBuf[20];
	strftime( dayBuf, 20, "%Y-%m-%d\0", timeinfo );
	strftime( timeBuf, 20, "%H:%M\0", timeinfo );
	file << "# Recording date: " << dayBuf << ", " << timeBuf << endl;
	file << "##########" << endl;
	file << "# Structure:" << endl;
	file << "# " << structure << endl;
	file << "##########" << endl << endl;
}

/**
 * @brief Handle incoming MAVLink packets
 */
// Timer for benchmarking
struct timeval tv;

static void mavlink_handler (const lcm_recv_buf_t *rbuf, const char * channel, const mavconn_mavlink_msg_container_t* container, void * user)
{
	lcm_t* lcmMavlink = (lcm_t*) user;
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t command;
		mavlink_msg_command_long_decode(msg, &command);
		if (command.command == MAV_CMD_DO_CONTROL_VIDEO)
		{
			if (command.param4 == 1 || command.param4 == 2)
			{
				mavlink_message_t msg;
				mavlink_statustext_t statustext;
				if (!recordData)
				{
					// start recording image data
					if (verbose) printf("Starting recording.\n");

					// preparing data folder and data file
					time_t rawtime;
					time( &rawtime );
					struct tm* timeinfo = localtime(&rawtime);
					captureDir = createCaptureDirectory(CAPTURE_DIR, timeinfo);
					captureDirLeft = captureDir + std::string("left/");
					captureDirRight = captureDir + std::string("right/");
					prepareCaptureFile(imageDataFile, captureDir, IMAGE_CAPTURE_FILE, "imagenumber, timestamp, roll, pitch, yaw, lat, lon, alt, local_z, ground truth X, ground truth Y, ground truth Z", "IMAGE", timeinfo);
					prepareCaptureFile(plainLogFile, captureDir, PLAIN_CAPTURE_FILE, "imagenumber, timestamp, roll, pitch, yaw, lat, lon, alt, pressure_alt, ground_distance, vdop, hdop, local_x, local_y, local_z, speedx, speedy, speedz, ground truth X, ground truth Y, ground truth Z, ground truth speed X, ground truth speed Y, ground truth speed Z", "IMAGE", timeinfo);

					char dateBuf[80];
					strftime( dateBuf, 80, "%Y%m%d_%H%M%S\0", timeinfo );
					mavlinkFile.open((string(CAPTURE_DIR) + string(dateBuf) + string(".mavlink")).c_str(), ios::binary | ios::out);

					sprintf((char*)&statustext.text, "MAVCONN: imagecapture: STARTING RECORDING");
					mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
					sendMAVLinkMessage(lcmMavlink, &msg);
					snprintf((char*)&statustext.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, "%s%s%s", CAPTURE_DIR, dateBuf, ".mavlink");
					mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
					sendMAVLinkMessage(lcmMavlink, &msg);
					recordData = true;
					bPause = false;
				}
				else if(bPause)
				{
					bPause = false;
					sprintf((char*)&statustext.text, "MAVCONN: imagecapture: CONTINUE RECORDING");
					mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
					sendMAVLinkMessage(lcmMavlink, &msg);
				}
			}
			else if (command.param4 == 0)
			{
				if (recordData)
				{
					// stop recording image data
					if (verbose) printf("Stop recording.\n");

					imageDataFile << endl << "### EOF" << endl;
					imageDataFile.close();
					plainLogFile << endl << "### EOF" << endl;
					plainLogFile.close();
					mavlinkFile.close();

					mavlink_message_t msg;
					mavlink_statustext_t statustext;
					sprintf((char*)&statustext.text, "MAVCONN: imagecapture: STOPPED RECORDING");
					mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
					sendMAVLinkMessage(lcmMavlink, &msg);
					recordData = false;
					bPause = false;
				}
			}
		}
	}
	break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			mavlink_gps_raw_int_t gps;
			mavlink_msg_gps_raw_int_decode(msg, &gps);
			if (!imageMetaContainsGPS)
			{
				lat = gps.lat/(double)1E7;
				lon = gps.lon/(double)1E7;
				alt = gps.alt/(double)1E3;
				hdop = gps.eph/(double)1E3;
				vdop = gps.epv/(double)1E3;
				satcount = gps.satellites_visible;
			}
		}
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			mavlink_local_position_ned_t pos;
			mavlink_msg_local_position_ned_decode(msg, &pos);
			if (!imageMetaContainsLocalPosition)
			{
				local_x = pos.x;
				local_y = pos.y;
				local_z = pos.z;
			}
			if (!imageMetaContainsLocalSpeed)
			{
				vx = pos.vx;
				vy = pos.vy;
				vz = pos.vz;
			}
		}
			break;

	default:
		break;
	}

	if(recordData && !bPause)
	{
		//write into mavlink logfile
		const int len = MAVLINK_MAX_PACKET_LEN+sizeof(uint64_t);
		uint8_t buf[len];
		uint64_t time = getSystemTimeUsecs();
		memcpy(buf, (void*)&time, sizeof(uint64_t));
		mavlink_msg_to_send_buffer(buf+sizeof(uint64_t), msg);
		mavlinkFile.write((char *)buf, len);
	}
}


void* lcm_image_wait(void* lcm_ptr)
{
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while(1)
	{
		lcm_handle (lcm);
	}
	return NULL;
}

/*
 * @brief Main
 */
int main(int argc, char* argv[])
{
	// ----- Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("sysid,a", config::value<int>(&sysid)->default_value(sysid), "ID of this system, 1-255")
		("compid,c", config::value<int>(&compid)->default_value(compid), "ID of this component")
		("silent,s", config::bool_switch(&silent)->default_value(false), "suppress outputs")
		("verbose,v", config::bool_switch(&verbose)->default_value(false), "verbose output")
		("stereo_front", config::bool_switch(&stereo_front)->default_value(false), "record front stereo")
		;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	// ----- Setting up communication and data for images
	// Creating LCM network provider
	lcm_t* lcmImage = lcm_create ("udpm://");
	lcm_t* lcmMavlink = lcm_create ("udpm://");
	if (!lcmImage || !lcmMavlink)
		exit(EXIT_FAILURE);

	PxSHMImageClient* cam = new PxSHMImageClient();
	if(stereo_front)
	{
		if (!cam->init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT))
		{
			printf("ERROR!\n");
			return -7;
		}
	}
	else
	{
		if (!cam->init(true, PxSHM::CAMERA_DOWNWARD_LEFT))
		{
			printf("ERROR!\n");
			return -7;
		}
	}
	mavconn_mavlink_msg_container_t_subscription_t * img_sub  = mavconn_mavlink_msg_container_t_subscribe (lcmImage, MAVLINK_IMAGES, &image_handler, cam);
	mavconn_mavlink_msg_container_t_subscription_t * comm_sub = mavconn_mavlink_msg_container_t_subscribe (lcmMavlink, MAVLINK_MAIN, &mavlink_handler, lcmMavlink);

	// ----- Creating thread for image handling
	GThread* lcm_imageThread;
	GThread* lcm_imageWriterThread;
	GError* err;

	if( !g_thread_supported() ) {
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}
	if (!image_grabbed_mutex)
	{
		image_grabbed_mutex = g_mutex_new();
	}

	if (!image_grabbed_cond)
	{
		image_grabbed_cond = g_cond_new();
	}

	// thread for IMAGE channel
	if( (lcm_imageThread = g_thread_create((GThreadFunc)lcm_image_wait, (void *)lcmImage, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err);
		exit(EXIT_FAILURE);
	}

	// thread for image_writer
	if( (lcm_imageWriterThread = g_thread_create((GThreadFunc)image_writer, NULL, TRUE, &err)) == NULL)
	{
		cout << "Thread create failed: " << err->message << "!!" << endl;
		g_error_free(err);
		exit(EXIT_FAILURE);
	}

	printf("IMAGE client ready, waiting.\n");

	mavlink_message_t msg;
	mavlink_statustext_t statustext;
	sprintf((char*)&statustext.text, "MAVCONN: imagecapture: Ready for capture");
	mavlink_msg_statustext_encode(sysid, compid, &msg, &statustext);
	sendMAVLinkMessage(lcmMavlink, &msg);

	while(1)
	{
		// blocking wait for MAVLINK channel
		lcm_handle(lcmMavlink);
	}

	mavconn_mavlink_msg_container_t_unsubscribe (lcmImage, img_sub);
	mavconn_mavlink_msg_container_t_unsubscribe (lcmMavlink, comm_sub);
	lcm_destroy (lcmImage);
	lcm_destroy (lcmMavlink);
	delete cam;

	return EXIT_SUCCESS;
}
