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
 *   This is an example showing how to read from PIXHAWK System V image transport layer
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Lionel Heng
 *
 */

// BOOST includes
#include <boost/program_options.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>

#include <mavconn.h>
#include <interface/shared_mem/PxSHMImageClient.h>

namespace config = boost::program_options;

int sysid;
int compid;
bool verbose;

int imageCounter = 0;
std::string fileBaseName("frame");
std::string fileExt(".png");

bool quit = false;

void
signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
		exit(EXIT_SUCCESS);
	}
}

/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void
imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
			 const mavconn_mavlink_msg_container_t* container, void* user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	std::vector<PxSHMImageClient>* clientVec =
			reinterpret_cast< std::vector<PxSHMImageClient>* >(user);

	cv::Mat imgToSave;

	for (size_t i = 0; i < clientVec->size(); ++i)
	{
		PxSHMImageClient& client = clientVec->at(i);
		if ((client.getCameraConfig() & PxSHMImageClient::getCameraNo(msg)) != PxSHMImageClient::getCameraNo(msg))
		{
			continue;
		}

		// read mono image data
		cv::Mat img;
		if (client.readMonoImage(msg, img))
		{
			struct timeval tv;
			gettimeofday(&tv, NULL);
			uint64_t currTime = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
			uint64_t timestamp = client.getTimestamp(msg);

			uint64_t diff = currTime - timestamp;

			if (verbose)
			{
				fprintf(stderr, "# INFO: Time from capture to display: %llu ms for camera %llu\n", diff / 1000, client.getCameraID(msg));
			}

			// Display if switched on
#ifndef NO_DISPLAY
			if ((client.getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT) == PxSHM::CAMERA_FORWARD_LEFT)
			{
				cv::namedWindow("Left Image (Forward Camera)");
				cv::imshow("Left Image (Forward Camera)", img);
			}
			else
			{
				cv::namedWindow("Left Image (Downward Camera)");
				cv::imshow("Left Image (Downward Camera)", img);
			}
#endif

			img.copyTo(imgToSave);
		}

		cv::Mat imgLeft, imgRight;
		if (client.readStereoImage(msg, imgLeft, imgRight))
		{
#ifndef NO_DISPLAY
			if ((client.getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT) == PxSHM::CAMERA_FORWARD_LEFT)
			{
				cv::namedWindow("Left Image (Forward Camera)");
				cv::imshow("Left Image (Forward Camera)", imgLeft);
				cv::namedWindow("Right Image (Forward Camera)");
				cv::imshow("Right Image (Forward Camera)", imgRight);
			}
			else
			{
				cv::namedWindow("Left Image (Downward Camera)");
				cv::imshow("Left Image (Downward Camera)", imgLeft);
				cv::namedWindow("Right Image (Downward Camera)");
				cv::imshow("Right Image (Downward Camera)", imgRight);
			}
#endif

			imgLeft.copyTo(imgToSave);
		}

		// read Kinect data
		cv::Mat imgBayer, imgDepth;
		if (client.readKinectImage(msg, imgBayer, imgDepth))
		{
#ifndef NO_DISPLAY
			cv::Mat imgColor;

			cv::cvtColor(imgBayer, imgColor, CV_BayerGB2BGR);
			cv::namedWindow("Image (Kinect Camera)");
			cv::imshow("Image (Kinect Camera)", imgColor);
#endif

			imgColor.copyTo(imgToSave);
		}
	}

#ifndef NO_DISPLAY
	int c = cv::waitKey(3);
	switch (static_cast<char>(c))
	{
		case 'f':
		{
			char index[20];
			sprintf(index, "%04d", imageCounter++);
			cv::imwrite(std::string(fileBaseName+index+fileExt).c_str(), imgToSave);
		}
		break;
	default:
		break;
	}
#endif
}

int main(int argc, char* argv[])
{
	// Handling Program options
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("sysid,a", config::value<int>(&sysid)->default_value(42), "ID of this system, 1-256")
		("compid,c", config::value<int>(&compid)->default_value(30), "ID of this component")
		("verbose,v", config::bool_switch(&verbose)->default_value(false), "Verbose output")
		;
	config::variables_map vm;
	config::store(config::parse_command_line(argc, argv, desc), vm);
	config::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		exit(EXIT_FAILURE);
	}

	std::vector<PxSHMImageClient> clientVec;
	clientVec.resize(4);

	clientVec.at(0).init(true, PxSHM::CAMERA_FORWARD_LEFT);
	clientVec.at(1).init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
	clientVec.at(2).init(true, PxSHM::CAMERA_DOWNWARD_LEFT);
	clientVec.at(3).init(true, PxSHM::CAMERA_DOWNWARD_LEFT, PxSHM::CAMERA_DOWNWARD_RIGHT);

	// Ready to roll
	fprintf(stderr, "# INFO: Image client ready, waiting for images..\n");

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES, &imageHandler, &clientVec);

	signal(SIGINT, signalHandler);

	while (!quit)
	{
		// Block waiting for new image messages,
		// once an image is received it will be
		// displayed
		lcm_handle(lcm);
	}

	mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
	lcm_destroy(lcm);

	return 0;
}
