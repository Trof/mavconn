/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009-2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
*   @brief Shared memory interface for writing images.
*
*   This interface is a wrapper around PxSHM.
*
*   @author Lionel Heng  <hengli@inf.ethz.ch>
*
*/

#ifndef PXSHMIMAGESERVER_H
#define PXSHMIMAGESERVER_H

#include <mavconn.h>
#include <opencv2/core/core.hpp>

#include "PxSHM.h"

class PxSHMImageServer
{
public:
	PxSHMImageServer();
	
	/**
	 * Initializes the image server.
	 *
	 * @param sysid System ID.
	 * @param compid Component ID.
	 * @param lcm LCM instance.
	 * @param cam1 Camera 1. If it is part of a stereo rig, it is the left camera.
	 * @param cam2 Camera 2. If it is part of a stereo rig, it is the right camera.
	 * 						 Otherwise, leave the parameter empty.
	 *
	 * @return Result of shared memory segment access.
	 */
	bool init(int sysid, int compid, lcm_t* lcm,
			  PxSHM::Camera cam1, PxSHM::Camera cam2 = PxSHM::CAMERA_NONE);
	
	int getCameraConfig(void) const;

	void writeMonoImage(const cv::Mat& img, uint64_t camId,
						uint64_t timestamp, const mavlink_image_triggered_t &image_data,
						uint32_t exposure);
	
	void writeStereoImage(const cv::Mat& imgLeft, uint64_t camIdLeft,
						  const cv::Mat& imgRight, uint64_t camIdRight,
						  uint64_t timestamp, const mavlink_image_triggered_t &image_data,
						  uint32_t exposure);
	
	void writeKinectImage(const cv::Mat& imgBayer, const cv::Mat& imgDepth,
						  uint64_t timestamp, float roll, float pitch, float yaw,
						  float z, float lon, float lat, float alt, float ground_x, float ground_y, float ground_z);
	
	void writeRGBDImage(const cv::Mat& img, const cv::Mat& imgDepth,
						uint64_t timestamp, float roll, float pitch, float yaw,
						float lon, float lat, float alt,
						float ground_x, float ground_y, float ground_z,
						const cv::Mat& cameraMatrix,
						const cv::Rect& roi = cv::Rect());

private:
	bool writeImage(PxSHM::CameraType cameraType, const cv::Mat& img,
					const cv::Mat& img2 = cv::Mat());

	bool writeImageWithCameraInfo(PxSHM::CameraType cameraType,
								  uint64_t timestamp,
								  float roll, float pitch, float yaw,
								  float lon, float lat, float alt,
								  float ground_x, float ground_y, float ground_z,
								  const cv::Mat& cameraMatrix,
								  const cv::Rect& roi,
								  const cv::Mat& img,
								  const cv::Mat& img2 = cv::Mat());

	int sysid;
	int compid;
	lcm_t* lcm;
	PxSHM::Camera cam1;
	PxSHM::Camera cam2;
	
	PxSHM shm;
	int key;
	
	std::vector<uint8_t> data;

	unsigned int imgSeq;
};

#endif
