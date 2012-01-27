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

#include "PxSHMImageServer.h"

PxSHMImageServer::PxSHMImageServer()
{
	
}
	
bool
PxSHMImageServer::init(int sysid, int compid, lcm_t* lcm,
					   PxSHM::Camera cam1, PxSHM::Camera cam2)
{
	this->sysid = sysid;
	this->compid = compid;
	this->lcm = lcm;
	this->cam1 = cam1;
	this->cam2 = cam2;
	key = cam1 | cam2;
	
	imgSeq = 0;
	
	data.reserve(1024 * 1024);
	return shm.init(key, PxSHM::SERVER_TYPE, 128, 1, 1024 * 1024, 9);
}

int
PxSHMImageServer::getCameraConfig(void) const
{
	return cam1 | cam2;
}

void
PxSHMImageServer::writeMonoImage(const cv::Mat& img, uint64_t camId,
								 uint64_t timestamp, const mavlink_image_triggered_t &image_data,
								 uint32_t exposure)
{
	PxSHM::CameraType cameraType;
	if (img.channels() == 1)
	{
		cameraType = PxSHM::CAMERA_MONO_8;
	}
	else
	{
		cameraType = PxSHM::CAMERA_MONO_24;
	}

	writeImage(cameraType, img);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = camId;
	imginfo.cam_no = cam1;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->imgSeq;
	imginfo.img_buf_index = 1;	//FIXME
	imginfo.width = img.cols;
	imginfo.height = img.rows;
	imginfo.depth = img.depth();
	imginfo.channels = img.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = exposure;
	imginfo.gain = 1;//gain;

	imginfo.roll = image_data.roll;
	imginfo.pitch = image_data.pitch;
	imginfo.yaw = image_data.yaw;
	imginfo.local_z = image_data.local_z;
	imginfo.lon = image_data.lon;
	imginfo.lat = image_data.lat;
	imginfo.alt = image_data.alt;
	imginfo.ground_x = image_data.ground_x;
	imginfo.ground_y = image_data.ground_y;
	imginfo.ground_z = image_data.ground_z;
	
	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	sendMAVLinkImageMessage(lcm, &msg);
	

	imgSeq++;
}

void
PxSHMImageServer::writeStereoImage(const cv::Mat& imgLeft, uint64_t camIdLeft,
									  const cv::Mat& imgRight, uint64_t camIdRight,
									  uint64_t timestamp, const mavlink_image_triggered_t &image_data,
									  uint32_t exposure)
{
	PxSHM::CameraType cameraType;
	if (imgLeft.channels() == 1)
	{
		cameraType = PxSHM::CAMERA_STEREO_8;
	}
	else
	{
		cameraType = PxSHM::CAMERA_STEREO_24;
	}

	writeImage(cameraType, imgLeft, imgRight);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = camIdLeft;
	imginfo.cam_no = cam1 | cam2;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->imgSeq;
	imginfo.img_buf_index = 2;	//FIXME
	imginfo.width = imgLeft.cols;
	imginfo.height = imgLeft.rows;
	imginfo.depth = imgLeft.depth();
	imginfo.channels = imgLeft.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = exposure;
	imginfo.gain = 1;//gain;
	
	imginfo.roll = image_data.roll;
	imginfo.pitch = image_data.pitch;
	imginfo.yaw = image_data.yaw;
	imginfo.local_z = image_data.local_z;
	imginfo.lon = image_data.lon;
	imginfo.lat = image_data.lat;
	imginfo.alt = image_data.alt;
	imginfo.ground_x = image_data.ground_x;
	imginfo.ground_y = image_data.ground_y;
	imginfo.ground_z = image_data.ground_z;

	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	sendMAVLinkImageMessage(lcm, &msg);
	
	imgSeq++;
}

void
PxSHMImageServer::writeKinectImage(const cv::Mat& imgBayer, const cv::Mat& imgDepth,
								   uint64_t timestamp, float roll, float pitch, float yaw,
								   float z, float lon, float lat, float alt, float ground_x, float ground_y, float ground_z)
{
	writeImage(PxSHM::CAMERA_KINECT, imgBayer, imgDepth);
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	uint64_t valid_until = now + (uint64_t)(100000);
	
	mavlink_image_available_t imginfo;
	imginfo.cam_id = 0;
	imginfo.cam_no = cam1;
	imginfo.timestamp = timestamp;
	imginfo.valid_until = valid_until;
	imginfo.img_seq = this->imgSeq;
	imginfo.img_buf_index = 1;	//FIXME
	imginfo.width = imgBayer.cols;
	imginfo.height = imgBayer.rows;
	imginfo.depth = imgBayer.depth();
	imginfo.channels = imgBayer.channels();
	imginfo.key = (int)this->key;
	imginfo.exposure = 0;
	imginfo.gain = 1;//gain;
	imginfo.roll = roll;
	imginfo.pitch = pitch;
	imginfo.yaw = yaw;
	imginfo.local_z = z;
	imginfo.lon = lon;
	imginfo.lat = lat;
	imginfo.alt = alt;
	imginfo.ground_x = ground_x;
	imginfo.ground_y = ground_y;
	imginfo.ground_z = ground_z;
	
	mavlink_message_t msg;
	mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
	sendMAVLinkImageMessage(lcm, &msg);
	
	imgSeq++;
}

void
PxSHMImageServer::writeRGBDImage(const cv::Mat& img, const cv::Mat& imgDepth,
								 uint64_t timestamp, float roll, float pitch, float yaw,
								 float lon, float lat, float alt,
								 float ground_x, float ground_y, float ground_z,
								 const cv::Mat& cameraMatrix,
								 const cv::Rect& roi)
{
	writeImageWithCameraInfo(PxSHM::CAMERA_RGBD,
							 timestamp, roll, pitch, yaw,
							 lon, lat, alt,
							 ground_x, ground_y, ground_z, cameraMatrix, roi,
							 img, imgDepth);

	imgSeq++;
}

bool
PxSHMImageServer::writeImage(PxSHM::CameraType cameraType, const cv::Mat& img,
							 const cv::Mat& img2)
{
	if (img.empty())
	{
		fprintf(stderr, "# WARNING: img parameter should not be empty.\n");
		return false;
	}

	uint32_t headerLength = 20;

	if (cameraType == PxSHM::CAMERA_STEREO_8 ||
		cameraType == PxSHM::CAMERA_STEREO_24 ||
		cameraType == PxSHM::CAMERA_KINECT ||
		cameraType == PxSHM::CAMERA_RGBD)
	{
		if (img2.empty())
		{
			fprintf(stderr, "# WARNING: img2 parameter should not be empty.\n");
			return false;
		}

		headerLength += 8;
	}

	uint32_t dataLength = headerLength + img.step[0] * img.rows +
						  img2.step[0] * img2.rows;

	while (data.capacity() < dataLength)
	{
		data.reserve(data.capacity() * 2);
	}

	data.resize(dataLength);

	int type = img.type();

	// write header
	memcpy(&(data[0]), &cameraType, 4);
	memcpy(&(data[4]), &(img.cols), 4);
	memcpy(&(data[8]), &(img.rows), 4);
	memcpy(&(data[12]), img.step.p, 4);
	memcpy(&(data[16]), &type, 4);

	//char dummy[img.step[0] * img.rows];
	memcpy(&(data[headerLength]), img.data, img.step[0] * img.rows);
	//memcpy(&(data[headerLength]), img.data, img.step[0] * img.rows);

	if (!img2.empty())
	{
		memcpy(&(data[20]), img2.step.p, 4);

		type = img2.type();
		memcpy(&(data[24]), &type, 4);

		memcpy(&(data[headerLength + img.step[0] * img.rows]), img2.data,
			   img2.step[0] * img2.rows);
	}

	shm.writeDataPacket(data);

	return true;
}

bool
PxSHMImageServer::writeImageWithCameraInfo(PxSHM::CameraType cameraType,
										   uint64_t timestamp,
										   float roll, float pitch, float yaw,
										   float lon, float lat, float alt,
										   float ground_x, float ground_y, float ground_z,
										   const cv::Mat& cameraMatrix,
										   const cv::Rect& roi,
										   const cv::Mat& img,
							 	 	 	   const cv::Mat& img2)
{
	if (img.empty())
	{
		fprintf(stderr, "# WARNING: img parameter should not be empty.\n");
		return false;
	}

	uint32_t headerLength = 116;

	if (cameraType == PxSHM::CAMERA_STEREO_8 ||
		cameraType == PxSHM::CAMERA_STEREO_24 ||
		cameraType == PxSHM::CAMERA_KINECT ||
		cameraType == PxSHM::CAMERA_RGBD)
	{
		if (img2.empty())
		{
			fprintf(stderr, "# WARNING: img2 parameter should not be empty.\n");
			return false;
		}

		headerLength += 8;
	}

	uint32_t dataLength = headerLength + img.step[0] * img.rows +
						  img2.step[0] * img2.rows;

	while (data.capacity() < dataLength)
	{
		data.reserve(data.capacity() * 2);
	}

	data.resize(dataLength);

	int type = img.type();

	// write header
	memcpy(&(data[0]), &cameraType, 4);
	memcpy(&(data[4]), &timestamp, 8);
	memcpy(&(data[12]), &roll, 4);
	memcpy(&(data[16]), &pitch, 4);
	memcpy(&(data[20]), &yaw, 4);
	memcpy(&(data[24]), &lon, 4);
	memcpy(&(data[28]), &lat, 4);
	memcpy(&(data[32]), &alt, 4);
	memcpy(&(data[36]), &ground_x, 4);
	memcpy(&(data[40]), &ground_y, 4);
	memcpy(&(data[44]), &ground_z, 4);

	assert(cameraMatrix.rows == 3);
	assert(cameraMatrix.cols == 3);

	int mark = 48;
	for (int i = 0; i < cameraMatrix.rows; ++i)
	{
		for (int j = 0; j < cameraMatrix.cols; ++j)
		{
			memcpy(&(data[mark]), &(cameraMatrix.at<float>(i,j)), 4);
			mark += 4;
		}
	}

	memcpy(&(data[mark]), &(roi.x), 4);
	memcpy(&(data[mark+4]), &(roi.y), 4);
	memcpy(&(data[mark+8]), &(roi.width), 4);
	memcpy(&(data[mark+12]), &(roi.height), 4);

	mark += 16;

	memcpy(&(data[mark]), &(img.cols), 4);
	memcpy(&(data[mark+4]), &(img.rows), 4);
	memcpy(&(data[mark+8]), img.step.p, 4);
	memcpy(&(data[mark+12]), &type, 4);

	mark += 16;

	memcpy(&(data[headerLength]), img.data, img.step[0] * img.rows);

	if (!img2.empty())
	{
		memcpy(&(data[mark]), img2.step.p, 4);

		type = img2.type();
		memcpy(&(data[mark+4]), &type, 4);

		memcpy(&(data[headerLength + img.step[0] * img.rows]), img2.data,
			   img2.step[0] * img2.rows);
	}

	shm.writeDataPacket(data);

	return true;
}
