#ifndef CAMERA_H
#define CAMERA_H

#include <glibmm.h>
#include <interface/shared_mem/PxSHMImageClient.h>

namespace px
{

class Camera
{
public:
	Camera(bool stereo, const std::string& orientation);

	bool start(void);

	bool grabFrame(cv::Mat& image);
	bool grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight);

private:
	void lcmHandler(lcm_t* lcm);

	static void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
						 const mavconn_mavlink_msg_container_t* container, void* user);

	bool mUseStereo;
	std::string mOrientation;

	Glib::Thread* mThreadLCM;
	PxSHMImageClient mSHMClient;

	cv::Mat mImage;
	cv::Mat mImageRight;

	Glib::Cond mImageCond;
	Glib::Mutex mImageMutex;
	bool mImageAvailable;
};

}

#endif
