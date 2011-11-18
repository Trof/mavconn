#include "Camera.h"

namespace px
{

double timeInSeconds(void)
{
	 struct timeval tv;

	 gettimeofday(&tv, 0);

	 return static_cast<double>(tv.tv_sec) +
			 static_cast<double>(tv.tv_usec) / 1000000.0;
}

Camera::Camera(bool stereo, const std::string& orientation)
 : mUseStereo(stereo)
 , mOrientation(orientation)
 , mThreadLCM(0)
{

}

bool
Camera::start(void)
{
	// initialize shared memory client
	if (mOrientation.compare("forward") == 0)
	{
		if (mUseStereo)
		{
			mSHMClient.init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
		}
		else
		{
			mSHMClient.init(true, PxSHM::CAMERA_FORWARD_LEFT);
		}
	}
	else if (mOrientation.compare("downward") == 0)
	{
		if (mUseStereo)
		{
			mSHMClient.init(true, PxSHM::CAMERA_DOWNWARD_LEFT, PxSHM::CAMERA_DOWNWARD_RIGHT);
		}
		else
		{
			mSHMClient.init(true, PxSHM::CAMERA_DOWNWARD_LEFT);
		}
	}
	else
	{
		fprintf(stderr, "# ERROR: Unknown camera orientation %s\n", mOrientation.c_str());
		return false;
	}

	mImageAvailable = false;

	lcm_t* lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		return false;
	}

	mThreadLCM = Glib::Thread::create(sigc::bind(sigc::mem_fun(this, &Camera::lcmHandler), lcm), true);

	usleep(1000000);

	return true;
}

bool
Camera::grabFrame(cv::Mat& image)
{
	assert(mUseStereo == false);

	mImageMutex.lock();

	double startTime = timeInSeconds();
	while (!mImageAvailable && timeInSeconds() - startTime < 1.0)
	{
		mImageCond.timed_wait(mImageMutex, Glib::TimeVal(0, 100000));
	}

	mImage.copyTo(image);

	mImageAvailable = false;

	mImageMutex.unlock();

	return true;
}

bool
Camera::grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight)
{
	assert(mUseStereo);

	mImageMutex.lock();

	double startTime = timeInSeconds();
	while (!mImageAvailable && timeInSeconds() - startTime < 1.0)
	{
		mImageCond.timed_wait(mImageMutex, Glib::TimeVal(0, 100000));
	}

	mImage.copyTo(imageLeft);
	mImageRight.copyTo(imageRight);

	mImageAvailable = false;

	mImageMutex.unlock();

	return true;
}

void
Camera::lcmHandler(lcm_t* lcm)
{
	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES, &imageHandler, this);

	while (1)
	{
		lcm_handle(lcm);
	}
}

void
Camera::imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
					 const mavconn_mavlink_msg_container_t* container, void* user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	Camera* cam = reinterpret_cast<Camera*>(user);

	cam->mImageMutex.lock();

	if (cam->mUseStereo)
	{
		cam->mSHMClient.readStereoImage(msg, cam->mImage, cam->mImageRight);
	}
	else
	{
		cam->mSHMClient.readMonoImage(msg, cam->mImage);
	}

	cam->mImageAvailable = true;
	cam->mImageCond.signal();

	cam->mImageMutex.unlock();
}

}
