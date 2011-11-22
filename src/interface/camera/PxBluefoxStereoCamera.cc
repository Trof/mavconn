#include "PxBluefoxStereoCamera.h"

#include <sys/time.h>

PxBluefoxStereoCamera::PxBluefoxStereoCamera(mvIMPACT::acquire::Device* _cameraLeft,
											 mvIMPACT::acquire::Device* _cameraRight)
 : lastSequenceNum(0)
 , externalTrigger(false)
{
	cameraLeft = std::tr1::shared_ptr<PxBluefoxCamera>(new PxBluefoxCamera(_cameraLeft));
	cameraRight = std::tr1::shared_ptr<PxBluefoxCamera>(new PxBluefoxCamera(_cameraRight));
}

PxBluefoxStereoCamera::~PxBluefoxStereoCamera()
{

}

bool
PxBluefoxStereoCamera::init(void)
{
	if (!cameraLeft->init())
	{
		return false;
	}
	if (!cameraRight->init())
	{
		return false;
	}

	return true;
}

void
PxBluefoxStereoCamera::destroy(void)
{
	cameraLeft->destroy();
	cameraRight->destroy();
}

bool
PxBluefoxStereoCamera::setConfig(const PxCameraConfig& config)
{
	frameRate = config.getFrameRate();
	timeout_ms = 1.0f / frameRate * 4000.0f;

	mode = config.getMode();

	if (!cameraLeft->setConfig(config))
	{
		return false;
	}
	if (!cameraRight->setConfig(config))
	{
		return false;
	}

	// manually set gain and exposure for right camera based on those with
	// which latest image from left camera was taken
	if (mode == PxCameraConfig::AUTO_MODE &&
		!cameraRight->setMode(PxCameraConfig::MANUAL_MODE))
	{
		return false;
	}

	externalTrigger = config.getExternalTrigger();

	return true;
}

bool
PxBluefoxStereoCamera::start(void)
{
	exitImageThreads = false;
	imageAvailableCond.reset(new Glib::Cond);
	imageAvailable = 0;

	try
	{
		leftImageThread = Glib::Thread::create(sigc::mem_fun(this, &PxBluefoxStereoCamera::leftImageHandler), true);
		rightImageThread = Glib::Thread::create(sigc::mem_fun(this, &PxBluefoxStereoCamera::rightImageHandler), true);
	}
	catch (const Glib::ThreadError& e)
	{
		fprintf(stderr, "# ERROR: Cannot create image handling thread.\n");
		return false;
	}

	struct timeval tv;
	gettimeofday(&tv, NULL);
	double startTime = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	double currentTime = startTime;
	double timeout = 5.0;

	while (imageAvailable != ALL_IMAGES_AVAILABLE &&
		   currentTime - startTime < timeout)
	{
		gettimeofday(&tv, NULL);
		currentTime = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	}

	return true;
}

bool
PxBluefoxStereoCamera::stop(void)
{
	exitImageThreads = true;

	leftImageThread->join();
	rightImageThread->join();

	return true;
}

bool
PxBluefoxStereoCamera::grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
								 uint32_t& skippedFrames, uint32_t& sequenceNum)
{
	imageMutex.lock();
	do
	{
		imageAvailableCond->wait(imageMutex);
	}
	while (imageAvailable != ALL_IMAGES_AVAILABLE);

//	if (cameraLeft->imageSequenceNr != cameraRight->imageSequenceNr)
//	{
//		fprintf(stderr, "# ERROR: Left and right image sequence numbers mismatch, grabbing failed.\n");
//		return false;
//	}

	cameraLeft->image.copyTo(imageLeft);
	cameraRight->image.copyTo(imageRight);
	sequenceNum = cameraLeft->imageSequenceNr;

	imageAvailable = 0;
	imageMutex.unlock();

	if (sequenceNum > lastSequenceNum)
	{
		skippedFrames = sequenceNum - lastSequenceNum - 1;
		lastSequenceNum = sequenceNum;
	}
	else
	{
		// sequence numbers wrong!
		return false;
	}

	return true;
}

void
PxBluefoxStereoCamera::leftImageHandler(void)
{
	int maxRequests = 2;

	mvIMPACT::acquire::SystemSettings systemSettings(cameraLeft->dev);
	systemSettings.requestCount.write(maxRequests);

	for (int i = 0; i < maxRequests; ++i)
	{
		cameraLeft->functionInterface->imageRequestSingle();
	}

	while (!exitImageThreads)
	{
		int requestNr = cameraLeft->functionInterface->imageRequestWaitFor(timeout_ms);

		if (cameraLeft->functionInterface->isRequestNrValid(requestNr))
		{
			const mvIMPACT::acquire::Request* request = cameraLeft->functionInterface->getRequest(requestNr);
			if (cameraLeft->functionInterface->isRequestOK(request))
			{
				imageMutex.lock();
				cameraLeft->convertToCvMat(request, cameraLeft->image);
				cameraLeft->imageSequenceNr = request->infoFrameNr.read();

				if (mode == PxCameraConfig::AUTO_MODE)
				{
					// set gain & exposure of right camera based on image
					// from left camera
					cameraRight->setExposureTime(request->infoExposeTime_us.read());
					cameraRight->setGainDB(request->infoGain_dB.read());
				}

				imageAvailable |= LEFT_IMAGE_AVAILABLE;
				imageAvailableCond->signal();
				imageMutex.unlock();
			}

			cameraLeft->functionInterface->imageRequestUnlock(requestNr);
			cameraLeft->functionInterface->imageRequestSingle();
		}

		Glib::Thread::yield();
	}

	cameraLeft->functionInterface->imageRequestReset(0, 0);
}

void
PxBluefoxStereoCamera::rightImageHandler(void)
{
	int maxRequests = 2;

	mvIMPACT::acquire::SystemSettings systemSettings(cameraRight->dev);
	systemSettings.requestCount.write(maxRequests);

	for (int i = 0; i < maxRequests; ++i)
	{
		cameraRight->functionInterface->imageRequestSingle();
	}

	while (!exitImageThreads)
	{
		int requestNr = cameraRight->functionInterface->imageRequestWaitFor(timeout_ms);

		if (cameraRight->functionInterface->isRequestNrValid(requestNr))
		{
			const mvIMPACT::acquire::Request* request = cameraRight->functionInterface->getRequest(requestNr);
			if (cameraRight->functionInterface->isRequestOK(request))
			{
				imageMutex.lock();
				cameraRight->convertToCvMat(request, cameraRight->image);
				cameraRight->imageSequenceNr = request->infoFrameNr.read();

				imageAvailable |= RIGHT_IMAGE_AVAILABLE;
				imageAvailableCond->signal();
				imageMutex.unlock();
			}

			cameraRight->functionInterface->imageRequestUnlock(requestNr);
			cameraRight->functionInterface->imageRequestSingle();
		}

		Glib::Thread::yield();
	}

	cameraRight->functionInterface->imageRequestReset(0, 0);
}
