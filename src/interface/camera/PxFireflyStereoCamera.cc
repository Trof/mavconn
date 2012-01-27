#include "PxFireflyStereoCamera.h"

PxFireflyStereoCamera::PxFireflyStereoCamera(dc1394camera_t* _cameraLeft, dc1394camera_t* _cameraRight)
 : externalTrigger(false)
{
	cameraLeft = std::tr1::shared_ptr<PxFireflyCamera>(new PxFireflyCamera(_cameraLeft));
	cameraRight = std::tr1::shared_ptr<PxFireflyCamera>(new PxFireflyCamera(_cameraRight));
}

PxFireflyStereoCamera::~PxFireflyStereoCamera()
{

}

bool
PxFireflyStereoCamera::init(void)
{
	if (!cameraLeft->init())
	{
		return false;
	}
	if (!cameraRight->init())
	{
		return false;
	}

//	fprintf(stderr, "# INFO: Setting up left camera sending strobe\n");
//
//	fprintf(stderr, "# Activating strobe on GPIO_2\n");
//	if (!cameraLeft->setStrobe(2))
//	{
//		destroy();
//		return false;
//	}

	return true;
}

void
PxFireflyStereoCamera::destroy(void)
{
	cameraLeft->destroy();
	cameraRight->destroy();
}

bool
PxFireflyStereoCamera::setConfig(const PxCameraConfig& config)
{
	if (!cameraLeft->setConfig(config))
	{
		return false;
	}
//	//the right camera has always to listen to trigger (we wan't to force the sync cable)
//	PxCameraConfig right_config(config);
//	right_config.setExternalTrigger(true);

	if (!cameraRight->setConfig(config))
	{
		return false;
	}

	externalTrigger = config.getExternalTrigger();

	return true;
}

bool
PxFireflyStereoCamera::start(void)
{
	//start transmission - always start slave camera first
	fprintf(stderr, "# INFO: Start transmission\n");
	if (!cameraRight->start())
	{
		fprintf(stderr, "# ERROR: Error starting transmission for slave camera.\n");
		return false;
	}
	if (!cameraLeft->start())
	{
		fprintf(stderr, "# ERROR: Error starting transmission for master camera.\n");
		return false;
	}

	usleep(250000);

	return true;
}

bool
PxFireflyStereoCamera::stop(void)
{
	if (!cameraLeft->stop())
	{
		return false;
	}
	if (!cameraRight->stop())
	{
		return false;
	}

	return true;
}

bool
PxFireflyStereoCamera::grabFrame(cv::Mat& imageLeft, cv::Mat& imageRight,
								 uint32_t& skippedFrames, uint32_t& sequenceNum)
{
	dc1394video_frame_t *frameLeft = 0, *frameRight = 0;

	dc1394error_t error = dc1394_capture_dequeue(cameraLeft->camera, DC1394_CAPTURE_POLICY_WAIT, &frameLeft);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error dequeuing left frame: %s\n", dc1394_error_get_string(error));
		return false;
	}
	error = dc1394_capture_dequeue(cameraRight->camera, DC1394_CAPTURE_POLICY_WAIT, &frameRight);
	if (error != DC1394_SUCCESS)
	{
		error = dc1394_capture_enqueue(cameraLeft->camera, frameLeft); //enqueue the dequeued frame of left camera - perhaps we can continue...
		if (error != DC1394_SUCCESS)
		{
			fprintf(stderr, "# ERROR: Error enqueuing left frame after dequeue error, now we're totally screwed, sorry: %s\n", dc1394_error_get_string(error));
			return false;
		}
		else
		{
			fprintf(stderr, "# ERROR: Error dequeuing right frame: %s\n", dc1394_error_get_string(error));
			return false;
		}
	}
	//now check the field frames_behind to see if there are newer images in the buffer and skip to the newest image pair
	uint32_t skipped_frames1 = 0;
	uint32_t skipped_frames2 = 0;

	if (externalTrigger)
	{
		while (std::min(frameLeft->frames_behind, frameRight->frames_behind) > 0)
		{
			error = dc1394_capture_enqueue(cameraLeft->camera, frameLeft);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error enqueuing left frame.\n");
				return false;
			}
			error = dc1394_capture_dequeue(cameraLeft->camera, DC1394_CAPTURE_POLICY_POLL, &frameLeft);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error dequeuing left frame.\n");
				return false;
			}
			error = dc1394_capture_enqueue(cameraRight->camera, frameRight);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error enqueuing right frame.\n");
				return false;
			}
			error = dc1394_capture_dequeue(cameraRight->camera, DC1394_CAPTURE_POLICY_POLL, &frameRight);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error dequeuing right frame.\n");
				return false;
			}

			skipped_frames1++;
			skipped_frames2++;
		}
	}
	else
	{
		while (frameLeft->frames_behind > 0)
		{
			error = dc1394_capture_enqueue(cameraLeft->camera, frameLeft);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error enqueuing left frame.\n");
				return false;
			}
			error = dc1394_capture_dequeue(cameraLeft->camera, DC1394_CAPTURE_POLICY_POLL, &frameLeft);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error dequeuing left frame.\n");
				return false;
			}
			skipped_frames1++;
		}
		while (frameRight->frames_behind > 0)
		{
			error = dc1394_capture_enqueue(cameraRight->camera, frameRight);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error enqueuing right frame.\n");
				return false;
			}
			error = dc1394_capture_dequeue(cameraRight->camera, DC1394_CAPTURE_POLICY_POLL, &frameRight);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error dequeuing right frame.\n");
				return false;
			}
			skipped_frames2++;
		}
	}

	skippedFrames = skipped_frames1;

	sequenceNum = frameLeft->image[0] << 24 | frameLeft->image[1] << 16 | frameLeft->image[2] << 8 | frameLeft->image[3];

	if (!cameraLeft->convertToCvMat(frameLeft, imageLeft))
	{
		fprintf(stderr, "# ERROR: Error converting left frame to cv::Mat.\n");
		return false;
	}
	if (!cameraRight->convertToCvMat(frameRight, imageRight))
	{
		fprintf(stderr, "# ERROR: Error converting right frame to cv::Mat.\n");
		return false;
	}

	//now enqueue the processed images
	error = dc1394_capture_enqueue(cameraLeft->camera, frameLeft);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error enqueuing the left frame.\n");
		return false;
	}
	error = dc1394_capture_enqueue(cameraRight->camera, frameRight);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error enqueuing the right frame.\n");
		return false;
	}

	return true;
}
