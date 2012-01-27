/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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
 *   @brief Definition of Point Grey Firefly MV interface
 *
 *   @author Pascal Dufour <mavteam@student.ethz.ch>
 *   @author Lionel Heng <hengli@inf.ethz.ch>
 *
 */

#include "PxFireflyCamera.h"

const int kDMAbuffers = 10;

typedef union _fint32
{
	uint32_t reg;
	float freg;
} fint32_t;

PxFireflyCamera::PxFireflyCamera(dc1394camera_t* _camera)
 : camera(_camera)
{

}

PxFireflyCamera::~PxFireflyCamera()
{

}

bool
PxFireflyCamera::init(void)
{
	//reset camera
	if (verbose)
	{
		fprintf(stderr, "# INFO: Reset camera\n");
	}
	dc1394error_t error = dc1394_camera_reset(camera);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not reset camera 1\n");
		destroy();
		return false;
	}
	usleep(250000);

	//reset bus - this is very rude and may disrupt other cameras from working
	if (verbose)
	{
		fprintf(stderr, "# INFO: Reset bus\n");
	}
	error = dc1394_reset_bus(camera);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not reset bus of camera.\n");
		destroy();
		return false;
	}
	usleep(250000);

	//set video mode to 640x480 Mono 8
	if (verbose)
	{
		fprintf(stderr, "# INFO: Set video mode to 640x480x8\n");
	}
	error = dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_640x480_MONO8);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set video mode for camera.\n");
		destroy();
		return false;
	}

	//set capture flags
	if (verbose)
	{
		fprintf(stderr, "# INFO: Set DMA buffer size to %d images\n", kDMAbuffers);
	}
	error = dc1394_capture_setup(camera, kDMAbuffers, DC1394_CAPTURE_FLAGS_DEFAULT);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not setup flags for camera.\n");
		destroy();
		return false;
	}

	//turn on the embedding of the timestamp in each frame
	//set 0x12F8 FRAME_INFO register (bit 31 to 1)
	if (verbose)
	{
		fprintf(stderr, "# INFO: Set embedded frame counter\n");
	}
	error = dc1394_set_control_register(camera, 0x12f8, 0x80000040);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: could not write to control register 0x12f8 of camera.\n");
		destroy();
		return false;
	}

	//read the absolute shutter value
	fint32_t fancyReg;
	error = dc1394_get_control_register(camera, 0x918, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read absolute shutter time of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Shutter time: %f ms\n", fancyReg.freg * 1000.f);
	}

	error = dc1394_get_control_register(camera, 0x928, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read absolute brightness of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Gain: %f dB\n", fancyReg.freg);
	}

	error = dc1394_get_control_register(camera, 0x938, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read absolute gain of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Brightness: %f%%\n", fancyReg.freg);
	}

	error = dc1394_get_control_register(camera, 0x948, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read absolute gamma of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Gamma: %f\n", fancyReg.freg);
	}

	//debugging information
	error = dc1394_get_control_register(camera, 0x1028, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read extended shutter register of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Extended shutter available: %s\n", (fancyReg.reg&1)?"yes":"no");
	}

	error = dc1394_get_control_register(camera, 0x1104, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read gpio extra register of camera.\n");
		destroy();
		return false;
	}
	else
	{
		error = dc1394_set_control_register(camera, 0x1104, fancyReg.reg | 0x2);
		if (error != DC1394_SUCCESS)
		{
			fprintf(stderr, "# ERROR: Could not write gpio extra register of camera.\n");
			destroy();
			return false;
		}
	}
	error = dc1394_get_control_register(camera, 0x1104, &fancyReg.reg);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read gpio extra register of camera.\n");
		destroy();
		return false;
	}
	if (verbose)
	{
		fprintf(stderr, "# INFO: Trigger is %s if arrived during exposure\n", (fancyReg.reg&2)?"dropped":"queued");
	}

	// clear the image buffer
	dc1394video_frame_t *frame1;
	error = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame1);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error dequeuing frame: %s\n", dc1394_error_get_string(error));
		destroy();
		return false;
	}
	if (frame1)
	{
		uint32_t i = 1;
		if (verbose)
		{
			fprintf(stderr, "# INFO: %u.. ", i++);
			fflush(stdout);
		}
		uint32_t frames_to_skip = frame1->frames_behind;
		while (frames_to_skip > 0)
		{
			error = dc1394_capture_enqueue(camera, frame1);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error enqueuing frame.\n");
				destroy();
				return false;
			}
			error = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame1);
			if (error != DC1394_SUCCESS)
			{
				fprintf(stderr, "# ERROR: Error dequeuing frame.\n");
				destroy();
				return false;
			}
			if (verbose)
			{
				fprintf(stderr, "# INFO: %u.. ", i++); fflush(stdout);
			}
			frames_to_skip = frame1->frames_behind;
		}
		error = dc1394_capture_enqueue(camera, frame1);
		if (error != DC1394_SUCCESS)
		{
			fprintf(stderr, "# ERROR: Error enqueuing frame\n");
			return false;
		}
		if (verbose)
		{
			fprintf(stderr, "\n");
		}
	}

	if (verbose)
	{
		fprintf(stderr, "# INFO: Camera initialized successfully!\n");
	}

	return true;
}

void
PxFireflyCamera::destroy(void)
{
	if (verbose)
	{
		fprintf(stderr, "# INFO: Reset camera and free resources\n");
	}
	if (camera != NULL)
	{
		dc1394_camera_reset(camera);
		dc1394_camera_free(camera);
		camera = NULL;
	}
}

bool
PxFireflyCamera::setConfig(const PxCameraConfig& config)
{
	if (!setExposureTime(config.getExposureTime()))
	{
		return false;
	}
	if (!setGain(config.getGain()))
	{
		return false;
	}
	if (!setMode(config.getMode()))
	{
		return false;
	}
	if (!setFrameRate(config.getFrameRate()))
	{
		return false;
	}
	if (config.getExternalTrigger())
	{
		if (!setExternalTrigger())
		{
			return false;
		}
	}
	if (!setGamma(config.getGamma()))
	{
		return false;
	}
	if (!setBrightness(config.getBrightness()))
	{
		return false;
	}

	return true;
}

bool
PxFireflyCamera::start(void)
{
	//start transmission
	if (verbose)
	{
		fprintf(stderr, "# INFO: Start transmission.\n");
	}

	if (dc1394_video_set_transmission(camera, DC1394_ON) != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error starting transmission.\n");
		return false;
	}

	usleep(250000);

	return true;
}

bool
PxFireflyCamera::stop(void)
{
	return true;
}

bool
PxFireflyCamera::grabFrame(cv::Mat& image, uint32_t& skippedFrames,
						   uint32_t& sequenceNum)
{
	dc1394video_frame_t *frame;

	dc1394error_t error = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error dequeuing frame: %s\n", dc1394_error_get_string(error));
		return false;
	}

	//now check the field frames_behind to see if there are newer images in the buffer and skip to the newest image
	uint32_t skipped_frames1 = 0;
	while (frame->frames_behind > 0)
	{
		error = dc1394_capture_enqueue(camera, frame);
		if (error != DC1394_SUCCESS)
		{
			fprintf(stderr, "# ERROR: Error enqueuing frame\n");
			return false;
		}
		error = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame);
		if (error != DC1394_SUCCESS)
		{
			fprintf(stderr, "# ERROR: Error dequeuing frame\n");
			return false;
		}

		skipped_frames1++;
	}

	skippedFrames = skipped_frames1;

	sequenceNum = frame->image[0] << 24 | frame->image[1] << 16 | frame->image[2] << 8 | frame->image[3];

	//convert to cv::Mat
	if (!convertToCvMat(frame, image))
	{
		fprintf(stderr, "# ERROR: Error converting frame to cv::Mat.\n");
		return false;
	}

	//now enqueue the processed images
	error = dc1394_capture_enqueue(camera, frame);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Error enqueuing the frame.\n");
		return false;
	}

	return true;
}

bool
PxFireflyCamera::setExternalTrigger(void)
{
	uint32_t desiredPinDirection[4] = {0, 1, 1, 0};

	// check, and if necessary, correct pin direction settings
	// GPIO0 and GPIO3 should be set as inputs while GPIO1 and GPIO2 should be set as outputs
	for (int i = 0; i < 4; ++i)
	{
		uint32_t pinDirection = 0;
		if (getGPIOPinDirection(i, pinDirection))
		{
			if (pinDirection != desiredPinDirection[i])
			{
				if (!setGPIOPinDirection(i, desiredPinDirection[i]))
				{
					return false;
				}
			}
		}
		else
		{
			return false;
		}
	}

	if (verbose)
	{
		fprintf(stderr, "# INFO: Activating external trigger on GPIO_0 for camera.\n");
	}
	dc1394error_t error = dc1394_external_trigger_set_mode(camera, DC1394_TRIGGER_MODE_0);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set trigger for camera.\n");
		return false;
	}
	error = dc1394_external_trigger_set_source(camera, DC1394_TRIGGER_SOURCE_0);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set trigger for camera.\n");
		return false;
	}
	error = dc1394_external_trigger_set_polarity(camera, DC1394_TRIGGER_ACTIVE_LOW);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set trigger for camera.\n");
		return false;
	}
	error = dc1394_external_trigger_set_power(camera, DC1394_ON);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set trigger for camera.\n");
		return false;
	}

	// recheck that the pin directions are set correctly
	for (int i = 0; i < 4; ++i)
	{
		uint32_t pinDirection = 0;
		if (getGPIOPinDirection(i, pinDirection) == true)
		{
			if (pinDirection != desiredPinDirection[i])
			{
				if (setGPIOPinDirection(i, desiredPinDirection[i]) != true)
				{
					return false;
				}
			}
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool
PxFireflyCamera::setStrobe(uint32_t pin)
{
	bool flag = false;
	if (!isStrobeEnabled(pin, flag))
	{
		fprintf(stderr, "# ERROR: Could not read strobe control inquiry register of camera.\n");
		return false;
	}
	if (!flag)
	{
		fprintf(stderr, "# ERROR: Strobe feature not available on GPIO%d on camera.\n", pin);
		return false;
	}

	if (!isStrobeConfigurable(pin, flag))
	{
		fprintf(stderr, "# ERROR: Could not read strobe control inquiry register of camera.\n");
		return false;
	}
	if (!flag)
	{
		fprintf(stderr, "# ERROR: Strobe feature for GPIO%d not configurable on camera.\n", pin);
		return false;
	}

	if (!setStrobeSource(pin))
	{
		fprintf(stderr, "# ERROR: Could not set strobe for GPIO%d of camera.\n", pin);
		return false;
	}

	return true;
}

bool
PxFireflyCamera::setFrameRate(float frameRate)
{
	dc1394error_t error;
	if (frameRate == 7.5f)
	{
		error = dc1394_video_set_framerate(camera, DC1394_FRAMERATE_7_5);
	}
	else if (frameRate == 15.0f)
	{
		error = dc1394_video_set_framerate(camera, DC1394_FRAMERATE_15);
	}
	else if (frameRate == 30.0f)
	{
		error = dc1394_video_set_framerate(camera, DC1394_FRAMERATE_30);
	}
	else if (frameRate == 60.0f)
	{
		error = dc1394_video_set_framerate(camera, DC1394_FRAMERATE_60);
	}
	else
	{
		fprintf(stderr, "# ERROR: Unknown frame rate: %.2f fps.\n", frameRate);
		return false;
	}

	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set frame rate (%.2f fps) for camera.\n", frameRate);
		return false;
	}

	return true;
}

bool
PxFireflyCamera::setMode(PxCameraConfig::Mode mode)
{
	dc1394error_t error;

	if (mode == PxCameraConfig::MANUAL_MODE)
	{
		//set manual brightness
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable manual brightness feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn off auto brightness mode\n");
			return false;
		}

		//set manual shutter
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_SHUTTER, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable manual shutter feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn off auto shutter\n");
			return false;
		}

		//set manual gain
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_GAIN, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable manual gain feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn off auto gain\n");
			return false;
		}

		//set manual gamma
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable manual gamma feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn off auto gamma\n");
			return false;
		}
	}
	else if (mode == PxCameraConfig::AUTO_MODE)
	{
		//set auto brightness
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable auto brightness feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn on auto brightness mode\n");
			return false;
		}

		//set auto shutter
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_SHUTTER, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable auto shutter feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn on auto shutter\n");
			return false;
		}

		//set auto gain
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_GAIN, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable auto gain feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn on auto gain\n");
			return false;
		}

		//set auto gamma
		error = dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_ON);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not enable auto gamma feature\n");
			return false;
		}
		error = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_AUTO);
		if (error != DC1394_SUCCESS)
		{
			printf("Could not turn on auto gamma\n");
			return false;
		}
	}
	else
	{
		printf("Unknown camera mode\n");
		return false;
	}

	return true;
}

bool
PxFireflyCamera::setExposureTime(uint32_t exposureTime)
{
	// shutter is given in microseconds, compute the correct register value
	// (the constant factor can change with different camera models, this one is correct for PTGrey Firefly MV)
//	if (special)
//	{
//		shutter *= 0.004096;
//	}
//	else
//	{
		exposureTime *= 0.032768;
//	}

	dc1394error_t error = dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER, exposureTime);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set exposure time.\n");
		return false;
	}
	return true;
}

bool
PxFireflyCamera::setGain(uint32_t gain)
{
	dc1394error_t error = dc1394_feature_set_value(camera, DC1394_FEATURE_GAIN, gain);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set gain.\n");
		return false;
	}
	return true;
}

bool
PxFireflyCamera::setGamma(uint32_t gamma)
{
	dc1394error_t error = dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA, gamma);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set gamma.\n");
		return false;
	}
	return true;
}

bool
PxFireflyCamera::setBrightness(uint32_t brightness)
{
	dc1394error_t error = dc1394_feature_set_value(camera, DC1394_FEATURE_BRIGHTNESS, brightness);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not set brightness.\n");
		return false;
	}
	return true;
}

bool
PxFireflyCamera::getGPIOPinDirection(uint32_t pin, uint32_t& pDirection)
{
	uint32_t val = 0;
	dc1394error_t error = dc1394_get_control_register(camera, (uint64_t) 0x11F8, &val);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read pin direction register of camera.\n");
		return false;
	}

	pDirection = (val >> (31 - pin)) & 0x1;

	return true;
}

bool
PxFireflyCamera::setGPIOPinDirection(uint32_t pin, uint32_t pDirection)
{
	uint32_t val = 0;
	dc1394error_t error = dc1394_get_control_register(camera, (uint64_t) 0x11F8, &val);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not read pin direction register of camera.\n");
		return false;
	}

	// ‘0’ indicates it is a input/trigger, and ‘1’ indicates it is an output/strobe
	if (pDirection == 0)
	{
		uint32_t mask = ~(0x1 << (31 - pin));
		val &= mask;
	}
	else
	{
		uint32_t mask = 0x1 << (31 - pin);
		val |= mask;
	}

	error = dc1394_set_control_register(camera, (uint64_t) 0x11F8, val);
	if (error != DC1394_SUCCESS)
	{
		fprintf(stderr, "# ERROR: Could not write correct pin direction register of camera.\n");
		return false;
	}

	return true;
}

bool
PxFireflyCamera::isStrobeEnabled(uint32_t pin, bool& flag)
{
	uint32_t val = 0;

	dc1394error_t error = dc1394_get_control_register(camera, (uint64_t) 0x1300, &val);
	if (error != DC1394_SUCCESS)
	{
		return false;
	}

	flag = (val >> (31 - pin)) & 0x1;
	return true;
}

bool
PxFireflyCamera::isStrobeConfigurable(uint32_t pin, bool& flag)
{
	uint64_t registerOffset[4] = {0x1400, 0x1404, 0x1408, 0x140C};

	uint32_t val = 0;
	dc1394error_t error = dc1394_get_control_register(camera, registerOffset[pin], &val);
	if (error != DC1394_SUCCESS)
	{
		return false;
	}

	if ((val & 0x8E000000) != 0x8E000000)
	{
		return false;
	}

	return true;
}

bool
PxFireflyCamera::setStrobeSource(uint32_t pin)
{
	uint64_t registerOffset[4] = {0x1500, 0x1504, 0x1508, 0x150C};

	dc1394error_t error = dc1394_set_control_register(camera, registerOffset[pin], (uint32_t) 0x82000800);
	if (error != DC1394_SUCCESS)
	{
		return false;
	}

	return true;
}

bool
PxFireflyCamera::convertToCvMat(dc1394video_frame_t* frame, cv::Mat& image)
{
	if (!frame)
	{
		fprintf(stderr, "# ERROR: NULL pointer in convertToCvMat().\n");
		return false;
	}

	cv::Mat temp(cv::Size(frame->size[0], frame->size[1]), CV_8UC1, frame->image, frame->stride);

	//if the given image has not the same format release its data and allocate new one
	temp.copyTo(image);

	return true;
}
