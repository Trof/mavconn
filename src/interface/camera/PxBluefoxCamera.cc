#include "PxBluefoxCamera.h"

#include <sys/time.h>

PxBluefoxCamera::PxBluefoxCamera(mvIMPACT::acquire::Device* _dev)
 : dev(_dev)
 , imageThread(0)
 , lastSequenceNum(0)
{
	serialNum = atoi(_dev->serial.read().c_str());
}

bool
PxBluefoxCamera::init(void)
{
	try
	{
		dev->open();
	}
	catch (const mvIMPACT::acquire::ImpactAcquireException& e)
	{
		// this e.g. might happen if the same device is already opened in another process...
		fprintf(stderr, "# ERROR: Cannot open Bluefox camera (#%lu). "
						"Error code: %d (%s)\n",
						serialNum, e.getErrorCode(),
						e.getErrorCodeAsString().c_str());
		return false;
	}

	functionInterface.reset(new mvIMPACT::acquire::FunctionInterface(dev));
	cameraSettings.reset(new mvIMPACT::acquire::CameraSettingsBlueFOX(dev));
	io.reset(new mvIMPACT::acquire::IOSubSystemBlueFOX(dev));
	stats.reset(new mvIMPACT::acquire::Statistics(dev));

	try
	{
		typedef std::vector<std::pair<std::string, mvIMPACT::acquire::TCameraPixelClock> > dict_type;
		dict_type dict;

		cameraSettings->pixelClock_KHz.getTranslationDict(dict);

		int pixelclock = 0; // in KHz
		for (dict_type::const_iterator i = dict.begin(); i != dict.end(); ++i)
		{
			if (i->second > pixelclock)
			{
				pixelclock = i->second;
			}
		}

		int difference = std::numeric_limits<int>::max();

		dict_type::const_iterator it;
		for (dict_type::const_iterator i = dict.begin(); i != dict.end(); ++i)
		{
			if (std::abs(i->second - pixelclock) < difference)
			{
				difference = std::abs(i->second - pixelclock);
				it = i;
			}
		}
		cameraSettings->pixelClock_KHz.writeS(it->first);
	}
	catch (const mvIMPACT::acquire::ImpactAcquireException& e)
	{
		fprintf(stderr, "# ERROR: Cannot set camera pixel clock. "
						"Error code: %d (%s)\n",
						e.getErrorCode(),
						e.getErrorCodeAsString().c_str());
		return false;
	}

	imageAvailable = false;

	return true;
}

void
PxBluefoxCamera::destroy(void)
{
	if (dev)
	{
		if (dev->isOpen())
		{
			dev->close();
		}
	}
}

bool
PxBluefoxCamera::setConfig(const PxCameraConfig& config)
{
	if (!setMode(config.getMode()))
	{
		return false;
	}
	frameRate = config.getFrameRate();
	timeout_ms = 1.0f / frameRate * 4000.0f;
	if (!setFrameRate(frameRate))
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

	if (config.getMode() != PxCameraConfig::AUTO_MODE)
	{
		if (!setExposureTime(config.getExposureTime()))
		{
			return false;
		}
		if (!setGain(config.getGain()))
		{
			return false;
		}
	}

	return true;
}

bool
PxBluefoxCamera::start(void)
{
	exitImageThread = false;
	imageAvailableCond.reset(new Glib::Cond);

	try
	{
		imageThread = Glib::Thread::create(sigc::mem_fun(this, &PxBluefoxCamera::imageHandler), true);
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
	double timeout = 0.5;

	while (!imageAvailable && currentTime - startTime < timeout)
	{
		gettimeofday(&tv, NULL);
		currentTime = tv.tv_sec + static_cast<double>(tv.tv_usec) / 1000000.0;
	}

	return true;
}

bool
PxBluefoxCamera::stop(void)
{
	exitImageThread = true;

	imageThread->join();

	return true;
}

bool
PxBluefoxCamera::grabFrame(cv::Mat& image, uint32_t& skippedFrames,
						   uint32_t& sequenceNum)
{
	imageMutex.lock();
	imageAvailableCond->wait(imageMutex);

	if (imageAvailable)
	{
		this->image.copyTo(image);
		sequenceNum = imageSequenceNr;

		imageAvailable = false;
		imageMutex.unlock();

		if (sequenceNum > lastSequenceNum)
		{
			skippedFrames = sequenceNum - lastSequenceNum - 1;

			lastSequenceNum = sequenceNum;
		}

		return true;
	}
	else
	{
		imageMutex.unlock();

		return false;
	}
}

bool
PxBluefoxCamera::setExternalTrigger(void)
{
	mvIMPACT::acquire::CameraSettingsBlueFOX triggerSettings(dev);

	// use hardware real-time controller to define image frequency
	triggerSettings.triggerSource.write(mvIMPACT::acquire::ctsRTCtrl);

	// ctmOnRisingEdge is not available with the CMOS sensor.
	// ctmOnHighLevel is the next best alternative.
	triggerSettings.triggerMode.write(mvIMPACT::acquire::ctmOnHighLevel);

	if (io->RTCtrProgramCount() == 0)
	{
		fprintf(stderr, "# ERROR: setExternalTrigger() only works if a HRTC controller is available.\n");
		return false;
	}

	// write program to wait for input to reach logical value 1, and trigger immediately
	// 0. WaitDigin On
	// 1. TriggerSet 1
	// 2. WaitClocks( <trigger pulse width in us> )
	// 3. TriggerReset
	// 4. Jump 0
	mvIMPACT::acquire::RTCtrProgram* program = io->getRTCtrProgram(0);
	if (!program)
	{
		// this should only happen if the system is short of memory
		return false;
	}

	program->mode.write(mvIMPACT::acquire::rtctrlModeStop);
	program->setProgramSize(5);

	int i = 0;
	mvIMPACT::acquire::RTCtrProgramStep* step =	program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitDigin);
	step->digitalInputs.write(mvIMPACT::acquire::digioOn);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerSet);
	step->frameID.write(1);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitClocks);
	step->clocks_us.write(100);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerReset);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgJumpLoc);

	program->mode.write(mvIMPACT::acquire::rtctrlModeRun);

	return true;
}

bool
PxBluefoxCamera::setFrameRate(float frameRate)
{
	// frame rate may fall below specified frame rate if scene requires long
	// exposure time, etc.

	mvIMPACT::acquire::CameraSettingsBlueFOX triggerSettings(dev);

	// use hardware real-time controller to define image frequency
	triggerSettings.triggerSource.write(mvIMPACT::acquire::ctsRTCtrl);

	// ctmOnRisingEdge is not available with the CMOS sensor.
	// ctmOnHighLevel is the next best alternative.
	triggerSettings.triggerMode.write(mvIMPACT::acquire::ctmOnHighLevel);

	if (io->RTCtrProgramCount() == 0)
	{
		fprintf(stderr, "# ERROR: setFrameRate() only works if a HRTC controller is available.\n");
		return false;
	}

	// write program to achieve image frequency
	// 0. WaitClocks( <frame time in us> - <trigger pulse width in us>) )
	// 1. TriggerSet 1
	// 2. WaitClocks( <trigger pulse width in us> )
	// 3. TriggerReset
	// 4. Jump 0
	mvIMPACT::acquire::RTCtrProgram* program = io->getRTCtrProgram(0);
	if (!program)
	{
		// this should only happen if the system is short of memory
		return false;
	}

	program->mode.write(mvIMPACT::acquire::rtctrlModeStop);
	program->setProgramSize(5);

	int i = 0;
	mvIMPACT::acquire::RTCtrProgramStep* step =	program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitClocks);
	step->clocks_us.write(static_cast<int>(1000000.0 / frameRate) - 100);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerSet);
	step->frameID.write(1);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgWaitClocks);
	step->clocks_us.write(100);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgTriggerReset);

	step = program->programStep(i++);
	step->opCode.write(mvIMPACT::acquire::rtctrlProgJumpLoc);

	program->mode.write(mvIMPACT::acquire::rtctrlModeRun);

	return true;
}

bool
PxBluefoxCamera::setMode(PxCameraConfig::Mode mode)
{
	try
	{
		if (mode == PxCameraConfig::MANUAL_MODE)
		{
			// turn off automatic gain control
			cameraSettings->autoGainControl.write(mvIMPACT::acquire::agcOff);

			// turn off automatic exposure control
			cameraSettings->autoExposeControl.write(mvIMPACT::acquire::aecOff);
		}
		else if (mode == PxCameraConfig::AUTO_MODE)
		{
			// configure auto control settings
			if (cameraSettings->autoControlParameters.isAvailable())
			{
				cameraSettings->autoControlParameters.controllerSpeed.write(mvIMPACT::acquire::acsUserDefined);
				cameraSettings->autoControlParameters.controllerGain.write(0.2);
				cameraSettings->autoControlParameters.controllerIntegralTime_ms.write(1000.0);
				cameraSettings->autoControlParameters.controllerDerivativeTime_ms.write(0.0);
				cameraSettings->autoControlParameters.controllerDelay_Images.write(0);
			}

			cameraSettings->autoControlMode.write(mvIMPACT::acquire::acmStandard);

			// turn on automatic gain control
			cameraSettings->autoGainControl.write(mvIMPACT::acquire::agcOn);

			// turn on automatic exposure control
			cameraSettings->autoExposeControl.write(mvIMPACT::acquire::aecOn);
		}
		else
		{
			fprintf(stderr, "# WARNING: Unknown camera mode\n");
			return false;
		}
	}
	catch (const mvIMPACT::acquire::ImpactAcquireException& e)
	{
		fprintf(stderr, "# ERROR: Cannot access camera settings. "
						"Error code: %d (%s)\n",
						e.getErrorCode(),
						e.getErrorCodeAsString().c_str());
		return false;
	}

	return true;
}

bool
PxBluefoxCamera::setExposureTime(uint32_t exposureTime)
{
	// set exposure time (microseconds)
	cameraSettings->expose_us.write(exposureTime);

	return true;
}

bool
PxBluefoxCamera::setGain(uint32_t gain)
{
	double gain_dB = 10.0 * log10(gain);

	return setGainDB(gain_dB);
}

bool
PxBluefoxCamera::setGainDB(float gain_dB)
{
	double minGain_dB = cameraSettings->gain_dB.read(mvIMPACT::acquire::PROP_MIN_VAL);
	if (gain_dB < minGain_dB)
	{
		fprintf(stderr, "# WARNING: Gain (%.2f dB) is too low. "
						"Setting gain to minimum allowable value (%.2f dB).\n",
						gain_dB, minGain_dB);
		gain_dB = minGain_dB;
	}

	double maxGain_dB = cameraSettings->gain_dB.read(mvIMPACT::acquire::PROP_MAX_VAL);
	if (gain_dB > maxGain_dB)
	{
		fprintf(stderr, "# WARNING: Gain (%.2f dB) is too high. "
						"Setting gain to maximum allowable value (%.2f dB).\n",
						gain_dB, maxGain_dB);
		gain_dB = maxGain_dB;
	}

	cameraSettings->gain_dB.write(gain_dB);

	return true;
}

float
PxBluefoxCamera::getFramesPerSecond(void)
{
	return stats->framesPerSecond.read();
}

void
PxBluefoxCamera::imageHandler(void)
{
	int maxRequests = 4;

	mvIMPACT::acquire::SystemSettings systemSettings(dev);
	systemSettings.requestCount.write(maxRequests);

	for (int i = 0; i < maxRequests; ++i)
	{
		functionInterface->imageRequestSingle();
	}

	while (!exitImageThread)
	{
		int requestNr = functionInterface->imageRequestWaitFor(timeout_ms);

		if (functionInterface->isRequestNrValid(requestNr))
		{
			const mvIMPACT::acquire::Request* request = functionInterface->getRequest(requestNr);
			if (functionInterface->isRequestOK(request))
			{
				convertToCvMat(request, image);

				imageSequenceNr = request->infoFrameNr.read();

				imageMutex.lock();
				imageAvailable = true;
				imageAvailableCond->signal();
				imageMutex.unlock();
			}

			functionInterface->imageRequestUnlock(requestNr);
			functionInterface->imageRequestSingle();
		}

		Glib::Thread::yield();
	}

	functionInterface->imageRequestReset(0, 0);
}

bool
PxBluefoxCamera::convertToCvMat(const mvIMPACT::acquire::Request* request, cv::Mat& image)
{
	cv::Mat temp(cv::Size(request->imageWidth.read(), request->imageHeight.read()),
			     CV_8UC1, request->imageData.read(), request->imageLinePitch.read());

	//if the given image has not the same format release its data and allocate new one
	temp.copyTo(image);

	return true;
}
