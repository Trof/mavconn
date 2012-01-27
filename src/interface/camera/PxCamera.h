#ifndef PXCAMERA_H
#define PXCAMERA_H

#include <stdint.h>
#include <glibmm.h>
#include <opencv2/core/core.hpp>
#include <tr1/memory>

class PxCameraConfig
{
public:
	typedef enum
	{
		MANUAL_MODE = 0,
		AUTO_MODE = 1
	} Mode;

	PxCameraConfig(Mode _mode = AUTO_MODE,
				   float _frameRate = 60.0f,
				   bool _externalTrigger = false,
				   uint32_t _exposureTime = 2000,
				   uint32_t _gain = 120,
				   uint32_t _gamma = 0,
				   uint32_t _brightness = 2047)
	 : mode(_mode)
	 , frameRate(_frameRate)
	 , externalTrigger(_externalTrigger)
	 , exposureTime(_exposureTime)
	 , gain(_gain)
	 , gamma(_gamma)
	 , brightness(_brightness)
	{

	}

	Mode getMode(void) const { return mode; }
	float getFrameRate(void) const { return frameRate; }
	bool getExternalTrigger(void) const { return externalTrigger; }
	uint32_t getExposureTime(void) const { return exposureTime; }
	uint32_t getGain(void) const { return gain; }
	uint32_t getGamma(void) const { return gamma; }
	uint32_t getBrightness(void) const { return brightness; }

	void setMode(Mode _mode) { mode = _mode; }
	void setFrameRate(float fps) { frameRate = fps; }
	void setExternalTrigger(bool trigger) { externalTrigger = trigger; }
	void setExposureTime(uint32_t exposure) { exposureTime = exposure; }
	void setGain(uint32_t _gain) { gain = _gain; }
	void setGamma(uint32_t _gamma) { gamma = _gamma; }
	void setBrightness(uint32_t _brightness) { brightness = _brightness; }


private:
	Mode mode;
	float frameRate;
	bool externalTrigger;
	uint32_t exposureTime; // in microseconds
	uint32_t gain;
	uint32_t gamma;
	uint32_t brightness;
};

class PxCamera
{
public:
	PxCamera();
	virtual ~PxCamera();

	virtual bool init(void) = 0;
	virtual void destroy(void) = 0;

	virtual bool setConfig(const PxCameraConfig& config) = 0;

	virtual bool start(void) = 0;
	virtual bool stop(void) = 0;

	virtual bool grabFrame(cv::Mat& image, uint32_t& skippedFrames,
						   uint32_t& sequenceNum) = 0;

protected:
	bool verbose;

private:
	Glib::Thread* mGrabThread;

	bool mTerminateGrabThread;
};

typedef std::tr1::shared_ptr<PxCamera> PxCameraPtr;

#endif
