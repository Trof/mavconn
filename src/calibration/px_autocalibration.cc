/*======================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

Original Authors:
  @author Petri Tanskanen <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):
  @author Lionel Heng <hengli@inf.ethz.ch>

Todo:

(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

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

========================================================================*/

#include <boost/program_options.hpp>
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <tr1/memory>

#include "Calibration.h"
#include "Camera.h"
#include "Chessboard.h"

namespace config = boost::program_options;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2, BENCHMARK = 3 };

int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
double fontScale = 1.0;

double timeInSeconds(void)
{
	 struct timeval tv;

	 gettimeofday(&tv, 0);

	 return static_cast<double>(tv.tv_sec) +
			 static_cast<double>(tv.tv_usec) / 1000000.0;
}

int main(int argc, char** argv)
{
	std::string camOrientation;
	bool useStereo = false;
	uint32_t numWidth = 0;
	uint32_t numHeight = 0;
	int imageCount = 0;
	std::string cameraCalibrationFile;
	float squareSize = 0.0f;
	double delay;
	bool undistortImage = true;
	bool silent;
	bool verbose;

	//========= Handling Program options =========
	config::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("orientation", config::value<std::string>(&camOrientation)->default_value("downward"), "Orientation of camera: [downward|forward]")
		("stereo", config::bool_switch(&useStereo)->default_value(false), "Enable stereo mode")
		("width,w", config::value<uint32_t>(&numWidth)->default_value(8), "Number of corners on the chessboard pattern in x direction")
		("height,h", config::value<uint32_t>(&numHeight)->default_value(5), "Number of corners on the chessboard pattern in y direction")
		("size,s", config::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
		("delay", config::value<double>(&delay)->default_value(0.5), "Minimum delay in seconds between captured images")
		("count", config::value<int>(&imageCount)->default_value(50), "Number of images to be taken for the calibration")
		("camera-calibration,c", config::value<std::string>(&cameraCalibrationFile)->default_value("firefly_mv_"), "The output camera calibration file, without serial number and the file extension.")
		("silent,s", config::bool_switch(&silent)->default_value(false), "Surpress outputs")
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
		return 1;
	}

	//========= Initialize threading =========
	// Only initialize g thread if not already done
	if (!Glib::thread_supported())
	{
		Glib::thread_init();
	}

	//========= Initialize camera =========
	px::Camera camera(useStereo, camOrientation);
	if (!camera.start())
	{
		return 1;
	}

	fprintf(stderr, "# INFO: Grabbing one frame to get image size... ");
	fflush(stderr);

	cv::Mat frame;
	cv::Mat frameRight;
	cv::Mat mapxL, mapyL, mapxR, mapyR;

	//========= Grab first frame to get the image size =========
	if (useStereo)
	{
		if (!camera.grabFrame(frame, frameRight))
		{
			fprintf(stderr, "# ERROR: Error grabbing first frame.\n");
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		if (!camera.grabFrame(frame))
		{
			fprintf(stderr, "# ERROR: Error grabbing first frame.\n");
			exit(EXIT_FAILURE);
		}
	}

	const cv::Size frameSize = frame.size();
	printf("%ix%i\n", frameSize.width, frameSize.height);

	cv::Size boardSize(numWidth, numHeight);

	std::tr1::shared_ptr<px::Calibration> calibration(new px::Calibration(frameSize, boardSize, squareSize));
	std::tr1::shared_ptr<px::Calibration> calibrationRight;

	if (useStereo)
	{
		calibrationRight.reset(new px::Calibration(frameSize, boardSize, squareSize));
	}

	int mode = DETECTION;
	bool bLoop = true;

	double lastTime = timeInSeconds();

	while (bLoop)
	{
		bool newFrame = false;
		if (useStereo)
		{
			newFrame = camera.grabFrame(frame, frameRight);
		}
		else
		{
			newFrame = camera.grabFrame(frame);
		}
		if (!newFrame)
		{
			if (!silent)
			{
				fprintf(stderr, "# WARNING: Failed grabbing frame!\n");
			}
			continue;
		}

		if (mode == BENCHMARK)
		{
			fprintf(stderr, "# INFO: Inter-frame time: %.3f s\n", timeInSeconds() - lastTime);
			lastTime = timeInSeconds();

			cv::imshow("Capture Left", frame);
			if (useStereo)
			{
				cv::imshow("Capture Right", frameRight);
			}
		}
		else
		{
			bool addChessboardData = (timeInSeconds() - lastTime > delay);
			bool foundChessboard = false;

			cv::Mat imgSketch, imgView, imgViewRight;

			if (useStereo)
			{
				imgSketch = cv::Mat(frameSize.height, frameSize.width * 2, CV_8UC3);
				imgView = cv::Mat(imgSketch, cv::Rect(0, 0, frameSize.width, frameSize.height));
				imgViewRight = cv::Mat(imgSketch, cv::Rect(frameSize.width, 0, frameSize.width, frameSize.height));
			}
			else
			{
				imgSketch = cv::Mat(frameSize, CV_8UC3);
				imgView = imgSketch;
			}

			if (useStereo)
			{
				px::Chessboard chessboardLeft(boardSize, frame);
				px::Chessboard chessboardRight(boardSize, frameRight);

				if (mode == DETECTION ||
					(mode == CAPTURING && calibration->getSampleCount() < imageCount))
				{
					Glib::Thread* threadLeft = Glib::Thread::create(sigc::mem_fun(&chessboardLeft, &px::Chessboard::findCorners), true);
					Glib::Thread* threadRight = Glib::Thread::create(sigc::mem_fun(&chessboardRight, &px::Chessboard::findCorners), true);

					threadLeft->join();
					threadRight->join();

					foundChessboard = chessboardLeft.cornersFound() &&
									  chessboardRight.cornersFound();

					if (mode == CAPTURING && foundChessboard && addChessboardData)
					{
						calibration->addChessboardData(chessboardLeft.getCorners());
						calibrationRight->addChessboardData(chessboardRight.getCorners());
						lastTime = timeInSeconds();
					}
				}

				chessboardLeft.getSketch().copyTo(imgView);
				chessboardRight.getSketch().copyTo(imgViewRight);
			}
			else
			{
				px::Chessboard chessboard(boardSize, frame);

				if (mode == DETECTION ||
					(mode == CAPTURING && calibration->getSampleCount() < imageCount))
				{
					chessboard.findCorners();
					foundChessboard = chessboard.cornersFound();

					if (mode == CAPTURING && foundChessboard && addChessboardData)
					{
						calibration->addChessboardData(chessboard.getCorners());
						lastTime = timeInSeconds();
					}
				}

				chessboard.getSketch().copyTo(imgView);
			}

			// create visual indicator of chessboard data added to calibration
			if (mode == CAPTURING && foundChessboard && addChessboardData)
			{
				cv::bitwise_not(imgView, imgView);
				if (useStereo)
				{
					cv::bitwise_not(imgViewRight, imgViewRight);
				}
			}

			// If we have all images for calibration, run it!
			if (mode == CAPTURING && calibration->getSampleCount() == imageCount)
			{
				if (useStereo)
				{
					Glib::Thread* threadLeft = Glib::Thread::create(sigc::mem_fun(calibration.get(), &px::Calibration::calibrateCamera), true);
					Glib::Thread* threadRight = Glib::Thread::create(sigc::mem_fun(calibrationRight.get(), &px::Calibration::calibrateCamera), true);

					threadLeft->join();
					threadRight->join();

					calibration->writeParamsARTKFormat(cameraCalibrationFile + "left.cal");
					calibrationRight->writeParamsARTKFormat(cameraCalibrationFile + "right.cal");
				}
				else
				{
					calibration->calibrateCamera();
					calibration->writeParamsARTKFormat(cameraCalibrationFile + "left.cal");
				}

				mode = CALIBRATED;

				if (useStereo)
				{
					std::vector< std::vector<cv::Point3f> > objectPoints;

					for (int i = 0; i < calibration->getSampleCount(); ++i)
					{
						std::vector<cv::Point3f> objectPointsInView;
						for (int j = 0; j < boardSize.height; ++j)
						{
							for (int k = 0; k < boardSize.width; ++k)
							{
								objectPointsInView.push_back(cv::Point3f(j * squareSize, k * squareSize, 0.0));
							}
						}
						objectPoints.push_back(objectPointsInView);
					}

					cv::Mat R, T, E, F;
					cv::stereoCalibrate(objectPoints,
										calibration->getImagePoints(),
										calibrationRight->getImagePoints(),
										calibration->getCameraMatrix(),
										calibration->getDistCoeffs(),
										calibrationRight->getCameraMatrix(),
										calibrationRight->getDistCoeffs(),
										frameSize, R, T, E, F);

					cv::Mat rvec;
					cv::Rodrigues(R, rvec);

					printf("rot:\t%f\t%f\t%f\n", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
					printf("trans:\t%f\t%f\t%f\n", T.at<double>(0), T.at<double>(1), T.at<double>(2));

					//===== store stereo config file ======
					std::string str("calib_stereo.scf");
					FILE* fp = fopen(str.c_str(), "w+"); // Write to file (overwrite if it exists)
					fprintf(fp, "config/%s\n", std::string(cameraCalibrationFile + "left.cal").c_str());
					fprintf(fp, "config/%s\n", std::string(cameraCalibrationFile + "right.cal").c_str());
					fprintf(fp, "%f %f %f\n", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
					fprintf(fp, "%f %f %f", T.at<double>(0), T.at<double>(1), T.at<double>(2));
					fclose(fp);

					//===== create undistort maps ======
					cv::Mat R1, R2, P1, P2, Q;
					cv::stereoRectify(calibration->getCameraMatrix(),
									  calibration->getDistCoeffs(),
									  calibrationRight->getCameraMatrix(),
									  calibrationRight->getDistCoeffs(),
									  frameSize,
									  R, T, R1, R2, P1, P2, Q);

					cv::initUndistortRectifyMap(calibration->getCameraMatrix(),
												calibration->getDistCoeffs(),
												R1, P1,
												frameSize,
												CV_32FC1,
												mapxL, mapyL);
					cv::initUndistortRectifyMap(calibrationRight->getCameraMatrix(),
												calibrationRight->getDistCoeffs(),
												R2, P2,
												frameSize,
												CV_32FC1,
												mapxR, mapyR);
				}
			}

			if (mode == CALIBRATED && undistortImage)
			{
				if (useStereo)
				{
					cv::Mat imgDistortLeft = imgView.clone();
					cv::Mat imgDistortRight = imgViewRight.clone();

					cv::remap(imgDistortLeft, imgView, mapxL, mapyL, CV_INTER_LINEAR);
					cv::remap(imgDistortRight, imgViewRight, mapxR, mapyR, CV_INTER_LINEAR);
				}
				else
				{
					cv::Mat imgDistort = imgView.clone();

					cv::undistort(imgDistort, imgView,
								  calibration->getCameraMatrix(),
								  calibration->getDistCoeffs());
				}
			}

			char text[100];
			if (mode == CAPTURING)
			{
				if (imageCount > 0)
				{
					sprintf(text, "%d/%d", calibration->getSampleCount(), imageCount);
				}
				else
				{
					sprintf(text, "%d/?", calibration->getSampleCount());
				}
			}
			else if (mode == CALIBRATED)
			{
				sprintf(text, "Calibrated");
			}
			else
			{
				sprintf(text, "Press 'g' to start");
			}

			int baseline = 0;
			cv::Size textSize = cv::getTextSize(text,
												fontFace, fontScale, 1,
												&baseline);

			cv::Point textOrigin;
			textOrigin.x = (imgSketch.cols - textSize.width) / 2;
			textOrigin.y = imgSketch.rows - baseline - 10;

			cv::putText(imgSketch, text, textOrigin, fontFace, fontScale,
						mode != CALIBRATED ? cv::Scalar(255,0,0) : cv::Scalar(0,255,0));

			cv::imshow("Capture", imgSketch);
		}

		int key = cv::waitKey(3);
		switch((char) key)
		{
			case -1: // nothing pressed
				break;

			case 27: // Escape
				bLoop = false;
				break;

			case 'u':
				undistortImage = !undistortImage;
				break;

			case 'g':
				if (mode == DETECTION)
				{
					mode = CAPTURING;
				}
				else if (mode == CAPTURING)
				{
					mode = DETECTION;
					calibration->clear();
					if (useStereo)
					{
						calibrationRight->clear();
					}
				}
				break;

			case 'b':
				if (mode == DETECTION)
				{
					mode = BENCHMARK;
				}
				else if (mode == BENCHMARK)
				{
					mode = DETECTION;
				}
				break;

			default:
				break;
		}
	}
}
