#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/core/core.hpp>

namespace px
{

class Calibration
{
public:
	Calibration(const cv::Size& imageSize, const cv::Size& boardSize,
			    float squareSize);

	void clear(void);

	void addChessboardData(const std::vector<cv::Point2f>& corners);

	void calibrateCamera(void);

	int getSampleCount(void) const;
	const std::vector< std::vector<cv::Point2f> >& getImagePoints(void) const;
	cv::Mat& getCameraMatrix(void);
	cv::Mat& getDistCoeffs(void);

	void writeParamsARTKFormat(const std::string& filename) const;

private:
	float computeReprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
								   const std::vector< std::vector<cv::Point2f> >& imagePoints,
								   const cv::Mat& cameraMatrix,
								   const cv::Mat& distCoeffs,
								   const std::vector<cv::Mat>& rvecs,
								   const std::vector<cv::Mat>& tvecs,
								   cv::Mat& perViewErrors);

	cv::Size mImageSize;
	cv::Size mBoardSize;
	float mSquareSize;
	cv::Mat mCameraMatrix;
	cv::Mat mDistCoeffs;
	cv::Mat mExtrParams;
	cv::Mat mReprojErrs;
	float mAvgReprojErr;

	std::vector< std::vector<cv::Point2f> > mImagePoints;
};

}

#endif
