#include "Calibration.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace px
{

Calibration::Calibration(const cv::Size& imageSize, const cv::Size& boardSize,
						 float squareSize)
 : mImageSize(imageSize)
 , mBoardSize(boardSize)
 , mSquareSize(squareSize)
{

}

void
Calibration::clear(void)
{
	mImagePoints.clear();
}

void
Calibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
	mImagePoints.push_back(corners);
}

void
Calibration::calibrateCamera(void)
{
    int imageCount = mImagePoints.size();

    std::vector< std::vector<cv::Point3f> > objectPoints;
    for (int i = 0; i < imageCount; ++i)
    {
    	std::vector<cv::Point3f> objectPointsInView;
    	for (int j = 0; j < mBoardSize.height; ++j)
    	{
    		for (int k = 0; k < mBoardSize.width; ++k)
    		{
    			objectPointsInView.push_back(cv::Point3f(j * mSquareSize, k * mSquareSize, 0.0));
    		}
    	}
    	objectPoints.push_back(objectPointsInView);
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(objectPoints, mImagePoints,
                        mImageSize, mCameraMatrix, mDistCoeffs,
                        rvecs, tvecs, 0);

    mExtrParams = cv::Mat(imageCount, 6, CV_64F);
    for (int i = 0; i < imageCount; ++i)
    {
    	mExtrParams.at<double>(i,0) = rvecs.at(i).at<double>(0);
    	mExtrParams.at<double>(i,1) = rvecs.at(i).at<double>(1);
    	mExtrParams.at<double>(i,2) = rvecs.at(i).at<double>(2);
    	mExtrParams.at<double>(i,3) = tvecs.at(i).at<double>(0);
    	mExtrParams.at<double>(i,4) = tvecs.at(i).at<double>(1);
    	mExtrParams.at<double>(i,5) = tvecs.at(i).at<double>(2);
    }

    mAvgReprojErr = computeReprojectionError(objectPoints, mImagePoints,
											 mCameraMatrix, mDistCoeffs,
											 rvecs, tvecs, mReprojErrs);
}

int
Calibration::getSampleCount(void) const
{
	return mImagePoints.size();
}

const std::vector< std::vector<cv::Point2f> >&
Calibration::getImagePoints(void) const
{
	return mImagePoints;
}

cv::Mat&
Calibration::getCameraMatrix(void)
{
	return mCameraMatrix;
}

cv::Mat&
Calibration::getDistCoeffs(void)
{
	return mDistCoeffs;
}

void
Calibration::writeParamsARTKFormat(const std::string& filename) const
{
	FILE* fp = fopen(filename.c_str(),"w+"); // Write to file (overwrite if it exists)

    time_t t;
    time(&t);
    struct tm* t2 = localtime(&t);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    // Write out:
    // width, height, principal point x, principal point y, focal length x, focal length y,
    // dist coeff 1, dist coeff 2, dist coeff 3, dist coeff 4, dist coeff 5, 0, 0, 10

    fprintf(fp, "ARToolKitPlus_CamCal_Rev02\n");
    fprintf(fp, "%d %d %f %f %f %f", mImageSize.width, mImageSize.height,
			mCameraMatrix.at<double>(0,2), mCameraMatrix.at<double>(1,2),
			mCameraMatrix.at<double>(0,0), mCameraMatrix.at<double>(1,1));
    fprintf(fp, " %.15f %.15f %.15f %.15f 0.00000 0.00000 10\n",
    		mDistCoeffs.at<double>(0,0), mDistCoeffs.at<double>(0,1),
    		mDistCoeffs.at<double>(0,2), mDistCoeffs.at<double>(0,3));
    fprintf(fp, "#\n");
    fprintf(fp, "# Images used: %zu\n", mImagePoints.size());
    fprintf(fp, "# avg_reprojection_error: %f\n", mAvgReprojErr);
    fclose(fp);
}

float
Calibration::computeReprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
									  const std::vector< std::vector<cv::Point2f> >& imagePoints,
									  const cv::Mat& cameraMatrix,
									  const cv::Mat& distCoeffs,
									  const std::vector<cv::Mat>& rvecs,
									  const std::vector<cv::Mat>& tvecs,
									  cv::Mat& perViewErrors)
{
    int imageCount = objectPoints.size();
    size_t pointsSoFar = 0;
    float totalErr = 0.0;

    perViewErrors = cv::Mat(1, imageCount, CV_32F);

    for (int i = 0; i < imageCount; ++i)
    {
    	size_t pointCount = imagePoints.at(i).size();

        pointsSoFar += pointCount;

        std::vector<cv::Point2f> estImagePoints;
        cv::projectPoints(cv::Mat(objectPoints.at(i)),
						  rvecs.at(i), tvecs.at(i),
                          cameraMatrix, distCoeffs,
                          estImagePoints);

        float err = 0.0;
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
        	err += fabsf(imagePoints.at(i).at(j).x - estImagePoints.at(j).x);
        	err += fabsf(imagePoints.at(i).at(j).y - estImagePoints.at(j).y);
        }

        perViewErrors.at<float>(i) = err / pointCount;

        totalErr += err;
    }

    return totalErr / pointsSoFar;
}

}
