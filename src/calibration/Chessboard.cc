#include "Chessboard.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace px
{

Chessboard::Chessboard(cv::Size boardSize, cv::Mat& image)
 : mBoardSize(boardSize)
 , mCornersFound(false)
{
	if (image.channels() == 1)
	{
		cv::cvtColor(image, mSketch, CV_GRAY2BGR);
		image.copyTo(mImage);
	}
	else
	{
		image.copyTo(mSketch);
		cv::cvtColor(image, mImage, CV_BGR2GRAY);
	}
}

void
Chessboard::findCorners(void)
{
	mCornersFound = cv::findChessboardCorners(mImage, mBoardSize, mCorners,
											  CV_CALIB_CB_ADAPTIVE_THRESH +
											  CV_CALIB_CB_FILTER_QUADS +
											  CV_CALIB_CB_FAST_CHECK);

	// improve the found corners' coordinate accuracy
	if (mCornersFound)
	{
		cv::cornerSubPix(mImage, mCorners,
						 cv::Size(11,11), cv::Size(-1,-1),
						 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
										  30, 0.1));
	}

	// draw chessboard corners
	cv::drawChessboardCorners(mSketch, mBoardSize, mCorners, mCornersFound);
}

const std::vector<cv::Point2f>&
Chessboard::getCorners(void) const
{
	return mCorners;
}

bool
Chessboard::cornersFound(void) const
{
	return mCornersFound;
}

const cv::Mat&
Chessboard::getSketch(void) const
{
	return mSketch;
}

}
