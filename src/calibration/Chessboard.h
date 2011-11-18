#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include <opencv2/core/core.hpp>

namespace px
{

class Chessboard
{
public:
	Chessboard(cv::Size boardSize, cv::Mat& image);

	void findCorners(void);
	const std::vector<cv::Point2f>& getCorners(void) const;
	bool cornersFound(void) const;

	const cv::Mat& getSketch(void) const;

private:
	cv::Mat mImage;
	cv::Mat mSketch;
	std::vector<cv::Point2f> mCorners;
	cv::Size mBoardSize;
	bool mCornersFound;
};

}

#endif
