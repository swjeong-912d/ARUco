#ifndef ARUCO_MARKER_DETECTOR_HPP
#define ARUCO_MARKER_DETECTOR_HPP
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "dictionary.hpp"
using std::vector;
using std::cout;
using std::endl;
using namespace cv;
typedef vector<Point2f> Contour;
typedef vector<Contour> ContourArray;

struct param {

	int borderBits, cellSize, dictionaryId;

	int windowSize, maxPixelValue, C;

	float eps; // DP approximation parameter eps

	bool showImage;

	bool verbal; // print intermediate process

	float errorCorrectionRate;
	param() {
		borderBits = 1;
		cellSize = 10;
		dictionaryId = 10;

		maxPixelValue = 255;
		// Must be odd value
		windowSize = 77;
		C = 8;

		eps = 0.05;

		verbal = false;

		errorCorrectionRate = 1.0f;

		showImage = false;
	}
};
struct MarkerInfo {
	int markerId;
	vector<Point2f> markerCorners;
	MarkerInfo(int markerId, const vector<Point2f>& markerCorners)
	{
		this->markerId = markerId;
		this->markerCorners = markerCorners;
	}

};

class MarkerDetector
{
	param params;
	Ptr<aruco::Dictionary> dictionary;
	Mat inputImage;
	Mat binaryImage;
	vector<MarkerInfo>  finalDetectedMarkers;
	int WAIT_TIME;

	void _findSquareContours(InputArray binaryImage, ContourArray& inputMarkerContours);
	void _arrangeContourPointsCCW(std::vector<Point2f>& cornerPoints);

	void _detectMarkerCandidates(const ContourArray& inputMarkerContours, const Mat& greyInputImage, vector<Mat>& bitMatrices, ContourArray& candidateMarkerContours);
	bool _isBorder(int x, int y, int BorderBits, int markerBitsWithBorder);
	bool _isWhiteCell(int x, int y, int cellSize, const Mat& warpedInputImage);

	void _identifyCandidates(const vector<Mat>& bitMatrices, const ContourArray& candidateMarkerContours, vector<MarkerInfo>& output);
	bool _identify(const Mat& onlyBits, int& idx, int& rotation, float maxCorrectionRate);
	Mat  _getByteListFromBits(const Mat& bits);

public:
	void detectMarkers(std::string filename, vector<MarkerInfo>& output, const param& params);
	void setParameters(const param& params);
	void getInputImage(Mat& output);
};

#endif