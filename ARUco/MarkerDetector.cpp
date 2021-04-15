#include "MarkerDetector.hpp"

void MarkerDetector::setParameters(const param& params)
{
	this->params = params;
}
void MarkerDetector::detectMarkers(std::string filename, vector<MarkerInfo>& output, const param& params)
{
	// step 0. Load Image
	inputImage = imread(filename, IMREAD_COLOR);

	// step 1. Convert to grey image
	Mat greyInputImage;
	cvtColor(inputImage, greyInputImage, COLOR_BGR2GRAY);

	// step 2. Binarize grey image using adaptive threshold with Gaussian filter
	adaptiveThreshold(greyInputImage, binaryImage,
		params.maxPixelValue, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, params.windowSize, params.C);

	// step 3. Find contours of marker candidates obtained from contours of the binarized image
	vector<vector<Point2f> > inputMarkerContours;
	_findSquareContours(binaryImage, inputMarkerContours);

	// step 4. Compute candidate bitmaps from contours
	vector<Mat>			bitMatrices;
	vector<vector<Point2f> > candidateMarkerContours;
	_detectMarkerCandidates(inputMarkerContours, greyInputImage, bitMatrices, candidateMarkerContours);

	// step 5. Identify candidates using dictionary
	_identifyCandidates(bitMatrices, candidateMarkerContours, output);
}

void MarkerDetector::_findSquareContours(InputArray binaryImage, ContourArray& inputMarkerContours)
{
	// step 3.1. Find contours
	vector<vector<Point> > contours;
	findContours(binaryImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	if (params.showImage)
	{
		Mat binaryImageC3;
		cvtColor(binaryImage, binaryImageC3, COLOR_GRAY2BGR, 0);
		drawContours(binaryImageC3, contours, -1, Scalar(0, 0, 255), 2);
		imshow("Original contour image", binaryImageC3);
		waitKey(WAIT_TIME);
	}

	// step 3.2. Polygonal approximation using Douglas-Peucker algorithm. 
	vector<vector<Point> > squareContours;
	vector<Point2f> polyApprox;
	for (const auto& contour : contours)
	{
		approxPolyDP(Mat(contour), polyApprox, arcLength(Mat(contour), true) * 0.05, true);
		if (polyApprox.size() == 4) // Convex square contours
		{
			squareContours.push_back(contour);
			vector<cv::Point2f> cornerPoints;
			for (int j = 0; j < 4; j++)
				cornerPoints.push_back(cv::Point2f(polyApprox[j].x, polyApprox[j].y));

			_arrangeContourPointsCCW(cornerPoints);
			inputMarkerContours.push_back(cornerPoints);
		}
	}
	if (params.showImage)
	{
		Mat binaryImageC3;
		cvtColor(binaryImage, binaryImageC3, COLOR_GRAY2BGR, 0);
		drawContours(binaryImageC3, squareContours, -1, Scalar(0, 0, 255), 2);
		imshow("Simplified contour image", binaryImageC3);
		waitKey(WAIT_TIME);
	}
}
void MarkerDetector::_arrangeContourPointsCCW(std::vector<Point2f>& cornerPoints)
{
	cv::Point v1 = cornerPoints[1] - cornerPoints[0];
	cv::Point v2 = cornerPoints[2] - cornerPoints[0];
	double o = (v1.x * v2.y) - (v1.y * v2.x);
	if (o < 0.0)
		swap(cornerPoints[1], cornerPoints[3]);
}

void MarkerDetector::_detectMarkerCandidates(const ContourArray& inputMarkerContours,const Mat& greyInputImage, vector<Mat> &bitMatrices, ContourArray&  candidateMarkerContours)
{
	dictionary = aruco::getPredefinedDictionary(params.dictionaryId);
	int markerBits = dictionary->markerSize;
	int cellSize = params.cellSize;
	int BorderBits = params.borderBits;
	int markerBitsWithBorder = markerBits + 2 * BorderBits;
	int markerSampleSize = markerBitsWithBorder * cellSize;

	// step 4.1. Compute coordinates of sample marker contours
	vector<Point2f>			 sampleMarkerContour;
	sampleMarkerContour.push_back(cv::Point2f(0, 0));
	sampleMarkerContour.push_back(cv::Point2f(markerSampleSize - 1, 0));
	sampleMarkerContour.push_back(cv::Point2f(markerSampleSize - 1, markerSampleSize - 1));
	sampleMarkerContour.push_back(cv::Point2f(0, markerSampleSize - 1));


	// step 4.2. compute bitmaps from marker candidates
	for (const auto& inputMarkerContour : inputMarkerContours)
	{
		// step 4.2.1. Compute perspective transformation matrix and warp image
		Mat PerspectiveTransformMatrix = getPerspectiveTransform(inputMarkerContour, sampleMarkerContour);
		Mat	warpedInputImage;
		warpPerspective(greyInputImage, warpedInputImage, PerspectiveTransformMatrix, Size(markerSampleSize, markerSampleSize));

		// step 4.2.2. Binarize candidate images using Otsu's method (histogram based global threshold)
		threshold(warpedInputImage, warpedInputImage, 125, 255, THRESH_BINARY | THRESH_OTSU);

		// step 4.2.3. check wheather boundary contains white cell
		bool whiteBorderCellExist = false;
		for (int x = 0; x < markerBitsWithBorder; x++)
			for (int y = 0; y < markerBitsWithBorder; y++)
			{
				if (
					!whiteBorderCellExist &&
					_isBorder(x, y, BorderBits, markerBitsWithBorder) &&
					_isWhiteCell(x, y, cellSize, warpedInputImage)
					)
					whiteBorderCellExist = true;
			}

		// step 4.2.4. Extract bits from warpedInputImage surrounded with only black borders
		if (!whiteBorderCellExist)
		{
			Mat bitMatrix(markerBits, markerBits, CV_8UC1);
			for (int x = BorderBits; x < markerBits + BorderBits; x++)
				for (int y = BorderBits; y < markerBits + BorderBits; y++)
				{
					if (_isWhiteCell(x, y, cellSize, warpedInputImage))
						bitMatrix.at<uchar>(y - BorderBits, x - BorderBits) = 1;
					else
						bitMatrix.at<uchar>(y - BorderBits, x - BorderBits) = 0;
				}
			bitMatrices.push_back(bitMatrix);
			candidateMarkerContours.push_back(inputMarkerContour);
		}
	}

	if (params.showImage)
	{
		vector<vector<Point> > squareContours;
		for (const auto& contour : candidateMarkerContours)
		{
			vector<Point> contour_;	
			for (const auto& point : contour)
				contour_.push_back(Point(point.x, point.y));
			squareContours.push_back(contour_);
		}
		Mat binaryImageC3;
		cvtColor(binaryImage, binaryImageC3, COLOR_GRAY2BGR, 0);
		drawContours(binaryImageC3, squareContours, -1, Scalar(0, 0, 255), 2);
		imshow("Original contour image", binaryImageC3);
		waitKey(WAIT_TIME);
	}
}
inline bool MarkerDetector::_isBorder(int x, int y, int BorderBits, int markerBitsWithBorder)
{
	return y < BorderBits || y >= markerBitsWithBorder - BorderBits ||
		x < BorderBits || x >= markerBitsWithBorder - BorderBits;

}
bool MarkerDetector::_isWhiteCell(int x, int y, int cellSize, const Mat& warpedInputImage)
{
	int cellX = x * cellSize;
	int cellY = y * cellSize;
	cv::Mat cell = warpedInputImage(Rect(cellX, cellY, cellSize, cellSize));
	return  countNonZero(cell) > (cellSize * cellSize) / 2;
}

void MarkerDetector::_identifyCandidates(const vector<Mat>& bitMatrices, const ContourArray& candidateMarkerContours, vector<MarkerInfo>& output)
{
	int rotation = -1;
	int markerInputId = -1;
	// step 5.1. identify bitmaps and insert only valid id into output
	for (int i = 0; i < candidateMarkerContours.size(); i++)
	{
		Mat bitMatrix = bitMatrices[i];
		vector<Point2f> markerContour = candidateMarkerContours[i];

		if (!_identify(bitMatrix, markerInputId, rotation, 1))
			cout << "Marker not found" << endl;
		else {

			if (rotation != 0) {
				// rearrange marker contours into pre-defined order
				std::rotate(markerContour.begin(), markerContour.begin() + 4 - rotation, markerContour.end());
			}
			cout << "marker ID: " << markerInputId << endl;
			finalDetectedMarkers.push_back(MarkerInfo(markerInputId, markerContour));
		}
	}
	output = finalDetectedMarkers;

}
bool MarkerDetector::_identify(const Mat& onlyBits, int& idx, int& rotation, float maxCorrectionRate) 
{

	// Convert bit matrix to byte lists
	Mat candidateBytes = _getByteListFromBits(onlyBits);

	idx = -1;
	rotation = -1;

	int MinDistance = dictionary->markerSize * dictionary->markerSize + 1;

	// Error correction bit - up to half of the minimum correlation (inter-byte hamming distance)
	int maxCorrectionRecalculed = static_cast<int>(
										(double)maxCorrectionRate * dictionary->maxCorrectionBits
									);
	for (int m = 0; m < dictionary->bytesList.rows; m++)
	{
		for (unsigned int r = 0; r < 4; r++)
		{
			int currentHamming =
				hal::normHamming
				(
					dictionary->bytesList.ptr(m) + r * candidateBytes.cols,
					candidateBytes.ptr(),
					candidateBytes.cols
				);

			// Update Hamming distance
			if (currentHamming < MinDistance)
			{
				MinDistance = currentHamming;
				rotation = r;
				idx = m;
			}
		}
		// If maxCorrection is fulfilled, return this one
		if (MinDistance <= maxCorrectionRecalculed)
			break;
	}

	return idx != -1;
}
Mat MarkerDetector::_getByteListFromBits(const Mat& bits) {
	
	// integer ceil
	int nbytes = (bits.cols * bits.rows + 8 - 1) / 8;

	Mat candidateByteList(1, nbytes, CV_8UC1, Scalar::all(0));
	unsigned char currentBit = 0;
	int currentByte = 0;

	uchar* rot0 = candidateByteList.ptr();

	for (int row = 0; row < bits.rows; row++) {
		for (int col = 0; col < bits.cols; col++) {
			// circular shift
			rot0[currentByte] <<= 1;

			// set bit
			rot0[currentByte] |= bits.at<uchar>(row, col);

			currentBit++;
			if (currentBit == 8) {
				// next byte
				currentBit = 0;
				currentByte++;
			}
		}
	}
	return candidateByteList;
}


void MarkerDetector::getInputImage(Mat& output)
{
	output = inputImage.clone();
}
