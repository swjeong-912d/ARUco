#include <opencv2/opencv.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "dictionary.hpp"
#include "MarkerDetector.hpp"
#include <vector>
#include <sstream>
using namespace cv;


// wait time constant. 0 for infinity
const int WAIT_TIME = 0;

namespace {
	const char* about = "Detect ArUco markers from image";
	const char* keys =
		"{@outfile |<none> | Output image }"
		"{name     |       | Input filename }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{param    |       | marker detection parameter filename }"
		"{cam      |       | camera parameter filename }"
		"{si       | false | show generated image }"
		"{ml       | 0.035 | marker size }"
		"{al       | 1     | axis size (relative to ml) }"
		"{verb     | false | print pipeline completion message }";
}

int main(int argc, char* argv[]) {
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	if (argc < 5) {
		parser.printMessage();
		return 0;
	}
	MarkerDetector detector;

	String filename = parser.get<String>("name");
	String camParams = parser.get<String>("cam");
	String detectionParams = parser.get<String>("param");
	int dictionaryId = parser.get<int>("d");
	float markerSize = parser.get<float>("ml");
	float axisSize = parser.get<float>("al");
	bool showImage = parser.get<bool>("si");
	bool verbal = parser.get<bool>("verb");
	String outFilename = parser.get<String>(0);
	if (!parser.check()) {
		parser.printErrors();
		return 0;
	}


	// Parameter setting
	FileStorage fs_param(detectionParams, FileStorage::READ);
	if (!fs_param.isOpened())
		return false;
	param params;
	fs_param["adaptiveThresC"] >> params.adaptiveThresC;
	fs_param["adaptiveThresWindowSize"] >> params.adaptiveThresWindowSize;
	fs_param["borderBits"] >> params.borderBits;
	fs_param["cellSize"] >> params.cellSize;
	fs_param["errorCorrectionRate"] >> params.errorCorrectionRate;
	fs_param["polyApproxAccuracyRate"] >> params.polyApproxAccuracyRate;
	fs_param.release();
	params.showImage = showImage;
	params.dictionaryId = dictionaryId;
	params.verbal = verbal;
	detector.setParameters(params);


	vector<MarkerInfo> finalDetectedMarkers;
	detector.detectMarkers(filename, finalDetectedMarkers, params);


	Mat camMatrix, distCoeffs;
	FileStorage fs_cam(camParams, FileStorage::READ);
	if (!fs_cam.isOpened())
		return false;
	fs_cam["camera_matrix"] >> camMatrix;
	fs_cam["distortion_coefficients"] >> distCoeffs;
	fs_cam.release();


	vector<cv::Point3f> markerCorners3d;
	markerCorners3d.push_back(cv::Point3f(markerSize, markerSize, 0));
	markerCorners3d.push_back(cv::Point3f(-markerSize, markerSize, 0));
	markerCorners3d.push_back(cv::Point3f(-markerSize, -markerSize, 0));
	markerCorners3d.push_back(cv::Point3f(markerSize, -markerSize, 0));


	Mat OutputImage;
	detector.getInputImage(OutputImage);
	for (const auto& marker : finalDetectedMarkers)
	{
		//Compute translation and rotation vectors
		Mat rotation_vector, translation_vector;
		solvePnP(markerCorners3d, marker.markerCorners, camMatrix, distCoeffs, rotation_vector, translation_vector);
		if (params.verbal)
		{
			cout << "markerID " << marker.markerId << endl;
			cout << "rotation_vector" << endl << rotation_vector << endl;
			cout << "translation_vector" << endl << translation_vector << endl;
		}

		drawFrameAxes(OutputImage, camMatrix, distCoeffs, rotation_vector, translation_vector, markerSize* axisSize, 4);


		// Find Id display position using projection matrix
		vector<Point3f> idPos3d;
		vector<Point2f> idPos;
		idPos3d.push_back(Point3f(0, 0, 0));
		projectPoints(idPos3d, rotation_vector, translation_vector, camMatrix, distCoeffs, idPos);

		std::stringstream s;
		s << "Id=" << marker.markerId;
		putText(OutputImage, s.str(), idPos[0], FONT_HERSHEY_SIMPLEX, 0.6,
			cv::Scalar(100,200, 0), 2);

	}
	
	if (params.showImage)
	{
		imshow("Axis Image", OutputImage);
		waitKey(WAIT_TIME);
	}
	imwrite(outFilename, OutputImage);

}
