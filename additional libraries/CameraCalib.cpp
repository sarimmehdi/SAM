#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "tinyxml2.h"
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <ctime>

using namespace std;

#include "CameraCalib.h"
#include "ArucoAndMap.h"

//Taken from https://www.youtube.com/watch?v=v7jutAmWJVQ
void CameraCalib::GetChessboardCorners(const cv::Size &dimensions, vector<cv::Mat> &images, vector<vector<cv::Point2f>> &allFoundCorners, 
	bool showResults)
{
	int i = 0;

	//create a new folder to store the images with chessboard corners drawn on them
	string folderName = "chessboard_corners"; ArucoAndMap temp(false);
	if (temp.dirExists(folderName) == false) { string folderCreateCommand = "mkdir " + folderName; system(folderCreateCommand.c_str()); }
	bool found; vector<cv::Point2f> pointBuf; ostringstream convert; string fullpath; string imageName = "chessboardImage_";
	for (vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		found = cv::findChessboardCorners(*iter, dimensions, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found) { allFoundCorners.push_back(pointBuf); }
		if (showResults)
		{
			cv::drawChessboardCorners(*iter, dimensions, pointBuf, found);
			convert << folderName << "/" << imageName << i << ".jpg"; fullpath = convert.str(); convert.str("");
			cv::imwrite(fullpath, *iter);
		}
		i++;
	}
}

//Taken from https://www.youtube.com/watch?v=v7jutAmWJVQ
void CameraCalib::CreateKnownBoardPosition(const cv::Size &chessboardDimension, const float &squareLength, vector<cv::Point3f>& corners)
{
	for (int i = 0; i < chessboardDimension.height; i++)
	{
		for (int j = 0; j < chessboardDimension.width; j++)
		{
			corners.push_back(cv::Point3f(j * squareLength, i * squareLength, 0.0f));
		}
	}
}

void CameraCalib::getCallibration(const char* filename, cv::Mat &camera_callib, cv::Mat &dist_coeffs)
{
	tinyxml2::XMLDocument camera; tinyxml2::XMLError myReader = camera.LoadFile(filename); auto theRoot = camera.FirstChildElement("camera");
	
	//first get the camera_callib parameters from xml file
	auto theElement = theRoot->FirstChildElement("camera_callib"); double value;	
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			myReader = theElement->QueryDoubleText(&value);
			camera_callib.at<double>(r, c) = value;
			if (theElement->NextSiblingElement("camera_callib") != nullptr)
			{
				theElement = theElement->NextSiblingElement("camera_callib");
			}
		}
	}

	//then get the dist_coeffs from xml file (notice that the dist_coeffs mat is just a column vector of 5 elements)
	theElement = theRoot->FirstChildElement("dist_coeffs");
	for (int r = 0; r < 5; r++)
	{
		for (int c = 0; c < 1; c++)
		{
			myReader = theElement->QueryDoubleText(&value);
			dist_coeffs.at<double>(r, c) = value;
			if (theElement->NextSiblingElement("dist_coeffs") != nullptr)
			{
				theElement = theElement->NextSiblingElement("dist_coeffs");
			}
		}
	}
}

//Taken from https://www.youtube.com/watch?v=GYIQiV9Aw74
void CameraCalib::saveCameraCallibration(cv::Mat &cameraMatrix, cv::Mat &distCoefficients)
{
	tinyxml2::XMLDocument callibrationDoc; tinyxml2::XMLNode * pRoot = callibrationDoc.NewElement("camera");
	callibrationDoc.InsertFirstChild(pRoot); double value;
	
	//insert elements of the camera matrix
	for (int r = 0; r < cameraMatrix.rows; r++)
	{
		for (int c = 0; c < cameraMatrix.cols; c++)
		{
			value = cameraMatrix.at<double>(r, c); cout << value << endl;
			tinyxml2::XMLElement * pElement = callibrationDoc.NewElement("camera_callib");
			pElement->SetText(value); pRoot->InsertEndChild(pElement);
		}
	}

	//insert elements of the distance coefficients
	for (int r = 0; r < distCoefficients.rows; r++)
	{
		for (int c = 0; c < distCoefficients.cols; c++)
		{
			value = distCoefficients.at<double>(r, c); cout << value << endl;
			tinyxml2::XMLElement * pElement = callibrationDoc.NewElement("dist_coeffs");
			pElement->SetText(value); pRoot->InsertEndChild(pElement);
		}
	}

	//save your camera parameters as an xml file
	tinyxml2::XMLError eResult = callibrationDoc.SaveFile("camera.xml");
}

//Taken from https://www.youtube.com/watch?v=GYIQiV9Aw74
void CameraCalib::cameraCallibration(const cv::Size &boardSize, const float &squareEdgeLength, cv::Mat &cameraMatrix, 
	cv::Mat &distanceCoefficients)
{
	vector<vector<cv::Point2f>> checkerBoardImageSpacePoints; vector<cv::String> fn;
	cv::glob("C:/Users/sarim/Downloads/OpenCV_Project/OpenCV_Project/callibration_images/*.jpg", fn, false);
	vector<cv::Mat> callibrationImages;

	int count = fn.size(); //number of png files in images folder
	for (size_t i = 0; i < count; i++) { callibrationImages.push_back(cv::imread(fn[i])); }

	//get the chessboard corners and push them to your checkerBoardImageSpacePoints vector above
	clock_t beginCorner = clock(); GetChessboardCorners(boardSize, callibrationImages, checkerBoardImageSpacePoints, true);
	clock_t endCorner = clock(); double elapsed_secs_corner = double(endCorner - beginCorner) / CLOCKS_PER_SEC;
	cout << "GETTING CHESSBOARD CORNERS TOOK " << fixed << setprecision(2) << elapsed_secs_corner << " SECONDS" << endl;

	//create a single vector that contains the points of your known 'hypothetical' chessboard
	vector<vector<cv::Point3f>> worldSpaceCornerPoints(1); CreateKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);	

	/*suppose you took 50 pictures of your chessboard. So, now you will resize your previous matrix to 50 and each element of that new
	vector contains, initially, the coordinates of the corners of that hypothetical chessboard from CreateKnownBoardPosition*/
	worldSpaceCornerPoints.resize(checkerBoardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	vector<cv::Mat> rVectors, tVectors;

	/*now just use the following built-in opencv function. Because of the resize thing we did previously, our worldSpaceCornerPoints is now
	the exact same size as the checkerBoardImageSpacePoints which contains all the detected points from the matrices we fed it earlier. So,
	our built-in opencv function simply callibrates by comparing the two vectors*/
	cout << "BEGIN CAMERA CALIBRATION" << endl; clock_t beginCalib = clock();
	cv::calibrateCamera(worldSpaceCornerPoints, checkerBoardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
	clock_t endCalib = clock(); double elapsed_secs_calib = double(endCalib - beginCalib) / CLOCKS_PER_SEC;
	cout << "CAMERA CALLIBRATION TOOK " << fixed << setprecision(2) << elapsed_secs_calib << " SECONDS" << endl;
}

//Taken from https://www.youtube.com/watch?v=l4gGX-5_5q0
cv::Mat CameraCalib::SeeChessBoardPoints(const cv::Size &chessboardDimensions, cv::Mat &frame)
{
	//cameraMatrix is an identity matrix of dimensions 3 by 3
	cv::Mat drawToFrame, cameraMatrix = cv::Mat::eye(3, 3, CV_64F), distanceCoefficients;

	vector<cv::Mat> savedImages; vector<vector<cv::Point2f>> markerCorners, rejectedCandidates; vector<cv::Vec2f> foundPoints;
	bool found = false;

	found = cv::findChessboardCorners(frame, chessboardDimensions, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
	frame.copyTo(drawToFrame); cv::drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);	
	if (found) { return drawToFrame; }
	else { return frame; }
}

CameraCalib::~CameraCalib() {}