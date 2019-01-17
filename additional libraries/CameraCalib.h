//Copyright 2018, Sarim Mehdi, All rights reserved.

#pragma once
#ifndef CAMERACALIB_H
#define CAMERACALIB_H

class CameraCalib
{
private:
	bool getCameraMatrix;  //if you want to callibrate camera

	//get the chessboard corners
	void GetChessboardCorners(const cv::Size &dimensions, vector<cv::Mat> &images, vector<vector<cv::Point2f>> &allFoundCorners, 
		bool showResults = true);

	/*In callibration, you compare detected points of a rotated and translated chessboard with those of a 'hypothetical' chessboard that is
	looking straight towards your camera. So, here, you get the coordinates of all detected corners on such a chessboard so that you can
	compare them (to get the camera matrix and the rotation and translation vectors) with the corners of the real chessboard you feed through
	your camera*/
	void CreateKnownBoardPosition(const cv::Size &chessboardDimension, const float &squareLength, vector<cv::Point3f>& corners);
public:
	//get the parameters of your camera matrix from the xml file
	void getCallibration(const char* filename, cv::Mat &camera_callib, cv::Mat &dist_coeffs);

	//check if you want to callibrate your matrix
	bool doYouWantCameraMatrix() { return this->getCameraMatrix; }

	//callibrate your camera
	void cameraCallibration(const cv::Size &boardSize, const float &squareEdgeLength, cv::Mat &cameraMatrix, cv::Mat &distanceCoefficients);

	//visualize points on a chessboard
	cv::Mat SeeChessBoardPoints(const cv::Size &chessboardDimensions, cv::Mat &frame);

	//save the parameters of your camera callibration as an xml file
	void saveCameraCallibration(cv::Mat &cameraMatrix, cv::Mat &distCoefficients);

	CameraCalib(bool IWantCamera = false) : getCameraMatrix(IWantCamera) {}
	~CameraCalib();
};

#endif