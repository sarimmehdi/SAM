//Copyright 2018, Sarim Mehdi, All rights reserved.

#pragma once
#ifndef LANETRACKING_H
#define LANETRACKING_H

class LaneTracker
{
private:
	bool gotROI;    //check if you have ROI points
	bool clickForROI;   //if you want to manually get a new ROI
	vector<cv::Point> vertices;    //points that define your ROI
	float previousDifference, errorDifference;   //global variables to be used in PID for steering
	int maxSpeed, minSpeed;   //the maximum and minimum speed of your car
	float Kp, Kd, Ki;    //my constants for PID controller
	vector<cv::Point> lanePoints;   //points of detected lines

	//struct for points that represent positive (left lane) and negative (right lane) points
	typedef struct RightLeftPoints
	{
		vector<cv::Point> positivePoints;
		vector<cv::Point> negativePoints;
		int laneCenter;
	} myLanePoints;

	//sort in ascending order of x coordinates
	struct myclass {
		bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x); }
	} myobject;

	//sort in ascending order of y coordinates
	struct newclass {
		bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y); }
	} newobject;

	//separate left and right points of lane and find midpoint of your lane and follow it
	RightLeftPoints clustering_algo(int searchRadius = 150);

	//use Otsu's algorithm to find the middle point of the lane (placeholder algorithm for the moment)
	RightLeftPoints OtsuAlgo();

	//here I draw the approximated quadratic curve on the image (mathematical details in the cpp file)
	void drawQuadraticCurve(RightLeftPoints &giveMePoints, cv::Mat &img);

	//crop the image to get your region of interest based on the ROI coordinates in filename
	cv::Mat roi(cv::Mat &img, const char* filename);

public:
	//struct for receiving the image with hough lines on it and the list of all detected lines
	typedef struct processedImageAndLines
	{
		cv::Mat theImage;
		cv::Mat theNoROIImage;
		vector<cv::Vec4i> theLines;
	} myImageAndLines;

	//get hough lines here
	processedImageAndLines process_img(cv::Mat &original_image, bool getData);

	//draw lane lines using quadratic approximation
	int draw_lanes(cv::Mat &img, cv::Mat &dataImg, bool getData, int &middle_point, int pointLimit, float resolution = 0.1, 
		bool drawLines = true);

	//get speed based on steering value (speed = minSpeed + (((maxSpeed - minSpeed)) * pow(tuningFactor, steerVal)) 
	int calculateSpeed(int steerVal, float tuningFactor) { return(minSpeed + ((maxSpeed - minSpeed)) * pow(tuningFactor, steerVal)); }

	//check if you want to take a new ROI or not
	bool doYouWantROI() { return clickForROI; }

	//set your private variables in constructor
	LaneTracker(float &proportional, float &differential, float &integral, int &yourMaxSpeed, int &yourMinSpeed, const char* filename, 
		bool IWantROI) : previousDifference(0), errorDifference(0), gotROI(false), maxSpeed(yourMaxSpeed), minSpeed(yourMinSpeed),
		Kp(proportional), Kd(differential), Ki(integral), clickForROI(IWantROI) {}
	~LaneTracker();
};

#endif