//Copyright 2018, Sarim Mehdi, All rights reserved.

#pragma once
#ifndef ARUCOANDMAP_H
#define ARUCOANDMAP_H

#include <windows.h>
#include <string>

class ArucoAndMap
{
private:
	bool getArucoMarkers;    //if you want to generate a new set of aruco markers

	//my roadsigns are described by 3 integers: x and y position and the orientation of car (wrt +ve x-axis) that looks directly towards it
	vector<cv::Point3i> signDescriptors;
	vector<int> signAngles;

	vector<int> interSigns = { 0, 12, 9, 13, 14, 4 };   //roadsigns for an intersection

	//struct for car's position and orientation
	typedef struct carData
	{
		cv::Point carPos;
		int angle;
	} myCar;
public:
	//struct that deals with executing maneuvers and keeping car on the right path
	typedef struct markerAndMap
	{
		cv::Mat theMarker;       //Mat on which detected aruco markers are drawn
		cv::Mat theMapSign;      //Mat on which the map is drawn
		bool nextMan;          //start executing the maneuver
		double distToMarker;    //distance to detected aruco marker
		bool rightLane;         //is true if car is in the right lane
		int signBias;      //add this to steering value while avoiding collision with sign based on whether the sign is to your left or right
		int screenX;       //midpoint of screen, if you are too far to the right, deteced aruco marker will be far away from this point
		int horizontalLimitR;      //distance of aruco marker from midpoint of screen must not be more than this distance (to the right)
		float interDist;      //distance of car from intersection
		double timeLimit;    //car gets a certain amount of time to execute the maneuver before it continues following lanes
		clock_t manTime;     //when it's time to execute the maneuver, this timer is activated
		unsigned int manIter;   //keep track of where you are in the maneuver list using this
	} myMarkAndMap;

	//check if folder exists: https://stackoverflow.com/questions/8233842/how-to-check-if-directory-exist-using-c-and-winapi
	bool dirExists(const std::string& dirName_in)
	{
		DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
		if (ftyp == INVALID_FILE_ATTRIBUTES)
			return false;  //something is wrong with your path!

		if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
			return true;   // this is a directory!

		return false;    // this is not a directory!
	}

	//check if you want to draw a new set of aruco markers
	bool doYouWantAruco() { return getArucoMarkers; }

	cv::Mat drawMap(const char* filename, const char* roadSigns);   //draw the map
	void CreateArucoMarkers(int borderWidth = 50);    //print aruco markers, each aruco marker corresponds to a traffic sign
	  
	//detects aruco markers in the given image
	void DetectArucoMarkers(cv::Mat &image, cv::Mat &theMapToDraw, const float &arucoMarkerLength, cv::Mat &yourCamCalib, 
		cv::Mat &yourDistCoeff, markerAndMap &retVal);

	ArucoAndMap(bool IWantAruco) : getArucoMarkers(IWantAruco) {}
	~ArucoAndMap();
};

#endif