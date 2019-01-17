//Copyright 2018, Sarim Mehdi, All rights reserved.

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/flann.hpp"
#include "opencv2/aruco.hpp"
#include "tinyxml2.h"
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <string>
#include <ctime>

using namespace std;

#include "ArucoAndMap.h"

//Taken from https://www.youtube.com/watch?v=R3RRKDcW2RU
void ArucoAndMap::DetectArucoMarkers(cv::Mat &image, cv::Mat &theMapToDraw, const float &arucoMarkerLength, 
	cv::Mat &yourCamCalib, cv::Mat &yourDistCoeff, markerAndMap &retVal)
{
	retVal.theMarker = image; retVal.theMapSign = theMapToDraw.clone();

	//initialize the dictionary, marker ids and corners here
	cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
	vector<int> ids; vector<vector<cv::Point2f> > corners; vector<cv::Vec3d> rotationVectors, translationVectors;
	
	cv::aruco::detectMarkers(retVal.theMarker, markerDictionary, corners, ids);

	cv::Mat R, cameraPose; cv::Point signPos; carData theCar; cv::Point3d rotToSign; cv::Point2f tempPos;
	int myCos, mySin;
	if (ids.size() > 0)     //make sure you detect markers
	{
		cv::aruco::drawDetectedMarkers(retVal.theMarker, corners, ids);
		cv::aruco::estimatePoseSingleMarkers(corners, arucoMarkerLength, yourCamCalib, yourDistCoeff, rotationVectors, translationVectors);

		//distance calculation for aruco markers
		for (int i = 0; i < ids.size(); i++)
		{
			cv::aruco::drawAxis(retVal.theMarker, yourCamCalib, yourDistCoeff, rotationVectors[i], translationVectors[i], 0.1);

			//check for invalid detected aruco markers that don't exist but get detected anyway
			if (ids[i] > signDescriptors.size())
			{
				cv::putText(retVal.theMapSign, "INVALID SIGN " + to_string(ids[i]) + " DETECTED", cv::Point(50, 400), \
					cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
			}
			else
			{
				cv::Rodrigues(rotationVectors[i], R); cameraPose = -R.t() * cv::Mat(translationVectors[i]);
				signPos = cv::Point(signDescriptors[ids[i]].x, signDescriptors[ids[i]].y);
				rotToSign = cv::Point3d(rotationVectors[i][0], rotationVectors[i][1], rotationVectors[i][2]);

				//rotate your car's position about the aruco reference frame and add the coordinates of the sign to get coordinates in map frame
				//car position given according to aruco xz axis in the aruco frame of reference (in aruco frame of reference, the z axis is
				//the horizontal axis increasing in right direction and the x axis is the vertical axis increasing in the downwards direction)

				//pre-assign cos and sin values for efficiency
				switch (signAngles[ids[i]])
				{
				case(0): myCos = 1; mySin = 0; break;
				case(90): myCos = 0; mySin = 1; break;
				case(-90): myCos = 0; mySin = -1; break;
				case(180): myCos = -1; mySin = 0; break;
				}

				//tempPos.x = (cameraPose.at<double>(2, 0) * cos(signAngles[ids[i]])) - (cameraPose.at<double>(0, 0) * sin(signAngles[ids[i]]));
				//tempPos.y = (cameraPose.at<double>(2, 0) * sin(signAngles[ids[i]])) + (cameraPose.at<double>(0, 0) * cos(signAngles[ids[i]]));
				tempPos.x = (cameraPose.at<double>(0, 0) * myCos) - (cameraPose.at<double>(2, 0) * mySin);
				tempPos.y = (cameraPose.at<double>(0, 0) * mySin) + (cameraPose.at<double>(2, 0) * myCos);
				theCar.carPos.x = tempPos.x + signPos.x; theCar.carPos.y = tempPos.y + signPos.y;
				retVal.distToMarker = pow(tempPos.x * tempPos.x + tempPos.y * tempPos.y, 0.5);

				//logic for executing maneuvers done here
				for (size_t j = 0; j < interSigns.size(); j++)
				{
					//check if detected sign belongs to an intersection sign	
					if (ids[i] == interSigns[j])
					{
						//if you see an intersection sign, check if it is close enough to allow you to execute next maneuver
						if (retVal.distToMarker < retVal.interDist && retVal.nextMan == false)
						{
							retVal.nextMan = true; retVal.manTime = clock();
						}
					}
				}

				int res = corners[i][0].x - retVal.screenX;
				
				//if I am too close to the sign and the sign is to my left or right, I simply add this to my steering to avoid collision
				if (retVal.distToMarker <= 7)
				{
					if (res >= 0) { retVal.signBias = -5; }
					else { retVal.signBias = 5; }
				}

				//make sure you are driving on right side of road by checking your horizontal distance from the nearest aruco marker
				if (retVal.distToMarker >= 20 && retVal.distToMarker <= 50)
				{
					if (res >= retVal.horizontalLimitR + (retVal.distToMarker * 100) / 50 && res > 0) 
					{ 
						retVal.rightLane = false; 
					}
				}

				//write down these for debugging purposes
				//cv::putText(retVal.theMapSign, "SAW A SIGN WITH ANGLE " + to_string(signAngles[ids[i]]) + " IT'S SIN IS " + to_string(mySin) + " IT'S COS IS " + to_string(myCos), cv::Point(50, 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "MY CAMERA POSE MATRIX HAS " + to_string(cameraPose.rows) + " ROWS AND " + to_string(cameraPose.cols) + " COLUMNS", cv::Point(50, 30), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "MY CAMERA POSE IS (" + to_string(cameraPose.at<double>(0,0)) + ", " + to_string(cameraPose.at<double>(1, 0)) + ", " + to_string(cameraPose.at<double>(2, 0)) + ")", cv::Point(50, 40), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				cv::putText(retVal.theMapSign, "SAW A SIGN WITH ID " + to_string(ids[i]), cv::Point(50, 50 + (i * 10)), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "I SAW A SIGN AT (" + to_string(signPos.x) + ", " + to_string(signPos.y) + ")", cv::Point(50, 60), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "THE TRANSLATION TO SIGN IS (" + to_string(translationVectors[i][0]) + ", " + to_string(translationVectors[i][1]) + ", " + to_string(translationVectors[i][2]) + ")", cv::Point(50, 70), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "THE ROTATION TO SIGN IS (" + to_string(rotToSign.x) + ", " + to_string(rotToSign.y) + ", " + to_string(rotToSign.z) + ")", cv::Point(50, 80), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "THE COORDINATE AFTER ROTATING IS (" + to_string(tempPos.x) + ", " + to_string(tempPos.y) + ")", cv::Point(50, 90), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "THE TOTAL DISTANCE TO SIGN IS " + to_string(cv::norm(translationVectors[i])), cv::Point(50, 100), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));
				//cv::putText(retVal.theMapSign, "THE CAR PSEUDO POS IS (" + to_string(cameraPose.at<double>(2, 0) + signPos.x) + ", " + to_string(cameraPose.at<double>(0, 0) + signPos.y) + ")", cv::Point(50, 110), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(100, 200, 100));
				//cv::putText(retVal.theMapSign, "THE CAR POS IS (" + to_string(theCar.carPos.x) + ", " + to_string(theCar.carPos.y) + ")", cv::Point(50, 120), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(200, 200, 250));

				//draw your car and the detected sign as a circle on the map
				//cv::circle(retVal.theMapSign, cv::Point((int)(cameraPose.at<double>(0, 0) + signPos.x), (int)(cameraPose.at<double>(2, 0) + signPos.y)), 1, cv::Scalar(200, 0, 255), -1);
				cv::circle(retVal.theMapSign, cv::Point(theCar.carPos.x, theCar.carPos.y), 1, cv::Scalar(100, 0, 255), -1);
				cv::circle(retVal.theMapSign, cv::Point(signPos.x, signPos.y), 1, cv::Scalar(0, 100, 0), -1);
			}
		}
	}

	//check if you have taken more than the alloted time for executing the maneuver
	if (retVal.nextMan)
	{
		cv::putText(retVal.theMarker, "EXECUTING MANEUVER " + to_string(retVal.manIter), cv::Point(400, 120), \
			cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0));
		if ((double)(clock() - retVal.manTime) >= retVal.timeLimit) { retVal.nextMan = false; retVal.manIter++; }
	}
}

void ArucoAndMap::CreateArucoMarkers(int borderWidth)
{
	cv::Mat outputMarker;
	string folderName = "aruco_markers";    //create a new folder to store all the aruco markers
	if (dirExists(folderName) == false) { string folderCreateCommand = "mkdir " + folderName; system(folderCreateCommand.c_str()); }

	//get the simplest aruco markers which are really fast to detect (we have less than 50 traffic signs anyway)
	cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	cv::Mat whiteBorder;
	int rowNumbers, colNumbers, widthStep, row_iterator, col_iterator;
	bool canIChange;
	clock_t beginAruco = clock();
	for (int i = 0; i < 50; i++)
	{
		cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		whiteBorder = outputMarker;
		rowNumbers = whiteBorder.rows;
		colNumbers = whiteBorder.cols;
		widthStep = whiteBorder.step;
		row_iterator, col_iterator; //iterators for rows and columns

		//fill the upper row (according to given width of border) with white pixels
		for (row_iterator = 0; row_iterator < rowNumbers; row_iterator++)
		{
			for (col_iterator = 0; col_iterator < colNumbers; col_iterator++)
			{
				canIChange = true;
				//first check which row you are scanning
				if (row_iterator >= borderWidth && row_iterator < (rowNumbers - borderWidth))
				{
					//then check which column of that specific row you are on
					if (col_iterator >= borderWidth && col_iterator < (colNumbers - borderWidth))
					{
						canIChange = false;
					}
				}

				//to make sure you only put white pixels at the border
				if (canIChange) { whiteBorder.data[(row_iterator * widthStep) + col_iterator] = 255; }
			}
		}

		ostringstream convert;
		string imageName = "4x4Marker_"; 
		convert << folderName << "/" << imageName << i << ".jpg"; 
		string fullpath = convert.str();
		convert.str("");
		cv::imwrite(fullpath, whiteBorder);
	}
	cout << "CREATING ARUCO MARKERS TOOK " << double(clock() - beginAruco) / CLOCKS_PER_SEC << " SECONDS" << endl;
}

cv::Mat ArucoAndMap::drawMap(const char* filename, const char* roadSigns)
{
	tinyxml2::XMLDocument reader;
	tinyxml2::XMLError myReader = reader.LoadFile(filename);
	auto theRoot = reader.FirstChildElement("map");

	//the map has a top left corner (anchor) which is offset from the actual screen's top left for easy viewing
	auto theElement = theRoot->FirstChildElement("anchor");
	int xAnchor; myReader = theElement->QueryIntAttribute("x", &xAnchor);   //get the x coordinate of anchor
	int yAnchor; myReader = theElement->QueryIntAttribute("y", &yAnchor);   //get the y coordinate of anchor
	theElement = theRoot->FirstChildElement("endpoint");    //the map also has a bottom right corner
	int width; myReader = theElement->QueryIntAttribute("x", &width);   //get the x coordinate of bottom right point
	int height; myReader = theElement->QueryIntAttribute("y", &height);   //get the y coordinate of bottom right point
	cv::Mat map(width, height, CV_8UC3, cv::Scalar(0, 0, 0));
	theElement = theRoot->FirstChildElement("tiles");     //now start iterating through the tiles
	auto tileBlock = theElement->FirstChildElement("block");
	string name; int x, y, length, outward, a, b, radius, i, previousX, yValue1, yValue2;
	while (tileBlock != nullptr)
	{
		name = tileBlock->Attribute("name"); i = 0;

		//check which block it is and start drawing (to draw inner lanes, we use outward variable which represents the width of each lane)
		if (name.compare("straightUp") == 0)
		{
			tileBlock->QueryIntAttribute("x", &x); tileBlock->QueryIntAttribute("y", &y); tileBlock->QueryIntAttribute("length", &length);
			x += xAnchor; y += yAnchor;     //offset by the given anchor
			cv::line(map, cv::Point(x, y), cv::Point(x, y - length), cv::Scalar(0, 255, 255), 1);

			//draw the inner lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			cv::line(map, cv::Point(x + outward, y), cv::Point(x + outward, y - length), cv::Scalar(0, 255, 255), 1);
			cv::line(map, cv::Point(x + outward + outward, y), cv::Point(x + outward + outward, y - length), cv::Scalar(0, 255, 255), 1);
		}
		else if (name.compare("straightDown") == 0)
		{
			tileBlock->QueryIntAttribute("x", &x); tileBlock->QueryIntAttribute("y", &y); tileBlock->QueryIntAttribute("length", &length);
			x += xAnchor; y += yAnchor;     //offset by the given anchor
			cv::line(map, cv::Point(x, y), cv::Point(x, y + length), cv::Scalar(0, 255, 255), 1);

			//draw the inner lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			cv::line(map, cv::Point(x + outward, y), cv::Point(x + outward, y + length), cv::Scalar(0, 255, 255), 1);
			cv::line(map, cv::Point(x + outward + outward, y), cv::Point(x + outward + outward, y + length), cv::Scalar(0, 255, 255), 1);
		}
		else if (name.compare("straightLeft") == 0)
		{
			tileBlock->QueryIntAttribute("x", &x); tileBlock->QueryIntAttribute("y", &y); tileBlock->QueryIntAttribute("length", &length);
			x += xAnchor; y += yAnchor;     //offset by the given anchor
			cv::line(map, cv::Point(x, y), cv::Point(x - length, y), cv::Scalar(0, 255, 255), 1);

			//draw the inner lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			cv::line(map, cv::Point(x, y + outward), cv::Point(x - length, y + outward), cv::Scalar(0, 255, 255), 1);
			cv::line(map, cv::Point(x, y + outward + outward), cv::Point(x - length, y + outward + outward), cv::Scalar(0, 255, 255), 1);
		}
		else if (name.compare("straightRight") == 0)
		{
			tileBlock->QueryIntAttribute("x", &x); tileBlock->QueryIntAttribute("y", &y); tileBlock->QueryIntAttribute("length", &length);
			x += xAnchor; y += yAnchor;     //offset by the given anchor
			cv::line(map, cv::Point(x, y), cv::Point(x + length, y), cv::Scalar(0, 255, 255), 1);

			//draw the inner lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			cv::line(map, cv::Point(x, y + outward), cv::Point(x + length, y + outward), cv::Scalar(0, 255, 255), 1);
			cv::line(map, cv::Point(x, y + outward + outward), cv::Point(x + length, y + outward + outward), cv::Scalar(0, 255, 255), 1);
		}

		/*it is impossible to draw curves in opencv, so we use formula of circle to mark closely spaced points on the curve and draw straight
		lines between points to give the illusion of a curve. We move the x value back or forward depending on direction of curve (x goes back
		if curve is facing left and x goes forward if curve is facing right) and then we calculate the y value using the following formula:
		pow(x-a,2) + pow(y-b,2) = pow(radius,2)  ====> where a and b are the x and y coordinates of the center of the quadrant (a quadrant
														is just a circle divided in 4 parts and our curve is a quadrant)
		from the above formula, we get two variations: (keep in mind that the y axis increases downwards in opencv)
		y = b - pow(pow(radius,2)-pow(x-a,2),0.5)   =======> if the curve is facing upwards (while being left or right)
		y = b + pow(pow(radius,2)-pow(x-a,2),0.5)   =======> if the curve is facing downwards (while being left or right)*/
		else if (name.compare("curveUpRight") == 0)
		{
			tileBlock->QueryIntAttribute("centerX", &a); tileBlock->QueryIntAttribute("centerY", &b); tileBlock->QueryIntAttribute("radius", &radius);
			a += xAnchor; b += yAnchor;     //offset by the given anchor

			//draw the road with its lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			while (i < 3)
			{
				previousX = a; yValue1 = b - pow(pow(radius, 2) - pow(previousX - a, 2), 0.5);
				for (int x = a + 1; x <= (a + radius); x++)
				{
					yValue2 = b - pow(pow(radius, 2) - pow(x - a, 2), 0.5);
					cv::line(map, cv::Point(previousX, yValue1), cv::Point(x, yValue2), cv::Scalar(0, 255, 255), 1);
					previousX = x; yValue1 = yValue2;
				}
				i++; radius += outward;
			}
		}
		else if (name.compare("curveUpLeft") == 0)
		{
			tileBlock->QueryIntAttribute("centerX", &a); tileBlock->QueryIntAttribute("centerY", &b); tileBlock->QueryIntAttribute("radius", &radius);
			a += xAnchor; b += yAnchor;     //offset by the given anchor

			//draw the road with its lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			while (i < 3)
			{
				previousX = a; yValue1 = b - pow(pow(radius, 2) - pow(previousX - a, 2), 0.5);
				for (int x = a - 1; x >= (a - radius); x--)
				{
					yValue2 = b - pow(pow(radius, 2) - pow(x - a, 2), 0.5);
					cv::line(map, cv::Point(previousX, yValue1), cv::Point(x, yValue2), cv::Scalar(0, 255, 255), 1);
					previousX = x; yValue1 = yValue2;
				}
				i++; radius += outward;
			}
		}
		else if (name.compare("curveDownRight") == 0)
		{
			tileBlock->QueryIntAttribute("centerX", &a); tileBlock->QueryIntAttribute("centerY", &b); tileBlock->QueryIntAttribute("radius", &radius);
			a += xAnchor; b += yAnchor;     //offset by the given anchor

			//draw the road with its lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			while (i < 3)
			{
				previousX = a; yValue1 = b + pow(pow(radius, 2) - pow(previousX - a, 2), 0.5);
				for (int x = a + 1; x <= (a + radius); x++)
				{
					yValue2 = b + pow(pow(radius, 2) - pow(x - a, 2), 0.5);
					cv::line(map, cv::Point(previousX, yValue1), cv::Point(x, yValue2), cv::Scalar(0, 255, 255), 1);
					previousX = x; yValue1 = yValue2;
				}
				i++; radius += outward;
			}
		}
		else if (name.compare("curveDownLeft") == 0)
		{
			tileBlock->QueryIntAttribute("centerX", &a); tileBlock->QueryIntAttribute("centerY", &b); tileBlock->QueryIntAttribute("radius", &radius);
			a += xAnchor; b += yAnchor;     //offset by the given anchor

			//draw the road with its lanes
			tileBlock->QueryIntAttribute("outward", &outward);
			while (i < 3)
			{
				previousX = a; yValue1 = b + pow(pow(radius, 2) - pow(previousX - a, 2), 0.5);
				for (int x = a - 1; x >= (a - radius); x--)
				{
					yValue2 = b + pow(pow(radius, 2) - pow(x - a, 2), 0.5);
					cv::line(map, cv::Point(previousX, yValue1), cv::Point(x, yValue2), cv::Scalar(0, 255, 255), 1);
					previousX = x; yValue1 = yValue2;
				}
				i++; radius += outward;
			}
		}
		tileBlock = tileBlock->NextSiblingElement("block");
	}

	//load the roadsigns xml file
	tinyxml2::XMLDocument roadSignReader;
	tinyxml2::XMLError mySigns = roadSignReader.LoadFile(roadSigns);
	auto roadRoot = roadSignReader.FirstChildElement("roadsigns");
	auto theChild = roadRoot->FirstChildElement("startsign");
	int id, angle;
	while (theChild != nullptr)
	{
		//each road sign has an (x, y) position, an id and the angle that the car would make with positive x-axis when looking at it
		theChild->QueryIntAttribute("id", &id); theChild->QueryIntAttribute("x", &x); theChild->QueryIntAttribute("y", &y); theChild->QueryIntAttribute("angle", &angle);
		x += xAnchor; y += yAnchor;     //offset by the given anchor
		signDescriptors.push_back(cv::Point3i(x, y, id)); signAngles.push_back(angle);

		//draw the road signs on your map
		cv::circle(map, cv::Point(x, y), 2, cv::Scalar(120, 120, 120), -1);
		theChild = theChild->NextSiblingElement("startsign");
	}
	return map;
}

ArucoAndMap::~ArucoAndMap() {}