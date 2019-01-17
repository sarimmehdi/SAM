//Copyright 2018, Sarim Mehdi, All rights reserved.

#include "opencv2/imgproc.hpp"
#include "tinyxml2.h"
#include <Windows.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <omp.h>
#include <stdlib.h>
#include <ctime>
#include <algorithm>

using namespace std;

#include "LaneTracking.h"

cv::Mat LaneTracker::roi(cv::Mat &img, const char* filename)
{
	//I only want to read the XML file once to get ROI coordinates
	if (gotROI == false)
	{
		//load the roi xml file
		tinyxml2::XMLDocument reader; tinyxml2::XMLError myReader = reader.LoadFile(filename);
		auto theRoot = reader.FirstChildElement("points");  auto theElement = theRoot->FirstChildElement("number");
		int IterLimit; myReader = theElement->QueryIntText(&IterLimit);	theElement = theRoot->FirstChildElement("coordinates");
		auto theCoord = theElement->FirstChildElement("xCoord");
		int xValue, yValue;	//points that define your ROI
		for (int i = 0; i < IterLimit; i++)
		{
			myReader = theCoord->QueryIntText(&xValue); theCoord = theCoord->NextSiblingElement("yCoord");
			myReader = theCoord->QueryIntText(&yValue);
			if (i != IterLimit - 1) { theCoord = theCoord->NextSiblingElement("xCoord"); }
			vertices.push_back(cv::Point(xValue, yValue));
		}
		gotROI = true;
	}

	vector<vector<cv::Point>> ppt; ppt.push_back(vertices);
	cv::Mat mask(img.size(), img.type(), cv::Scalar(0)); cv::fillPoly(mask, ppt, cv::Scalar(255));
	cv::Mat masked(img.size(), img.type(), cv::Scalar(0)); cv::bitwise_and(img, mask, masked);
	return masked;
}

LaneTracker::RightLeftPoints LaneTracker::clustering_algo(int searchRadius)
{
	RightLeftPoints yourPoints;
	sort(lanePoints.begin(), lanePoints.end(), myobject);  //arrange all points in ascending order of coordinates

	//move from left to right until you arrive at a lane point which is the last point of your left lane
	cv::Point threshold = lanePoints[0]; yourPoints.negativePoints.push_back(lanePoints[0]);
	bool foundThresh = false; unsigned int i = 1;
	while ((foundThresh == false) && (i < lanePoints.size()))
	{
		//check if the next lane point is too far away or not, if not, then it probably is part of your left lane otherwise not
		if ((lanePoints[i].x - threshold.x) > searchRadius) { foundThresh = true; }
		else { threshold = lanePoints[i]; yourPoints.negativePoints.push_back(threshold); i++; }
	}

	//arrange negative points in ascending order of y coordinates
	sort(yourPoints.negativePoints.begin(), yourPoints.negativePoints.end(), newobject);
	yourPoints.laneCenter = yourPoints.negativePoints[0].x;

	//put all the remaining points in your right lane and find lane center
	while (i < lanePoints.size()) { yourPoints.positivePoints.push_back(lanePoints[i]); i++; }

	//arrange positive points in ascending order of y coordinates and compute lane center
	if (!yourPoints.positivePoints.empty()) 
	{ 
		sort(yourPoints.positivePoints.begin(), yourPoints.positivePoints.end(), newobject);
		yourPoints.laneCenter += yourPoints.positivePoints[0].x;
		yourPoints.laneCenter = yourPoints.laneCenter / 2;
	}
	return yourPoints;
}

LaneTracker::RightLeftPoints LaneTracker::OtsuAlgo()
{
	//first find the mean
	double mean = 0; cv::Point theX;
#pragma omp parallel for default(shared) reduction(+:mean)
	for (int i = 0; i < lanePoints.size(); i++)
	{
		theX = lanePoints[i];
		mean += theX.y;
	}
	mean = mean / lanePoints.size();

	//arrange all points in ascending order
	sort(lanePoints.begin(), lanePoints.end(), myobject);

	//start with threshold of 1 (the lane point of index 0) and move from left to right
	bool foundThresh = false; int thresh = 1; double Q1_t = ((double)thresh) / (double)lanePoints.size();
	double sum1 = (double)lanePoints[thresh - 1].y, sum2 = (mean * (double)lanePoints.size()) - (double)lanePoints[thresh - 1].y;
	double num1 = (double)(thresh), num2 = (double)(lanePoints.size() - thresh);
	double mean1 = sum1 / num1, mean2 = sum2 / num2;
	double betweenGroup = Q1_t * (1 - Q1_t) * pow(mean1 - mean2, 2); double temp;
	
	//iterate until you find the threshold
	while (!foundThresh)
	{
		thresh++; 
		Q1_t = ((double)thresh) / (double)lanePoints.size();
		sum1 += (double)lanePoints[thresh - 1].y; sum2 -= (double)lanePoints[thresh - 1].y;
		num1 = (double)(thresh); num2 = (double)(lanePoints.size() - thresh);
		mean1 = sum1 / num1; mean2 = sum2 / num2;
		temp = Q1_t * (1 - Q1_t) * pow(mean1 - mean2, 2);
		if (temp < betweenGroup) { foundThresh = true; }
		else { betweenGroup = temp; }
	}

	RightLeftPoints yourPoints;
	for (int i = 0; i < thresh; i++) { yourPoints.negativePoints.push_back(lanePoints[i]); }
	for (int i = thresh; i < lanePoints.size(); i++) { yourPoints.positivePoints.push_back(lanePoints[i]); }

	/*the lane center is offset a little bit to the right to keep the car more biased towards the right and, therefore (hopefully)
	it might have a greater chance of staying in the right lane*/
	yourPoints.laneCenter = yourPoints.negativePoints[thresh].x - yourPoints.positivePoints[0].x;
	yourPoints.laneCenter = yourPoints.negativePoints[thresh].x + (0.75 * yourPoints.laneCenter);
	return yourPoints;
}

void LaneTracker::drawQuadraticCurve(RightLeftPoints &giveMePoints, cv::Mat &img)
{
	vector<cv::Point> positive_contours = giveMePoints.positivePoints;
	vector<cv::Point> negative_contours = giveMePoints.negativePoints;

	//compute the best Quadratic curve and draw it
	bool gotEmptySoSkip; unsigned int n;
	int xLowest, xLargest;
	double sumX, sumXPow2, sumXPow3, sumXPow4, xNormalized, sumY, sumXY, sumXPow2AndY, yNormalized, D, D1, D2, D3, a, b, c;
	double xNormalizedOneStepAhead, yFirst, ySecond;
	for (int i = 0; i < 2; i++)
	{
		gotEmptySoSkip = false; n = 0;

		if (i == 0)
		{
			//always check if the vector is empty or not otherwise you will get error while drawing in opencv
			if (!negative_contours.empty())
			{
				n = negative_contours.size();
			}
			else { gotEmptySoSkip = true; }
		}
		else
		{
			//always check if the vector is empty or not otherwise you will get error while drawing in opencv
			if (!positive_contours.empty())
			{
				n = positive_contours.size();
			}
			else { gotEmptySoSkip = true; }
		}

		if (!gotEmptySoSkip)
		{
			//store the lowest x value here
			xLowest = 9999;

			//store the largest x value here
			xLargest = 0;

			//use the following math to fit a quadratic curve though your points (https://www.youtube.com/watch?v=AzroLr1XS5E&t=9m)
			sumX = 0; sumXPow2 = 0; sumXPow3 = 0; sumXPow4 = 0;
			sumY = 0; sumXY = 0; sumXPow2AndY = 0;
#pragma omp parallel for default(shared) reduction(+:sumX, sumXPow2, sumXPow3, sumXPow4, sumY, sumXY, sumXPow2AndY)
			for (int j = 0; j < n; j++)
			{
				cv::Point p = (i == 0) ? negative_contours[j] : positive_contours[j];

				/*we normalize the x and y values by dividing with screen width and height respectively (otherwise the numerical operation
				becomes too large, imagine 724 raised to the power of 4 as an example)*/

				xNormalized = (double)p.x / (double)img.cols; yNormalized = (double)p.y / (double)img.rows;

				sumX += xNormalized; sumXPow2 += pow(xNormalized, 2); sumXPow3 += pow(xNormalized, 3); sumXPow4 += pow(xNormalized, 4);
				sumY += yNormalized; sumXY += (xNormalized * yNormalized); sumXPow2AndY += (pow(xNormalized, 2) * yNormalized);

#pragma omp critical
				{
					//compare x values to get the smallest x value of all points
					if (p.x < xLowest) { xLowest = p.x; }

					//compare x values to get the largest x value of all points
					if (p.x > xLargest) { xLargest = p.x; }
				}
			}
			//use Cramer's rule to solve for coefficients of Quadratic equation
			D = ((double)n * ((sumXPow2 * sumXPow4) - pow(sumXPow3, 2))) - (sumX * ((sumX * sumXPow4) - (sumXPow3 * sumXPow2))) + (sumXPow2 * ((sumX * sumXPow3) - pow(sumXPow2, 2)));
			D1 = (sumY * ((sumXPow2 * sumXPow4) - pow(sumXPow3, 2))) - (sumX * ((sumXY * sumXPow4) - (sumXPow3 * sumXPow2AndY))) + (sumXPow2 * ((sumXY * sumXPow3) - (sumXPow2 * sumXPow2AndY)));
			D2 = ((double)n * ((sumXY * sumXPow4) - (sumXPow3 * sumXPow2AndY))) - (sumY * ((sumX * sumXPow4) - (sumXPow3 * sumXPow2))) + (sumXPow2 * ((sumX * sumXPow2AndY) - (sumXY * sumXPow2)));
			D3 = ((double)n * ((sumXPow2 * sumXPow2AndY) - (sumXY * sumXPow3))) - (sumX * ((sumX * sumXPow2AndY) - (sumXY * sumXPow2))) + (sumY * ((sumX * sumXPow3) - pow(sumXPow2, 2)));

			a = D1 / D, b = D2 / D, c = D3 / D;

			//draw the quadratic curve by discretizing it and drawing lines between the points to create the 'illusion' of a curve
#pragma omp parallel for default(shared)
			for (int k = xLowest; k < xLargest; k += 1)
			{
				//you found coefficients for a normalized image plane, so feed normalized x values
				xNormalized = (double)k / (double)img.cols; xNormalizedOneStepAhead = ((double)k + 1) / (double)img.cols;
				yFirst = ((double)a) + ((double)b * xNormalized) + ((double)c * pow(xNormalized, 2));
				yFirst = yFirst * img.rows;     //convert back to the original image plane to display
				ySecond = ((double)a) + ((double)b * (xNormalizedOneStepAhead)) + ((double)c * pow(xNormalizedOneStepAhead, 2));
				ySecond = ySecond * img.rows;
#pragma omp critical
				{
					line(img, cv::Point(k, (int)yFirst), cv::Point(k + 1, (int)ySecond), cv::Scalar(0, 255, 0), 5);
				}
			}
		}
	}
}

int LaneTracker::draw_lanes(cv::Mat &img, cv::Mat &dataImg, bool getData, int &middle_point, int pointLimit, float resolution, bool drawLines)
{
	int leftLimit = middle_point - pointLimit, rightLimit = middle_point + pointLimit;  //limits for left and right
	int steeringAngle = 0;  //by default I am not steering and, therefore, going straight

	//to visualize the middle
	line(img, cv::Point(leftLimit, 150), cv::Point(rightLimit, 150), cv::Scalar(0, 0, 255), 5);
	circle(img, cv::Point(middle_point, 170), 6, cv::Scalar(255, 255, 0), -1, 8);

	//I separate left and right line points
	RightLeftPoints theseMyPoints = clustering_algo();
	vector<cv::Point> positive_contours = theseMyPoints.positivePoints;
	vector<cv::Point> negative_contours = theseMyPoints.negativePoints;

	//draw the center of lanes here
	circle(img, cv::Point(theseMyPoints.laneCenter, 100), 6, cv::Scalar(255, 255, 0), -1, 8);

	//draw the lane lines for visualization purpose
	if (drawLines) { drawQuadraticCurve(theseMyPoints, img); }

	//if you are gathering data, draw lines on the image where you got the hough lines in the first place
	if (getData) { drawQuadraticCurve(theseMyPoints, dataImg); }

	//lane following
	if (theseMyPoints.laneCenter < leftLimit || theseMyPoints.laneCenter > rightLimit)
	{ 
		float theLaneCenter = theseMyPoints.laneCenter; float theMiddlePoint;
		if (theseMyPoints.laneCenter < leftLimit) { theMiddlePoint = leftLimit; }
		else { theMiddlePoint = rightLimit; }
		float finalSteeringValue = ((theLaneCenter - theMiddlePoint) / theMiddlePoint);
		float changeInError = finalSteeringValue - previousDifference;
		float errorBuildUp = finalSteeringValue + previousDifference;
		
		//PID controller for steering, watch this video to understand how: https://www.youtube.com/watch?v=4Y7zG48uHRo
		float PIDresult = (finalSteeringValue * Kp) + (changeInError * Kd) + (errorBuildUp * Ki);

		//store the difference for tuning Kd and Ki of PID
		previousDifference = finalSteeringValue;

		//scale your value to 30
		finalSteeringValue = PIDresult * 30;
		steeringAngle = (int)finalSteeringValue;

		if (steeringAngle <= -30) { steeringAngle = -30; }

		//driver assist text
		if (theseMyPoints.laneCenter < leftLimit)
		{
			putText(img, "YOU SHOULD TURN LEFT WITH STEERING " + to_string(steeringAngle), cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2,
				cv::Scalar(200, 200, 250));
		}
		else
		{
			putText(img, "YOU SHOULD TURN RIGHT WITH STEERING " + to_string(steeringAngle), cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2,
				cv::Scalar(200, 200, 250));
		}
	}
	return steeringAngle;
}

LaneTracker::processedImageAndLines LaneTracker::process_img(cv::Mat &original_image, bool getData)
{
	cv::Mat processed_img = original_image.clone(); cv::Mat grad_x, grad_y; cv::Mat abs_grad_x, abs_grad_y;

	//preprocessing steps before applying hough line detector (get edges, blur the image and then enhance the edges)
	cvtColor(processed_img, processed_img, cv::COLOR_BGR2GRAY);
	Canny(processed_img, processed_img, 200, 300);
	GaussianBlur(processed_img, processed_img, cv::Size(11, 11), 0, 0);
	Sobel(processed_img, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT); convertScaleAbs(grad_x, abs_grad_x);	
	Sobel(processed_img, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT); convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, processed_img);

	//if you are gathering data, you need the version of hough line populated image that is not cropped according to ROI
	processedImageAndLines returnVal; returnVal.theNoROIImage = processed_img.clone();

	//crop out everything except the region of interest so that your car AI only sees that part of the road
	processed_img = roi(processed_img, "XMLfiles/roi.xml");

	//Get hough lines
	vector<cv::Vec4i> lines;
	HoughLinesP(processed_img, lines, 1, CV_PI / 180, 20, 0, 0);

	//Make sure lines vector is not empty, otherwise it will throw exception
	//Draw the lines on the processed image
	//make sure vector is empty before putting in a new set of points
	if (!lines.empty())
	{
		if (!lanePoints.empty()) { lanePoints.clear(); }
		cv::Vec4i l;
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			l = lines[i];

			//push the two points of every detected line into this vector
			lanePoints.push_back(cv::Point(l[0], l[1])); lanePoints.push_back(cv::Point(l[2], l[3]));
			line(processed_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3);
		}
		returnVal.theLines = lines;
	}
	returnVal.theImage = processed_img;

	if (getData)
	{
		//Get hough lines on the whole image instead of just the ROI cropped image as before
		vector<cv::Vec4i> lines_new;
		HoughLinesP(returnVal.theNoROIImage, lines_new, 1, CV_PI / 180, 100, 0, 0);
		if (!lines_new.empty())
		{
			cv::Vec4i l_new; returnVal.theNoROIImage = original_image.clone();
			for (unsigned int i = 0; i < lines_new.size(); i++)
			{
				l_new = lines_new[i];
				line(returnVal.theNoROIImage, cv::Point(l_new[0], l_new[1]), cv::Point(l_new[2], l_new[3]), cv::Scalar(255, 255, 255), 3);
			}
		}
	}
	return returnVal;
}

LaneTracker::~LaneTracker() {}