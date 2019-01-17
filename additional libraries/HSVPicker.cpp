//Taken from https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "tinyxml2.h"
#include <iostream>

using namespace std;

#include "HSVPicker.h"

void HSVPicker::lowHtrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::highHtrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::lowStrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::highStrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::lowVtrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::highVtrackbar(int v, void* ptr)
{
	// resolve 'this':
	HSVPicker *that = (HSVPicker*)ptr;
	that->on_low_H_thresh_trackbar();
}

void HSVPicker::on_low_H_thresh_trackbar()
{
	low_H = min(high_H - 1, low_H);
	cv::setTrackbarPos("Low H", "Video Capture", low_H);
}

void HSVPicker::on_high_H_thresh_trackbar()
{
	high_H = max(high_H, low_H + 1);
	cv::setTrackbarPos("High H", "Video Capture", high_H);
}

void HSVPicker::on_low_S_thresh_trackbar()
{
	low_S = min(high_S - 1, low_S);
	cv::setTrackbarPos("Low S", "Video Capture", low_S);
}

void HSVPicker::on_high_S_thresh_trackbar()
{
	high_S = max(high_S, low_S + 1);
	cv::setTrackbarPos("High S", "Video Capture", high_S);
}

void HSVPicker::on_low_V_thresh_trackbar()
{
	low_V = min(high_V - 1, low_V);
	cv::setTrackbarPos("Low V", "Video Capture", low_V);
}

void HSVPicker::on_high_V_thresh_trackbar()
{
	high_V = max(high_V, low_V + 1);
	cv::setTrackbarPos("High V", "Video Capture", high_V);
}

void HSVPicker::windowAndTrackBar()
{
	cv::namedWindow("Video Capture");

	// Trackbars to set thresholds for HSV values
	cv::createTrackbar("Low H", "Video Capture", &low_H, max_value_H, lowHtrackbar, this);
	cv::createTrackbar("High H", "Video Capture", &high_H, max_value_H, highHtrackbar, this);
	cv::createTrackbar("Low S", "Video Capture", &low_S, max_value, lowStrackbar, this);
	cv::createTrackbar("High S", "Video Capture", &high_S, max_value, highStrackbar, this);
	cv::createTrackbar("Low V", "Video Capture", &low_V, max_value, lowVtrackbar, this);
	cv::createTrackbar("High V", "Video Capture", &high_V, max_value, highVtrackbar, this);
}

cv::Mat HSVPicker::threshold(cv::Mat &frame)
{
	cv::Mat frame_HSV, frame_threshold;
	// Convert from BGR to HSV colorspace
	cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
	// Detect the object based on HSV Range Values and store those HSV values
	cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
	this->lowerThresh = cv::Scalar(low_H, low_S, low_V); this->upperThresh = cv::Scalar(high_H, high_S, high_V);
	//mask the original image
	cv::Mat masked(frame.size(), frame.type(), cv::Scalar(0));
	cv::bitwise_and(frame, frame, masked, frame_threshold);
	return masked;
}

HSVPicker::HSVThresh HSVPicker::getThreshValues()
{
	HSVThresh myThresh; 
	myThresh.theLowerThresh = this->lowerThresh; 
	myThresh.theUpperThresh = this->upperThresh;
	return myThresh;
}

cv::Mat HSVPicker::applyHSV(cv::Mat &img, const char* filename)
{
	//I only want to read the XML file once to get HSV values
	if (!this->gotHSV)
	{
		int lowH, lowS, lowV, highH, highS, highV;
		tinyxml2::XMLDocument reader;
		tinyxml2::XMLError myReader = reader.LoadFile(filename);    //load the hsv xml file
		auto theRoot = reader.FirstChildElement("HSV");    //the first child of your cursor is HSV

		//start getting the lower and upper threshold values (no need for a loop)
		auto theElement = theRoot->FirstChildElement("lowerHThresh");
		myReader = theElement->QueryIntText(&lowH);
		theElement = theElement->NextSiblingElement("upperHThresh");
		myReader = theElement->QueryIntText(&highH);
		theElement = theElement->NextSiblingElement("lowerSThresh");
		myReader = theElement->QueryIntText(&lowS);
		theElement = theElement->NextSiblingElement("upperSThresh");
		myReader = theElement->QueryIntText(&highS);
		theElement = theElement->NextSiblingElement("lowerVThresh");
		myReader = theElement->QueryIntText(&lowV);
		theElement = theElement->NextSiblingElement("upperVThresh");
		myReader = theElement->QueryIntText(&highV);

		//store them here so you don't have to open and read the xml file every time
		this->lowerThresh = cv::Scalar(lowH, lowS, lowV); this->upperThresh = cv::Scalar(highH, highS, highV); this->gotHSV = true;
	}

	cv::Mat newImg = img.clone(), frame_HSV, frame_threshold;
	// Convert from BGR to HSV colorspace
	cv::cvtColor(newImg, frame_HSV, cv::COLOR_BGR2HSV);
	// Detect the object based on HSV Range Values
	cv::inRange(frame_HSV, lowerThresh, upperThresh, frame_threshold);
	//mask the original image
	cv::Mat masked(newImg.size(), newImg.type(), cv::Scalar(0));
	cv::bitwise_and(newImg, newImg, masked, frame_threshold);
	return masked;
}

HSVPicker::~HSVPicker() {}