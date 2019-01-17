//Taken from https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#pragma once
#ifndef HSVPICKER_H
#define HSVPICKER_H

class HSVPicker
{
private:
	bool getHSV;   //check if you want to get HSV to threshold image to get road only
	bool gotHSV;   //check if you have loaded the HSV threshold values from the xml file

	//store your thresh values
	cv::Scalar lowerThresh;
	cv::Scalar upperThresh;

	const int max_value_H = 360 / 2;
	const int max_value = 255;
	int low_H = 0, low_S = 0, low_V = 0;
	int high_H = max_value_H, high_S = max_value, high_V = max_value;

	//static functions from where you go to non-static function where you deal with changes to trackbar
	static void lowHtrackbar(int v, void* ptr);
	static void highHtrackbar(int v, void* ptr);
	static void lowStrackbar(int v, void* ptr);
	static void highStrackbar(int v, void* ptr);
	static void lowVtrackbar(int v, void* ptr);
	static void highVtrackbar(int v, void* ptr);

	void on_low_H_thresh_trackbar();
	void on_high_H_thresh_trackbar();
	void on_low_S_thresh_trackbar();
	void on_high_S_thresh_trackbar();
	void on_low_V_thresh_trackbar();
	void on_high_V_thresh_trackbar();
public:
	//struct for storing upper and lower threshold values of HSV
	typedef struct HSVThresh
	{
		cv::Scalar theLowerThresh;
		cv::Scalar theUpperThresh;
	} myHSVThresh;

	//call this to start segmenting image based on HSV
	bool doYouWantHSV() { return this->getHSV; }

	void windowAndTrackBar();    //draw the trackbar for adjusting HSV values
	HSVThresh getThreshValues();   //get your upper and lower threshold values to store in XML
	cv::Mat applyHSV(cv::Mat &img, const char* filename);   //apply HSV threshold to filter out everything except road lanes
	cv::Mat threshold(cv::Mat &frame);//this method thresholds your image based on HSV chosen via trackbar and returns a Mat for you to display

	HSVPicker(bool IWantHSV) : getHSV(IWantHSV), gotHSV(false) {}
	~HSVPicker();
};

#endif
