//Copyright 2018, Sarim Mehdi, All rights reserved.

#pragma once
#ifndef INPUTANDSIMULATION_H
#define INPUTANDSIMULATION_H

class InputSim
{
private:
	bool canISendInput;  //if true, it means your controller sends inputs to simulation otherwise no

	//the mouse callback function to be called once
	static void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
	{
		// resolve 'param':
		InputSim* that = reinterpret_cast<InputSim*>(param);
		that->mouseClick(event, x, y, flag);
	}

	//record mouse click event
	void mouseClick(int  event, int  x, int  y, int  flag)
	{
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			// Store point coordinates
			pt.x = x;
			pt.y = y;
			newCoords = true;
		}
	}

	//global variables to deal with mouse click
	cv::Point pt = cv::Point(-1, -1);
	bool newCoords = true;
	int numOfROIcoords = 0;
	vector <cv::Point> myROIcoords;
public:
	//struct for vector of ROI coordinates
	typedef struct ROIdata
	{
		vector <cv::Point> theROIcoords;
		int ROIcoordnum;
	} myROIdata;

	//function to return ROI data in the form of struct
	ROIdata giveMeROI() { ROIdata temp; temp.ROIcoordnum = this->numOfROIcoords; temp.theROIcoords = this->myROIcoords; return temp; }
	
	cv::Mat hwnd2mat(HWND hwnd);  //here we get the stream from our car simulator window

	void InitializeKey(INPUT &key) { key.type = INPUT_KEYBOARD; key.ki.wScan = 0; key.ki.time = 0; key.ki.dwExtraInfo = 0; }

	void PressKeyNormal(INPUT &key, WORD code) { key.ki.dwFlags = 0; key.ki.wVk = code; SendInput(1, &key, sizeof(INPUT)); }

	void ReleaseKeyNormal(INPUT &key, WORD code) { key.ki.dwFlags = KEYEVENTF_KEYUP; key.ki.wVk = code; SendInput(1, &key, sizeof(INPUT)); }

	//use this function to get coordinates of point where you clicked and store them in an xml file
	void OnMouseClick()
	{
		//Show last point clicked, if valid (uncomment the bottom function only when you want to get coordinates of a new ROI)
		if (pt.x != -1 && pt.y != -1)
		{
			if (newCoords)
			{
				std::cout << "Clicked coordinates: " << pt << std::endl;
				newCoords = false;
				numOfROIcoords++;
				myROIcoords.push_back(pt);
			}
		}
	}

	//check if you are allowed to send input
	bool checkInputStatus() { return this->canISendInput; }

	void SetTheCallback() { cv::setMouseCallback("THIS IS WHAT THE CAR SEES", mouse_callback, this); }

	void PressKey(INPUT &key, int &digit);

	void ReleaseKey(INPUT &key, int &digit);

	InputSim(bool allowInput = true) : canISendInput(allowInput) {}
	~InputSim();
};

#endif