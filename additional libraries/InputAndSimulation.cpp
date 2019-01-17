//Copyright 2018, Sarim Mehdi, All rights reserved.

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <Windows.h>
#include <iostream>
#include <stdlib.h>

using namespace std;

#include "InputAndSimulation.h"

cv::Mat InputSim::hwnd2mat(HWND hwnd)
{
	HDC hwindowDC, hwindowCompatibleDC;

	int height, width, srcheight, srcwidth;
	HBITMAP hbwindow;
	cv::Mat src;
	BITMAPINFOHEADER  bi;

	hwindowDC = GetDC(hwnd);
	hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);

	RECT windowsize;    // get the height and width of the screen
	GetClientRect(hwnd, &windowsize);

	srcheight = windowsize.bottom;
	srcwidth = windowsize.right + 380;
	height = windowsize.bottom / 1;  //change this to whatever size you want to resize to
	width = windowsize.right / 1;

	src.create(height, width, CV_8UC4);

	// create a bitmap
	hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
	bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
	bi.biWidth = width;
	bi.biHeight = -height;  //this is the line that makes it draw upside down or not
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	// use the previously created device context with the bitmap
	SelectObject(hwindowCompatibleDC, hbwindow);
	// copy from the window device context to the bitmap device context
	StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, 0, 100, srcwidth, srcheight, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO *)&bi, DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow

	// avoid memory leak
	DeleteObject(hbwindow);
	DeleteDC(hwindowCompatibleDC);
	ReleaseDC(hwnd, hwindowDC);

	return src;
}

void InputSim::PressKey(INPUT &key, int &digit)
{
	switch (digit)
	{
	case 0: key.ki.wVk = 0x30; break;
	case 1: key.ki.wVk = 0x31; break;
	case 2: key.ki.wVk = 0x32; break;
	case 3: key.ki.wVk = 0x33; break;
	case 4: key.ki.wVk = 0x34; break;
	case 5: key.ki.wVk = 0x35; break;
	case 6: key.ki.wVk = 0x36; break;
	case 7: key.ki.wVk = 0x37; break;
	case 8: key.ki.wVk = 0x38; break;
	case 9: key.ki.wVk = 0x39; break;
	}

	key.ki.dwFlags = 0;  // 0 for key press
	SendInput(1, &key, sizeof(INPUT));
}

void InputSim::ReleaseKey(INPUT &key, int &digit)
{
	switch (digit)
	{
	case 0: key.ki.wVk = 0x30; break;
	case 1: key.ki.wVk = 0x31; break;
	case 2: key.ki.wVk = 0x32; break;
	case 3: key.ki.wVk = 0x33; break;
	case 4: key.ki.wVk = 0x34; break;
	case 5: key.ki.wVk = 0x35; break;
	case 6: key.ki.wVk = 0x36; break;
	case 7: key.ki.wVk = 0x37; break;
	case 8: key.ki.wVk = 0x38; break;
	case 9: key.ki.wVk = 0x39; break;
	}

	key.ki.dwFlags = KEYEVENTF_KEYUP;  // release the key
	SendInput(1, &key, sizeof(INPUT));
}

InputSim::~InputSim() {}