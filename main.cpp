//Copyright 2018, Sarim Mehdi, All rights reserved.

#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "additional libraries/tinyxml2.h"
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <ctime>
#include <cmath>
#include <vector>

//uncomment this line if you want to see debug images
//#define DEBUG

using namespace std;
using namespace cv;

//my controller libraries
#include "additional libraries/LaneTracking.h"
#include "additional libraries/CameraCalib.h"
#include "additional libraries/ArucoAndMap.h"
#include "additional libraries/InputAndSimulation.h"
#include "additional libraries/HSVPicker.h"

//your chessboard dimensions
const Size myChessBoard(9, 6);
const float LengthOfSquare = 0.06624;

//your aruco marker length
const float myArucoLength = 0.6503;

int main()
{
	clock_t loadTime, logTime, beginResize, trainTime, manTime;          //variables for logging time
	vector<bool> myBools; bool myTemp;       //booleans used to initialize objects of each class

	//load the xml file for setting booleans
	const char* cliFile = "XMLfiles/cli_args.xml";    
	tinyxml2::XMLDocument cliReader;
	tinyxml2::XMLError mycliReader = cliReader.LoadFile(cliFile);
	auto thecliRoot = cliReader.FirstChildElement("CLI"); 
	auto thecliElement = thecliRoot->FirstChildElement("argv");
	mycliReader = thecliElement->QueryBoolText(&myTemp); 
	while (thecliElement != nullptr) 
	{ 
		mycliReader = thecliElement->QueryBoolText(&myTemp);
		myBools.push_back(myTemp);
		thecliElement = thecliElement->NextSiblingElement("argv"); 
	}

	//load the controller xml file and get the values
	const char* controlFile = "XMLfiles/controller.xml";
	tinyxml2::XMLDocument cntReader;
	tinyxml2::XMLError mycntReader = cntReader.LoadFile(controlFile);
	auto thecntRoot = cntReader.FirstChildElement("controller"); float prop, diff, integ; int maxS, minS;
	auto thecntElement = thecntRoot->FirstChildElement("Kp"); mycntReader = thecntElement->QueryFloatText(&prop);
	thecntElement = thecntElement->NextSiblingElement("Kd"); mycntReader = thecntElement->QueryFloatText(&diff);
	thecntElement = thecntElement->NextSiblingElement("Ki"); mycntReader = thecntElement->QueryFloatText(&integ);
	thecntElement = thecntElement->NextSiblingElement("maxSpeed"); mycntReader = thecntElement->QueryIntText(&maxS);
	thecntElement = thecntElement->NextSiblingElement("minSpeed"); mycntReader = thecntElement->QueryIntText(&minS);

	//load the maneuver xml file and get list of maneuvers: 0 = left, 1 = right, 2 = straight, 3 = park
	const char* manFile = "XMLfiles/maneuver.xml";
	tinyxml2::XMLDocument manReader;
	tinyxml2::XMLError mymanReader = manReader.LoadFile(manFile);
	auto themanRoot = manReader.FirstChildElement("maneuverList"); vector<int> maneuevrs;
	auto themanElement = themanRoot->FirstChildElement("maneuver"); string manName;
	while (themanElement != nullptr)
	{
		manName = themanElement->Attribute("name");
		if (manName.compare("left") == 0) { maneuevrs.push_back(0); }
		else if (manName.compare("right") == 0) { maneuevrs.push_back(1); }
		else if (manName.compare("straight") == 0) { maneuevrs.push_back(2); }
		else if (manName.compare("park") == 0) { maneuevrs.push_back(3); }
		themanElement = themanElement->NextSiblingElement("maneuver");
	}

	LaneTracker myTracker(prop, diff, integ, maxS, minS, "XMLfiles/maneuver.xml", myBools[0]); //set car controller here
	HSVPicker myHSV(myBools[1]);    //threshold image?
	ArucoAndMap myArucoMap(myBools[3]);  //generate new aruco markers?
	if (myArucoMap.doYouWantAruco()) { myArucoMap.CreateArucoMarkers(); }
	InputSim myInputSim(myBools[4]);      //send input to simulation?
	CameraCalib myCalib(myBools[2]); Mat theCallibs = Mat(cv::Size(3, 3), CV_64F), theDistCoefs = Mat::zeros(cv::Size(1, 5), CV_64F);   //callibrate camera?
	//CALL THIS WHENEVER YOU WANT TO CALLIBRATE YOUR CAMERA. ALSO, MAKE SURE ALL YOUR CALLIBRATION IMAGES ARE
	//LOCATED IN callibration_images FOLDER IN YOUR PROJECT FOLDER
	if (myCalib.doYouWantCameraMatrix())
	{
		Mat myDistCoeffs = Mat::zeros(8, 1, CV_64F); Mat myCameraMatrix = Mat::eye(3, 3, CV_64F);
		myCalib.cameraCallibration(myChessBoard, LengthOfSquare, myCameraMatrix, myDistCoeffs);
		myCalib.saveCameraCallibration(myCameraMatrix, myDistCoeffs);
		myCalib.getCallibration("XMLfiles/camera.xml", theCallibs, theDistCoefs);
	}
	else
	{
		myCalib.getCallibration("XMLfiles/camera.xml", theCallibs, theDistCoefs);
	}

	Mat theMap = myArucoMap.drawMap("XMLfiles/map.xml", "XMLfiles/roadsigns.xml"); //draw the map

	//This structure will be used to create the keyboard input event
	INPUT inputKey; memset(&inputKey, 0, sizeof(inputKey));
	myInputSim.InitializeKey(inputKey);  //initialize your keys here

	//Set the name of the window where your game or simulator is running
	LPCTSTR WindowName = "Unity 2018.2.18f1 Personal (64bit) - SampleScene.unity - OpenCV_TEST - PC, Mac & Linux Standalone <DX11>";
	//LPCTSTR WindowName = "Unity 2018.2.15f1 Personal (64bit) - CharacterFirstPerson.unity - OpenCV_TEST - PC, Mac & Linux Standalone <DX11>";
	HWND GameWindow = FindWindow(NULL, WindowName); //get handle to game window
	int key = 0;
	int midPoint = myInputSim.hwnd2mat(GameWindow).cols / 2;     //midpoint is the center of the screen

	//for saving images for calibration
	//ostringstream save;
	//string imageName = "calibImg_";
	//int saveImageCounter = 0;

	double frameTime; int dataType; string folderName, resizedFolderName, resizedImageName;

	//gather data
	if (myBools[6])
	{
		cout << "What kind of data do you want to gather? (0 = training, 1 = validation, 2 = test)" << endl; cin >> dataType;
		switch (dataType)
		{
		case 0: folderName = "training_images"; break;
		case 1: folderName = "validation_images"; break;
		case 2: folderName = "test_images"; break;
		}

		if (myArucoMap.dirExists(folderName) == false) { string folderCreateCommand = "mkdir " + folderName; system(folderCreateCommand.c_str()); }
	}
	
	//label data (MAKE SURE THE FOLDERS HAVE THE EXACT NAME AS IN THIS IF-CONDITION)
	if (myBools[7])
	{
		cout << "What kind of data do you want to label? (0 = training, 1 = validation, 2 = test)" << endl; cin >> dataType;
		switch (dataType)
		{
		case 0: folderName = "training_images"; break;
		case 1: folderName = "validation_images"; break;
		case 2: folderName = "test_images"; break;
		}
	}

	Mat src, src_copy, src_HSV, resizedImg;
	ArucoAndMap::markerAndMap receivedValMarkMap; receivedValMarkMap.nextMan = false;
	receivedValMarkMap.screenX = midPoint; receivedValMarkMap.manIter = 0; receivedValMarkMap.horizontalLimitR = 200;
	receivedValMarkMap.timeLimit = 4000; receivedValMarkMap.interDist = 15;
	LaneTracker::processedImageAndLines receivedValForHoughLines;
	int steeringValueToCar, speed, digit1, digit2, speedDigit1, speedDigit2, speedDigit3, straightCounter = 0, straightSteering = -2;
	bool prevOnJuntion = false; int varForPrint, trainingImg = 0, junction = 0;
	
	ostringstream convert; string imageName = "data_"; string fullpath;
	ostringstream resizedConvert; resizedImageName = "final_data_"; string resizedFullpath;
	vector<String> fn; vector<Mat> trainingImages, validImages, testImages; int count, labelCounter = 0, labelSpeed = 1; bool allLabeled = false;
	bool repeatUntilAll = false, allResized = false; if (myBools[8] == true || myBools[10] == true) { repeatUntilAll = true; }
	bool allTrained = false;

	//load images for labeling
	if (myBools[7])
	{
		glob("C:/Users/sarim/Downloads/OpenCV_Project/OpenCV_Project/" + folderName +  "/*.jpg", fn);
		count = fn.size(); //number of jpg files in images folder

		//make sure you load all the images sequentially
		loadTime = clock();
		for (size_t i = 0; i < count; i++) 
		{ 
			convert << folderName << "/" << imageName << i << ".jpg";
			fullpath = convert.str();
			trainingImages.push_back(imread(fullpath, IMREAD_COLOR));
			convert.str("");
			cout << '\r' << (int)(((double)(i + 1) / count) * 100); cout.flush();
		}
		cout << endl;
		cout << "LOADING ALL IMAGES TOOK " << double(clock() - loadTime) / CLOCKS_PER_SEC << " SECONDS" << endl;
		cout << trainingImages.size() << " IMAGES LOADED" << endl; cout << endl;
		cout << "How many seconds do you want to wait before the next image is processed? (only integers and in milliseconds)" << endl;
		cin >> labelSpeed;
	}

	//load all images for resizing (MAKE SURE THE FOLDERS HAVE THE EXACT NAME AS IN THIS IF-CONDITION)
	int i_repeat = 0; vector<Mat> placeHolder; loadTime = clock(); if (myBools[10]) { imageName = "final_data_"; }
	while (repeatUntilAll)
	{
		switch (i_repeat)
		{
			case 0: folderName = "training_images"; cout << "LOADING TRAINING IMAGES" << endl; break;
			case 1: folderName = "validation_images"; cout << "LOADING VALIDATION IMAGES" << endl; break;
			case 2: folderName = "test_images"; cout << "LOADING TEST IMAGES" << endl; repeatUntilAll = false; break;
		}
		if (myBools[10]) { folderName = "final_" + folderName; }
		glob("C:/Users/sarim/Downloads/OpenCV_Project/OpenCV_Project/" + folderName + "/*.jpg", fn);
		count = fn.size(); //number of jpg files in images folder

		//make sure you load all the images sequentially
		cout << "PERCENT IMAGES LOADED:" << endl;
		for (size_t i = 0; i < count; i++)
		{
			convert << folderName << "/" << imageName << i << ".jpg";
			fullpath = convert.str();
			switch (i_repeat)
			{
			case 0: trainingImages.push_back(imread(fullpath, IMREAD_COLOR)); break;
			case 1: validImages.push_back(imread(fullpath, IMREAD_COLOR)); break;
			case 2: testImages.push_back(imread(fullpath, IMREAD_COLOR)); break;
			}
			convert.str("");
			cout << '\r' << (int)(((double)(i+1) / count) * 100); cout.flush();
		}
		cout << endl; cout << endl; i_repeat++;
	}
	int control = 0;

	//placeholder for my SNN
	if (myBools[8] == true || myBools[10] == true)
	{
		cout << "LOADING ALL IMAGES TOOK " << double(clock() - loadTime) / CLOCKS_PER_SEC << " SECONDS" << endl;
		cout << trainingImages.size() << " TRAINING IMAGES LOADED" << endl;
		cout << validImages.size() << " VALIDATION IMAGES LOADED" << endl;
		cout << testImages.size() << " TEST IMAGES LOADED" << endl;
		if (myBools[8]) { cout << "RESIZING IMAGES" << endl; beginResize = clock(); i_repeat = 0; repeatUntilAll = true; cout << endl; }
		if (myBools[10]) 
		{ 
			cout << "Do you want control during training? (1 = yes, 2 = timer, 0 = no)" << endl; cin >> control;
			if (control == 2) 
			{
				cout << "How many seconds do you want to wait before the next data is processed? (only integers and in milliseconds)" << endl;
				cin >> labelSpeed;
			}
		}
	}

	//record video of lane and hough lines for debugging purpose
	VideoWriter out_capture("out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(myInputSim.hwnd2mat(GameWindow).cols, myInputSim.hwnd2mat(GameWindow).rows), true);
	VideoWriter out_captureHough("outHough.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(myInputSim.hwnd2mat(GameWindow).cols, myInputSim.hwnd2mat(GameWindow).rows), true);
	VideoWriter out_captureMap("outMap.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(theMap.cols, theMap.rows), true);
	VideoWriter out_captureAruco("outAruco.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(myInputSim.hwnd2mat(GameWindow).cols, myInputSim.hwnd2mat(GameWindow).rows), true);
	VideoWriter out_captureOriginal("outOriginal.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(myInputSim.hwnd2mat(GameWindow).cols, myInputSim.hwnd2mat(GameWindow).rows), true);

	//create trackbars if you want to get HSV
	if (myHSV.doYouWantHSV()) { myHSV.windowAndTrackBar(); }

	//xml file for training
	tinyxml2::XMLDocument trainingDoc;
	tinyxml2::XMLNode * tRoot = trainingDoc.NewElement("trainingData");  
	trainingDoc.InsertFirstChild(tRoot);
	tinyxml2::XMLElement *tElement = trainingDoc.NewElement("junction");
	trainTime = clock();
	if (myBools[6]) { cout << "RECORDING DATA..." << endl; }

	while (key != 27)        
	{
		//ALWAYS MAKE SURE SIMULATION WINDOW IS OPEN NO MATTER WHAT YOU ARE DOING OR ELSE YOU WILL GET ERROR

		logTime = clock();
		src = myInputSim.hwnd2mat(GameWindow);          //convert your game screen to an opencv Mat 
		if (myBools[9])
		{
			cout << "CONVERTING SIMULATION WINDOW TO OPENCV MAT TOOK " << double(clock() - logTime) / CLOCKS_PER_SEC << " SECONDS" << endl;
		}

		//execute simulation if you are not looking for HSV threshold or labeling data
		if (myHSV.doYouWantHSV() == false && myBools[7] == false)
		{
			loadTime = clock();      //start recording time after taking frame
			myInputSim.SetTheCallback(); //Set callback
			src_HSV = myHSV.applyHSV(src, "XMLfiles/hsv.xml");  //apply HSV threshold
			cvtColor(src, src_copy, COLOR_BGRA2BGR);     //aruco marker detection need 3 channel image

			//save << imageName << saveImageCounter << ".jpg";
			//string finalPath = save.str();
			//save.str(" ");
			//imwrite(finalPath, src);
			//saveImageCounter++;			

			logTime = clock();
			receivedValMarkMap.distToMarker = 100; receivedValMarkMap.rightLane = true;
			myArucoMap.DetectArucoMarkers(src_copy, theMap, myArucoLength, theCallibs, theDistCoefs, receivedValMarkMap);
			if (myBools[9])
			{
				cout << "DETECTING ARUCO MARKERS TOOK " << double(clock() - logTime) / CLOCKS_PER_SEC << " SECONDS" << endl;
			}

			//click to get ROI points for tracking lane
			if (myTracker.doYouWantROI()) { myInputSim.OnMouseClick(); }

			receivedValForHoughLines = myTracker.process_img(src_HSV, myBools[6]);    //get hough lines

			//draw lanes using hough lines
			if (!receivedValForHoughLines.theLines.empty())   
			{
				logTime = clock();
				steeringValueToCar = myTracker.draw_lanes(src_HSV, receivedValForHoughLines.theNoROIImage, myBools[6], midPoint, 30);
				if (myBools[9])
				{
					cout << "GETTING STEERING VALUE " << double(clock() - logTime) / CLOCKS_PER_SEC << " SECONDS" << endl;
				}
			}

			//get to right lane if you are in left lane
			if (!receivedValMarkMap.rightLane) { steeringValueToCar = 15; }

			//calculate speed based on inverse relationship between speed and absolute value of steering
			speed = myTracker.calculateSpeed(abs(steeringValueToCar), 0.5);

			/*execute maneuver if you haven't reached the end of the maneuver list
			IDEA: For example, if I have to turn right at a junction, I manually turn right but if hough lines are already telling me to turn
			right, I just follow them instead. Same idea for left*/
			if (receivedValMarkMap.nextMan == true && receivedValMarkMap.manIter < maneuevrs.size())
			{
				switch (maneuevrs[receivedValMarkMap.manIter])
				{
				case(0):
					if ((steeringValueToCar >= 0 || steeringValueToCar <= -10) && (double)(clock() - receivedValMarkMap.manTime) >= 1500)
					{ 
						steeringValueToCar = -5; speed = 200; 
					}
					else if ((double)(clock() - receivedValMarkMap.manTime) < 1500) { steeringValueToCar = -1; speed = 200; }
					else { steeringValueToCar = -5; speed = 200; }
					putText(src, "EXECUTING MANEUVER: LEFT", cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
					break;
				case(1):
					if ((steeringValueToCar >= 10 || steeringValueToCar <= 0) && (double)(clock() - receivedValMarkMap.manTime) >= 1500)
					{ 
						steeringValueToCar = 5; speed = 200; 
					}
					else if ((double)(clock() - receivedValMarkMap.manTime) < 1500) { steeringValueToCar = 1; speed = 200; }
					else { steeringValueToCar = 5; speed = 200; }
					putText(src, "EXECUTING MANEUVER: RIGHT", cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
					break;
				case(2):     //finicky way, best thing I could think of at the moment (wiggle left and right through a junction)
					if ((double)(clock() - receivedValMarkMap.manTime) < straightCounter * 1000) 
					{ 
						steeringValueToCar = straightSteering; speed = 200; 
					}
					else { straightCounter++; straightSteering = -straightSteering; }
					steeringValueToCar = -steeringValueToCar; speed = 150;
					putText(src, "EXECUTING MANEUVER: STRAIGHT", cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
					break;
				}
			}
			else if (receivedValMarkMap.manIter == maneuevrs.size())
			{ 
				putText(src, "ALL MANEUVERS HAVE BEEN EXECUTED!", cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
				steeringValueToCar = 0; speed = 0; 
			}

			//reset finicky variables every time you complete a maneuver
			if (!receivedValMarkMap.nextMan) { straightCounter = 0; straightSteering = -2; }

			//finicky way to avoid collision with aruco marker on intersection (because in simulation I only have camera)
			if (receivedValMarkMap.distToMarker <= 7) { steeringValueToCar = -steeringValueToCar + receivedValMarkMap.signBias; }

			//gather training data
			if (myBools[6]) 
			{ 
				convert << folderName << "/" << imageName << trainingImg << ".jpg";
				fullpath = convert.str();
				convert.str("");
				imwrite(fullpath, receivedValForHoughLines.theNoROIImage);
				trainingImg++;
			}

			//separate the three-number speed value into the three digits
			speedDigit1 = speed / 100;     //left digit of the number
			speedDigit2 = (speed % 100) / 10;     //middle digit of the number
			speedDigit3 = (speed % 100) % 10;     //right digit of the number

			//calculate time difference and display on main game window
			frameTime = double(clock() - loadTime) / CLOCKS_PER_SEC;

			//set result to two decimal places before displaying
			if (frameTime != 0)
			{
				double average = 1 / frameTime; stringstream stream;  stream << fixed << setprecision(2) << average; string s = stream.str();
				putText(src_HSV, "FRAME-RATE: " + s, cv::Point(350, 220), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			}

			if (myTracker.doYouWantROI()) { imshow("CLICK TO SET ROI POINT", src_HSV); }    //image for ROI seen here

			//you can visualize the detected points of your chessboard here
			if (myCalib.doYouWantCameraMatrix()) { imshow("chessboard", myCalib.SeeChessBoardPoints(myChessBoard, src)); }
#ifdef DEBUG
			imshow("hough lines", receivedValForHoughLines.theImage);
			imshow("what the car sees", src);
			imshow("map", receivedValMarkMap.theMapSign);
			imshow("markers", receivedValMarkMap.theMarker);
#endif

			//convert to BGR before writing to a video file (otherwise the video file is not made)
			cvtColor(receivedValMarkMap.theMarker, receivedValMarkMap.theMarker, COLOR_RGB2BGR);
			cvtColor(src_HSV, src_HSV, COLOR_RGB2BGR); cvtColor(src, src, COLOR_RGB2BGR);
			cvtColor(receivedValForHoughLines.theImage, receivedValForHoughLines.theImage, COLOR_GRAY2BGR);
			out_captureMap.write(receivedValMarkMap.theMapSign);   //no need to convert before saving map
			out_captureAruco.write(receivedValMarkMap.theMarker);
			out_capture.write(src_HSV); out_captureOriginal.write(src);
			out_captureHough.write(receivedValForHoughLines.theImage);

			if (myInputSim.checkInputStatus())
			{
				//press and release minus key if negative steering
				if (steeringValueToCar < 0)
				{
					myInputSim.PressKeyNormal(inputKey, 0xBD); myInputSim.ReleaseKeyNormal(inputKey, 0xBD);

					//make steering positive because we send the minus sign separately later on
					steeringValueToCar = -steeringValueToCar;
				}

				//separate the two-number steering value into the two digits
				digit1 = steeringValueToCar / 10;     //left digit of the number
				digit2 = steeringValueToCar % 10;     //right digit of the number

				//press and release the key for left digit of your steering number
				myInputSim.PressKey(inputKey, digit1); myInputSim.ReleaseKey(inputKey, digit1);

				//press and release the key for right digit of your steering number
				myInputSim.PressKey(inputKey, digit2); myInputSim.ReleaseKey(inputKey, digit2);

				//press and release key for '.' to start giving digits for speed
				myInputSim.PressKeyNormal(inputKey, 0xBE); myInputSim.ReleaseKeyNormal(inputKey, 0xBE);

				//press and release the key for left digit of your speed number
				myInputSim.PressKey(inputKey, speedDigit1); myInputSim.ReleaseKey(inputKey, speedDigit1);

				//press and release the key for middle digit of your speed number
				myInputSim.PressKey(inputKey, speedDigit2); myInputSim.ReleaseKey(inputKey, speedDigit2);

				//press and release the key for right digit of your speed number
				myInputSim.PressKey(inputKey, speedDigit3); myInputSim.ReleaseKey(inputKey, speedDigit3);

				//press and release the enter key to send steering and speed value to car
				myInputSim.PressKeyNormal(inputKey, 0x0D); myInputSim.ReleaseKeyNormal(inputKey, 0x0D);
			}
		}	

		//get HSV values
		if (myHSV.doYouWantHSV() == true)
		{ 
			imshow("Object Detection", myHSV.threshold(src)); 
		}

		//label training data, Q = junction and E = not junction (only responds to lower case press of the button)
		if (myBools[7] == true && allLabeled == false)
		{
			putText(trainingImages[labelCounter], "TRAINING IMAGE " + to_string(labelCounter), cv::Point(350, 70), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			if (junction == 0)
			{
				putText(trainingImages[labelCounter], "LABELING IMAGES AS NON-JUNCTION", cv::Point(350, 170), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			}
			else
			{
				putText(trainingImages[labelCounter], "LABELING IMAGES AS JUNCTION", cv::Point(350, 170), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			}
			putText(trainingImages[labelCounter], "Q TO LABEL AS JUNCTION", cv::Point(350, 210), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			putText(trainingImages[labelCounter], "E TO LABEL AS NON-JUNCTION", cv::Point(350, 270), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(200, 200, 250));
			imshow("LABEL ME!", trainingImages[labelCounter]);
			tElement->SetAttribute("Image", labelCounter); tElement->SetAttribute("Verdict", junction); tRoot->InsertEndChild(tElement);
			tElement = trainingDoc.NewElement("junction");

			//press Q
			if (key == 113) { junction = 1; }

			//press E
			else if (key == 101) { junction = 0; }
			labelCounter++;
			if (labelCounter == trainingImages.size()) { cout << "YOU HAVE SUCCESSFULLY LABELED ALL IMAGES" << endl; allLabeled = true; }
		}

		//compress the labeled data to 32 by 32
		if (myBools[8] == true && allResized == false)
		{
			if (repeatUntilAll)
			{
				switch (i_repeat)
				{
				case 0: placeHolder = trainingImages; repeatUntilAll = false; cout << "PERCENTAGE OF TRAINING IMAGES RESIZED:" << endl;
					resizedFolderName = "final_training_images"; labelCounter = 0; break;
				case 1: placeHolder = validImages; repeatUntilAll = false; cout << "PERCENTAGE OF VALIDATION IMAGES RESIZED:" << endl;
					resizedFolderName = "final_validation_images"; labelCounter = 0; break;
				case 2: placeHolder = testImages; repeatUntilAll = false; cout << "PERCENTAGE OF TEST IMAGES RESIZED:" << endl;
					resizedFolderName = "final_test_images"; labelCounter = 0; break;
				}
			}
 			resize(placeHolder[labelCounter], placeHolder[labelCounter], Size(32, 32), 0, 0, INTER_CUBIC);
			resizedConvert << resizedFolderName << "/" << resizedImageName << labelCounter << ".jpg";
			resizedFullpath = resizedConvert.str();
			resizedConvert.str("");
			imwrite(resizedFullpath, placeHolder[labelCounter]);
			labelCounter++;
			cout << '\r' << (int)(((double)(labelCounter + 1) / placeHolder.size()) * 100); cout.flush();
			if (labelCounter == placeHolder.size()) 
			{ 
				cout << endl;
				repeatUntilAll = true;
				if (i_repeat == 2)
				{
					cout << "YOU HAVE SUCCESSFULLY RESIZED ALL IMAGES" << endl; allResized = true;
					cout << "RESIZING ALL IMAGES TOOK " << double(clock() - beginResize) / (60 * CLOCKS_PER_SEC) << " MINUTES" << endl;
				}
				i_repeat++; cout << endl;
			}
		}

		//wait for esc key to be pressed (while labeling your data, console will ask you to give this value)
		key = waitKey(labelSpeed);
	}

	//close the video writers
	out_capture.release(); out_captureHough.release(); out_captureMap.release(); out_captureAruco.release(); out_captureOriginal.release();

	//store labeled data in the xml file
	if (myBools[7] == true) 
	{ 
		tElement = trainingDoc.NewElement("numberOfLabeledImages");
		tElement->SetText(labelCounter); tRoot->InsertEndChild(tElement);
		switch (dataType)
		{
		case 0: trainingDoc.SaveFile("XMLfiles/training_labels.xml"); break;
		case 1: trainingDoc.SaveFile("XMLfiles/validation_labels.xml"); break;
		case 2: trainingDoc.SaveFile("XMLfiles/test_labels.xml"); break;
		}
		cout << "LABELING DATA TOOK " << double(clock() - logTime) / (60 * CLOCKS_PER_SEC) << " MINUTES" << endl;
	}

	//store HSV threshold values in an xml file
	if (myHSV.doYouWantHSV())
	{
		HSVPicker::HSVThresh threshStuff = myHSV.getThreshValues();

		tinyxml2::XMLDocument xmlDoc;
		tinyxml2::XMLNode * pRoot = xmlDoc.NewElement("HSV");    //create the root of your xml file
		xmlDoc.InsertFirstChild(pRoot);     //put the root in your xml file

		tinyxml2::XMLElement *pElementLower = xmlDoc.NewElement("lowerHThresh");
		tinyxml2::XMLElement *pElementUpper = xmlDoc.NewElement("upperHThresh");
		//store the lower threshold values
		for (int i = 0; i < 3; i++)
		{
			switch (i)
			{
			case 0:
				break;
			case 1:
				pElementLower = xmlDoc.NewElement("lowerSThresh");
				pElementUpper = xmlDoc.NewElement("upperSThresh");
				break;
			case 2:
				pElementLower = xmlDoc.NewElement("lowerVThresh");
				pElementUpper = xmlDoc.NewElement("upperVThresh");
				break;
			}
			pElementLower->SetText(threshStuff.theLowerThresh[i]);
			pRoot->InsertEndChild(pElementLower);
			pElementUpper->SetText(threshStuff.theUpperThresh[i]);
			pRoot->InsertEndChild(pElementUpper);
		}
		tinyxml2::XMLError eResult = xmlDoc.SaveFile("XMLfiles/hsv.xml");
	}

	//store all your clicked ROI coordinates in an xml file here
	if (myTracker.doYouWantROI())
	{
		InputSim::ROIdata roiStuff = myInputSim.giveMeROI();

		tinyxml2::XMLDocument xmlDoc;
		tinyxml2::XMLNode * pRoot = xmlDoc.NewElement("points");    //create the root of your xml file
		xmlDoc.InsertFirstChild(pRoot);     //put the root in your xml file

		//store the first child of points here along with its data
		tinyxml2::XMLElement * pElement = xmlDoc.NewElement("number");
		pElement->SetText(roiStuff.ROIcoordnum);
		pRoot->InsertEndChild(pElement);

		//start adding coordinates
		pElement = xmlDoc.NewElement("coordinates");
		for (int i = 0; i < roiStuff.ROIcoordnum; i++)
		{
			//first put the x value
			tinyxml2::XMLElement * pListElementX = xmlDoc.NewElement("xCoord");
			pListElementX->SetText(roiStuff.theROIcoords[i].x);
			pElement->InsertEndChild(pListElementX);

			//and then put the y value
			tinyxml2::XMLElement * pListElementY = xmlDoc.NewElement("yCoord");
			pListElementY->SetText(roiStuff.theROIcoords[i].y);
			pElement->InsertEndChild(pListElementY);
		}
		pRoot->InsertEndChild(pElement);   //close your list of coordinates
		tinyxml2::XMLError eResult = xmlDoc.SaveFile("XMLfiles/roi.xml");
	}

	return 0;
}