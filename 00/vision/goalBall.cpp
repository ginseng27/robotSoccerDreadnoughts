// Dreadnoughts
// VISION HQ

#include <stdio.h>
#include <time.h>
#include <string>

#include <vector>
#include <exception>

#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <fstream>

#include <cmath>

using namespace std;
using namespace cv;

void setYellow();
void setBlue();
void setGreen();
void setRed();
void setOrange();


ifstream infile;
// Open CV vars
//initial min and max HSV filter values.
//these will be changed using trackbars
bool graphics = false;
bool trackObjects = true;
bool useMorphOps = true;

float depth = 0;
int x = 0, y = 0;
int yx = 0, yy = 0;
int zx = 0, zy = 0;
double yArea,rArea = 0;
//int bx=0,by=0;
int bx[2];
int by[2];
int bw[2];
//int gx=0,gy=0;
int gx[2];
int gy[2];
int gw[2];
//int ox=0,oy=0;
int ox[2];
int oy[2];
int ow[2];
//int rx=0,ry=0;
int rx[2];
int ry[2];
int rw[2];


//Pillar Data
int pillar1x, pillar1y = -1;
int pillar2x, pillar2y = -1;
int pillar3x, pillar3y = -1;
int pillar4x, pillar4y = -1;
int pillar5x, pillar5y = -1;
int pillar6x, pillar6y = -1;
double pillar1d, pillar2d, pillar3d, pillar4d, pillar5d, pillar6d = -1;
float  pillar1a, pillar2a, pillar3a, pillar4a, pillar5a, pillar6a = -1;
//Ball Data
int ballx, bally = -1;
double balld = -1;
float  balla = -1;

//goal Data
double goald = -1;
float  goala = -1;

//Threshold
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

// NOONISH
//BALL
int Y_H_MIN = 0;
int Y_H_MAX = 256;
int Y_S_MIN = 0;
int Y_S_MAX = 256;
int Y_V_MIN = 0;
int Y_V_MAX = 256;

//BLUE PILLARS
int B_H_MIN = 0;
int B_H_MAX = 256;
int B_S_MIN = 0;
int B_S_MAX = 256;
int B_V_MIN = 0;
int B_V_MAX = 256;

//GREEN PILLARS
int G_H_MIN = 0;
int G_H_MAX = 256;
int G_S_MIN = 0;
int G_S_MAX = 256;
int G_V_MIN = 0;
int G_V_MAX = 256;

//ORANGE PILLARS
int O_H_MIN = 0;
int O_H_MAX = 256;
int O_S_MIN = 0;
int O_S_MAX = 256;
int O_V_MIN = 0;
int O_V_MAX = 256;

//RED PILLARS
int R_H_MIN = 0;
int R_H_MAX = 256;
int R_S_MIN = 0;
int R_S_MAX = 256;
int R_V_MIN = 0;
int R_V_MAX = 256;

// NIGHT TIME
/*
//BALL
int Y_H_MIN = 30;
int Y_H_MAX = 62;
int Y_S_MIN = 102;
int Y_S_MAX = 175;
int Y_V_MIN = 58;
int Y_V_MAX = 256;

//BLUE PILLARS
int B_H_MIN = 99;
int B_H_MAX = 121;
int B_S_MIN = 131;
int B_S_MAX = 209;
int B_V_MIN = 122;
int B_V_MAX = 233;

//GREEN PILLARS
int G_H_MIN = 82;
int G_H_MAX = 104;
int G_S_MIN = 80;
int G_S_MAX = 256;
int G_V_MIN = 92;
int G_V_MAX = 256;

//ORANGE PILLARS
int O_H_MIN = 3;
int O_H_MAX = 10;
int O_S_MIN = 123;
int O_S_MAX = 195;
int O_V_MIN = 245;
int O_V_MAX = 256;

//RED PILLARS
int R_H_MIN = 165;
int R_H_MAX = 181;
int R_S_MIN = 37;
int R_S_MAX = 218;
int R_V_MIN = 111;
int R_V_MAX = 256;
*/

int ballCount = 0;
bool objectFound = false;
ofstream myfile;

//SET HEIGHTS
#define CORNER_PILLAR_HEIGHT 0.182
#define MIDDLE_PILLAR_HEIGHT 0.13081
#define PI 3.14159265
#define NULL_VAL -999
#define REFAREA 23.24272072

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 15 * 15;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
// FOV 
const float FOV_HEIGHT = 63.2;
const float FOV_WIDTH = 49.3;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
void on_trackbar(int, void*)
{//This function gets called whenever a
	// trackbar position is changed

}

float horizontalAngle(int x) {
	double degreePixel = (double)(FOV_HEIGHT/2) / (FRAME_WIDTH/2);
	double angle = x * degreePixel;
	if (x < (FRAME_WIDTH / 2)) { //Left side
		return ((FOV_HEIGHT / 2) - angle)* PI / 180.0;
	}
	else { //Right Side
		return -(angle - (FOV_HEIGHT / 2))* PI / 180.0;
	}
}

float verticalAngle(int y) {
	double degreePixel = (FOV_WIDTH / 2) / (FRAME_HEIGHT/2);
	double angle = y * degreePixel;
	if (y < (FRAME_HEIGHT / 2)) {//up
		return (FOV_WIDTH / 2 - angle);
	}
	else { //down
		return -(angle - FOV_WIDTH / 2);
	}
}

double calculateDistance(bool cornerPillar, float vertAngle, int pixelWidth) {
	if (pixelWidth > 85) { // we are too close to get accurate depth
		return NULL_VAL;
	}
	/*else if (pixelWidth < 30) { // we are too far away for accurate depth
		return NULL_VAL;
	}*/
	else /*if (pixelWidth >= 39) */{ // we should use angle depth
		double distance;
		double tangent = tan(vertAngle * PI / 180.0);
		//d = (l*w)/  (2xtan(alpha/2)) where alpha=78?
		// where l = length of object (6cm). w = width of screen (640)
		// where x = width of object in pixels in picture
		if (cornerPillar) {
			return (CORNER_PILLAR_HEIGHT / tangent);
		}
		else {
			return (MIDDLE_PILLAR_HEIGHT / tangent);
		}
	}
	/*else { // we should use equation depth
		return ((pixelWidth - 139.8) / -9.6571)/10;
	}*/	
}

double calculateDistance(float vertAngle, int pixelWidth) {
    return calculateDistance(false, vertAngle, pixelWidth);
}

double calculateBallDistance(double area) {
    return 20/sqrt(area);
}

int calculateWidth(vector< vector<Point> > contours, int indexFound) {
	Point pt1, pt2;
	Rect rect = boundingRect((Mat)contours[indexFound]);
	pt1.x = rect.x;
	pt1.y = rect.y;
	pt2.x = rect.x + rect.width;
	pt2.y = rect.y + rect.height;
	//rectangle(cameraFeed, pt1, pt2, CV_RGB(255, 255, 255), 0.1);
	return rect.width;
	
	/*Rect rect = boundingRect((Mat)contours[indexFound]);
	//cout << "Original Rect: " << rect.width;
	//cout << "distance: "<< ((rect.width - 139.8) / -9.6571)/10;
	
	RotatedRect rect2 = minAreaRect((Mat)contours[indexFound]);
	if(rect2.size.width < rect2.size.height){
	    //cout << "  RotatedRect: " <<rect2.size.width;
	    //cout << "distance: "<< ((rect2.size.width - 139.8) / -9.6571)/10<<endl;
		return rect2.size.width;
	}
	else {
	    //cout << "  RotatedRect: " <<rect2.size.height;
	    //cout << "distance: "<< ((rect2.size.height - 139.8) / -9.6571)/10<<endl;
		return rect2.size.height;
	}*/
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
	//create window for trackbars


	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	//sprintf(TrackbarName, "H_MIN", H_MIN);
	//sprintf(TrackbarName, "H_MAX", H_MAX);
	//sprintf(TrackbarName, "S_MIN", S_MIN);
	//sprintf(TrackbarName, "S_MAX", S_MAX);
	//sprintf(TrackbarName, "V_MIN", V_MIN);
	//sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);


}
void drawObject(int x, int y, Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(6, 6));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);



}

void clearData(){
	yx = -1;
	yy = -1;
	
	yArea = -1;
	rArea = -1;

    goald = NULL_VAL;
    goala = NULL_VAL;
    
	zx = -1;
	zy = -1;
	for (int i = 0; i < 2; i++) {
		bx[i] = -1;
		by[i] = -1;
		bw[i] = -1;
		gx[i] = -1;
		gy[i] = -1;
		gw[i] = -1;
		ox[i] = -1;
		oy[i] = -1;
		ow[i] = -1;
		rx[i] = -1;
		ry[i] = -1;
		rw[i] = -1;

	}

	pillar1x = -1;
	pillar1y = -1;
	pillar1d = NULL_VAL;
	pillar1a = NULL_VAL;

	pillar2x = -1;
	pillar2y = -1;
	pillar2d = NULL_VAL;
	pillar2a = NULL_VAL;

	pillar3x = -1;
	pillar3y = -1;
	pillar3d = NULL_VAL;
	pillar3a = NULL_VAL;

	pillar4x = -1;
	pillar4y = -1;
	pillar4d = NULL_VAL;
	pillar4a = NULL_VAL;

	pillar5x = -1;
	pillar5y = -1;
	pillar5d = NULL_VAL;
	pillar5a = NULL_VAL;

	pillar6x = -1;
	pillar6y = -1;
	pillar6d = NULL_VAL;
	pillar6a = NULL_VAL;

	ballx = -1;
	bally = -1;
	balld = NULL_VAL;
	balla = NULL_VAL;
}


void trackMultiFilteredObject(int x[], int y[], int  w[], Mat threshold, Mat &cameraFeed)
{
	Mat temp;
	threshold.copyTo(temp);

	// These two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// Use moments method to find our filtered object
	double refArea[2] = { 0, 0 };
	int objectIndex[2] = { -1, -1 };
	bool objectFound[2] = { false, false };
	int tempX[2];
	int tempY[2];
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS)
		{
			// find largest two blobs
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA)
				{
					if (area > refArea[0])
					{
						// Check to see if we need to move the 0 postion to
						// postion 1
						if (refArea[0] > refArea[1])
						{
							refArea[1] = refArea[0];
							objectIndex[1] = objectIndex[0];
						}
						refArea[0] = area;
						objectIndex[0] = index;
					}
					else if (area > refArea[1])
					{
						refArea[1] = area;
						objectIndex[1] = index;
					}
				}
			}

			for (int i = 0; i <= 1; i++)
			{
				if (objectIndex[i] > -1)
				{
					Moments moment = moments((cv::Mat)contours[objectIndex[i]]);
					double area = moment.m00;

					tempX[i] = moment.m10 / area;
					tempY[i] = moment.m01 / area;
					objectFound[i] = true;
					//draw object location on screen
					//float f = horizontalAngle(tempX[i]);
					////cout << f << endl;
					//drawObject(tempX[i],tempY[i],cameraFeed);
					w[i] = calculateWidth(contours, objectIndex[i]);
					x[i] = tempX[i];
					y[i] = tempY[i];
				}
				else {
					w[i] = -1;
					x[i] = -1;
					y[i] = -1;
				}

			}


		}
		else
		{
			////cout << "NULL" << endl;
		}
	}
}


void trackFilteredObject(int &x, int &y, double &area, Mat threshold, Mat &cameraFeed){
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int indexFound = -1;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					indexFound = index;
				}
				else {
					objectFound = false;
					x = -1;
					y = -1;
				}


			}
			//let user know you found an object
			// bounded rectangle????? tilted
			//RotatedRect rect = minAreaRect((Mat)contours[indexFound]);



			if (objectFound == true){
                area = refArea;
                /*float angle = horizontalAngle(x);
                ////cout << angle << endl;
				//double d = calculateDistance(a);
				////cout << refArea << endl;
				balld = calculateBallDistance(refArea);
				////cout << balld << endl;
				balla = angle;
				////cout << f << endl;
				ballx = x;
				bally = y;*/
			}
		}
	}
	else {
		////cout << "NULL_VAL" << endl;
	}
}

bool compareCoordinates(int x1, int y1, int x2, int y2) {
	if (abs(y1 - y2) < 300)
	if (abs(x1 - x2) < 75) // they are one pillar
		return true;
	return false;
}

void findBall() {
    if(yx == -1 || yy == -1) { // no good values
            balla = NULL_VAL;
            balld = NULL_VAL;
    } else {    
        float angle = horizontalAngle(yx);
        ////cout << angle << endl;
        //double d = calculateDistance(a);
        ////cout << refArea << endl;
        balld = calculateBallDistance(yArea);
        ////cout << balld << endl;
        balla = angle;
        ////cout << f << endl;
    }
    ballx = x;
    bally = y;

}

void findGoal() {
    if(zx == -1 || zy == -1) { // no good values
            balla = NULL_VAL;
            balld = NULL_VAL;
    } else {    
        float angle = horizontalAngle(zx);
        ////cout << angle << endl;
        //double d = calculateDistance(a);
        ////cout << refArea << endl;
        //balld = calculateBallDistance(yArea);
        ////cout << balld << endl;
        goala = angle;
        goald = NULL_VAL;
        ////cout << f << endl;
        //ballx = x;
        //bally = y;
    }

}

void findPillars() {

	if (rx[0] > -1 && rx[1] > -1) {//Two red pillars
		if (ox[0] > -1) {//I see orange
			// Look for pillar3
			if (compareCoordinates(ox[0], oy[0], rx[0], ry[0])) {
				// rx1, ry1 = middle pillar
				pillar2x = rx[1];
				pillar2y = ry[1];

				pillar3x = rx[0];
				pillar3y = (ry[0]+oy[0])/2;
				////cout << "Found Pillars 2 3" << endl;
				float angle = verticalAngle(pillar2y);
				float distance = calculateDistance(false, angle, rw[1]);
				pillar2a = horizontalAngle(pillar2x);
				pillar2d = distance;
				angle = verticalAngle(pillar3y);
				distance = calculateDistance(true, angle, rw[0]);
				pillar3a = horizontalAngle(pillar3x);
				pillar3d = distance;
			}
			else if (compareCoordinates(ox[0], oy[0], rx[1], ry[1])) {
				// rx0, ry0 = middle pillar
				pillar2x = rx[0];
				pillar2y = ry[0];

				pillar3x = rx[1];
				pillar3y = (ry[1]+oy[0])/2;
				////cout << "Found Pillars 2 3" << endl;
				float angle = verticalAngle(pillar2y);
				float distance = calculateDistance(false, angle, rw[0]);
				pillar2a = horizontalAngle(pillar2x);
				pillar2d = distance;
				angle = verticalAngle(pillar3y);
				distance = calculateDistance(true, angle, rw[1]);
				pillar3a = horizontalAngle(pillar3x);
				pillar3d = distance;
			}
		}
		else if (gx[0] > -1) { // I see green
			// Look for pillar1
			if (compareCoordinates(gx[0], gy[0], rx[0], ry[0])) {
				// rx1, ry1 = middle pillar
				pillar2x = rx[1];
				pillar2y = ry[1];

				pillar1x = rx[0];
				pillar1y = (ry[0]+gy[0])/2;
				////cout << "Found Pillars 1 2" << endl;
				float angle = verticalAngle(pillar2y);
				float distance = calculateDistance(false, angle, rw[1]);
				pillar2a = horizontalAngle(pillar2x);
				pillar2d = distance;
				angle = verticalAngle(pillar1y);
				distance = calculateDistance(true, angle, rw[0]);
				pillar1a = horizontalAngle(pillar1x);
				pillar1d = distance;
			}
			else if (compareCoordinates(gx[0], gy[0], rx[1], ry[1])) {
				// rx0, ry0 = middle pillar
				pillar2x = rx[0];
				pillar2y = ry[0];

				pillar1x = rx[1];
				pillar1y = (ry[1]+gy[0])/2;
				////cout << "Found Pillars 1 2" << endl;
				float angle = verticalAngle(pillar2y);
				float distance = calculateDistance(false, angle, rw[0]);
				pillar2a = horizontalAngle(pillar2x);
				pillar2d = distance;
				angle = verticalAngle(pillar1y);
				distance = calculateDistance(true, angle, rw[1]);
				pillar1a = horizontalAngle(pillar1x);
				pillar1d = distance;
			}
		}
	}
	else if (rx[0] > -1) {// one red pillar
		if (ox[0] > -1) {//I see orange
			// rx0, ry0 = pillar 3
			pillar3x = rx[0];
			pillar3y = (ry[0]+oy[0])/2;
			////cout << "Found Pillar 3: ";
			float angle = verticalAngle(pillar3y);
			float distance = calculateDistance(angle, rw[0]);
			////cout << distance << endl;
			pillar3a = horizontalAngle(pillar3x);;
			pillar3d = distance;

		}
		else if (gx[0] > -1) { // I see green
			// rx0, ry0 = pillar 1
			pillar1x = rx[0];
			pillar1y = (ry[0]+gy[0])/2;
			////cout << "Found Pillar 1: ";
			float angle = verticalAngle(pillar1y);
			float distance = calculateDistance(angle, rw[0]);
			////cout << distance << endl;
			pillar1a = horizontalAngle(pillar1x);;
			pillar1d = distance;
		}
		else {
			// rx0, ry0 = pillar 2
			pillar2x = rx[0];
			pillar2y = ry[0];
			////cout << "Found Pillar 2: ";
			float angle = verticalAngle(pillar2y);
			////cout << "V_Angle " << angle << ": ";
			float distance = calculateDistance(angle, rw[0]);
			////cout << distance << endl;
			pillar2a = horizontalAngle(pillar2x);;
			pillar2d = distance;
		}
	}

	if (bx[0] > -1 && bx[1] > -1) {//Two blue pillars
		if (ox[0] > -1) {//I see orange
			// Look for pillar4
			if (compareCoordinates(ox[0], oy[0], bx[0], by[0])) {
				// bx1, by1 = middle pillar
				pillar5x = bx[1];
				pillar5y = by[1];

				pillar4x = bx[0];
				pillar4y = (by[0]+oy[0])/2;
				////cout << "Found Pillars 4 5" << endl;
				float angle = verticalAngle(pillar4y);
				float distance = calculateDistance(true, angle, bw[0]);
				pillar4a = horizontalAngle(pillar4x);;
				pillar4d = distance;
				angle = verticalAngle(pillar5y);
				distance = calculateDistance(false, angle, bw[1]);
				pillar5a = horizontalAngle(pillar5x);;
				pillar5d = distance;
			}
			else if (compareCoordinates(ox[0], oy[0], bx[1], by[1])) {
				// bx0, by0 = middle pillar
				pillar5x = bx[0];
				pillar5y = by[0];

				pillar4x = bx[1];
				pillar4y = (by[1]+oy[0])/2;
				////cout << "Found Pillars 4 5" << endl;
				float angle = verticalAngle(pillar4y);
				float distance = calculateDistance(true, angle, bw[1]);
				pillar4a = horizontalAngle(pillar4x);;
				pillar4d = distance;
				angle = verticalAngle(pillar5y);
				distance = calculateDistance(false, angle, bw[0]);
				pillar5a = horizontalAngle(pillar5x);;
				pillar5d = distance;
			}
		}
		else if (gx[0] > -1) { // I see green
			// Look for pillar6
			if (compareCoordinates(gx[0], gy[0], bx[0], by[0])) {
				// bx1, by1 = middle pillar
				pillar5x = bx[1];
				pillar5y = by[1];

				pillar6x = bx[0];
				pillar6y = (by[0]+gy[0])/2;
				////cout << "Found Pillars 5 6" << endl;
				float angle = verticalAngle(pillar6y);
				float distance = calculateDistance(true, angle, bw[0]);
				pillar6a = horizontalAngle(pillar6x);;
				pillar6d = distance;
				angle = verticalAngle(pillar5y);
				distance = calculateDistance(false, angle, bw[1]);
				pillar5a = horizontalAngle(pillar5x);;
				pillar5d = distance;
			}
			else if (compareCoordinates(gx[0], gy[0], bx[1], by[1])) {
				// bx0, by0 = middle pillar
				pillar5x = bx[0];
				pillar5y = by[0];

				pillar6x = bx[1];
				pillar6y = (by[1]+gy[0])/2;
				////cout << "Found Pillars 5 6" << endl;
				float angle = verticalAngle(pillar6y);
				float distance = calculateDistance(true, angle, bw[1]);
				pillar6a = horizontalAngle(pillar6x);;
				pillar6d = distance;
				angle = verticalAngle(pillar5y);
				distance = calculateDistance(false, angle, bw[0]);
				pillar5a = horizontalAngle(pillar5x);;
				pillar5d = distance;
			}
		}
	}
	else if (bx[0] > -1) { //one blue pillar
		if (ox[0] > -1) {//I see orange
			// bx0, by0 = pillar 4
			pillar4x = bx[0];
			pillar4y = (by[0]+oy[0])/2;
			////cout << "Found Pillar 4 :";
			float angle = verticalAngle(pillar4y);
			float distance = calculateDistance(angle, bw[0]);
			////cout << distance << endl;
			pillar4a = horizontalAngle(pillar4x);;
			pillar4d = distance;
		}
		else if (gx[0] > -1) { // I see green
			// bx0, by0 = pillar 6
			pillar6x = bx[0];
			pillar6y = (by[0]+gy[0])/2;
			////cout << "Found Pillar 6 :";
			float angle = verticalAngle(pillar6y);
			float distance = calculateDistance(angle, bw[0]);
			////cout << distance << endl;
			pillar6a = horizontalAngle(pillar6x);;
			pillar6d = distance;
		}
		else {
			// bx0, by0 = pillar 5
			pillar5x = bx[0];
			pillar5y = by[0];
			////cout << "Found Pillar 5 :";
			float angle = verticalAngle(pillar5y);
			float distance = calculateDistance(angle, bw[0]);
			////cout << distance << endl;
			pillar5a = horizontalAngle(pillar5x);;
			pillar5d = distance;
		}
	}
}

string objToString(double d) {
	stringstream ss;
	ss << d;
	return (d == NULL_VAL) ? "\"NULL\"" : ss.str();
}

string generateBroadcast() {
	stringstream ss;
	ss << "{" <<
		"\"ball\":{\"distance\":" << objToString(balld) << ",\"angle\":" << objToString(balla) << "}," <<
		/*"\"pillar1\":{\"distance\":" << objToString(pillar1d) << ",\"angle\":" << objToString(pillar1a) << "}," <<*/
		"\"goal\":{\"distance\":" << objToString(NULL_VAL) << ",\"angle\":" << objToString(goala) << "}" <<
		/*"\"pillar3\":{\"distance\":" << objToString(pillar3d) << ",\"angle\":" << objToString(pillar3a) << "}," <<
		"\"pillar4\":{\"distance\":" << objToString(pillar4d) << ",\"angle\":" << objToString(pillar4a) << "}," <<
		"\"pillar5\":{\"distance\":" << objToString(pillar5d) << ",\"angle\":" << objToString(pillar5a) << "}," <<
		"\"pillar6\":{\"distance\":" << objToString(pillar6d) << ",\"angle\":" << objToString(pillar6a) << "}" <<*/
		"}" << endl;
	return ss.str();
}
/*
void blackoutHalf(Mat A) {
    for(int i=0; i<A.rows;i++){
        uchar* rowi = A.ptr(i);
        for(int j=0; j<A.cols; j++){
            doProcessOnPixel(rowi[j]);
        }
    }
}
*/
void printHSV() {
   
    //cout << "orange values: " << endl;
    //cout << O_H_MIN << endl;
    //cout << O_H_MAX << endl;
    //cout << O_S_MIN << endl;
    //cout << O_S_MAX << endl;
    //cout << O_V_MIN << endl;
    //cout << O_V_MAX << endl;
    //cout << "blue values: " << endl;
    //cout << B_H_MIN << endl;
    //cout << B_H_MAX << endl;
    //cout << B_S_MIN << endl;
    //cout << B_S_MAX << endl;
    //cout << B_V_MIN << endl;
    //cout << B_V_MAX << endl;
    //cout << "yellow values: " << endl;
    //cout << Y_H_MIN << endl;
    //cout << Y_H_MAX << endl;
    //cout << Y_S_MIN << endl;
    //cout << Y_S_MAX << endl;
    //cout << Y_V_MIN << endl;
    //cout << Y_V_MAX << endl;
    //cout << "green values: " << endl;
    //cout << G_H_MIN << endl;
    //cout << G_H_MAX << endl;
    //cout << G_S_MIN << endl;
    //cout << G_S_MAX << endl;
    //cout << G_V_MIN << endl;
    //cout << G_V_MAX << endl;
    //cout << "red values: " << endl;
    //cout << R_H_MIN << endl;
    //cout << R_H_MAX << endl;
    //cout << R_S_MIN << endl;
    //cout << R_S_MAX << endl;
    //cout << R_V_MIN << endl;
    //cout << R_V_MAX << endl;


}



void setConfig(char* fileName){
    char data[500];
    string tempstring;
    string object_hsv;
    infile.open(fileName);
    ////cout << "Reading from file" << endl;
    while(! infile.eof() ) 
    {
        infile >> data;
        tempstring = data;
        ////cout << "tempstring: " << tempstring << endl;
        if(tempstring.find("orange_pillar") != std::string::npos)
        {
            ////cout << "ORANGE PILLAR 2!" << endl;
            setOrange();
        }
        else if(tempstring.find("blue_pillar") != std::string::npos)
        {
            ////cout << "BLUE PILLAR" << endl;
            setBlue();
        }
        else if(tempstring.find("ball") != std::string::npos)
        {
            ////cout << "BALL" << endl;
            setYellow();
        }
        else if(tempstring.find("green_pillar") != std::string::npos)
        {
            ////cout << "GREEN PILLAR" << endl;
            setGreen();
        }
        else if(tempstring.find("red_pillar") != std::string::npos)
        {
            ////cout << "RED PILLAR" << endl;
            setRed();
        }
        else if(tempstring.find("{") != std::string::npos ||
                tempstring.find("}") != std::string::npos)
        {
            ////cout << "FOUND { OR }" << endl;
        }
    }
    infile.close();
}



int main(int argc, char* argv[])
{
	char* myFile = argv[1];
	if (myFile == NULL)
	    return 0;
	setConfig(myFile);

    //printHSV();


	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed, original;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	Mat ythresh;
	Mat bthresh;
	Mat gthresh;
	Mat othresh;
	Mat rthresh;
	//x and y values for the location of the object
	int x = 0, y = 0;
	//create slider bars for HSV filtering
	if (graphics)
		createTrackbars();
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (1){
		//store image to matrix
		capture.read(cameraFeed);
		for (int i = 1; i < 4; i += 2)
		{
			//GaussianBlur( image, image, Size( i, i ), 0, 0 );
			blur(cameraFeed, cameraFeed, Size(i, i), Point(-1, -1));
			//bilateralFilter ( original, cameraFeed, i, i*2, i/2 ); 
		}
        //bilateralFilter ( original, cameraFeed, 3, 30, ); 
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		//inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		inRange(HSV, Scalar(Y_H_MIN, Y_S_MIN, Y_V_MIN), Scalar(Y_H_MAX, Y_S_MAX, Y_V_MAX), ythresh);
		inRange(HSV, Scalar(B_H_MIN, B_S_MIN, B_V_MIN), Scalar(B_H_MAX, B_S_MAX, B_V_MAX), bthresh);
		inRange(HSV, Scalar(G_H_MIN, G_S_MIN, G_V_MIN), Scalar(G_H_MAX, G_S_MAX, G_V_MAX), gthresh);
		inRange(HSV, Scalar(O_H_MIN, O_S_MIN, O_V_MIN), Scalar(O_H_MAX, O_S_MAX, O_V_MAX), othresh);
		inRange(HSV, Scalar(R_H_MIN, R_S_MIN, R_V_MIN), Scalar(R_H_MAX, R_S_MAX, R_V_MAX), rthresh);

		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if (useMorphOps) {
			//morphOps(threshold);
			morphOps(ythresh);
			morphOps(bthresh);
			morphOps(gthresh);
			morphOps(othresh);
			morphOps(rthresh);
		}
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object

		if (trackObjects) {
			clearData();
			//trackFilteredObject(x,y,threshold,cameraFeed);
			trackFilteredObject(zx,zy,rArea,rthresh,cameraFeed);
			trackFilteredObject(yx,yy,yArea,ythresh,cameraFeed);
			//trackMultiFilteredObject(bx,by,bw,bthresh,cameraFeed);
			//trackMultiFilteredObject(gx,gy,gw,gthresh,cameraFeed);
			//trackMultiFilteredObject(ox,oy,ow,othresh,cameraFeed);
			//trackMultiFilteredObject(rx,ry,rw,rthresh,cameraFeed);
		}

		//FIND PILLARS
		//findPillars();
		findBall();
		findGoal();

		string toBroadcast = generateBroadcast();
		cout << toBroadcast;

//        imwrite("yellow.jpg",ythresh);
//        imwrite("red.jpg",rthresh);
//        imwrite("original.jpg",cameraFeed);

		if (graphics) {
			drawObject(ballx, bally, cameraFeed);
			drawObject(pillar1x,pillar1y,cameraFeed);
			drawObject(pillar2x,pillar2y,cameraFeed);
			drawObject(pillar3x,pillar3y,cameraFeed);
			drawObject(pillar4x,pillar4y,cameraFeed);
			drawObject(pillar5x,pillar5y,cameraFeed);
			drawObject(pillar6x,pillar6y,cameraFeed);		

			//show frames 
			//imshow(windowName2,threshold);
			imshow("Yellow", ythresh);
			imshow("Blue", bthresh); 
			imshow("Green", gthresh); 
			imshow("Red", rthresh);
			imshow("Orange", othresh);
			imshow(windowName, cameraFeed);
			imshow(windowName1, HSV);
		}
		//cvFlip(g_videoImage, NULL, 1);
		waitKey(15);
	}

	return 0;
}


//Set Colors
//ORANGE
void setOrange()
{
    //cout << "Setting orange" << endl;
    char data[500];
    string tempstring;
    string stripped;
    int value;
    int count = 0;

    while(count < 6)
    {
        infile >> data;
        tempstring = data;
        ////cout << "tempstring is: " << tempstring << endl;
        if(tempstring.find("H_MAX") != std::string::npos)
        {
            ////cout << "found h_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_H_MAX;
            count ++;
        }
        else if(tempstring.find("H_MIN") != std::string::npos)
        {
            ////cout << "found h_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_H_MIN;
            count ++;
        }
        else if(tempstring.find("S_MAX") != std::string::npos)
        {
            ////cout << "found s_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_S_MAX;
            count ++;
        }
        else if(tempstring.find("S_MIN") != std::string::npos)
        {
            ////cout << "found s_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_S_MIN;
            count ++;
        }
        else if(tempstring.find("V_MAX") != std::string::npos)
        {
            ////cout << "found v_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_V_MAX;
            count ++;
        }
        else if(tempstring.find("V_MIN") != std::string::npos)
        {
            ////cout << "found v_mni" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> O_V_MIN;
            count ++;
        }
    }
}

//BLUE
void setBlue()
{
    //cout << "Setting blue" << endl;
    char data[500];
    string tempstring;
    string stripped;
    int value;
    int count = 0;

    while(count < 6)
    {
        infile >> data;
        tempstring = data;
        //cout << "tempstring is: " << tempstring << endl;
        if(tempstring.find("H_MAX") != std::string::npos)
        {
            //cout << "found h_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_H_MAX;
            count ++;
        }
        else if(tempstring.find("H_MIN") != std::string::npos)
        {
            //cout << "found h_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_H_MIN;
            count ++;
        }
        else if(tempstring.find("S_MAX") != std::string::npos)
        {
            //cout << "found s_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_S_MAX;
            count ++;
        }
        else if(tempstring.find("S_MIN") != std::string::npos)
        {
            //cout << "found s_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_S_MIN;
            count ++;
        }
        else if(tempstring.find("V_MAX") != std::string::npos)
        {
            //cout << "found v_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_V_MAX;
            count ++;
        }
        else if(tempstring.find("V_MIN") != std::string::npos)
        {
            //cout << "found v_mni" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> B_V_MIN;
            count ++;
        }
    }
}

//YELLOW
void setYellow()
{
    //cout << "Setting orange" << endl;
    char data[500];
    string tempstring;
    string stripped;
    int value;
    int count = 0;

    while(count < 6)
    {
        infile >> data;
        tempstring = data;
        //cout << "tempstring is: " << tempstring << endl;
        if(tempstring.find("H_MAX") != std::string::npos)
        {
            //cout << "found h_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_H_MAX;
            count ++;
        }
        else if(tempstring.find("H_MIN") != std::string::npos)
        {
            //cout << "found h_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_H_MIN;
            count ++;
        }
        else if(tempstring.find("S_MAX") != std::string::npos)
        {
            //cout << "found s_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_S_MAX;
            count ++;
        }
        else if(tempstring.find("S_MIN") != std::string::npos)
        {
            //cout << "found s_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_S_MIN;
            count ++;
        }
        else if(tempstring.find("V_MAX") != std::string::npos)
        {
            //cout << "found v_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_V_MAX;
            count ++;
        }
        else if(tempstring.find("V_MIN") != std::string::npos)
        {
            //cout << "found v_mni" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> Y_V_MIN;
            count ++;
        }
    }
}

//GREEN
void setGreen()
{
    //cout << "Setting orange" << endl;
    char data[500];
    string tempstring;
    string stripped;
    int value;
    int count = 0;

    while(count < 6)
    {
        infile >> data;
        tempstring = data;
        //cout << "tempstring is: " << tempstring << endl;
        if(tempstring.find("H_MAX") != std::string::npos)
        {
            //cout << "found h_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_H_MAX;
            count ++;
        }
        else if(tempstring.find("H_MIN") != std::string::npos)
        {
            //cout << "found h_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_H_MIN;
            count ++;
        }
        else if(tempstring.find("S_MAX") != std::string::npos)
        {
            //cout << "found s_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_S_MAX;
            count ++;
        }
        else if(tempstring.find("S_MIN") != std::string::npos)
        {
            ////cout << "found s_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_S_MIN;
            count ++;
        }
        else if(tempstring.find("V_MAX") != std::string::npos)
        {
            ////cout << "found v_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_V_MAX;
            count ++;
        }
        else if(tempstring.find("V_MIN") != std::string::npos)
        {
            ////cout << "found v_mni" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> G_V_MIN;
            count ++;
        }
    }
}

//RED
void setRed()
{
    ////cout << "Setting orange" << endl;
    char data[500];
    string tempstring;
    string stripped;
    int value;
    int count = 0;

    while(count < 6)
    {
        infile >> data;
        tempstring = data;
        ////cout << "tempstring is: " << tempstring << endl;
        if(tempstring.find("H_MAX") != std::string::npos)
        {
            ////cout << "found h_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_H_MAX;
            count ++;
        }
        else if(tempstring.find("H_MIN") != std::string::npos)
        {
            ////cout << "found h_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_H_MIN;
            count ++;
        }
        else if(tempstring.find("S_MAX") != std::string::npos)
        {
            ////cout << "found s_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_S_MAX;
            count ++;
        }
        else if(tempstring.find("S_MIN") != std::string::npos)
        {
            ////cout << "found s_min" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_S_MIN;
            count ++;
        }
        else if(tempstring.find("V_MAX") != std::string::npos)
        {
            ////cout << "found v_max" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_V_MAX;
            count ++;
        }
        else if(tempstring.find("V_MIN") != std::string::npos)
        {
            ////cout << "found v_mni" << endl;
            infile >> data;
            tempstring = data;
            stripped = tempstring;
            if(tempstring.find(",") != std::string::npos)
                stripped = tempstring.substr(0,tempstring.size() -1);
            stringstream(stripped) >> R_V_MIN;
            count ++;
        }
    }
}


