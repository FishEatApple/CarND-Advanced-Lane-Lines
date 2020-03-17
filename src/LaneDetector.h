/*
 * laneDetector.h
 *
 *  Created on: Jan 16, 2020
 *      Author: Gong
 */

#ifndef LANEDETECTOR_H_
#define LANEDETECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
//#include <algorithm>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Line {
public:
	Line():n(5),detected(false){}
	// n frames computing the moving average
	int n ;
	// was the line detected in last iteration?
	bool detected ;
	// x values of the last n fits  of the line
	vector<int> recent_xfitted;
	// average x values of the fitted line over the last n iterations
	double bestx;
	// polynomial coefficients averaged over the last n iterations
	vector<float> best_fit;
	// polynomial coefficients for the most recent fit
	vector<vector<float> > current_fit;
	// radius of curvature of the line in some units
	double radius_of_curvature;
	// distance in meters of vehicle center from the line
	double line_base_pos;
	// difference in fit coefficients between last and new fits
	vector<float> diffs;
	// x values for detected line pixels
	vector<float> allx;
	// y values for detected line pixels
	vector<float> ally;

};



struct Camera {

	Mat cameraMat;
	Mat distCoeff;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

};



class LaneDetector {
private:
	int imgHeight;
	int imgWidth;
	float distToCtr;
	Camera cam;
	Line leftLine;
	Line rightLine;
	Mat Mtrans;
	Mat Minv;
	Point2f src[4];
	Point2f dst[4];
public:
	LaneDetector(int h, int w): imgHeight(h), imgWidth(w), distToCtr(0)	
	{
		src[0]=Point2f(250, imgHeight),src[1]=Point2f(595, 450),src[2]=Point2f(687, 450),src[3]=Point2f(1170, imgHeight);
		dst[0]=Point2f(320, imgHeight),dst[1]=Point2f(320, 0),dst[2]=Point2f(960, 0),dst[3]=Point2f(960, imgHeight);
	}

	void detect(InputArray img, OutputArray out, OutputArray debug);

	void cameraCalib(const vector<string>& chessboard_imgs, int nx=9, int ny=6);

	void undistImage(InputArray in, OutputArray out,  OutputArray undist_v);

	void thresholding(InputArray in, OutputArray out, OutputArray threshedCol);

	void projectForward(InputArray img, OutputArray out);

	void slidingWindow(InputArray img, OutputArray detected, OutputArray debug);

	void compCurveRadius();

	void compDistToCtr();

	void projectBackward(InputArray img, InputArray det, OutputArray overlay,
			OutputArray wrpdbck);

};



#endif /* LANEDETECTOR_H_ */
