/*
 * main.cpp
 *
 *  Created on: Jan 16, 2020
 *      Author: Gong
 */

#include <iostream>
//#include <glob.h>

#include "LaneDetector.h"


inline void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_MOUSEMOVE) {
        cout << "(" << x << ", " << y << ")" << endl;
    }
}


//inline void getFiles(const string& pttrn, vector<string>& fileList) {
//	glob_t buf;
//	glob(pttrn.c_str(), GLOB_TILDE, NULL, &buf);
//	for (int i = 0; i < buf.gl_pathc; ++i) {
//		fileList.push_back(buf.gl_pathv[i]);
//	}
//	if (buf.gl_pathc > 0)
//		globfree(&buf);
//}
#include<Windows.h>
#include<ctime>
int main() {
	using namespace std;
	LARGE_INTEGER t1,t2,tc;

	Mat first=imread("../../test_images/test2.jpg");
	imshow("first",first);
	LaneDetector ld(first.rows, first.cols);

	//LaneDetector ld(0, 0);
	vector<string> chessboard_images;

	for(int i=1;i<21;i++)
	{
		string path_name_start="../../camera_cal/calibration";
		string path_name_end=to_string(i).append(".jpg");
		chessboard_images.push_back(path_name_start.append(path_name_end));
		cout<<chessboard_images[i-1]<<endl;
	}

	ld.cameraCalib(chessboard_images);
	Mat img = imread("../../camera_cal/calibration1.jpg");
	Mat undist;
	Mat undist_v;
	ld.undistImage(img, undist, undist_v);
	imshow("dist", img);
	imshow("undist", undist);
	waitKey();

	// initialize
	//VideoCapture cap("project_video.mp4");

	//if (!cap.isOpened()) return -1;
	//Mat first;
	//cap >> first;
	
	if(first.empty()) return -1;
	Mat overlay, monitor;
	QueryPerformanceCounter(&t1);
	ld.detect(first, overlay, monitor);
	QueryPerformanceCounter(&t2);
	cout<<"  Use Time:"<<(t2.QuadPart - t1.QuadPart)*1.0/tc.QuadPart*1000<<"ms"<<endl;
	imshow("overlay",overlay);
	imshow("monitor",monitor);
	waitKey();
	///* camera calibration */

	//vector<string> chessboard_images;
	//getFiles("camera_cal/*.jpg", chessboard_images);

	//ld.cameraCalib(chessboard_images);

	/* Process  */
//	VideoWriter out("output_images/out_project_video.avi",
//			VideoWriter::fourcc('M','J','P','G'), 30, first.size());
	//VideoWriter outMonitor("output_images/out_project_video_monitor.avi",
	//		VideoWriter::fourcc('M','J','P','G'), 30,
	//		Size(int(first.cols*3/2), int(first.rows*2/2)));

//	int countFrame = 0;
//	for (;;) {
//		Mat frame;
//		cap >> frame;
//		cout << countFrame << endl;
//		countFrame++;
//		if (frame.empty()) break;
//		Mat overlay, monitor;
//		ld.detect(frame, overlay, monitor);
////		out.write(overlay);
//		outMonitor.write(monitor);
//	}
////	out.release();
//	outMonitor.release();

	return 0;
}























