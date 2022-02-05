#include "MatrixReaderWriter.h"
#include "Camera.h"
#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, const char** argv){
	// Read 3D points
	if (argc!=2) {
		printf("Usage: FV filename\n");
		exit(0);
	}
	MatrixReaderWriter mrw(argv[1]);
	printf("%d %d\n",mrw.rowNum,mrw.columnNum);
	
	Mat resImg;
	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.

	// Set image and initial camera parameters (see Camera.h)
	Size size(800, 600); // Image size
	resImg = Mat::zeros(size, CV_8UC3);
	float fu = 3000,
		fv = 3000,
		u0 = 400,
		v0 = 300;
	Camera camera(fu, fv, u0, v0, mrw);
	float v = 1.0;
	float u = 0.5;
	float rad = 100.0;
	
	// Display image
	camera.drawPoints(u, v, rad, resImg);
	imshow("Display window", resImg);                

	// Camera movement
	char key;
	while(true){
		key= cv::waitKey(0);
		if(key==27) break;

		switch(key){
		case 'd':
			u += 0.1;
			break;
		case 'a':
			u -= 0.1;
			break;
		case 'w':
			v += 0.1;
			break;
		case 's':
			v -= 0.1;
			break;
		case 'q':
			rad *= 1.1;
			break;
		case 'e':
			rad /= 1.1;
			break;
    }

	// Display updated image
	resImg = Mat::zeros(size, CV_8UC3);
	camera.drawPoints(u, v, rad, resImg);
	imshow("Display window", resImg);             
  }

}
