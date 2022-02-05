

#include "MatrixReaderWriter.h"
#include "Camera.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "MovingPoints.h"
#include "PLYWriter.h"
#include "PlaneEstimation.h"
#include "CylinderEstimation.h"
#include "SphereEstimation.h"


#include<dos.h>

#include<thread>



using std::filesystem::directory_iterator;

using namespace cv;






#define THERSHOLD 0.2  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

#define RANSAC_ITER 30   //RANSAC iteration

#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering



int main(int argc, const char** argv) {
	// Read 3D points
	
	std::string dir = "PointCloudsFrames";

	int count = 0;
	for (const auto& entry : directory_iterator(dir)) 
	{
		count++;
	}
	const int constcount = count+1;
	string*  FramesFiles = new string[constcount];
	for (const auto& entry : directory_iterator(dir)) {
		string path = entry.path().string();

		int loc = path.find_first_of('_');
		string FrameNumberStr =path.substr(dir.length()+1,(path.find_first_of('_')-(dir.length() + 1)));
		int FrameNumber = stoi(FrameNumberStr);

		/*string str_obj(path);
		char* pathpinter = &str_obj[0];*/

		FramesFiles[FrameNumber] = path;


	}

	int FrameCounter = 1;

	std::thread t;
	int er = 0;

	int timepassed_ms = 0;

	float PrevTime = 0;
	float LastTime = 0;

	float PrevDistance = 0;
	float LastDistance = 0;

	int number_of_messurment = 0;


	while (true)
	{
		if (FrameCounter + 5 > count) {
			return 0;
		}

		cout << "-------------------------------------------------------------------------------- ";
		cout << "FrameCounter :  " + FrameCounter;
		cout << "-------------------------------------------------------------------- ";

		string FirstFrames = FramesFiles[FrameCounter];

		string SecondFrames = FramesFiles[FrameCounter + 5];

		timepassed_ms += 500;

		/*	string FirstFrames = FramesFiles[6];

	        string SecondFrames = FramesFiles[11];*/


		/*char* FirstFrame;

		char* SecondFrame;*/
		/*const char* arg1 = "PointClouds/1_frame.xyz";
		const char* arg2 = "PointClouds/5_frame.xyz";*/

		const char* arg1 = &FirstFrames[0];
		const  char* arg2 = &SecondFrames[0];
		bool SaveImages = false;


		MatrixReaderWriter mrw1(arg1);

		vector<Point3f> Frame1_Points;
		for (int i = 0; i < mrw1.rowNum; i++) {
			float x = mrw1.data[3 * i];
			float y = mrw1.data[3 * i + 1];
			float z = mrw1.data[3 * i + 2];


			float distFromOrigo = sqrt(x * x + y * y + z * z);


			// First filter: minimal work distance for a LiDAR limited.        
			if (distFromOrigo > FILTER_LOWEST_DISTANCE) {
				Point3f newPt;
				newPt.x = x;
				newPt.y = y;
				newPt.z = z;
				Point3f p = Point3f(x, y, z);
				Frame1_Points.push_back(p);
			}

			
		}


		MatrixReaderWriter mrw2(arg2);

		vector<Point3f> Frame2_Points;
		for (int i = 0; i < mrw2.rowNum; i++) {
			float x = mrw2.data[3 * i];
			float y = mrw2.data[3 * i + 1];
			float z = mrw2.data[3 * i + 2];


			float distFromOrigo = sqrt(x * x + y * y + z * z);


			// First filter: minimal work distance for a LiDAR limited.        
			if (distFromOrigo > FILTER_LOWEST_DISTANCE) {
				Point3f newPt;
				newPt.x = x;
				newPt.y = y;
				newPt.z = z;
				Point3f p = Point3f(x, y, z);
				Frame2_Points.push_back(p);
			}



			
		}

		int MovingPointsCount = 0;

		vector<int> MovingPointIndes;

		vector<Point3f> Moving_Points;

		vector<Point3f> Fixed_Points;


		//int* MovingPointsMap = GetMovingPoints(Frame1_Points, Frame2_Points, 0.02, MovingPointsCount, MovingPointIndes);

		int b = 1;

		GetMovingPoints3(Frame1_Points, Frame2_Points, 0.08, Moving_Points, Fixed_Points);


		Point3i InlierColor = Point3i(255, 0, 0);
		Point3i OutLayerColor = Point3i(255, 255, 255);

		if (SaveImages) {
			WritePLY_Inlier_Outliers("resault.ply", Moving_Points, Fixed_Points, InlierColor, OutLayerColor);
		}


		bool fitCylinder = true;

		RANSACDiffs differences;
		array<float, 4> params;

		if (fitCylinder) {
			// Vertical cylinder fitting
			// RANSAC-based robust cylinder estimation
			params = EstimateCylinderRANSAC(Moving_Points, THERSHOLD, RANSAC_ITER);
			printf("Cylinder params RANSAC:\n px:%f py:%f pz:%f r:%f \n", params[0], params[1], params[2], params[3]);

			// Compute differences of the fitted cylinder in order to separate inliers from outliers
			differences = CylinderPointRANSACDifferences(Moving_Points, params, THERSHOLD);
		}
		else {
			// Sphere cylinder fitting
			// RANSAC-based robust sphere estimation
			params = EstimateSphereRANSAC(Moving_Points, THERSHOLD, RANSAC_ITER);
			printf("Sphere params RANSAC:\n px:%f py:%f pz:%f r:%f \n", params[0], params[1], params[2], params[3]);

			// Compute differences of the fitted sphere in order to separate inliers from outliers
			differences = SpherePointRANSACDifferences(Moving_Points, params, THERSHOLD);
		}

		vector<Point3f> Moving_PointsInliers;
		for (int i = 0; i < Moving_Points.size(); i++) {
			if (differences.isInliers.at(i)) {
				Moving_PointsInliers.push_back(Moving_Points[i]);
			}
			else {
				Fixed_Points.push_back(Moving_Points[i]);
			}
		}
		if (SaveImages) {
			WritePLY_Inlier_Outliers("resault2.ply", Moving_PointsInliers, Fixed_Points, InlierColor, OutLayerColor);
		}


		namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.

		printf("%d %d\n", mrw1.rowNum, mrw1.columnNum);

		/*if (er == 0) {
			t = std::thread(Draw_World, Moving_PointsInliers, Fixed_Points);

			t.detach();
			er++;
		}*/
	
		Mat resImg;
		

		// Set image and initial camera parameters (see Camera.h)
		// 
		// 
		Size size(800, 600); // Image size
		resImg = Mat::zeros(size, CV_8UC3);
		float fu = 3000,
			fv = 3000,
			u0 = 400,
			v0 = 300;
		Camera camera(fu, fv, u0, v0);
		float v = 1.0;
		float u = 3;
		float rad = 90.0;

		// Display image


		Point3f CreavityCenter;

		for (int i = 0; i < Moving_PointsInliers.size(); i++ ) 
		{
			CreavityCenter.x += Moving_PointsInliers[i].x;
			CreavityCenter.y += Moving_PointsInliers[i].y;
			CreavityCenter.z += Moving_PointsInliers[i].z;
		}

		CreavityCenter.x = CreavityCenter.x / Moving_PointsInliers.size();
		CreavityCenter.y = CreavityCenter.y / Moving_PointsInliers.size();
		CreavityCenter.z = CreavityCenter.z / Moving_PointsInliers.size();

		vector<float> DistanceFromCenter;

		float AvreageDistanceFrom_CreavityCenter = 0;
		for (int i = 0; i < Moving_PointsInliers.size(); i++)
		{
			float distance = sqrt(pow((Moving_PointsInliers[i].x - CreavityCenter.x), 2) + pow((Moving_PointsInliers[i].y - CreavityCenter.y), 2) + pow((Moving_PointsInliers[i].z - CreavityCenter.z), 2));
			AvreageDistanceFrom_CreavityCenter += distance;
			DistanceFromCenter.push_back(distance);
		}

		AvreageDistanceFrom_CreavityCenter = AvreageDistanceFrom_CreavityCenter / Moving_PointsInliers.size();

		vector<Point3f> Final_ObjectPoints;
		for (int i = 0; i < Moving_PointsInliers.size(); i++)
		{
			if (DistanceFromCenter[i] < AvreageDistanceFrom_CreavityCenter * 1.8) {
				Final_ObjectPoints.push_back(Moving_PointsInliers[i]);
			}
			else {
				Fixed_Points.push_back(Moving_PointsInliers[i]);
			}

			//AvreageDistanceFrom_CreavityCenter += sqrt(pow((Moving_PointsInliers[i].x - CreavityCenter.x), 2) + pow((Moving_PointsInliers[i].y - CreavityCenter.y), 2) + pow((Moving_PointsInliers[i].z - CreavityCenter.z), 2));
		}


		camera.drawPoints_Inlier_outlyer(u, v, rad, resImg, Final_ObjectPoints, Fixed_Points, CreavityCenter);


		if (Final_ObjectPoints.size() > 0) {

			float DistanceFrom_Orgin = sqrt(CreavityCenter.x * CreavityCenter.x + CreavityCenter.y * CreavityCenter.y + CreavityCenter.z * CreavityCenter.z);
			
			PrevTime = LastTime;
			LastTime = timepassed_ms;
			
			PrevDistance = LastDistance;
			LastDistance = DistanceFrom_Orgin;

			number_of_messurment++;

			if (number_of_messurment > 1) {
				float DistancePassed = PrevDistance - LastDistance;
				float timepassed = (LastTime - PrevTime)/1000;
				float speed_KM_Hour = (DistancePassed/1000) / (timepassed/3600);
				float Time_To_Colision = ((LastDistance/1000) / speed_KM_Hour)*3600;

				string InfoOnImage1 = "Speed = " + to_string(speed_KM_Hour) + "KM/H Distance : " + to_string(LastDistance) + "meter ";
				string InfoOnImage2 =  "Time To Colision : " + to_string(Time_To_Colision) + " Second";

				if (Time_To_Colision > 5) {
					putText(resImg, InfoOnImage1, Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(118, 185, 0), 2);
					putText(resImg, InfoOnImage2, Point(50, 100), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(118, 185, 0), 2);
				}
				else {
					putText(resImg, InfoOnImage1, Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 2);
					putText(resImg, InfoOnImage2, Point(50, 100), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 2);
				}
				
			}
		}

	
	

		imshow("Display window", resImg);
		char key = cv::waitKey(100);




	/*	std::this_thread::sleep_for(2000ms);*/
		// Camera movement
		//char key;
		//while (true) {
		//	key = cv::waitKey(0);
		//	if (key == 27) break;

		//	switch (key) {
		//	case 'd':
		//		u += 0.1;
		//		break;
		//	case 'a':
		//		u -= 0.1;
		//		break;
		//	case 'w':
		//		v += 0.1;
		//		break;
		//	case 's':
		//		v -= 0.1;
		//		break;
		//	case 'q':
		//		rad *= 1.1;
		//		break;
		//	case 'e':
		//		rad /= 1.1;
		//		break;
		//	}

		//	// Display updated image
		//	resImg = Mat::zeros(size, CV_8UC3);
		//	camera.drawPoints_Inlier_outlyer(u, v, rad, resImg, Moving_PointsInliers, Fixed_Points);
		//	imshow("Display window", resImg);
		//}

		FrameCounter = FrameCounter + 5;
		int r = 0;
	}

		
	

	return 0;
	
	/*  const char* arg1 = "PointClouds//1_frame.xyz";
   const  char* arg2 = "PointClouds//5_frame.xyz";*/
  // const char* arg3 = "Images/7/Dev2_Image_w960_h600_fn1007.jpg";


	/*if (argc != 2) {
		printf("Usage: FV filename\n");
		exit(0);
	}*/
	

}



//void Draw_World(vector<Point3f> MovingObject, vector<Point3f> FixedPoints) {
//	Size size(800, 600); // Image size
//
//	Mat resImg;
//	resImg = Mat::zeros(size, CV_8UC3);
//	float fu = 3000,
//		fv = 3000,
//		u0 = 400,
//		v0 = 300;
//	Camera camera(fu, fv, u0, v0);
//	float v = 1.0;
//	float u = 3;
//	float rad = 90.0;
//
//	
//
//	camera.drawPoints_Inlier_outlyer(u, v, rad, resImg, MovingObject, FixedPoints);
//	imshow("Display window", resImg);
//
//	/*	std::this_thread::sleep_for(2000ms);*/
//		// Camera movement
//		char key;
//		while (true) {
//			key = 'y';
//			if (key == 27) break;
//
//			switch (key) {
//			case 'd':
//				u += 0.1;
//				break;
//			case 'a':
//				u -= 0.1;
//				break;
//			case 'w':
//				v += 0.1;
//				break;
//			case 's':
//				v -= 0.1;
//				break;
//			case 'q':
//				rad *= 1.1;
//				break;
//			case 'e':
//				rad /= 1.1;
//				break;
//			}
//
//			// Display updated image
//			resImg = Mat::zeros(size, CV_8UC3);
//			camera.drawPoints_Inlier_outlyer(u, v, rad, resImg, MovingObject, FixedPoints);
//			imshow("Display window", resImg);
//		}
//
//}