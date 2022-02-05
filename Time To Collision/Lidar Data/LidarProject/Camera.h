#pragma once
#include "MatrixReaderWriter.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class Camera
{
private:
	Mat C; // Camera intrinsic parameters
	Mat R; // Rotatrion matrix
	Mat T; // Translation vector
	MatrixReaderWriter mrw; // 3D data points
	int NUM; // No. of 3D points
	Scalar color; // Color of the points (blue, green, red) 

public: 
	/*
	* Creates a camera
	* 
	* fu, fv : focal length * pixel scale 
	* u0, v0 : principal point
	* mrw_in: 3D points
	*/
	Camera(float fu, float fv, float u0, float v0);
	
	/*
	* Project 3D points to the image
	*
	* u, v: horizonatl and vertical rotation angle (spherical)
	* rad : distance from the view
	* resImg : final image with projected points
	*/
	void drawPoints(float u, float v, float rad, Mat& resImg);
	void drawPoints_Inlier_outlyer(float u, float v, float rad, Mat& resImg , vector<Point3f>& MovingObject, vector<Point3f>& FixedPoints , Point3f CravityCenter);
	
};





//void Draw_World(vector<Point3f> MovingObject, vector<Point3f> FixedPoints);

