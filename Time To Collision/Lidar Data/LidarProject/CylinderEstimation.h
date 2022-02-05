#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include<numeric>
#include "PlaneEstimation.h"

using namespace std;
using namespace cv;

//Cylinder: array<float, 4>, where c[0]; c[1]; c[2] is a point of the cylinder axis, and c[4] is the radius

array<float, 4> EstimateCylinderMinimal(const vector<Point3f>&);
array<float, 4> EstimateCylinderLsq(vector<Point3f>, const array<float, 4>&);
array<float, 4> EstimateCylinderRANSAC(const vector<Point3f>&, const float, const int);
RANSACDiffs CylinderPointRANSACDifferences(const vector<Point3f>& pts, const array<float, 4>& cylinder, const float threshold);

float normP3fXY(const Point3f& pt);
float normP3f(const Point3f& pt);
float normP2f(const Point2f& pt);