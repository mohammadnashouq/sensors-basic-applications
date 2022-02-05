#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include "PlaneEstimation.h"
#include "CylinderEstimation.h"

using namespace std;
using namespace cv;

//Sphere: array<float, 4>, where c[0]; c[1]; c[2] is a center coordinates of the sphere, and c[4] is the radius

array<float, 4> EstimateSphereMinimal(const vector<Point3f>&);
array<float, 4> EstimateSphereLsq(vector<Point3f>, const array<float, 4>&);
array<float, 4> EstimateSphereRANSAC(const vector<Point3f>&, const float, const int);
RANSACDiffs SpherePointRANSACDifferences(const vector<Point3f>& pts, const array<float, 4>& sphere, const float threshold);