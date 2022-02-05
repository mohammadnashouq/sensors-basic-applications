#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int* GetMovingPoints(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame , float threshold , int& MovingPointsCount, vector<int>& InlierIndexes);


void GetMovingPoints2(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame, float threshold, vector<Point3f>& MovingPoints, vector<Point3f>& FixedPoints);

void GetMovingPoints3(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame, float threshold, vector<Point3f>& MovingPoints, vector<Point3f>& FixedPoints);
