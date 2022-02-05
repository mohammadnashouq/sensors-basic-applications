/* PLY format writer
 *
 * Implemented by Levente Hajder
 * hajder@inf.elte.hu
 * 30-06-2021
 */

#ifndef PLY_WRITER
#define PLY_WRITER

#include <iostream>
#include <fstream>
#include "stdlib.h"
#include <opencv2/opencv.hpp>


#include <vector>


using namespace std;
using namespace cv;


void WritePLY(const char*, vector<Point3f>, vector<Point3i>);

void WritePLY_movingPoints(const char* , vector<Point3f>& , int* );


void WritePLY_Inlier_Outliers(const char*, vector<Point3f>&, vector<Point3f>& , Point3i& , Point3i&);

#endif
