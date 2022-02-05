#include "MatrixReaderWriter.h"
#include "Camera.h"
#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "MovingPoints.h"


int* GetMovingPoints(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame,float threshold, int& MovingPointsCount, vector<int>& MovingPointsIndexes ) {

	MovingPointsCount = 0;
	const int SecondFrameSize = SecondFrame.size();
	int* Moving_PointsMap= new int[SecondFrameSize];
	/*vector<Point3f> Moving_Points;*/
	for (int i = 0; i < SecondFrame.size(); i++) {
		bool PointFound = false;
		for (int j = 0; j < FirstFramce.size(); j++) {

			if ((abs(SecondFrame[i].x - FirstFramce[j].x) < threshold) && (abs(SecondFrame[i].y - FirstFramce[j].y) < threshold) &&
				(abs(SecondFrame[i].z - FirstFramce[j].z) < threshold)) {
				PointFound = true;
				break;
			}
		}

		if (!PointFound) {
			MovingPointsCount++;
			MovingPointsIndexes.push_back(i);
			Moving_PointsMap[i] = 1;
			//Moving_PointsIndexes.push_back(SecondFrame[i]);
		}else{
			Moving_PointsMap[i] = 0;
		}
	}

	return Moving_PointsMap;

}


void GetMovingPoints2(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame, float threshold, vector<Point3f>& MovingPoints, vector<Point3f>& FixedPoints) {

	const int SecondFrameSize = SecondFrame.size();
	
	/*vector<Point3f> Moving_Points;*/
	for (int i = 0; i < SecondFrame.size(); i++) {
		bool PointFound = false;
		for (int j = 0; j < FirstFramce.size(); j++) {

			if ((abs(SecondFrame[i].x - FirstFramce[j].x) < threshold) && (abs(SecondFrame[i].y - FirstFramce[j].y) < threshold) &&
				(abs(SecondFrame[i].z - FirstFramce[j].z) < threshold)) {
				PointFound = true;
				break;
			}
		}

		if (!PointFound) {
			
			MovingPoints.push_back(SecondFrame[i]);
			
			//Moving_PointsIndexes.push_back(SecondFrame[i]);
		}
		else {
			FixedPoints.push_back(SecondFrame[i]);
		}
	}

	
}



void GetMovingPoints3(vector<Point3f>& FirstFramce, vector<Point3f>& SecondFrame, float threshold, vector<Point3f>& MovingPoints, vector<Point3f>& FixedPoints) {

	const int SecondFrameSize = SecondFrame.size();

	/*vector<Point3f> Moving_Points;*/
	for (int i = 0; i < SecondFrame.size(); i++) {
		bool PointFound = false;
		int from = 0;
		int to = 0;
		if ((i - 200) >= 0) {
			from = i-200;
		}
		else {
			from = 0;
		}

		if ((i + 200) < SecondFrame.size() && (i + 200) < FirstFramce.size()) {
			to = i+200;
		}
		else {
			to = min(SecondFrame.size()-1 , FirstFramce.size()-1);
		}


		for (int j = from; j < to; j++) {

			if ((abs(SecondFrame[i].x - FirstFramce[j].x) < threshold) && (abs(SecondFrame[i].y - FirstFramce[j].y) < threshold) &&
				(abs(SecondFrame[i].z - FirstFramce[j].z) < threshold)) {
				PointFound = true;
				break;
			}
		}

		if (!PointFound) {

			MovingPoints.push_back(SecondFrame[i]);

			//Moving_PointsIndexes.push_back(SecondFrame[i]);
		}
		else {
			FixedPoints.push_back(SecondFrame[i]);
		}
	}


}