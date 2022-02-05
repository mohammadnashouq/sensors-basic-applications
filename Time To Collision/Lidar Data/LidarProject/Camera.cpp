#include "Camera.h"

Camera::Camera(float fu, float fv, float u0, float v0)
{
	C = Mat::eye(3, 3, CV_32F);
	C.at<float>(0, 0) = fu;
	C.at<float>(1, 1) = fv;
	C.at<float>(0, 2) = u0;
	C.at<float>(1, 2) = v0;

	R = Mat(3, 3, CV_32F); 
	T = Mat(3, 1, CV_32F); 
	
	
	NUM = mrw.rowNum;
	color = Scalar(255, 255, 255); // B, G, R 
}

void Camera::drawPoints(float u, float v, float rad, Mat& resImg)
{
	// Spherical coordinates from two angles
	float tx = cos(u) * sin(v);
	float ty = sin(u) * sin(v);
	float tz = cos(v);

	//Translation vector = scale * position
	T.at<float>(0, 0) = rad * tx;
	T.at<float>(1, 0) = rad * ty;
	T.at<float>(2, 0) = rad * tz;

	// Rotation matrix = 3 ortogonal normal vector 
	// Axes: Z (at - eye), X (right), Y (up)) 
	Point3f Z(-tx, -ty, -tz);
	Point3f X(ty, -tx, 0.0f);
	X /= norm(X);
	int HowManyPi = (int)floor(v / 3.1415); // Due to overturning at y = 1 or -1
	if (!(HowManyPi % 2))
		X *= -1;
	Point3f Y = X.cross(Z);

	R.at<float>(1, 0) = Y.x;
	R.at<float>(1, 1) = Y.y;
	R.at<float>(1, 2) = Y.z;

	R.at<float>(0, 0) = X.x;
	R.at<float>(0, 1) = X.y;
	R.at<float>(0, 2) = X.z;

	R.at<float>(2, 0) = Z.x;
	R.at<float>(2, 1) = Z.y;
	R.at<float>(2, 2) = Z.z;

	for (int i = 0; i < NUM; i++) {
		Mat p_W(3, 1, CV_32F);
		p_W.at<float>(0, 0) = mrw.data[3 * i];
		p_W.at<float>(1, 0) = mrw.data[3 * i + 1];
		p_W.at<float>(2, 0) = mrw.data[3 * i + 2];

		Mat p_C = R * (p_W - T); // Projection from p_W to p_C
		p_C = C * p_C; // Projection from p_C (3D) to u_C (2D)
		p_C /= p_C.at<float>(2, 0); // Homogeneous division

		// Plot a circle at u_C with r = 2 radius
		circle(resImg, Point((int)p_C.at<float>(0, 0), (int)p_C.at<float>(1, 0)), 1.0, color, 1, 8);
		
	}
}



void Camera::drawPoints_Inlier_outlyer(float u, float v, float rad, Mat& resImg, vector<Point3f>& MovingObject , vector<Point3f>& FixedPoints, Point3f CravityCenter)
{
	// Spherical coordinates from two angles
	float tx = cos(u) * sin(v);
	float ty = sin(u) * sin(v);
	float tz = cos(v);

	//Translation vector = scale * position
	T.at<float>(0, 0) = rad * tx;
	T.at<float>(1, 0) = rad * ty;
	T.at<float>(2, 0) = rad * tz;

	// Rotation matrix = 3 ortogonal normal vector 
	// Axes: Z (at - eye), X (right), Y (up)) 
	Point3f Z(-tx, -ty, -tz);
	Point3f X(ty, -tx, 0.0f);
	X /= norm(X);
	int HowManyPi = (int)floor(v / 3.1415); // Due to overturning at y = 1 or -1
	if (!(HowManyPi % 2))
		X *= -1;
	Point3f Y = X.cross(Z);

	R.at<float>(1, 0) = Y.x;
	R.at<float>(1, 1) = Y.y;
	R.at<float>(1, 2) = Y.z;

	R.at<float>(0, 0) = X.x;
	R.at<float>(0, 1) = X.y;
	R.at<float>(0, 2) = X.z;

	R.at<float>(2, 0) = Z.x;
	R.at<float>(2, 1) = Z.y;
	R.at<float>(2, 2) = Z.z;

	for (int i = 0; i < MovingObject.size(); i++) {
		Mat p_W(3, 1, CV_32F);
		p_W.at<float>(0, 0) = MovingObject[i].x;
		p_W.at<float>(1, 0) = MovingObject[i].y;
		p_W.at<float>(2, 0) = MovingObject[i].z;

		Mat p_C = R * (p_W - T); // Projection from p_W to p_C
		p_C = C * p_C; // Projection from p_C (3D) to u_C (2D)
		p_C /= p_C.at<float>(2, 0); // Homogeneous division

		// Plot a circle at u_C with r = 2 radius
		Scalar Color = Scalar(0, 0, 255);
		circle(resImg, Point((int)p_C.at<float>(0, 0), (int)p_C.at<float>(1, 0)), 1.0, Color, 2, 8);

	}

	for (int i = 0; i < FixedPoints.size(); i++) {
		Mat p_W(3, 1, CV_32F);
		p_W.at<float>(0, 0) = FixedPoints[i].x;
		p_W.at<float>(1, 0) = FixedPoints[i].y;
		p_W.at<float>(2, 0) = FixedPoints[i].z;

		Mat p_C = R * (p_W - T); // Projection from p_W to p_C
		p_C = C * p_C; // Projection from p_C (3D) to u_C (2D)
		p_C /= p_C.at<float>(2, 0); // Homogeneous division

		// Plot a circle at u_C with r = 2 radius
		Scalar Color = Scalar(255, 255, 255);
		circle(resImg, Point((int)p_C.at<float>(0, 0), (int)p_C.at<float>(1, 0)), 1.0, Color, 1, 8);

	}


	if (MovingObject.size() > 0) {
		Mat p_W(3, 1, CV_32F);
		p_W.at<float>(0, 0) = CravityCenter.x;
		p_W.at<float>(1, 0) = CravityCenter.y;
		p_W.at<float>(2, 0) = CravityCenter.z;

		Mat p_C = R * (p_W - T); // Projection from p_W to p_C
		p_C = C * p_C; // Projection from p_C (3D) to u_C (2D)
		p_C /= p_C.at<float>(2, 0); // Homogeneous division

		Point CravityCenter2d = Point((int)p_C.at<float>(0, 0), (int)p_C.at<float>(1, 0));

		Scalar Color = Scalar(0, 255, 0);
		circle(resImg, CravityCenter2d, 2.0, Color, 2, 8);


		p_W.at<float>(0, 0) = 0;
		p_W.at<float>(1, 0) = 0;
		p_W.at<float>(2, 0) = 0;

		p_C = R * (p_W - T); // Projection from p_W to p_C
		p_C = C * p_C; // Projection from p_C (3D) to u_C (2D)
		p_C /= p_C.at<float>(2, 0); // Homogeneous division


		Point Origin = Point((int)p_C.at<float>(0, 0), (int)p_C.at<float>(1, 0));

		Color = Scalar(255, 0, 0);
		circle(resImg, Origin, 2.0, Color, 2, 8);

		line(resImg, CravityCenter2d, Origin, Scalar(255, 255, 0), 2, 8, 0);
	}

	

	



}
