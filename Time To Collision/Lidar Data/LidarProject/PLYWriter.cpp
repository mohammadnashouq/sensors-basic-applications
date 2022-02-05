/* PLY format writer
 *
 * Implemented by Levente Hajder
 * hajder@inf.elte.hu
 * 30-06-2021
 */

#include "PLYWriter.h"


 /* Write a PLY file
  *
  * Input:
  *
  * fileName: nam of the file written
  * vector<Point3f> pts: spatial points
  * vector<Point3i> colors: colors. RGB color values are given by coordinates of Point3i
  *
  *
  */

//int* MovingIndises

void WritePLY(const char* fileName, vector<Point3f> pts, vector<Point3i> colors) {
    int num = pts.size();
    ofstream myfile;
    myfile.open(fileName);

    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << num << endl;
    myfile << "property float x\n";
    myfile << "property float y\n";
    myfile << "property float z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";


    for (int idx = 0; idx < num; idx++) {
        Point3f point = pts.at(idx);
        Point3i color = colors.at(idx);

        myfile << point.x << " " << point.y << " " << point.z << " " << color.x << " " << color.y << " " << color.z << endl;

    }

    myfile.close();



}


void WritePLY_movingPoints(const char* fileName, vector<Point3f>& pts ,int* MovingIndises) {
    int num = pts.size();
    ofstream myfile;
    myfile.open(fileName);

    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << num << endl;
    myfile << "property float x\n";
    myfile << "property float y\n";
    myfile << "property float z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";


    for (int idx = 0; idx < num; idx++) {
        Point3f point = pts.at(idx);
        Point3i color;
        if (MovingIndises[idx] == 1) {
            color = Point3i(255, 0, 0);
        }
        else {
            color = Point3i(255, 255, 255);
        }
       

        myfile << point.x << " " << point.y << " " << point.z << " " << color.x << " " << color.y << " " << color.z << endl;

    }

    myfile.close();



}


void WritePLY_Inlier_Outliers(const char* fileName, vector<Point3f>& Object_pts, vector<Point3f>& OutLiers_pts,Point3i& InliersColor , Point3i& OutLayerColor) {
   
    ofstream myfile;
    myfile.open(fileName);

    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << Object_pts.size()+ OutLiers_pts .size() << endl;
    myfile << "property float x\n";
    myfile << "property float y\n";
    myfile << "property float z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";

   
    for (int idx = 0; idx <  Object_pts.size(); idx++) {
        Point3f point = Object_pts.at(idx);
       
        myfile << point.x << " " << point.y << " " << point.z << " " << InliersColor.x << " " << InliersColor.y << " " << InliersColor.z << endl;

    }

    for (int idx = 0; idx < OutLiers_pts.size(); idx++) {
        Point3f point = OutLiers_pts.at(idx);
      
        myfile << point.x << " " << point.y << " " << point.z << " " << OutLayerColor.x << " " << OutLayerColor.y << " " << OutLayerColor.z << endl;

    }

    myfile.close();



}