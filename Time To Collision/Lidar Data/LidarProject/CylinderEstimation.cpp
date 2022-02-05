#include "CylinderEstimation.h"

 
array<float, 4> EstimateCylinderMinimal(const vector<Point3f>& pts){
    int num = pts.size();
    array<Point3f, 3> midP; 
    array<Point3f, 3> n;
    Mat nM(3, 2, CV_32F);
    Mat d(3, 1, CV_32F);
    Mat S0(2, 1, CV_32F);

    for (int idx = 0; idx < 3; ++idx)
    {
        midP.at(idx) = (pts.at(idx) + pts.at((idx + 1) % 3)) / 2;
        n.at(idx) = pts.at(idx) - midP.at(idx);
        n.at(idx) /= normP3fXY(n.at(idx));
        d.at<float>(idx, 0) = n.at(idx).x * midP.at(idx).x + n.at(idx).y * midP.at(idx).y;
        nM.at<float>(idx, 0) = n.at(idx).x;
        nM.at<float>(idx, 1) = n.at(idx).y;
    }
    cv::solve(nM, d, S0, cv::DECOMP_SVD); // nM * S0 = d
    Point2f r;
    r.x = S0.at<float>(0, 0) - pts.at(0).x;
    r.y = S0.at<float>(1, 0) - pts.at(0).y;

    array<float, 4> ret{ S0.at<float>(0, 0),
                         S0.at<float>(1, 0),
                         pts.at(0).z,
                         normP2f(r) };
    return ret;
}

array<float, 4> EstimateCylinderLsq(vector<Point3f> pts, const array<float, 4>& paramIni) {
    const int numIt = 1000;
    cv::Point3f sum = std::accumulate(
        pts.begin(), pts.end(), 
        cv::Point3f(0.0f, 0.0f, 0.0f),  
        std::plus<cv::Point3f>()     
    );
    const int numPts = pts.size();
    cv::Point3f mean = sum / numPts;

    for (int idx = 0; idx < numPts; ++idx) {
        pts.at(idx) -= mean;
    }

    Point2f S0(paramIni[0] - mean.x, paramIni[1] - mean.y); //initial estimation

    float r = paramIni[3];
    Point2f diff(1.0f, 1.0f);
    int it = 0;

    while (diff != Point2f(0.0f, 0.0f) && it < numIt) {
        float r_avg = 0;
        Point2f dir_avg(0.0f, 0.0f);
        Point2f S0_tmp = S0;
        for (int idx = 0; idx < numPts; idx++)
        {
            Point2f dir_i = S0 - Point2f(pts.at(idx).x, pts.at(idx).y);
            float r_i = normP2f(dir_i);
            r_avg += r_i;
            dir_avg += dir_i / r_i;
        }

        S0 = (r_avg * dir_avg) / (numPts*numPts);
        r = r_avg / numPts;
        diff = S0 - S0_tmp;
        ++it;
    }
    
    array<float, 4> ret{ S0.x + mean.x,
                         S0.y + mean.y,
                         mean.z, 
                         r};
    return ret;
}

 
array<float, 4> EstimateCylinderRANSAC(const vector<Point3f>& pts, const float threshold, const int iterateNum) {
    int num = pts.size();
    const float rMax = 1.0;
    const float rMin = 0.1;

    int bestSampleInlierNum = 0;
    array<float, 4> bestCylinder;
    cout << "No. of points:" << num << endl;

    for (int iter = 0; iter < iterateNum; iter++) {

        float rand1 = (float)(rand()) / RAND_MAX;
        float rand2 = (float)(rand()) / RAND_MAX;
        float rand3 = (float)(rand()) / RAND_MAX;

        //Generate three different(!) random numbers:
        int index1 = (int)(rand1 * num);
        int index2 = (int)(rand2 * num);
        while (index2 == index1) { rand2 = (float)(rand()) / RAND_MAX; index2 = (int)(rand2 * num); }
        int index3 = (int)(rand3 * num);
        while ((index3 == index1) || (index3 == index2)) { rand3 = (float)(rand()) / RAND_MAX; index3 = (int)(rand3 * num); }

        Point3f pt1 = pts.at(index1);
        Point3f pt2 = pts.at(index2);
        Point3f pt3 = pts.at(index3);

        //In each RANSAC cycle, a minimal sample with 3 points are formed

        vector<Point3f> minimalSample;
        minimalSample.push_back(pt1);
        minimalSample.push_back(pt2);
        minimalSample.push_back(pt3);

        array<float, 4> sampleCylinder = EstimateCylinderMinimal(minimalSample);//  EstimateCylinderLsq(minimalSample);
        //printf("Cylinder params: %f %f %f %f \n",sampleCylinder[0],sampleCylinder[1],sampleCylinder[2],sampleCylinder[3]);
        
        //Compute consensus set
        RANSACDiffs sampleResult = CylinderPointRANSACDifferences(pts, sampleCylinder, threshold);
        //printf("NumInliers: %d \n",sampleResult.inliersNum);

        //Check the new test is larger than the best one.
        if (sampleResult.inliersNum > bestSampleInlierNum && sampleCylinder[3] < rMax && sampleCylinder[3] > rMin) {
            bestSampleInlierNum = sampleResult.inliersNum;
            bestCylinder[0] = sampleCylinder[0];
            bestCylinder[1] = sampleCylinder[1];
            bestCylinder[2] = sampleCylinder[2];
            bestCylinder[3] = sampleCylinder[3];
            cout << "Inlier num. update: " << bestSampleInlierNum << "\t\tradius: " << bestCylinder[3] << endl;
        }//end if
    }//end for iter

    //Finally, the Cylinder is refitted from thew best consensus set
    RANSACDiffs bestResult = CylinderPointRANSACDifferences(pts, bestCylinder, threshold);

    vector<Point3f> inlierPts;

    for (int idx = 0; idx < num; idx++) {
        if (bestResult.isInliers.at(idx)) {
            inlierPts.push_back(pts.at(idx));
        }
    }

    array<float, 4> finalCylinder = EstimateCylinderLsq(inlierPts, bestCylinder);
    return finalCylinder;
}


RANSACDiffs CylinderPointRANSACDifferences(const vector<Point3f>& pts, const array<float, 4>& cylinder, const float threshold) {
    int num = pts.size();
    Point3f S0(cylinder[0], cylinder[1], cylinder[2]);
    float r = cylinder[3];

    RANSACDiffs ret;

    vector<bool> isInliers;
    vector<float> distances;

    int inlierCounter = 0;
    for (int idx = 0; idx < num; idx++) {
        Point3f pt = pts.at(idx) - S0;
        float diff = abs(normP3fXY(pt) - r);
        distances.push_back(diff);
        if (diff < threshold) {
            isInliers.push_back(true);
            inlierCounter++;
        }
        else {
            isInliers.push_back(false);
        }

    }//end for idx;

    ret.distances = distances;
    ret.isInliers = isInliers;
    ret.inliersNum = inlierCounter;

    return ret;
}

float normP3fXY(const Point3f& pt) {
    return sqrt(pt.x * pt.x + pt.y * pt.y);
}


float normP3f(const Point3f& pt) {
    return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

float normP2f(const Point2f& pt) {
    return sqrt(pt.x * pt.x + pt.y * pt.y);
}
