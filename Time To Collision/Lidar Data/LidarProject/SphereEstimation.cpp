#include "SphereEstimation.h"


 /* Sphere Estimation
  * Goal: Fit sphere to the given spatial points
  * Sphere is given by the center coordinates [px py pz] and the radius r
  * The center point considered as intersection of lines
  * and midP are the normal vectorsand a points lying on a line
  *  
  * Input:
  * vector<Point3f> pts: input points for sphere fitting
  *
  * Output:
  * array<float, 4> an array with four elements
  * return[0]: px
  * return[1]: py
  * return[2]: pz
  * return[3]: r
  *
  *
  */
array<float, 4> EstimateSphereMinimal(const vector<Point3f>& pts){
    int num = pts.size();
    array<Point3f, 4> midP; 
    array<Point3f, 4> n;
    Mat nM(4, 3, CV_32F);
    Mat d(4, 1, CV_32F);
    Mat S0(3, 1, CV_32F);

    for (int idx = 0; idx < 4; ++idx)
    {
        midP.at(idx) = (pts.at(idx) + pts.at((idx + 1) % 4)) / 2;
        n.at(idx) = pts.at(idx) - midP.at(idx);
        n.at(idx) /= normP3f(n.at(idx)); // norm in 3D!
        d.at<float>(idx, 0) = n.at(idx).x * midP.at(idx).x + n.at(idx).y * midP.at(idx).y + n.at(idx).z * midP.at(idx).z; // d in 3D!
        nM.at<float>(idx, 0) = n.at(idx).x;
        nM.at<float>(idx, 1) = n.at(idx).y;
        nM.at<float>(idx, 2) = n.at(idx).z;
    }
    cv::solve(nM, d, S0, cv::DECOMP_SVD); // nM * S0 = d
    Point3f r;
    r.x = S0.at<float>(0, 0) - pts.at(0).x;
    r.y = S0.at<float>(1, 0) - pts.at(0).y;
    r.z = S0.at<float>(2, 0) - pts.at(0).z;

    array<float, 4> ret{ S0.at<float>(0, 0),
                         S0.at<float>(1, 0),
                         S0.at<float>(2, 0),
                         normP3f(r) };
    return ret;
}

/* Sphere Estimation
 * Goal: Fit vertical sphere to the given spatial points
 * Vertical sphere is given by a point of the axis [px py pz] and the radius r
 * The implementation use sphere fitting with an iterative least-squares method
 * based on David Eberly, "Least Squares Fitting of Data"
 * (http://www.ncorr.com/download/publications/eberlyleastsquares.pdf)
 *
 * Input:
 * vector<Point3f> pts: input points for sphere fitting
 * const array<float, 4>& paramIni: initial estimation of the sphere
 *
 * Output:
 * array<float, 4> an array with four elements
 * return[0]: px
 * return[1]: py
 * return[2]: pz
 * return[3]: r
 *
 *
 */
array<float, 4> EstimateSphereLsq(vector<Point3f> pts, const array<float, 4>& paramIni) {
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

    Point3f S0(paramIni[0] - mean.x, paramIni[1] - mean.y, paramIni[2] - mean.z); //initial estimation in 3D

    float r = paramIni[3];
    Point3f diff(1.0f, 1.0f, 1.0f);
    int it = 0;

    while (diff != Point3f(0.0f, 0.0f, 0.0f) && it < numIt) {
        float r_avg = 0;
        Point3f dir_avg(0.0f, 0.0f, 0.0f);
        Point3f S0_tmp = S0;
        for (int idx = 0; idx < numPts; idx++)
        {
            Point3f dir_i = S0 - pts.at(idx);
            float r_i = normP3f(dir_i);
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
                         S0.z + mean.z,
                         r};
    return ret;
}

 /* Robust Sphere Estimation
  * Goal: Fit vertical Sphere to the given spatial points
  *       The fitting is robust, RANSAC method is applied
  * Vertical Sphere is given by a point of the axis [px py pz] and the radius r
  *
  * Input:
  * vector<Point3f> pts: input points for Sphere fitting
  * threshold: threshold for point-Sphere distabce
  * iterateNum: number of iteration within RANSAC
  *
  * Output:
  * array<float, 4> an array with four elements
  * return[0]: px
  * return[1]: py
  * return[2]: pz
  * return[3]: r
  */
array<float, 4> EstimateSphereRANSAC(const vector<Point3f>& pts, const float threshold, const int iterateNum) {
    int num = pts.size();
    const float rMax = 0.35;
    const float rMin = 0.1;

    int bestSampleInlierNum = 0;
    array<float, 4> bestSphere;
    cout << "No. of points:" << num << endl;

    for (int iter = 0; iter < iterateNum; iter++) {

        float rand1 = (float)(rand()) / RAND_MAX;
        float rand2 = (float)(rand()) / RAND_MAX;
        float rand3 = (float)(rand()) / RAND_MAX;
        float rand4 = (float)(rand()) / RAND_MAX;

        //Generate three different(!) random numbers:
        int index1 = (int)(rand1 * num);
        int index2 = (int)(rand2 * num);
        while (index2 == index1) { rand2 = (float)(rand()) / RAND_MAX; index2 = (int)(rand2 * num); }
        int index3 = (int)(rand3 * num);
        while ((index3 == index1) || (index3 == index2)) { rand3 = (float)(rand()) / RAND_MAX; index3 = (int)(rand3 * num); }
        int index4 = (int)(rand4 * num);
        while ((index4 == index1) || (index4 == index2) || (index4 == index3)) { rand4 = (float)(rand()) / RAND_MAX; index4 = (int)(rand4 * num); }

        Point3f pt1 = pts.at(index1);
        Point3f pt2 = pts.at(index2);
        Point3f pt3 = pts.at(index3);
        Point3f pt4 = pts.at(index4);

        //In each RANSAC cycle, a minimal sample with 4 points are formed
        vector<Point3f> minimalSample{ pt1, pt2, pt3, pt4 };

        array<float, 4> sampleSphere = EstimateSphereMinimal(minimalSample);

        //printf("Sphere params: %f %f %f %f \n",sampleSphere[0],sampleSphere[1],sampleSphere[2],sampleSphere[3]);
        
        //Compute consensus set
        RANSACDiffs sampleResult = SpherePointRANSACDifferences(pts, sampleSphere, threshold);
        //printf("NumInliers: %d \n",sampleResult.inliersNum);

        //Check the new test is larger than the best one.
        if (sampleResult.inliersNum > bestSampleInlierNum && sampleSphere[3] < rMax && sampleSphere[3] > rMin) {
            bestSampleInlierNum = sampleResult.inliersNum;
            bestSphere[0] = sampleSphere[0];
            bestSphere[1] = sampleSphere[1];
            bestSphere[2] = sampleSphere[2];
            bestSphere[3] = sampleSphere[3];
            cout << "Inlier num. update: " << bestSampleInlierNum << "\t\tradius: " << bestSphere[3] << endl;
        }//end if
    }//end for iter

    //Finally, the Sphere is refitted from thew best consensus set
    RANSACDiffs bestResult = SpherePointRANSACDifferences(pts, bestSphere, threshold);

    vector<Point3f> inlierPts;

    for (int idx = 0; idx < num; idx++) {
        if (bestResult.isInliers.at(idx)) {
            inlierPts.push_back(pts.at(idx));
        }
    }

    array<float, 4> finalSphere = EstimateSphereLsq(inlierPts, bestSphere);
    return finalSphere;
}

/* Sphere-point differences
 * Goal: This method calculates the sphere point differences, and determines if a point is an outlier.
 *
 * Input:
 * vector<Point3f> pts: input points
 * const array<float, 4> sphere: sphere parameters; sphere[0]:px, sphere[1]:py, sphere[2]:pz, sphere[3]:r
 * float threshold: the threshold for inlier/outlier separation
 *
 * Output:
 * RANSACDiffs
 * see PlaneEstimation.h for details
 *
 *
 */
RANSACDiffs SpherePointRANSACDifferences(const vector<Point3f>& pts, const array<float, 4>& sphere, const float threshold) {
    int num = pts.size();
    Point3f S0(sphere[0], sphere[1], sphere[2]);
    float r = sphere[3];

    RANSACDiffs ret;

    vector<bool> isInliers;
    vector<float> distances;



    int inlierCounter = 0;
    for (int idx = 0; idx < num; idx++) {
        Point3f pt = pts.at(idx) - S0;
        float diff = abs(normP3f(pt) - r);      // point-S0 distance in 3D
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