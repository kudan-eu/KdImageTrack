/**
 *    This file is part of KdImageTrack.
 *
 *   KdImageTrack is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   KdImageTrack is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with KdImageTrack. If not, see <https://www.gnu.org/licenses/>.
 *
 *    Copyright (c) 2015 Kudan. All rights reserved.
 **/

#include "HomographyFinding.hpp"
#include "Geometry.hpp"
#include "Helper.h"
#include "Logging.hpp"
#include <set>
#include <time.h>

#if !defined(M_PI)
	#define M_PI 3.141
#endif

Namespace(KdImageTrack)

/* Solve for homography from two sets of matching points with SVD
 Uses all points in the lists */
cv::Mat solveHsvd(std::vector<cv::Point2f> src, std::vector<cv::Point2f> dst)
{
    
    const size_t N = src.size();
    
    cv::Mat A = cv::Mat::zeros( int(N)*2, 9, CV_64F);

    for (int i = 0; i < N; i++) {
        int u = 0;
        
        
        float x1 = src[i].x;
        float y1 = src[i].y;
        float x2 = dst[i].x;
        float y2 = dst[i].y;
        
        A.at<double>(2 * i,   u) = -x1;        u++;
        A.at<double>(2 * i,   u) = -y1;        u++;
        A.at<double>(2 * i,   u) = -1;         u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = x2 * x1;    u++;
        A.at<double>(2 * i,   u) = x2 * y1;    u++;
        A.at<double>(2 * i,   u) = x2;         u++;
        
        u = 0;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = -x1;      u++;
        A.at<double>(2 * i + 1, u) = -y1;      u++;
        A.at<double>(2 * i + 1, u) = -1;       u++;
        A.at<double>(2 * i + 1, u) = y2 * x1;  u++;
        A.at<double>(2 * i + 1, u) = y2 * y1;  u++;
        A.at<double>(2 * i + 1, u) = y2;       u++;

    }
    

    cv::SVD svd(A, cv::SVD::FULL_UV);
    cv::Mat Vt = svd.vt;

    
    
    cv::Mat hv = Vt.row(Vt.rows - 1);
    
    
    // this really should be 1x9
    assert(hv.rows == 1);
    assert(hv.cols == 9);
    
    
    double lastVal = hv.at<double>(8); // final value, divide all by this to get 1 in bottom right
    // Reshape 1x9 into 3x3
    cv::Mat H(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            H.at<double>(j, i) = hv.at<double>(3 * j + i)/lastVal;
        }
    }
    
    return H;
    
}

/* Solve for homography from two sets of matching points with SVD
 Uses a list to index into the point lists */
cv::Mat solveHsvd(size_t* hyp, int N, std::vector<cv::Point2f> src, std::vector<cv::Point2f> dst)
{

    cv::Mat A = cv::Mat::zeros( N * 2, 9, CV_64F);
    
    for (int i = 0; i < N; i++) {
        int u = 0;
        
        size_t h = hyp[i];
        
        float x1 = src[h].x;
        float y1 = src[h].y;
        float x2 = dst[h].x;
        float y2 = dst[h].y;
        
        A.at<double>(2 * i,   u) = -x1;        u++;
        A.at<double>(2 * i,   u) = -y1;        u++;
        A.at<double>(2 * i,   u) = -1;         u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = 0;          u++;
        A.at<double>(2 * i,   u) = x2 * x1;    u++;
        A.at<double>(2 * i,   u) = x2 * y1;    u++;
        A.at<double>(2 * i,   u) = x2;         u++;
        
        u = 0;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = 0;        u++;
        A.at<double>(2 * i + 1, u) = -x1;      u++;
        A.at<double>(2 * i + 1, u) = -y1;      u++;
        A.at<double>(2 * i + 1, u) = -1;       u++;
        A.at<double>(2 * i + 1, u) = y2 * x1;  u++;
        A.at<double>(2 * i + 1, u) = y2 * y1;  u++;
        A.at<double>(2 * i + 1, u) = y2;       u++;
        
    }
    
    
    cv::SVD svd(A, cv::SVD::FULL_UV);
    cv::Mat Vt = svd.vt;
    
    
    
    cv::Mat hv = Vt.row(Vt.rows - 1);
    
    
    // this really should be 1x9
    assert(hv.rows == 1);
    assert(hv.cols == 9);
    
    
    double lastVal = hv.at<double>(8); // final value, divide all by this to get 1 in bottom right
    // Reshape 1x9 into 3x3
    cv::Mat H(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            H.at<double>(j, i) = hv.at<double>(3 * j + i)/lastVal;
        }
    }
    
    return H;
    
}


/* Project source points according to hypothesised homography, compute total error compared to estination.
 Counts how many are within the threshold (and returns their IDs) */
std::vector<size_t> getInliers(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, cv::Mat H, float reprojectionThreshold)
{
    std::vector<size_t> inlierIds;
    inlierIds.reserve(srcPoints.size());
    

    for (size_t p = 0; p < srcPoints.size(); p++) {
        cv::Point2f proj = projectWithH(srcPoints[p], H);
        float err = cv::norm(proj - dstPoints[p]);
        bool isInlier = (err < reprojectionThreshold);
        if (isInlier) {
            inlierIds.push_back(p);
        }

    }
    return inlierIds;
    
}

/* Update how many iterations to be done (in total), based on the probability of seeing something even better than what we just saw.
 Logic copied from OpenCV */
int updateMaxIterations(int maxIterations, double confidence, size_t N, size_t numInliers)
{
    double c = 1.0 - confidence;
    double logc = log(c);
    
    double goodness = (N - numInliers)/double(N);
    double d = 1 - (pow(1 - goodness, 4.0));
    double logd = log(d);
    
    if (logd >= 0) {
        return maxIterations;
    }
    if (logc <= logd * maxIterations) {
        return maxIterations;
    }
    
    return int(round(logc / logd));
}



/* Implementation of Speeding-up homography estimation in mobile devices by Pablo M ́arquez-Neila et al. */
bool marquezNeilaTest(size_t* hyp, std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst)
{
    
    for (int i = 0; i < HYP_SIZE; i++) {
        size_t a = hyp[i];
        size_t b = hyp[(i + 1) % 4];
        size_t c = hyp[(i + 2) % 4];
        
        cv::Point3f a0 (src[a].x, src[a].y, 1);
        cv::Point3f b0 (src[b].x, src[b].y, 1);
        cv::Point3f c0 (src[c].x, src[c].y, 1);
        cv::Point3f a1 (dst[a].x, dst[a].y, 1);
        cv::Point3f b1 (dst[b].x, dst[b].y, 1);
        cv::Point3f c1 (dst[c].x, dst[c].y, 1);
        
        cv::Point3f cb0 = b0.cross(c0);
        float mc0 =cb0.dot(a0);
        cv::Point3f cb1 = b1.cross(c1);
        float mc1 =cb1.dot(a1);
        
        // if they have different signs, their prodict is negative
        if (mc0 * mc1 < 0) {
            return false;
        }
        
    }
    return true;
    
}


/* Check that no angle between lines connecting points is less than the threshold
 This is to eliminate collinear configurations (and stretched shapes) */
bool minAngleTest(size_t* hyp, std::vector<cv::Point2f> &points, float threshold)
{
    

    for (int i = 0; i < HYP_SIZE; i++) {
        
        cv::Point2f a = points[hyp[i]];
        
        
        // take each of the three points as the centre in turn, find the minimum angle: that's the one to test
        for (int j = 1; j < 3; j++) {
            
            int ii = (i + j) % 4;
            int iii = (i + j + 1) % 4;
            cv::Point2f b = points[hyp[ ii ]];
            cv::Point2f c = points[hyp[ iii]];
            
            cv::Point2f ab = b - a;
            cv::Point2f ac = c - a;
            // if the lines are too short, don't bother, just say this angle wasy zero for now
            if (cv::norm(ab) < 1) {
                continue;
            }
            if (cv::norm(ac) < 1) {
                continue;
            }
            // absolute angle, because doesn't matter which way it's going
            float cosine = ab.dot(ac) / (cv::norm(ab)*cv::norm(ac)) ;
            if (fabs(cosine) > 1) {
                continue;
            }
            float ang = fabs( acos( cosine ) );
            
            ang = 360 * ang / (2 * M_PI);
            if (ang < threshold) {
                return false;
            }
           
        }
        
        
    }
    return true;
    
}

/* Check the minimum angle within the source and destination lists */
bool minAngleTest(size_t* hyp, std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, float threshold)
{
    if (!minAngleTest(hyp, src, threshold)) {
        return false;
    }
    if (!minAngleTest(hyp, dst, threshold)) {
        return false;
    }
    return true;
}



/* Test to see if the smallest distance between any pair of points is less than a threshold
 This is to avoid point configurations which are almost triangles (degenerate) */
float minDistanceTest(size_t* hyp, std::vector<cv::Point2f> &points, float threshold)
{
    for (int i = 0; i < HYP_SIZE; i++) {
        for (int j = i + 1; j < HYP_SIZE; j++) {
            float d = cv::norm(points[hyp[i]] - points[hyp[j]]);
            if (d < threshold) {
                return false;
            }
        }
    }
    return true;
}

/* Check the minimum distance within the source and destination lists */
float minDistanceTest(size_t* hyp, std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, float threshold)
{
    if (!minDistanceTest(hyp, src, threshold)) {
        return false;
    }
    if (!minDistanceTest(hyp, dst, threshold)) {
        return false;
    }
    return true;
}



/* Generate an ordered set of unique IDs in the given range [0,N)
 The ordering is necessary mostly because it makes it easier to enforce uniqueness without explicit sorting.
 Time complexity should be quadratic in list size? Which is currently =4. But otherwise linear. */
void generateHypothesisOrdered(size_t *hyp, size_t N)
{
    
    
    for (int i = 0; i < HYP_SIZE; i++) {
        
        // generate a random number from a slightly smaller version of the set each time:
        int rnum = rand() % (N - i);
        
        
        
        if (i == 0) {
            // Always insert the first element at the beginning, no checks needed:
            hyp[i] = rnum;
        }
        else {
            
            // Look over the ids written so far:
            for (int j = 0; j < i; j++) {
                size_t el = hyp[j];
                // If this is at least as big as each element in turn, increment it. This avoids collisions
                
                if (rnum >= el) {
                    rnum++;
                }
                
                
            }
            
            
            // Make sure the new element is inserted in the right order
            // Assume the others were already ordered, which just means you need to find the right place to add the new one
            // Don't necessarily write it at the end, but write it before the first element greater than it
            
            bool isInserted = false;
            for (int j = 0; j < i; j++) {
                // find where in the list so far it is just SMALLER than the current element
                if (rnum < hyp[j]) {
                    // Shift everything from here to the end (leaves a duplicate)
                    
                    for (int k = std::min(i, HYP_SIZE - 2); k >= j; k--) {
                        hyp[k + 1] =  hyp[k];
                    }
                    
                    
                    // then put it in that place in the list:
                    hyp[j] = rnum;
                    
                    
                    // stop now, and record that it was put in the list
                    isInserted = true;
                    break;
                }
            }
            
            
            // if it was NOT put in the list, it's the biggest thing so far - simply write at the end
            if (!isInserted) {
                hyp[i] = rnum;
            }
            
        }
        
    }
    

}



/* Simpler way of generating hypothesis sets: pick 4 random numbers in the range [0,N)
 This is not ordered and might generate sets with duplicate IDs! */
void generateHypothesisSimple(size_t *h, int N)
{
    
    for (int i = 0; i < HYP_SIZE; i++) {
        h[i] = rand()%N;
    }

}


/* Generate a single unique number representing a hypothesis set - to check for duplicates */
long int getHypothesisId(size_t* h, size_t N)
{
 
    long int hd = 0;
    for (size_t i = 0; i < HYP_SIZE; i++) {
        hd += h[i] * pow(int(N), int(i));
    }
    return hd;
}


/* Robustly find a homography between two sets of points.
 Returns the homography correponding to the best minimal set (doFinalSolution=false) or the homography generated from all inliers (doFinalSolution=true)
 Returns an empty matrix if no solution can be found */
cv::Mat findHomographyRANSAC(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, float reprojectionThreshold, int maxIterations, bool doFinalSolution, std::vector<unsigned char> &inlierMask, int &iterationsDone)
{
    
    // Actual (pseudo)randomness!
    srand(int(time(0)));
   
    const size_t N = srcPoints.size();

    
    // Check there are enough points to do anything:
    if (N < HYP_SIZE) {
        return cv::Mat();
    }
    // If there is the minimum number, solve explicitly:
    if (N == HYP_SIZE) {
        inlierMask.clear();
        inlierMask.resize(N, 1);

        return solveHsvd( srcPoints, dstPoints);
    }
    
    
    const float confidence = 0.995;
    
    // Parameters for the early-quit tests:
    bool useMarquezTest = true;
    const float minAngleThreshold = 5.0;
    const float minDistanceThreshold = 10.0;

    
    
    
    
    cv::Mat bestH = cv::Mat::eye(3, 3, CV_64F);
    std::vector<size_t> bestInliers;
    size_t maxInliers = 0;
   

    int currentMaxIterations = maxIterations;
    int numIterations = 0;
    
    
  
    
    
    // Store the unique id of each hypothesis used, to make sure duplicates are avoided!
    std::set<long int> usedHypotheses;


    // Allocate memory to store each hypothesis set in turn (never have more than one at the same time)
    size_t *hyp = new size_t[HYP_SIZE];
    
    // ITERATE:
    for (int h = 0; h <  currentMaxIterations; h++) {
        
        generateHypothesisOrdered(hyp, N);

        // Check for duplicates:
        
        // Unique id:
        long int hi = getHypothesisId(hyp, N);

        // Insert returns whetehr it was inserted. If it was not inseted it was already used
        bool isAlreadyUsed = !usedHypotheses.insert(hi).second;
        if (isAlreadyUsed) {
            continue;
        }
        

        // * Pre-tests to avoid bad hypotheses:
        
        if (useMarquezTest && !marquezNeilaTest(hyp, srcPoints, dstPoints)) {
            continue;
        }
        
        if (minAngleThreshold >0 && !minAngleTest(hyp, srcPoints, dstPoints, minAngleThreshold)) {
            continue;
        }
        
        if (minDistanceThreshold >0 && !minDistanceTest(hyp, srcPoints, dstPoints, minDistanceThreshold)) {
            continue;
        }
        
        
        // generate the homography which maps src->dst;
        cv::Mat H = solveHsvd(hyp, HYP_SIZE, srcPoints, dstPoints);
        
       
        
        // evaluate!

        std::vector<size_t> inlierIds = getInliers(srcPoints, dstPoints, H, reprojectionThreshold);
        size_t numInliers = inlierIds.size();
        
        if (numInliers > maxInliers) {

            bestH = H;
            maxInliers = numInliers;
            bestInliers = inlierIds;

            // update how many iterations are needed
            currentMaxIterations = updateMaxIterations(currentMaxIterations, confidence, N, maxInliers);
            
        }
        
        // Count iterations actually done
        numIterations++;
        
        
        
    }
    
    cv::Mat finalH;
    
    // initialise output inlier mask and set all values to 0
    if (inlierMask.size() == N) {
        for (int n = 0; n < N ; n++) inlierMask[n] = 0;
    }
    else {
        inlierMask.clear();
        inlierMask.resize(N, 0);
    }
    
    // If there are fewer inliers than the minimal set, fail!
    if (bestInliers.size() < HYP_SIZE) {
        delete []hyp;
        return cv::Mat();
    }
    
    for (int i = 0; i < bestInliers.size(); i++ ) {
        inlierMask[bestInliers[i]] = 1;
    }
    
    
    
    if (doFinalSolution) {
        // Solve for the best homography between inilers using east squares:
        finalH = solveHsvd(&bestInliers[0], (int)bestInliers.size(), srcPoints, dstPoints);
        
    }
    else {
        // Return the best minimal hypothesis:
        finalH = bestH;
    }
    
    printlog(LOG_HOMOGRAPHY,"Done NEW homography with %i iterations \n", numIterations);
    
    iterationsDone = numIterations;
    
    // clear the memory used to store the hypothesis IDs!
    delete []hyp;
    
    return finalH;

    
}

NamespaceEnd
