/*
 * feature_extract.h
 *
 *  Created on: Aug 15, 2018
 *      Author: jdy
 */

#ifndef FEATURE_EXTRACT_H_
#define FEATURE_EXTRACT_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;
using namespace cv;

void find_feature_matches_orb ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches );

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t);

void find_feature_matches_sift( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches );


#endif /* FEATURE_EXTRACT_H_ */
