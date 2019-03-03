// for std
#include <iostream>
#include <stdio.h>
// for eigen
#include <Eigen/Core>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
//#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#define PI 3.14159265

// 相机内参
double cx = 270;
double cy = 270;
double fx = 270;
double fy = 270;

using namespace cv;
using namespace std;

int main() {
	Mat img1_1_c = imread("x_t_0_f.jpg");
	Mat img1_2_c = imread("x_t_1_f.jpg");
	
	if (!img1_1_c.data || !img1_2_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
		return -1;
	}
	
	Mat img1_1, img1_2;

	cvtColor(img1_1_c, img1_1, COLOR_BGR2GRAY);
	cvtColor(img1_2_c, img1_2, COLOR_BGR2GRAY);

	// Create Keypoint and descriptor extractor
	std::vector<KeyPoint> keypoints_1_1, keypoints_1_2;
	Mat descriptors_1_1, descriptors_1_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();

	// Create matcher
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	// Extractor keypoint and descriptor
	detector->detect(img1_1, keypoints_1_1);
	detector->detect(img1_2, keypoints_1_2);
	descriptor->compute(img1_1, keypoints_1_1, descriptors_1_1);
	descriptor->compute(img1_2, keypoints_1_2, descriptors_1_2);


	// Match
	vector<DMatch> matches1;
	matcher->match(descriptors_1_1, descriptors_1_2, matches1);

	double min_dist1 = 10000, max_dist1 = 0;	

	for (int i = 0; i < descriptors_1_1.rows; i++)	
	{
		double dist = matches1[i].distance;
		if (dist < min_dist1) min_dist1 = dist;
		if (dist > max_dist1) max_dist1 = dist;
	}


	// Optimize the match
	std::vector< DMatch > good_matches1;				
	for (int i = 0; i < descriptors_1_1.rows; i++)
	{
		if (matches1[i].distance <= max(2 * min_dist1, 50.0))
		{
			good_matches1.push_back(matches1[i]);
		}
	}


	Mat img_match1;
	Mat img_goodmatch1;
	drawMatches(img1_1, keypoints_1_1, img1_2, keypoints_1_2, matches1, img_match1);			//Front
	drawMatches(img1_1, keypoints_1_1, img1_2, keypoints_1_2, good_matches1, img_goodmatch1);


	namedWindow("Front_All match", WINDOW_NORMAL);
	imshow("Front_All match", img_goodmatch1);

	namedWindow("Front_All match2", WINDOW_NORMAL);
	imshow("Front_All match2", img_match1);

	// extract keypoint according to the match result 
		std::vector<KeyPoint> keypoints_1_1_match, keypoints_1_2_match;		//Front
		vector<Point2f> keypoints_1_1_match_2f((int)good_matches1.size()), keypoints_1_2_match_2f((int)good_matches1.size());
		for (int i = 0; i < (int)good_matches1.size(); i++)
		{
			keypoints_1_1_match.push_back(keypoints_1_1[good_matches1[i].queryIdx]);
			keypoints_1_2_match.push_back(keypoints_1_2[good_matches1[i].trainIdx]);
			keypoints_1_1_match_2f[i].x = keypoints_1_1_match[i].pt.x;
			keypoints_1_1_match_2f[i].y = keypoints_1_1_match[i].pt.y;
			keypoints_1_2_match_2f[i].x = keypoints_1_2_match[i].pt.x;
			keypoints_1_2_match_2f[i].y = keypoints_1_2_match[i].pt.y;
		}





	Mat F = findFundamentalMat(keypoints_1_1_match_2f,keypoints_1_2_match_2f,cv::FM_8POINT);
	Mat K = Mat::ones(3, 3, CV_64FC1);
	K.at<double>(0, 0) = 270;
	K.at<double>(0, 1) = 0;
	K.at<double>(0, 2) = 270;
	K.at<double>(1, 0) = 0;
	K.at<double>(1, 1) = -270;
	K.at<double>(1, 2) = 270;
	K.at<double>(2, 0) = 0;
	K.at<double>(2, 1) = 0;
	K.at<double>(2, 2) = 1;
	Mat E = K.t()*F*K;
	Mat R, t;
	recoverPose(E,keypoints_1_1_match_2f,keypoints_1_2_match_2f,K,R,t);


		//Construct map points
		cv::Mat P0 = K*cv::Mat::eye(3,4,CV_64F);
		cv::Mat Rt,X;
		cv::hconcat(R,t,Rt);
		cv::Mat P1 = K*Rt;
		cv::triangulatePoints(P0,P1,keypoints_1_1_match_2f,keypoints_1_2_match_2f,X);
		X.row(0) = X.row(0)/X.row(3);
		X.row(1) = X.row(1)/X.row(3);
		X.row(2) = X.row(2)/X.row(3);
		X.row(3) = 1;


	FILE* deltaio;
        deltaio = fopen("delta", "w");
	for(int c = 0;c<X.cols;c++){
		fprintf(deltaio, "%f %f %f\n", X.at<double>(0,c),X.at<double>(1,c),X.at<double>(2,c)); 
		fflush(deltaio);	
	}


	waitKey(0);



	return 0;

}
