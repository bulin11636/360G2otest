/*
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
*/

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
int angle = 100; // angle = 100 : +10 : 140

using namespace cv;
using namespace std;
//1. I'm trying to add g2o into my vo.
//2. I'm stuck at the transformation between eigen and cv(done).

Mat convert(int angle, Mat imgL, Mat imgF, Mat imgR); 
Mat bundleadjustment(vector<Point2f> points1,vector<Point2f> points2,Mat cv_pose_pre);

int main() {
	//================================Initial Step===================================//
					 // Open image file
	Mat img1_1, img1_2, img2_1, img2_2;
	//Front and Back
	char filename1_1[200], filename1_2[200], filename1_3[200], filename1_4[200], filename2_1[200], filename2_2[200], filename2_3[200], filename2_4[200];
	
	snprintf(filename1_1,sizeof(filename1_1), "EEFront/%05d.jpg", 537);
	snprintf(filename2_1,sizeof(filename2_1), "EEFront/%05d.jpg", 538);  
	snprintf(filename1_2,sizeof(filename1_2), "EEBack/%05d.jpg", 537);
	snprintf(filename2_2,sizeof(filename2_2), "EEBack/%05d.jpg", 538);
	snprintf(filename1_3,sizeof(filename1_3), "EERight/%05d.jpg", 537);
	snprintf(filename2_3,sizeof(filename2_3), "EERight/%05d.jpg", 538);
	snprintf(filename1_4,sizeof(filename1_4), "EELeft/%05d.jpg", 537);
	snprintf(filename2_4,sizeof(filename2_4), "EELeft/%05d.jpg", 538);
	//1->Front 2->Back 3->Right 4->Left
	cout << filename2_1 << endl;
	

	Mat img1_1_c = imread(filename1_1);
	Mat img1_2_c = imread(filename1_2);
	Mat img1_3_c = imread(filename1_3);
	Mat img1_4_c = imread(filename1_4);
	Mat img2_1_c = imread(filename2_1);
	Mat img2_2_c = imread(filename2_2);
	Mat img2_3_c = imread(filename2_3);
	Mat img2_4_c = imread(filename2_4);

	if (!img1_1_c.data || !img1_2_c.data || !img1_3_c.data || !img1_4_c.data || !img2_1_c.data || !img2_2_c.data || !img2_3_c.data || !img2_4_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
		return -1;
	}
	img1_1_c = convert(angle, img1_4_c, img1_1_c, img1_3_c);
	img1_2_c = convert(angle, img1_3_c, img1_2_c, img1_4_c);
	img2_1_c = convert(angle, img2_4_c, img2_1_c, img2_3_c);
	img2_2_c = convert(angle, img2_3_c, img2_2_c, img2_4_c);
	cvtColor(img1_1_c, img1_1, COLOR_BGR2GRAY);
	cvtColor(img1_2_c, img1_2, COLOR_BGR2GRAY);
	cvtColor(img2_1_c, img2_1, COLOR_BGR2GRAY);
	cvtColor(img2_2_c, img2_2, COLOR_BGR2GRAY);

	// Create Keypoint and descriptor extractor
	std::vector<KeyPoint> keypoints_1_1, keypoints_1_2, keypoints_2_1, keypoints_2_2;
	Mat descriptors_1_1, descriptors_1_2, descriptors_2_1, descriptors_2_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();

	// Create matcher
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	// Extractor keypoint and descriptor
	detector->detect(img1_1, keypoints_1_1);
	detector->detect(img1_2, keypoints_1_2);
	detector->detect(img2_1, keypoints_2_1);
	detector->detect(img2_2, keypoints_2_2);
	descriptor->compute(img1_1, keypoints_1_1, descriptors_1_1);
	descriptor->compute(img1_2, keypoints_1_2, descriptors_1_2);
	descriptor->compute(img2_1, keypoints_2_1, descriptors_2_1);
	descriptor->compute(img2_2, keypoints_2_2, descriptors_2_2);

	// Match
	vector<DMatch> matches1, matches2;
	matcher->match(descriptors_1_1, descriptors_2_1, matches1); //Front
	matcher->match(descriptors_1_2, descriptors_2_2, matches2); //Back

	double min_dist1 = 10000, max_dist1 = 0;	//Front
	double min_dist2 = 10000, max_dist2 = 0;	//Back

	for (int i = 0; i < descriptors_1_1.rows; i++)	//Front
	{
		double dist = matches1[i].distance;
		if (dist < min_dist1) min_dist1 = dist;
		if (dist > max_dist1) max_dist1 = dist;
	}

	for (int i = 0; i < descriptors_1_2.rows; i++)	//Back
	{
		double dist = matches2[i].distance;
		if (dist < min_dist2) min_dist2 = dist;
		if (dist > max_dist2) max_dist2 = dist;
	}

	// Optimize the match
	std::vector< DMatch > good_matches1;				//Front
	for (int i = 0; i < descriptors_1_1.rows; i++)
	{
		if (matches1[i].distance <= max(1.5 * min_dist1, 25.0))
		{
			good_matches1.push_back(matches1[i]);
		}
	}

	std::vector< DMatch > good_matches2;				//Back
	for (int i = 0; i < descriptors_1_2.rows; i++)
	{
		if (matches2[i].distance <= max(1.5 * min_dist2, 25.0))
		{
			good_matches2.push_back(matches2[i]);
		}
	}

	Mat img_match1, img_match2;
	Mat img_goodmatch1, img_goodmatch2;
	drawMatches(img1_1, keypoints_1_1, img2_1, keypoints_2_1, matches1, img_match1);			//Front
	drawMatches(img1_1, keypoints_1_1, img2_1, keypoints_2_1, good_matches1, img_goodmatch1);
	drawMatches(img1_2, keypoints_1_2, img2_2, keypoints_2_2, matches2, img_match2);			//Back
	drawMatches(img1_2, keypoints_1_2, img2_2, keypoints_2_2, good_matches2, img_goodmatch2);
	
	//namedWindow("Front_All match", WINDOW_NORMAL);
	//namedWindow("Front_good match", WINDOW_NORMAL);
	//namedWindow("Back_All match", WINDOW_NORMAL);
	//namedWindow("Back_good match", WINDOW_NORMAL);

	//imshow("Front_All match", img_match1);
	//imshow("Front_good match", img_goodmatch1);
	//imshow("Back_All match", img_match2);
	//imshow("Back_good match", img_goodmatch2);


	// extract keypoint according to the match result 
	std::vector<KeyPoint> keypoints_1_1_match, keypoints_2_1_match;		//Front //////////////////////////
	vector<Point2f> keypoints_1_1_match_2f((int)good_matches1.size()), keypoints_2_1_match_2f((int)good_matches1.size());
	for (int i = 0; i < (int)good_matches1.size(); i++)
	{
		keypoints_1_1_match.push_back(keypoints_1_1[good_matches1[i].queryIdx]);
		keypoints_2_1_match.push_back(keypoints_2_1[good_matches1[i].trainIdx]);
		keypoints_1_1_match_2f[i].x = keypoints_1_1_match[i].pt.x;
		keypoints_1_1_match_2f[i].y = keypoints_1_1_match[i].pt.y;
		keypoints_2_1_match_2f[i].x = keypoints_2_1_match[i].pt.x;
		keypoints_2_1_match_2f[i].y = keypoints_2_1_match[i].pt.y;
	}

	std::vector<KeyPoint> keypoints_1_2_match, keypoints_2_2_match;		//Back
	vector<Point2f> keypoints_1_2_match_2f((int)good_matches2.size()), keypoints_2_2_match_2f((int)good_matches2.size());
	for (int i = 0; i < (int)good_matches2.size(); i++)
	{
		keypoints_1_2_match.push_back(keypoints_1_2[good_matches2[i].queryIdx]);
		keypoints_2_2_match.push_back(keypoints_2_2[good_matches2[i].trainIdx]);
		keypoints_1_2_match_2f[i].x = keypoints_1_2_match[i].pt.x;
		keypoints_1_2_match_2f[i].y = keypoints_1_2_match[i].pt.y;
		keypoints_2_2_match_2f[i].x = keypoints_2_2_match[i].pt.x;
		keypoints_2_2_match_2f[i].y = keypoints_2_2_match[i].pt.y;
	}

	drawKeypoints(img1_1, keypoints_1_1_match, img_match1,Scalar(255,255,0));
	imshow("Front_good match", img_match1);


	// coordinate transformation(For Back only)
	// Define 
	Mat R_B = Mat::ones(3, 3, CV_64FC1);
	R_B.at<double>(0, 0) = -1;
	R_B.at<double>(0, 1) = 0;
	R_B.at<double>(0, 2) = 0;
	R_B.at<double>(1, 0) = 0;
	R_B.at<double>(1, 1) = 1;
	R_B.at<double>(1, 2) = 0;
	R_B.at<double>(2, 0) = 0;
	R_B.at<double>(2, 1) = 0;
	R_B.at<double>(2, 2) = -1;
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
	Mat H_B = K * R_B*K.inv();

	Mat temp_1 = Mat::ones(3, 1, CV_32FC1);
	Mat temp_2 = Mat::ones(3, 1, CV_32FC1);

	for (int i = 0; i < (int)good_matches2.size(); i++) {
		temp_1.at<float>(0, 0) = keypoints_1_2_match_2f[i].x;
		temp_1.at<float>(0, 1) = keypoints_1_2_match_2f[i].y;
		temp_1.at<float>(0, 2) = 1.0;
		temp_1.convertTo(temp_1, CV_64FC1);
		//temp_1 = H_B.inv()*temp_1;
		temp_1.convertTo(temp_1, CV_32FC1);
		keypoints_1_2_match_2f[i].x = temp_1.at<float>(0, 0);            //Teacher's version
		keypoints_1_2_match_2f[i].y = 540.0 - temp_1.at<float>(0, 1);    //Teacher's version
		//keypoints_1_2_match_2f[i].x = 540.0-temp_1.at<float>(0, 0);        //My version
		//keypoints_1_2_match_2f[i].y = temp_1.at<float>(0, 1);              //My version         
		//cout << keypoints_1_2_match_2f[i] << endl;

		temp_2.at<float>(0, 0) = keypoints_2_2_match_2f[i].x;
		temp_2.at<float>(0, 1) = keypoints_2_2_match_2f[i].y;
		temp_2.at<float>(0, 2) = 1.0;
		temp_2.convertTo(temp_2, CV_64FC1);
		//temp_2 = H_B.inv()*temp_2;
		temp_2.convertTo(temp_2, CV_32FC1);
		keypoints_2_2_match_2f[i].x = temp_2.at<float>(0, 0);            //Teacher's version
		keypoints_2_2_match_2f[i].y = 540.0 - temp_2.at<float>(0, 1);    //Teacher's version
		//keypoints_2_2_match_2f[i].x = 540-temp_2.at<float>(0, 0);          //My version
		//keypoints_2_2_match_2f[i].y = temp_2.at<float>(0, 1);              //My version
		//cout << keypoints_2_2_match_2f[i] << endl;
	}
	// Exchange first and sencond image keypoints(For Back only)
	
	vector<Point2f> keypoints_1_match_2f_temp((int)good_matches2.size()), keypoints_2_match_2f_temp((int)good_matches2.size());
	keypoints_1_match_2f_temp = keypoints_1_2_match_2f;
	keypoints_2_match_2f_temp = keypoints_2_2_match_2f;
	keypoints_1_2_match_2f = keypoints_2_match_2f_temp;
	keypoints_2_2_match_2f = keypoints_1_match_2f_temp;
	



	// Combine front and back keypoints
	vector<Point2f> points1, points2;
	points1.assign(keypoints_1_1_match_2f.begin(), keypoints_1_1_match_2f.end());	//Front
	points2.assign(keypoints_2_1_match_2f.begin(), keypoints_2_1_match_2f.end());


	//for (int i = 0; i < (int)good_matches2.size(); i++) {				//Back
	//	points1.push_back(keypoints_1_2_match_2f[i]);
	//	points2.push_back(keypoints_2_2_match_2f[i]);
	//}
	
//==================================================g2o====================================================================//
	Mat pose_g2o = Mat::eye(4,4,CV_64FC1);
	pose_g2o = bundleadjustment(points1,points2,pose_g2o);
	cout<<"pose_g2o = "<<pose_g2o<<endl;
	Mat R(3,3,CV_64F), R_f, t(3,1,CV_64F), t_f, mask;
	for(int i = 0; i<3;i++)   
		t.at<double>(i,0) = pose_g2o.at<double>(i,3);
	for(int i = 0; i<3;i++)
		for(int j = 0; j<3; j++)
			R.at<double>(j,i) = pose_g2o.at<double>(j,i);
	cout<<"t = "<<t<<endl;
	cout<<"R = "<<R<<endl;
//==============================================================================================================================//
	R_f = R.clone();
	t_f = t.clone();

	//Clear vector
	keypoints_1_1.clear();
	keypoints_1_2.clear();
	keypoints_2_1.clear();
	keypoints_2_2.clear();
	keypoints_1_1_match.clear();
	keypoints_1_2_match.clear();
	keypoints_2_1_match.clear();
	keypoints_2_2_match.clear();
	keypoints_1_1_match_2f.clear();
	keypoints_1_2_match_2f.clear();
	keypoints_2_1_match_2f.clear();
	keypoints_2_2_match_2f.clear();
	//keypoints_1_match_2f_temp.clear();
	//keypoints_2_match_2f_temp.clear();

	matches1.clear();
	matches2.clear();
	good_matches1.clear();
	good_matches2.clear();
	points1.clear();
	points2.clear();



	

	//==================================Loop Step=====================================//
	//MAP
	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	Mat traj2 = Mat::zeros(600, 600, CV_8UC3);
	Mat camera_pose = cv::Mat::eye(4,4,CV_64F);
	//Mappoint variable
	Mat X,X_pre;
	vector<Point2f> points2_pre;
	// Open image file
	//Mat img1_1, img1_2, img2_1, img2_2;
	//char filename1_1[200], filename1_2[200], filename2_1[200], filename2_2[200];
	for (int i = 538; i <= 2096; i++) {
		snprintf(filename1_1,sizeof(filename1_1), "EEFront/%05d.jpg", i);
		snprintf(filename2_1,sizeof(filename2_1), "EEFront/%05d.jpg", i + 1);
		snprintf(filename1_2,sizeof(filename1_2), "EEBack/%05d.jpg", i);
		snprintf(filename2_2,sizeof(filename2_2), "EEBack/%05d.jpg", i + 1);
		snprintf(filename1_3,sizeof(filename1_3), "EERight/%05d.jpg", i);
		snprintf(filename2_3,sizeof(filename2_3), "EERight/%05d.jpg", i + 1);
		snprintf(filename1_4,sizeof(filename1_4), "EELeft/%05d.jpg", i);
		snprintf(filename2_4,sizeof(filename2_4), "EELeft/%05d.jpg", i + 1);

		img1_1_c = imread(filename1_1);
		img1_2_c = imread(filename1_2);
		img1_3_c = imread(filename1_3);
		img1_4_c = imread(filename1_4);
		img2_1_c = imread(filename2_1);
		img2_2_c = imread(filename2_2);
		img2_3_c = imread(filename2_3);
		img2_4_c = imread(filename2_4);

		if (!img1_1_c.data || !img1_2_c.data || !img1_3_c.data || !img1_4_c.data || !img2_1_c.data || !img2_2_c.data || !img2_3_c.data || !img2_4_c.data) {
			std::cout << " --(!) Error reading images " << std::endl;
			return -1;
		}

		img1_1_c = convert(angle, img1_4_c, img1_1_c, img1_3_c);
		img1_2_c = convert(angle, img1_3_c, img1_2_c, img1_4_c);
		img2_1_c = convert(angle, img2_4_c, img2_1_c, img2_3_c);
		img2_2_c = convert(angle, img2_3_c, img2_2_c, img2_4_c);
		// we work with grayscale images
		cvtColor(img1_1_c, img1_1, COLOR_BGR2GRAY);
		cvtColor(img1_2_c, img1_2, COLOR_BGR2GRAY);
		cvtColor(img2_1_c, img2_1, COLOR_BGR2GRAY);
		cvtColor(img2_2_c, img2_2, COLOR_BGR2GRAY);

		// Extractor keypoint and descriptor
		detector->detect(img1_1, keypoints_1_1);
		detector->detect(img1_2, keypoints_1_2);
		detector->detect(img2_1, keypoints_2_1);
		detector->detect(img2_2, keypoints_2_2);
		descriptor->compute(img1_1, keypoints_1_1, descriptors_1_1);
		descriptor->compute(img1_2, keypoints_1_2, descriptors_1_2);
		descriptor->compute(img2_1, keypoints_2_1, descriptors_2_1);
		descriptor->compute(img2_2, keypoints_2_2, descriptors_2_2);

		matcher->match(descriptors_1_1, descriptors_2_1, matches1);	//Front
		matcher->match(descriptors_1_2, descriptors_2_2, matches2); 	//Back

		min_dist1 = 10000; max_dist1 = 0;	//Front
		min_dist2 = 10000; max_dist2 = 0;	//Back

		for (int i = 0; i < descriptors_1_1.rows; i++)	//Front
		{
			double dist = matches1[i].distance;
			if (dist < min_dist1) min_dist1 = dist;
			if (dist > max_dist1) max_dist1 = dist;
		}

		for (int i = 0; i < descriptors_1_2.rows; i++)	//Back
		{
			double dist = matches2[i].distance;
			if (dist < min_dist2) min_dist2 = dist;
			if (dist > max_dist2) max_dist2 = dist;
		}
		//cout << descriptors_1_2.rows << endl;
		// Optimize the match
		for (int i = 0; i < descriptors_1_1.rows; i++)
		{
			if (matches1[i].distance <= max(1.5 * min_dist1, 25.0))
			{
				good_matches1.push_back(matches1[i]);
			}
		}

		for (int i = 0; i < descriptors_1_2.rows; i++)
		{
			if (matches2[i].distance <= max(1.5 * min_dist2, 25.0))
			{
				good_matches2.push_back(matches2[i]);
			}
		}
		//drawMatches(img1_1, keypoints_1_1, img2_1, keypoints_2_1, matches1, img_match1);			//Front
		//drawMatches(img1_1, keypoints_1_1, img2_1, keypoints_2_1, good_matches1, img_goodmatch1);
		//drawMatches(img1_2, keypoints_1_2, img2_2, keypoints_2_2, matches2, img_match2);			//Back
		//drawMatches(img1_2, keypoints_1_2, img2_2, keypoints_2_2, good_matches2, img_goodmatch2);

		//namedWindow("Front_All match", WINDOW_NORMAL);
		//namedWindow("Front_good match", WINDOW_NORMAL);
		//namedWindow("Back_All match", WINDOW_NORMAL);
		//namedWindow("Back_good match", WINDOW_NORMAL);


		//imshow("Front_All match", img_match1);
		//imshow("Front_good match", img_goodmatch1);
		//imshow("Back_All match", img_match2);
		//imshow("Back_good match", img_goodmatch2);
		//Here!!!!
		// extract keypoint according to the match result 
		std::vector<KeyPoint> keypoints_1_1_match, keypoints_2_1_match;		//Front
		vector<Point2f> keypoints_1_1_match_2f((int)good_matches1.size()), keypoints_2_1_match_2f((int)good_matches1.size());
		for (int i = 0; i < (int)good_matches1.size(); i++)
		{
			keypoints_1_1_match.push_back(keypoints_1_1[good_matches1[i].queryIdx]);
			keypoints_2_1_match.push_back(keypoints_2_1[good_matches1[i].trainIdx]);
			keypoints_1_1_match_2f[i].x = keypoints_1_1_match[i].pt.x;
			keypoints_1_1_match_2f[i].y = keypoints_1_1_match[i].pt.y;
			keypoints_2_1_match_2f[i].x = keypoints_2_1_match[i].pt.x;
			keypoints_2_1_match_2f[i].y = keypoints_2_1_match[i].pt.y;
		}

		std::vector<KeyPoint> keypoints_1_2_match, keypoints_2_2_match;		//Back
		vector<Point2f> keypoints_1_2_match_2f((int)good_matches2.size()), keypoints_2_2_match_2f((int)good_matches2.size());
		for (int i = 0; i < (int)good_matches2.size(); i++)
		{
			keypoints_1_2_match.push_back(keypoints_1_2[good_matches2[i].queryIdx]);
			keypoints_2_2_match.push_back(keypoints_2_2[good_matches2[i].trainIdx]);
			keypoints_1_2_match_2f[i].x = keypoints_1_2_match[i].pt.x;
			keypoints_1_2_match_2f[i].y = keypoints_1_2_match[i].pt.y;
			keypoints_2_2_match_2f[i].x = keypoints_2_2_match[i].pt.x;
			keypoints_2_2_match_2f[i].y = keypoints_2_2_match[i].pt.y;
		}
		drawKeypoints(img1_1, keypoints_1_1_match, img_match1,Scalar(255,255,0));
		imshow("Front_good match", img_match1);

		// coordinate transformation(For Back only)
		//Define 
		R_B = Mat::ones(3, 3, CV_64FC1);
		R_B.at<double>(0, 0) = -1;
		R_B.at<double>(0, 1) = 0;
		R_B.at<double>(0, 2) = 0;
		R_B.at<double>(1, 0) = 0;
		R_B.at<double>(1, 1) = 1;
		R_B.at<double>(1, 2) = 0;
		R_B.at<double>(2, 0) = 0;
		R_B.at<double>(2, 1) = 0;
		R_B.at<double>(2, 2) = -1;
		K = Mat::ones(3, 3, CV_64FC1);
		K.at<double>(0, 0) = 270;
		K.at<double>(0, 1) = 0;
		K.at<double>(0, 2) = 270;
		K.at<double>(1, 0) = 0;
		K.at<double>(1, 1) = -270;
		K.at<double>(1, 2) = 270;
		K.at<double>(2, 0) = 0;
		K.at<double>(2, 1) = 0;
		K.at<double>(2, 2) = 1;
		H_B = K * R_B*K.inv();

		for (int i = 0; i < (int)good_matches2.size(); i++) {
			temp_1.at<float>(0, 0) = keypoints_1_2_match_2f[i].x;
			temp_1.at<float>(0, 1) = keypoints_1_2_match_2f[i].y;
			temp_1.at<float>(0, 2) = 1.0;
			temp_1.convertTo(temp_1, CV_64FC1);
			//temp_1 = H_B.inv()*temp_1;
			temp_1.convertTo(temp_1, CV_32FC1);
			keypoints_1_2_match_2f[i].x = temp_1.at<float>(0, 0);			//Teacher's version
			keypoints_1_2_match_2f[i].y = 540.0 - temp_1.at<float>(0, 1);         //Teacher's version
			//keypoints_1_2_match_2f[i].x = 540.0 - temp_1.at<float>(0, 0);           //My version
			//keypoints_1_2_match_2f[i].y = temp_1.at<float>(0, 1);                   //My version
			//cout << keypoints_1_2_match_2f[i] << endl;

			temp_2.at<float>(0, 0) = keypoints_2_2_match_2f[i].x;
			temp_2.at<float>(0, 1) = keypoints_2_2_match_2f[i].y;
			temp_2.at<float>(0, 2) = 1.0;
			temp_2.convertTo(temp_2, CV_64FC1);
			//temp_2 = H_B.inv()*temp_2;
			temp_2.convertTo(temp_2, CV_32FC1);
			keypoints_2_2_match_2f[i].x = temp_2.at<float>(0, 0);                 //Teacher's version
			keypoints_2_2_match_2f[i].y = 540.0 - temp_2.at<float>(0, 1);         //Teacher's version
			//keypoints_2_2_match_2f[i].x = 540 - temp_2.at<float>(0, 0);             //My version
			//keypoints_2_2_match_2f[i].y = temp_2.at<float>(0, 1);                   //My version
			//cout << keypoints_2_2_match_2f[i] << endl;
		}

		// Exchange first and sencond image keypoints(For Back only)

		vector<Point2f> keypoints_1_match_2f_temp((int)good_matches2.size()), keypoints_2_match_2f_temp((int)good_matches2.size());
		keypoints_1_match_2f_temp = keypoints_1_2_match_2f;
		keypoints_2_match_2f_temp = keypoints_2_2_match_2f;
		keypoints_1_2_match_2f = keypoints_2_match_2f_temp;
		keypoints_2_2_match_2f = keypoints_1_match_2f_temp;




		// Combine front and back keypoints
		points1.assign(keypoints_1_1_match_2f.begin(), keypoints_1_1_match_2f.end());				//Front
		points2.assign(keypoints_2_1_match_2f.begin(), keypoints_2_1_match_2f.end());


		//for (int i = 0; i < (int)good_matches2.size(); i++) {							//Back
		//	points1.push_back(keypoints_1_2_match_2f[i]);
		//	points2.push_back(keypoints_2_2_match_2f[i]);
		//}



//==================================================g2o====================================================================//
		//Mat pose_g2o = zero(4,4,CV_64FC1);
		pose_g2o = bundleadjustment(points1,points2,pose_g2o);
		cout<<"pose_g2o = "<<pose_g2o<<endl;
		for(int i = 0; i<3;i++)   //stuck at the code which transform the result from g2o to opencv.
			t.at<double>(i,0) = pose_g2o.at<double>(i,3);
		for(int i = 0; i<3;i++)
			for(int j = 0; j<3; j++)
				R.at<double>(j,i) = pose_g2o.at<double>(j,i);
		cout<<"t = "<<t<<endl;
		cout<<"R = "<<R<<endl;
//==============================================================================================================================//

		//Variable for map
		double scale = 100;
		char text[100];
		int fontFace = FONT_HERSHEY_PLAIN;
		double fontScale = 1;
		int thickness = 1;
		cv::Point textOrg(10, 50);


		//Construct map points
		X_pre = X;
		cv::Mat P0 = K*cv::Mat::eye(3,4,CV_64F);
		cv::Mat Rt;
		cv::hconcat(R,t,Rt);
		cv::Mat P1 = K*Rt;
		cv::triangulatePoints(P0,P1,points1,points2,X);
		X.row(0) = X.row(0)/X.row(3);
		X.row(1) = X.row(1)/X.row(3);
		X.row(2) = X.row(2)/X.row(3);
		X.row(3) = 1;
		//for(int c = 0;c<X.cols;c++)
			//cout<<"X:"<<X.at<double>(0,c)<<" Y:"<<X.at<double>(1,c)<<" Z:"<<X.at<double>(2,c)<<endl;
		cout<<"X.cols"<<X.cols<<endl;
		cout<<"points1.size()"<<points1.size()<<endl;
		
		Mat X_match1 = X_pre; //assign the type which X is.
		Mat X_match2 = X;
		int match_num = 0;
		//First, find the match map point

		if(!X_pre.data) //initial
		{
			scale = 1;
		}
		else
		{
			for(int j = 0;j<points2_pre.size();j++){
				for(int i = 0;i<points1.size();i++){
					if(sqrt(abs(points2_pre[j].x-points1[i].x)*abs(points2_pre[j].x-points1[i].x)+abs(points2_pre[j].y-points1[i].y)*abs(points2_pre[j].y-points1[i].y))<0.08)		
					{
						X_match1.at<double>(0,match_num) = X_pre.at<double>(0,j);
						X_match1.at<double>(1,match_num) = X_pre.at<double>(1,j);
						X_match1.at<double>(2,match_num) = X_pre.at<double>(2,j);
						X_match2.at<double>(0,match_num) = X.at<double>(0,i);
						X_match2.at<double>(1,match_num) = X.at<double>(1,i);
						X_match2.at<double>(2,match_num) = X.at<double>(2,i);
						match_num ++;
					}
		
				}
			}
	
			for(int k = 0;k<=match_num;k++)
			{
				cout<<"X_match1 X:"<<X_match1.at<double>(0,k)<<"X_match1 Y:"<<X_match1.at<double>(1,k)<<"X_match1 Z:"<<X_match1.at<double>(2,k)<<endl;
				cout<<"match_num: "<<match_num<<endl;
			}



		}
		




		//accumulate each pose
		//if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
			//Method_1
			//t_f = t_f + scale * (R_f*t);
			//R_f = R * R_f;
			//Method_2
			cv::Mat T = cv::Mat::eye(4,4,R.type());
			T(cv::Rect(0,0,3,3)) = R*1.0;
			T.col(3).rowRange(0,3) = t*1.0;
			camera_pose = camera_pose*T.inv();
			

		//}
			//cout<<"Camera_pose:"<<camera_pose<<endl;
			//cout<<"x = :"<<camera_pose.at<double>(0,3)*10<<endl;
			//cout<<"y = :"<<camera_pose.at<double>(2,3)*10<<endl;

		int x = int(camera_pose.at<double>(0,3)*100) + 300;
		int y = int(camera_pose.at<double>(2,3)*100) + 100;
		//int x = int(t_f.at<double>(0)) + 300;
		//int y = int(t_f.at<double>(2)) + 300;
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

		rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
		//snprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));//
		//putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
		imshow("Trajectory", traj);



		//Store pre points2 for mappoint matching
		points2_pre = points2;

		waitKey(1);

		//Clear vector
		keypoints_1_1.clear();
		keypoints_1_2.clear();
		keypoints_2_1.clear();
		keypoints_2_2.clear();
		keypoints_1_1_match.clear();
		keypoints_1_2_match.clear();
		keypoints_2_1_match.clear();
		keypoints_2_2_match.clear();
		keypoints_1_1_match_2f.clear();
		keypoints_1_2_match_2f.clear();
		keypoints_2_1_match_2f.clear();
		keypoints_2_2_match_2f.clear();
		//keypoints_1_match_2f_temp.clear();
		//keypoints_2_match_2f_temp.clear();

		matches1.clear();
		matches2.clear();
		good_matches1.clear();
		good_matches2.clear();
		points1.clear();
		points2.clear();


	}
	return 0;
}

Mat convert(int angle, Mat imgL, Mat imgF, Mat imgR)
{
	double angle_rad = angle * PI / 180;
	int L = imgF.rows;
	int ext_init = L / 2 + 1;
	int ext_end = ceil((L / 2)*tan(angle_rad / 2));

	int counter = 0;
	int x_mapR = 0, x_mapL = 0;
	int y_map = 0;
	double HR = 0, HL = 0;

	Mat img;
	Mat imgR_map = Mat::zeros(L, ext_end - ext_init + 1, CV_8UC3);
	Mat imgL_map = Mat::zeros(L, ext_end - ext_init + 1, CV_8UC3);
	for (x_mapR = ext_init; x_mapR <= ext_end; x_mapR++)
	{
		for (y_map = 1; y_map <= (L / 2); y_map++)
		{
			HR = pow((L / 2), 2) / x_mapR;
			x_mapL = ext_init + ext_end - x_mapR;
			HL = pow((L / 2), 2) / x_mapL;
			for (int ch = 0; ch < 3; ch++)
			{
				imgR_map.at<Vec3b>(L / 2 - y_map, counter)[ch] = imgR.at<Vec3b>(L / 2 - ceil((L*y_map) / (2 * x_mapR)), ceil(L / 2 - HR) - 1)[ch];
				imgR_map.at<Vec3b>(L / 2 + y_map - 1, counter)[ch] = imgR.at<Vec3b>(L / 2 + ceil((L*y_map) / (2 * x_mapR)) - 1, ceil(L / 2 - HR) - 1)[ch];
				imgL_map.at<Vec3b>(L / 2 - y_map, counter)[ch] = imgL.at<Vec3b>(L / 2 - ceil((L*y_map) / (2 * x_mapL)), ceil(L / 2 + HL) - 1)[ch];
				imgL_map.at<Vec3b>(L / 2 + y_map - 1, counter)[ch] = imgL.at<Vec3b>(L / 2 + ceil((L*y_map) / (2 * x_mapL)) - 1, ceil(L / 2 + HL) - 1)[ch];
			}
		}
		counter += 1;
	}
	hconcat(imgL_map, imgF, img);
	hconcat(img, imgR_map, img);

	return img;
}
Mat bundleadjustment(vector<Point2f> points1,vector<Point2f> points2,Mat cv_pose_pre)
{
	switch (angle)
	{
	case 90:
		cx = 270;
		break;
	case 100:
		cx = 322;
		break;
	case 110:
		cx = 386;
		break;
	case 120:
		cx = 468;
		break;
	case 130:
		cx = 580;
		break;
	case 140:
		cx = 742;
		break;
	default:
		cout << "illegal angle" << endl;
	}
	// 构造g2o中的图
	// 先构造求解器
	g2o::SparseOptimizer    optimizer;
	// 使用Cholmod中的线性方程求解器  
	g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
	// 6*3 的参数  
	g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));
	// L-M 下降   
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3>(block_solver));

	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);


 	// 添加节点
      	// 两个位姿节点
      	for ( int i=0; i<2; i++ )
      	{
          	g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
          	v->setId(i);
          	if ( i == 0)
              		v->setFixed( true ); // 第一个点固定为零
          	// 预设值为单位Pose，因为我们不知道任何信息
          	v->setEstimate( g2o::SE3Quat() );
          	optimizer.addVertex( v );
     	 }


  	// 很多个特征点的节点
      	// 以第一帧为准
      	for ( size_t i=0; i<points1.size(); i++ )
      	{
          	g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
          	v->setId( 2 + i );
          	// 由于深度不知道，只能把深度设置为1了
          	double z = 1;
          	double x = ( points1[i].x - cx ) * z / fx; 
          	double y = ( points1[i].y - cy ) * z / fy; 
          	v->setMarginalized(true);
          	v->setEstimate( Eigen::Vector3d(x,y,z) );
          	optimizer.addVertex( v );
     	}


  	// 准备相机参数
     	g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
     	camera->setId(0);
     	optimizer.addParameter( camera );
     
     	// 准备边
     	// 第一帧
     	vector<g2o::EdgeProjectXYZ2UV*> edges;
     	for ( size_t i=0; i<points1.size(); i++ )
     	{
         	g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
         	edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
         	edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
         	edge->setMeasurement( Eigen::Vector2d(points1[i].x, points1[i].y ) );
         	edge->setInformation( Eigen::Matrix2d::Identity() );
         	edge->setParameterId(0, 0);
         	// 核函数
         	edge->setRobustKernel( new g2o::RobustKernelHuber() );
         	optimizer.addEdge( edge );
         	edges.push_back(edge);
     	}

     	// 第二帧
     	for ( size_t i=0; i<points2.size(); i++ )
     	{
         	g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
         	edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
         	edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
         	edge->setMeasurement( Eigen::Vector2d(points2[i].x, points2[i].y ) );
         	edge->setInformation( Eigen::Matrix2d::Identity() );
         	edge->setParameterId(0,0);
         	// 核函数
         	edge->setRobustKernel( new g2o::RobustKernelHuber() );
         	optimizer.addEdge( edge );
         	edges.push_back(edge);
     	}

  	cout<<"开始优化"<<endl;
     	optimizer.setVerbose(true);
     	optimizer.initializeOptimization();
     	optimizer.optimize(10);
     	cout<<"优化完毕"<<endl;
     
     	//我们比较关心两帧之间的变换矩阵
     	g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
     	Eigen::Isometry3d pose = v->estimate();
     	//cout<<"Pose="<<endl<<pose.matrix()<<endl;

	// 估计inlier的个数
	float inliers = 0;
	float count   = 0;
	for (vector<g2o::EdgeProjectXYZ2UV*>::iterator e = edges.begin(); e != edges.end(); ++e)
	{
		(*e)->computeError();
		// chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
		if ((*e)->chi2() > 1)
		{
			//cout << "error = " << (*e)->chi2() << endl;
		}
		else
		{
			inliers++;
		}
		count++;
	}
	

	Mat cv_pose;

	cv::eigen2cv(pose.matrix(),cv_pose);
	if((inliers*100)/count >92){
		cout<<"inliers: "<<inliers<<"("<<(inliers*100)/count<<"%)"<<endl;
		return	cv_pose; 
	}
	else{
		cout<<"inliers: "<<inliers<<"("<<(inliers*100)/count<<"%)"<<endl;
		return cv_pose_pre;
	}

}

