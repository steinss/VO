//
// Created by da on 18-4-19.
//

#ifndef SLAM_ONE_COMMON_H
#define SLAM_ONE_COMMON_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using cv::Mat;

#include <sophus/so3.h>
#include <sophus/se3.h>
using Sophus::SE3;

#include <Eigen/Core>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
using namespace std;

void read_fold(string path,vector<string>& img_namelist);

void pose_estimation_3d3d (
        const vector<cv::Point3f>& pts1,
        const vector<cv::Point3f>& pts2,
        Mat& R, Mat& t
);
#endif //SLAM_ONE_COMMON_H
