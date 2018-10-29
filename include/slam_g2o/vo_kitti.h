//
// Created by da on 18-8-12.
//

#ifndef SLAM_ONE_VO_KITTI_H
#define SLAM_ONE_VO_KITTI_H
#include "slam_g2o/config.h"
#include "slam_g2o/Frame_kitti.h"
#include "slam_g2o/camera_kitti.h"
#include "slam_g2o/common.h"
#include "slam_g2o/map.h"
#include "slam_g2o/mappoint.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <queue>

namespace slam_g2o
{
    class VO_kitti
    {
    public:
        //parameter
        enum VOstate{
            INITIALIZING = -1,
            OK = 0,
            LOST
        };
        VOstate                             state_;
        typedef shared_ptr<VO_kitti>        Ptr;
        double                              baseline_;
        Frame_kitti::Ptr                    frame_curr_;
        Frame_kitti::Ptr                    frame_ref_;
        vector<cv::KeyPoint>                keypoints_;
        vector<cv::Point2f>                 points_right;
        vector<cv::DMatch>                  match_curr_;
        Mat                                 descriptor_curr_;
        Mat                                 descriptor_ref;
        Mat                                 inliers_points;
        vector<cv::Point3f>                 pts_ref_;
        Camera_kitti::Ptr                   camera_;
        cv::Ptr<cv::ORB>                    orb_;
        SE3                                 T_c_r_estimate;
        cv::FlannBasedMatcher               matcher_flann_;
        //other parameter
        int                                 num_of_features_ ;
        double                              scale_factor_;
        int                                 level_pyramid_;
        float                               match_ratio_;
        float                               max_num_lost_;
        int                                 min_inliers_;
        double                              key_frame_min_rot;
        double                              key_frame_min_trans;
        ///////////inlier
        int                                 num_inliers;
        int                                 num_lost;
        //
        Map::Ptr                            map_;
        vector<MapPoint::Ptr>               map_point_vector;
        double                              map_point_erase_ratio_;
        vector<int>                         mappoint_index_num_;
        bool                                largerot_state;
        queue<bool>                         rot_point_insert;
    public:
        //function
        VO_kitti( double baseline );
        ~VO_kitti(){};

        bool addFrame( Frame_kitti::Ptr frame);

    protected:
        //inner function
        void extractFeature();
        void computeOrb();
        void featureMatching();
        void setRef3Dpoint();
        void poseEstimatePnp();
        bool checkEstimatedPose();
        //
        void optimizeMap();
        void addKeyFrame();
        bool isKeyFrame();
        void addMapPoint();
        double getAngle( Frame_kitti::Ptr frame,MapPoint::Ptr mappoint );
        void poseEstimateICP();
        void triangulateThePoints();
        bool tri_ornot();
        bool largerot();


    };
}

#endif //SLAM_ONE_VO_KITTI_H
