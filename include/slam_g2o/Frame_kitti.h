//
// Created by da on 18-8-12.
//

#ifndef SLAM_ONE_FRAME_KITTI_H
#define SLAM_ONE_FRAME_KITTI_H
#include "slam_g2o/common.h"
#include "slam_g2o/camera_kitti.h"

namespace slam_g2o
{
    class Frame_kitti
    {
    public:
        typedef shared_ptr<Frame_kitti>     Ptr;
        unsigned long                       id_;
        double                              time_stamp_;
        Mat                                 left_,right_;
        SE3                                 T_c_w;
        Camera_kitti::Ptr                   camera_;
        bool                                isKeyFrame;

    public:
        Frame_kitti();
        Frame_kitti(long id, double time_stamp=0 , Mat left=Mat(), Mat right = Mat(), SE3 T_cw=SE3(),Camera_kitti::Ptr camera = nullptr );
        ~Frame_kitti();
        double getDepth( const cv::KeyPoint& keypoint, const cv::Point2f& point_right,double baseline );
        static Frame_kitti::Ptr createFrame ();
        bool isInFrame(const Vector3d& p_w);
    };
}
#endif //SLAM_ONE_FRAME_KITTI_H
