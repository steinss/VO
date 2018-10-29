//
// Created by da on 18-8-12.
//
#include "slam_g2o/Frame_kitti.h"
namespace slam_g2o{
    Frame_kitti::Frame_kitti()
            :id_(-1),time_stamp_(-1),camera_(nullptr){}

    Frame_kitti::Frame_kitti(long id,  double time_stamp, Mat left, Mat right, SE3 T_cw, Camera_kitti::Ptr camera)
            :id_(id),time_stamp_(time_stamp),left_(left),right_(right),T_c_w(T_cw),camera_(camera),isKeyFrame(false)
    {}

    Frame_kitti::~Frame_kitti() {}

    double Frame_kitti::getDepth(const cv::KeyPoint &keypoint, const cv::Point2f& point_right, double baseline )
    {
        double x = keypoint.pt.x;
        double x2 = point_right.x;
        double depth = camera_->fx_*baseline/(x-x2);
        return depth;
    }

    bool Frame_kitti::isInFrame(const Vector3d &p_w)
    {
        Vector2d p_p = camera_->world2pixel(p_w,T_c_w);
        double d = camera_->fx_*camera_->baseline_/p_w.z();
        return p_p(0,0)>0 &&
               p_p(0,0)<left_.cols &&
               p_p(1,0)>0 &&
               p_p(1,0)<left_.rows &&
               (p_p(0,0)-d)>0;
    }

    Frame_kitti::Ptr Frame_kitti::createFrame() {
        static long frame_id = 0;
        return Frame_kitti::Ptr ( new Frame_kitti(frame_id++) );
    }


}
