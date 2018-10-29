//
// Created by da on 18-8-12.
//

#ifndef SLAM_ONE_CAM_H
#define SLAM_ONE_CAM_H
#include "slam_g2o/common.h"

namespace slam_g2o
{
    class Camera_kitti
    {
        //typedef shared_ptr<Camera> Ptr;
    public:
        typedef shared_ptr<Camera_kitti> Ptr;
        float fx_,fy_,cx_,cy_;
        double baseline_;
    public:
        Camera_kitti();

        Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
        Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
        Vector2d camera2pixel( const Vector3d& p_c );
        Vector3d pixel2camera( const Vector2d& p_p,double depth = 1 );
        Vector3d pixel2world( const Vector2d& p_p, const SE3& T_c_w, double depth = 1 );
        Vector2d world2pixel( const Vector3d& p_w, const SE3& T_c_w );
    };
}
#endif //SLAM_ONE_CAM_H
