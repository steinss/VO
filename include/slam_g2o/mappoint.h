//
// Created by da on 18-4-20.
//

#ifndef SLAM_ONE_MAPPOINT_H
#define SLAM_ONE_MAPPOINT_H

#include "slam_g2o/common.h"
#include "slam_g2o/Frame_kitti.h"
namespace slam_g2o
{
    class MapPoint
    {
    public:
        typedef shared_ptr<MapPoint>    Ptr;
        static unsigned long            factory_id_;
        unsigned long                   id_;
        Vector3d                        pose_;
        Vector3d                        norm_;
        Mat                             descriptor_;
        bool                            isGoodPoint_;
        list<Frame_kitti*>              observed_frames_;
        int                             matched_times_;
        int                             observed_times_;

    public:
        MapPoint();
        MapPoint(unsigned long id, const Vector3d& position,
                 const Vector3d& norm, Frame_kitti* frame = nullptr,
                 const Mat& descriptor = Mat()
        );
        static MapPoint::Ptr createMapPoint();

        static MapPoint::Ptr createMapPoint(
                 const Vector3d& position,
                const Vector3d& norm, Frame_kitti* frame,
                const Mat& descriptor
        );
        inline cv::Point3f getPoint()
        {
            return cv::Point3f (
                    pose_(0,0),pose_(1,0),pose_(2,0)
            );
        }


    };
}
#endif //SLAM_ONE_MAPPOINT_H
