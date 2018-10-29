//
// Created by da on 18-4-26.
//
#include "slam_g2o/mappoint.h"
namespace slam_g2o
{
    unsigned long MapPoint::factory_id_ = 0;

    MapPoint::MapPoint()
    : id_(-1),pose_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)),observed_times_(0),matched_times_(0)
    ,isGoodPoint_(true){}

    MapPoint::MapPoint(unsigned long id, const Vector3d &position, const Vector3d &norm, Frame_kitti *frame,
                       const Mat &descriptor)
    : id_(id), pose_(position), norm_(norm),descriptor_(descriptor),observed_times_(1),
    matched_times_(1)
    {
        observed_frames_.push_back(frame);
    }

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        return MapPoint::Ptr (new
        MapPoint( factory_id_++,Vector3d(0,0,0),Vector3d(0,0,0)));
    }

    MapPoint::Ptr MapPoint::createMapPoint(const Vector3d &position, const Vector3d &norm,
                                           Frame_kitti *frame, const Mat &descriptor)
    {
        return MapPoint::Ptr
                (new MapPoint(factory_id_++,position,norm,frame,descriptor));
    }
}
