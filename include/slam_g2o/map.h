//
// Created by da on 18-4-26.
//

#ifndef SLAM_ONE_MAP_H
#define SLAM_ONE_MAP_H

#include "slam_g2o/common.h"
#include "slam_g2o/mappoint.h"
#include "slam_g2o/Frame_kitti.h"
namespace slam_g2o
{
    class Map
    {
    public:
        typedef                                         shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr>     map_points_;
        unordered_map<unsigned long, Frame_kitti::Ptr>  key_frames_;

    public:
        Map(){}
        void insertMapPoint( MapPoint::Ptr point_map );
        void insertKeyFrame( Frame_kitti::Ptr frame_map );

    };
}

#endif //SLAM_ONE_MAP_H
