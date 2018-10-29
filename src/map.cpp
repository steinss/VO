//
// Created by da on 18-4-26.
//

#include "slam_g2o/map.h"
namespace slam_g2o
{
    void Map::insertKeyFrame(Frame_kitti::Ptr frame_map)
    {
        if( key_frames_.find(frame_map->id_) == key_frames_.end() )
        {
            key_frames_.insert( make_pair(frame_map->id_,frame_map) );
        }
        else
        {
            key_frames_[frame_map->id_] = frame_map;
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr point_map)
    {
        if( map_points_.find(point_map->id_) == map_points_.end() )
        {
            map_points_.insert( make_pair(point_map->id_,point_map));
        }
        else
        {
            map_points_[point_map->id_] = point_map;
        }
    }
}