//
// Created by da on 18-4-20.
//

#ifndef SLAM_ONE_CONFIG_H
#define SLAM_ONE_CONFIG_H

#include "slam_g2o/common.h"

namespace slam_g2o
{
    class Config
    {
    public:
        static shared_ptr<Config> config_;
        Config(){}
        cv::FileStorage file_;

    public:
        ~Config();
        static void setParameter( const string& filename );

        template <typename T>
                static T getValue(const string& key )
        {
            return T( Config::config_->file_[key] );
        }




    };
}



#endif //SLAM_ONE_CONFIG_H
