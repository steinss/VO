//
// Created by da on 18-4-20.
//
#include "slam_g2o/config.h"

namespace slam_g2o
{
    Config::~Config() {
        if( file_.isOpened() ) file_.release();
    }
    shared_ptr<Config> Config::config_ = nullptr;

    void Config::setParameter(const string &filename) {
        if( config_ == nullptr ) config_ = shared_ptr<Config> (new Config);
        config_->file_ = cv::FileStorage( filename.c_str(),cv::FileStorage::READ );
        if( config_->file_.isOpened()== false ){
            cerr<< "The file "<<filename<< "is not existed"<<endl;
            config_->file_.release();
            return;
        }
    }


}

