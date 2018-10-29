//
// Created by da on 18-8-12.
//

#include "slam_g2o/vo_kitti.h"
#include "slam_g2o/bundle_adjustment.h"
#include <opencv2/opencv.hpp>

namespace slam_g2o
{
    VO_kitti::VO_kitti(double baseline)
            : state_(INITIALIZING),map_(new Map),baseline_(baseline), frame_ref_(nullptr),frame_curr_(nullptr),
              camera_(nullptr),num_lost(0),num_inliers(0),matcher_flann_( new cv::flann::LshIndexParams(5,10,2) )
    {
        num_of_features_    = 1500;
        scale_factor_       = 1.2;
        level_pyramid_      = 4;
        match_ratio_        = 2;
        max_num_lost_       = 10;
        min_inliers_        = 8;
        key_frame_min_rot   = 0.1;
        key_frame_min_trans = 8;
        map_point_erase_ratio_ = 0.1;
        orb_ = cv::ORB::create( num_of_features_,scale_factor_,level_pyramid_ );
    }

    bool VO_kitti::addFrame(Frame_kitti::Ptr pFrame)
    {
        switch (state_)
        {
            case INITIALIZING:
            {
                largerot_state = false;
                state_ = OK;
                frame_ref_ = frame_curr_=pFrame;
                extractFeature();
                computeOrb();

                addKeyFrame();
                cout<<"ok"<<endl;

                break;
            }
            case OK:
            {
                frame_curr_ = pFrame;
                frame_curr_->T_c_w = frame_ref_->T_c_w;
                extractFeature();
                computeOrb();
                featureMatching();
                poseEstimatePnp();

                if(checkEstimatedPose() == true)
                {
                    frame_curr_->T_c_w = T_c_r_estimate;
//                    if(tri_ornot()){
//                        triangulateThePoints();
//                    }
                    optimizeMap();
                    //addMapPoint();
                    num_lost = 0;
                    if(isKeyFrame())
                    {
                        addKeyFrame();
                        frame_ref_ = frame_curr_;
                        //addMapPoint();
                    }
                    cout<<"The map size is "<<map_->map_points_.size()<<endl;
                }
                else
                {
                    num_lost++;
                    if ( num_lost>max_num_lost_)
                        state_ = LOST;
                    return false;

                }
                break;
            }
            case LOST:
            {
                cout<<"the vo is lost"<<endl;
                break;
            }
        }
        return true;
    }

    void VO_kitti::extractFeature(){
        keypoints_.clear();
        points_right.clear();
        orb_->detect( frame_curr_->left_,keypoints_ );

        vector<cv::Point2f> pt1, pt2;
        for (auto &kp: keypoints_) pt1.push_back(kp.pt);
        vector<uchar> status;
        vector<float> error;
        cv::calcOpticalFlowPyrLK(frame_curr_->left_,frame_curr_->right_, pt1, pt2, status, error, cv::Size(16,16));
        //points_right = pt2;
        int i = 0;
        for(auto iter = keypoints_.begin();iter!=keypoints_.end();)
        {
            if( !status[i] || (abs(pt1[i].y-pt2[i].y)/pt1[i].y)>0.01 )
            {
                iter = keypoints_.erase(iter);
                i++;
                continue;
            }
            points_right.push_back(pt2[i]);
            i++;
            iter++;
        }
//        keypoints_.clear();
//        points_right.clear();
//        descriptor_curr_ = Mat();
//        vector<cv::KeyPoint> keypoint_left,keypoint_right;
//        Mat des_left,des_right;
//        orb_->detect( frame_curr_->left_,keypoint_left );
//        orb_->compute(frame_curr_->left_,keypoint_left, des_left);
//        orb_->detect( frame_curr_->right_,keypoint_right );
//        orb_->compute(frame_curr_->right_,keypoint_right, des_right);
//
//        vector<cv::DMatch> match_tmp_;
//        matcher_flann_.match( des_left, des_right ,match_tmp_);
//        float min_dist = 10000;
//        for(int i=0;i<des_left.rows;i++)
//        {
//            if( match_tmp_[i].distance<min_dist ) min_dist=match_tmp_[i].distance;
//        }
//        for(auto m: match_tmp_)
//        {
//            float dist = m.distance;
//            if( dist < max<float> (min_dist*2.0,45.0) && (abs(keypoint_left[m.queryIdx].pt.y-keypoint_right[m.trainIdx].pt.y)/keypoint_left[m.queryIdx].pt.y)< 0.01  )
//            {
//                keypoints_.push_back(keypoint_left[m.queryIdx]);
//                points_right.push_back(keypoint_right[m.trainIdx].pt);
//                descriptor_curr_.push_back(des_left.row(m.queryIdx).clone());
//            }
//        }
    }

    void VO_kitti::computeOrb() {
        orb_->compute(frame_curr_->left_,keypoints_,descriptor_curr_);
    }

    void VO_kitti::featureMatching()
    {
        //cv::BFMatcher matcher(cv::NORM_HAMMING);
        vector<cv::DMatch> match_tmp_;
        vector<MapPoint::Ptr> mappoint_tmp;
        pts_ref_.clear();
        descriptor_ref = Mat();
        map_point_vector.clear();
        mappoint_index_num_.clear();

        for(auto& mappoints_inf:map_->map_points_ )
        {
            MapPoint::Ptr& point_ptr = mappoints_inf.second;
            if( frame_curr_->isInFrame(point_ptr->pose_))
            {
                point_ptr->observed_times_++;
                pts_ref_.push_back(point_ptr->getPoint());
                mappoint_tmp.push_back(point_ptr);
                descriptor_ref.push_back(point_ptr->descriptor_);
            }

        }
        matcher_flann_.match( descriptor_ref,descriptor_curr_,match_tmp_);
        float min_dist = 10000;
        for(int i=0;i<descriptor_curr_.rows;i++)
        {
            if( match_tmp_[i].distance<min_dist ) min_dist=match_tmp_[i].distance;
        }
        match_curr_.clear();
        for(auto m: match_tmp_)
        {
            float dist = m.distance;
            if( dist < max<float> (min_dist*2,30.0) )
            {
                match_curr_.push_back(m);
                map_point_vector.push_back(mappoint_tmp[m.queryIdx]);
                mappoint_index_num_.push_back( m.trainIdx );
            }

        }
        cout<<"good matches: "<<match_curr_.size()<<endl;


    }



    void VO_kitti::poseEstimatePnp() {
        vector<cv::Point3f> pts_3d;
        vector<cv::Point2f> pts_2d;
        //pts2d
        for (auto m:match_curr_ )
        {
            pts_2d.push_back(keypoints_[m.trainIdx].pt);
            //pts_3d.push_back(pts_ref_[m.queryIdx]);
        }
        //pts3d
        for(auto mt: map_point_vector)
        {
            pts_3d.push_back( mt->getPoint());
            mt->matched_times_++;

        }


        Mat K, rvec, tvec,inliers;
        K = (cv::Mat_<double> (3,3)<<
                                   camera_->fx_,0,camera_->cx_,
                                    0,camera_->fy_,camera_->cy_,
                                    0,0,1
        );
        cv::solvePnPRansac(pts_3d,pts_2d,K,Mat(),rvec ,tvec, false ,100, 1 ,0.98 ,inliers,cv::SOLVEPNP_EPNP);
        num_inliers = inliers.rows;
        inliers_points = inliers;
        T_c_r_estimate = SE3( Sophus::SO3(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0))
                ,Vector3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0))
        );

//        //////////////////////////////////ba优化//////////////////////////////////////////////////////////
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr =  new Block(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm( solver );
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(g2o::SE3Quat(
                T_c_r_estimate.rotation_matrix(),T_c_r_estimate.translation()
        ));
        optimizer.addVertex(pose);
        //int index = 0;
        for (int i=0;i<inliers.rows;i++ )
        {
            int index = inliers.at<int> (i,0);
            EdgeXYZ2UV* edge = new EdgeXYZ2UV();
            edge->setId( i );
            edge->setVertex(0,pose);
            edge->point_ = Vector3d(pts_3d[index].x,pts_3d[index].y,pts_3d[index].z);
            edge->cameraa_ = frame_curr_->camera_.get();
            edge->setMeasurement(Vector2d(pts_2d[index].x,pts_2d[index].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
        }
        //optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(20);
        T_c_r_estimate = SE3(
                pose->estimate().rotation(),pose->estimate().translation()
        );
        /////////////////////////////////优化结束////////////////////////////////////////////////////////////
        /***********************************************************************************************/

        /****************************************ba优化******************************************/
//        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
//        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
//        Block* solver_ptr =  new Block(linearSolver);
//        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
//
//        g2o::SparseOptimizer optimizer;
//        optimizer.setAlgorithm( solver );
//        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
//        pose->setId(0);
//        pose->setEstimate(g2o::SE3Quat(
//                T_c_r_estimate.rotation_matrix(),T_c_r_estimate.translation()
//        ));
//        optimizer.addVertex(pose);
//        g2o::CameraParameters* camera1 = new g2o::CameraParameters(camera_->fx_,Eigen::Vector2d(camera_->cx_,camera_->cy_),0);
//        camera1->setId(0);
//        optimizer.addParameter(camera1);
//        int point_num = 1;
//        for (int i=0;i<inliers.rows;i++ )
//        {
//            int index = inliers.at<int> (i,0);
//            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
//            point->setId(point_num++);
//            point->setMarginalized(true);
//            point->setEstimate( Eigen::Vector3d(pts_3d[index].x,pts_3d[index].y,pts_3d[index].z));
//            optimizer.addVertex(point);
//        }
//        for (int i=0;i<inliers.rows;i++ )
//        {
//            int index = inliers.at<int> (i,0);
//            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
//            edge->setId( i+1 );
//            edge->setVertex(1,pose);
//            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+1)));
//            edge->setParameterId(0,0);
//            edge->setMeasurement(Vector2d(pts_2d[index].x,pts_2d[index].y));
//            edge->setInformation(Eigen::Matrix2d::Identity());
//            optimizer.addEdge(edge);
//        }
//        //optimizer.setVerbose(true);
//        optimizer.initializeOptimization();
//        optimizer.optimize(20);
//        T_c_r_estimate = SE3(
//                pose->estimate().rotation(),pose->estimate().translation()
//        );
//        for(int i=0;i<inliers.rows;i++)
//        {
//            int index = inliers.at<int> (i,0);
//            g2o::VertexSBAPointXYZ* point_te = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+1));
//            map_point_vector[index]->pose_ = point_te->estimate();
//        }
        ///////////////////////////////////优化结束////////////////////////////////////////////////////////////
    }

    bool VO_kitti::checkEstimatedPose()
    {
        if( num_inliers<min_inliers_)
        {
            cout<<"inliers is too small "<<num_inliers<<endl;
            return false;
        }
        SE3 ttmp = frame_ref_->T_c_w*T_c_r_estimate.inverse();
        Sophus::Vector6d d = ttmp.log();
        Vector3d d_trans = d.head<3>();
        Vector3d d_rot = d.tail<3>();
        if (d_rot.norm()>4*key_frame_min_rot||d_trans.norm()>5*key_frame_min_trans)
        {
            cout<<"motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }

    void VO_kitti::addKeyFrame()
    {
        if(map_->key_frames_.empty())
        {
            for( int i=0;i<keypoints_.size();i++)
            {
                double d = frame_curr_->getDepth(keypoints_[i],points_right[i],camera_->baseline_);
                if( d<0 ) continue;
                Vector3d pts_w = frame_curr_->camera_->pixel2world(
                        Vector2d(keypoints_[i].pt.x,keypoints_[i].pt.y),
                        frame_curr_->T_c_w, d
                );
                //Vector3d p_origin = frame_curr_->T_c_w.inverse().translation();
                Vector3d n = pts_w - frame_curr_->T_c_w.inverse().translation();
                n.normalize();
                MapPoint::Ptr map_poi = MapPoint::createMapPoint(pts_w,n,
                                                                 frame_curr_.get(),descriptor_curr_.row(i).clone());
                map_->insertMapPoint(map_poi);
            }
            //map_->map_points_
        }
        map_->insertKeyFrame( frame_curr_ );
    }

    void VO_kitti::optimizeMap()
    {
        for(auto iter=map_->map_points_.begin();iter!=map_->map_points_.end();)
        {
            if( !frame_curr_->isInFrame(iter->second->pose_))
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            float match_ratio = float(iter->second->matched_times_)/iter->second->observed_times_;
            if(match_ratio<map_point_erase_ratio_)
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }
            double angle = getAngle(frame_curr_,iter->second);
            if(angle > M_PI/6)
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }
            iter++;
        }
//        bool state_tmp = largerot_state;
//        rot_point_insert.push(state_tmp);
//        if(rot_point_insert.size()>2) rot_point_insert.pop();
//        bool insert_point_or_not = false;
//        queue<bool> qu_tmp = rot_point_insert;
//        while (!qu_tmp.empty())
//        {
//            bool w_tmp = qu_tmp.front();
//            if(w_tmp)
//            {
//                insert_point_or_not = true;
//                break;
//            }
//            qu_tmp.pop();
//        }
//
//        if(insert_point_or_not)
//        {
//            addMapPoint();
//        }
//        if ( largerot() )
//        {
//            //addMapPoint();
//            map_point_erase_ratio_+=0.1;
//            largerot_state = true;
//        }
//        else
//        {
//            largerot_state = false;
//        }
        if( map_point_vector.size()<500   )
        {
            addMapPoint();
        }
        if(map_->map_points_.size()>1500)
        {
            map_point_erase_ratio_+=0.05;
        } else
            map_point_erase_ratio_=0.1;
    }

    bool VO_kitti::isKeyFrame()
    {
        SE3 T_delta = frame_ref_->T_c_w * T_c_r_estimate.inverse();
        Sophus::Vector6d d = T_delta.log();
        Vector3d d_trans = d.head<3>();
        Vector3d d_rot = d.tail<3>();
        if (d_rot.norm()>key_frame_min_rot||d_trans.norm()>key_frame_min_trans)
            return true;
        return false;
    }

    void VO_kitti::addMapPoint()
    {
        vector<bool> matched_judge(keypoints_.size(), false);
        for(auto number : mappoint_index_num_)
            matched_judge[number] = true;
        for(int i=0;i<matched_judge.size();i++)
        {
            if(matched_judge[i])
            {
                continue;
            }
            double d= frame_curr_->getDepth( keypoints_[i],points_right[i],camera_->baseline_);
            if (d<0) continue;
            Vector3d p_w = camera_->pixel2world(
                    Vector2d( keypoints_[i].pt.x,keypoints_[i].pt.y ),
                    frame_curr_->T_c_w,d
            );
            Vector3d n = p_w - frame_curr_->T_c_w.inverse().translation();
            n.normalize();
            MapPoint::Ptr mapPoint_added = MapPoint::createMapPoint(p_w,n,frame_curr_.get(),descriptor_curr_.row(i).clone());
            map_->insertMapPoint(mapPoint_added);
        }
    }

    double VO_kitti::getAngle(Frame_kitti::Ptr frame, MapPoint::Ptr mappoint)
    {
        Vector3d n = mappoint->pose_-frame->T_c_w.inverse().translation();
        n.normalize();
        return acos(n.transpose()*mappoint->norm_);

    }

    void VO_kitti::poseEstimateICP() {
        vector<cv::Point3f> pts_3d;
        vector<cv::Point3f> pts_img;
        //pts2d
        for (auto m:match_curr_ )
        {
            int num = m.trainIdx;
            double d = frame_curr_->getDepth(keypoints_[num],points_right[num],baseline_);
            cv::Point3f p_tmp;
            p_tmp.x = keypoints_[num].pt.x;
            p_tmp.y = keypoints_[num].pt.y;
            p_tmp.z = d;
            pts_img.push_back(p_tmp);
            //pts_3d.push_back(pts_ref_[m.queryIdx]);
        }
        //pts3d
        for(auto mt: map_point_vector)
        {
            pts_3d.push_back( mt->getPoint());
            mt->matched_times_++;

        }


        Mat R,t ;
        pose_estimation_3d3d(pts_img,pts_3d,R,t);
        Eigen::Matrix3d R_tmp;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
                R_tmp(i,j) = R.at<double>(i,j);
        }
        Eigen::Vector3d t_tmp(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
        T_c_r_estimate = SE3( R_tmp,t_tmp );
        //////////////////////////////////ba优化//////////////////////////////////////////////////////////
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr =  new Block(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm( solver );
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(g2o::SE3Quat(
                T_c_r_estimate.rotation_matrix(),T_c_r_estimate.translation()
        ));
        optimizer.addVertex(pose);
        //int index = 0;
        for (int i=0;i<pts_img.size();i++ )
        {

            EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly();
            edge->setId( i );
            edge->setVertex(0,pose);
            edge->point_ = Vector3d(pts_3d[i].x,pts_3d[i].y,pts_3d[i].z);
            edge->setMeasurement(Vector3d(pts_img[i].x,pts_img[i].y,pts_img[i].z));
            edge->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addEdge(edge);
        }
        //optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        T_c_r_estimate = SE3(
                pose->estimate().rotation(),pose->estimate().translation()
        );

        ///////////////////////////////////优化结束////////////////////////////////////////////////////////////
    }

    void VO_kitti::triangulateThePoints(){
        vector<cv::Point2f> pts_2d;
        //pts2d
        for (auto m:match_curr_ )
        {
            pts_2d.push_back(keypoints_[m.trainIdx].pt);
        }
        Mat inliers = inliers_points;
        vector<cv::Point2f>pts1,pts2;
        for(int i=0;i<inliers.rows;i++)
        {
            int index = inliers.at<int> (i,0);
            Vector3d v_p = frame_ref_->T_c_w * map_point_vector[index]->pose_;
            cv::Point2f w_p;
            w_p.x = v_p(0,0);
            w_p.y = v_p(1,0);
//            w_p.x = map_point_vector[index]->pose_(0,0);
//            w_p.y = map_point_vector[index]->pose_(1,0);
            pts1.push_back(w_p);
        }
        for(int i=0;i<inliers.rows;i++)
        {
            int index = inliers.at<int> (i,0);
            cv::Point2f c_p;
            c_p.x = (pts_2d[index].x-camera_->cx_)/camera_->fx_;
            c_p.y = (pts_2d[index].y-camera_->cy_)/camera_->fy_;
            pts2.push_back(c_p);
        }
        SE3 t_tcw = frame_ref_->T_c_w;
        Mat T1 = (cv::Mat_<double>(3,4)<<
//                                       1,0,0,0,
//                                        0,1,0,0,
//                                        0,0,1,0);
                                       t_tcw.matrix()(0,0),t_tcw.matrix()(0,1),t_tcw.matrix()(0,2),t_tcw.matrix()(0,3),
                t_tcw.matrix()(0,0),t_tcw.matrix()(0,0),t_tcw.matrix()(1,2),t_tcw.matrix()(1,3),
                t_tcw.matrix()(2,0),t_tcw.matrix()(2,1),t_tcw.matrix()(2,2),t_tcw.matrix()(2,3));
        Mat T2 = (cv::Mat_<double>(3,4)<<
                                       T_c_r_estimate.matrix()(0,0),T_c_r_estimate.matrix()(0,1),T_c_r_estimate.matrix()(0,2),T_c_r_estimate.matrix()(0,3),
                T_c_r_estimate.matrix()(0,0),T_c_r_estimate.matrix()(0,0),T_c_r_estimate.matrix()(1,2),T_c_r_estimate.matrix()(1,3),
                T_c_r_estimate.matrix()(2,0),T_c_r_estimate.matrix()(2,1),T_c_r_estimate.matrix()(2,2),T_c_r_estimate.matrix()(2,3));
        Mat pts_4d;
        cv::triangulatePoints(T1,T2,pts1,pts2,pts_4d);
        for(int i=0;i<pts_4d.cols;i++)
        {
            int index = inliers.at<int> (i,0);
            Mat x = pts_4d.col(i);
            x /= x.at<float>(3,0);
            Vector3d pose_pts(x.at<double>(0,0),x.at<double>(1,0),x.at<double>(2,0));
            auto tmp = map_point_vector[index]->pose_;
            map_point_vector[index]->pose_ = (pose_pts*2+tmp)/3;
        }
    }

    bool VO_kitti::tri_ornot(){
        SE3 T_delta = frame_ref_->T_c_w * T_c_r_estimate.inverse();
        Sophus::Vector6d d = T_delta.log();
        Vector3d d_trans = d.head<3>();
        Vector3d d_rot = d.tail<3>();
        if ( d_rot.norm()<0.5*key_frame_min_rot && d_trans.norm()>key_frame_min_trans*0.8 )
            return true;
        return false;
    }

    bool VO_kitti::largerot()
    {
        SE3 T_delta = frame_ref_->T_c_w * T_c_r_estimate.inverse();
        Sophus::Vector6d d = T_delta.log();
        Vector3d d_rot = d.tail<3>();
        if (d_rot.norm()>0.5*key_frame_min_rot)
            return true;
        return false;
    }

}
