//
// Created by da on 18-8-12.
//


#include "slam_g2o/config.h"
#include "slam_g2o/camera_kitti.h"
#include "boost/timer.hpp"
#include <fstream>
#include "slam_g2o/vo_kitti.h"
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz.hpp>
//paras
//string my_file = "../pose_cal_2.txt";
//string filepath = "/media/da/My Passport/kitti/dataset/sequences/01";
double baseline = 0.5372;



int main(int argc,char** argv)
{
    if(argc!=3)
    {
        cout<<"file is lost"<<endl;
    }
    string my_file = argv[1];
    string filepath = argv[2];
    //读取img文件名
    vector<string> img_leftlist,img_rightlist;
    read_fold(filepath+"/image_0/",img_leftlist);
    read_fold(filepath+"/image_1/",img_rightlist);
    assert(img_leftlist.size()==img_rightlist.size());
    vector<double> the_time;
    //
    ofstream fout;
    fout.open( my_file, ios::trunc );

    slam_g2o::Camera_kitti::Ptr camera( new slam_g2o::Camera_kitti );
    slam_g2o::VO_kitti::Ptr VOexp( new slam_g2o::VO_kitti(baseline) );

    //cout<<SE3().matrix();
    VOexp->camera_ = camera;

    for(int i=0;i<img_leftlist.size();i++)
    {
        Mat img_left,img_right;
//        color = cv::imread(img_leftlist[i],0);
//        depth = cv::imread(img_rightlist[i],0);
        //
        img_left = cv::imread(img_leftlist[i],0);
        img_right = cv::imread(img_rightlist[i],0);
        //
        if(img_left.data == nullptr || img_right.data == nullptr) break;
        slam_g2o::Frame_kitti::Ptr pFframe = slam_g2o::Frame_kitti::createFrame();
        pFframe->camera_ = camera;
        pFframe->right_ = img_right;
        pFframe->left_ = img_left;
        pFframe->time_stamp_ = i;
        boost::timer timer;
        VOexp->addFrame(pFframe);
        cout<<"the time cost is "<<timer.elapsed()<<endl;
        the_time.push_back(timer.elapsed());
        SE3 Twc = pFframe->T_c_w.inverse();
/********************************************************/


        cv::imshow("ase",img_left);
        cv::waitKey(1);
        cout<<"#######################"<<i<<endl;

        for(int k=0;k<3;k++)
        {
            for(int l=0;l<4;l++)
            {
                fout<< Twc.matrix()(k,l)<<" ";
            }

        }
        fout<<"\n";

    }
/******************************************************/
    fout.close();
    double sum_time=0;
    for(int i=0;i<the_time.size();i++)
    {
        sum_time+=the_time[i];
    }
    cout<<"The average cost time is "<<sum_time/the_time.size()<<endl;

    return 0;
}


