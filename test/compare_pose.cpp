//
// Created by da on 18-8-14.
//
#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.h>

#include <fstream>
#include <pangolin/pangolin.h>
#include <boost/format.hpp>
using namespace std;
/****************************************************************/
//string pose_file = "../00.txt";
//string my_file = "../pose_cal.txt";

/*****************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2);
void read_pose(string file,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>& poses);

int main(int argc,char** argv){
/***********************************************************/
    if(argc!=3)
    {
        cout<<"file is lost"<<endl;
    }
    string pose_file = argv[1];
    string my_file = argv[2];
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_dataset;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_1;
    read_pose( pose_file,poses_dataset );
    read_pose( my_file,poses_1 );

/********************************************************/
/********************************************************/
    DrawTrajectory(poses_dataset,poses_1);

    return 0;

};

void read_pose(string file,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>& poses)
{
    ifstream fin(file);
    while (!fin.eof()) {
        double data[12];
        for (auto &d: data) fin >> d;
        Eigen::Matrix3d m_tmp;
        m_tmp << data[0], data[1],data[2],
                data[4],data[5],data[6],
                data[8],data[9],data[10];
        Eigen::Vector3d vec(data[3],data[7],data[11]);
        poses.push_back(Sophus::SE3(m_tmp,vec));
        if (!fin.good()) break;
    }
    fin.close();
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2)
{
    if (poses1.empty()||poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            //glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glColor3f(0.0f, 0.0f, 255.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses2.size() - 1; i++) {
            //glColor3f(1 - (float) i / poses2.size(), 0.0f, (float) i / poses2.size());
            glColor3f(255.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p3 = poses2[i], p4 = poses2[i + 1];
            glVertex3d(p3.translation()[0], p3.translation()[1], p3.translation()[2]);
            glVertex3d(p4.translation()[0], p4.translation()[1], p4.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}