/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);

    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);//对IMU进行预积分
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());//对图像进行处理
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);//对IMU进行预积分
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);//对图像进行处理
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();//把现在滑窗里面的未经过非线性话的原数数据（初始值预设值），转化为非线性优化参数para_**
    void double2vector();//把ceres求解出来的结果附加到滑窗内的变量中去.把非线性优化参数para_**转化为现在滑窗里面的数据。
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();//,用来预测最新P,V,Q的姿态
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    //用来对原始图像进行畸变校正，特征点采集，光流跟踪
    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];//imu到相机的外参
    Vector3d tic[2];

    Vector3d        Ps[(WINDOW_SIZE + 1)];//Ps,Vs,Rs（这个是绝对坐标系下的位姿）
    Vector3d        Vs[(WINDOW_SIZE + 1)];//Ps,Vs,Rs（这个是绝对坐标系下的位姿）
    Matrix3d        Rs[(WINDOW_SIZE + 1)];//Ps,Vs,Rs（这个是绝对坐标系下的位姿）
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;//last_R, last_P: 滑窗里面最新的位姿
    Vector3d back_P0, last_P, last_P0;//last_R0, last_P0: 滑窗里面最旧的位姿
    double Headers[(WINDOW_SIZE + 1)];//相机id

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;//预积分计算的是两帧图像之间积分的值，通过一帧帧imu帧的迭代求解来获得。通过对pre_integrations进行push_back（），可以迭代求解出最终两帧图像间预积分的值。但是要计算一个个预积分，需要知道这个预积分开头的帧当前的加速度以及角速度，以这个基准来建立的。每次使用完这个函数都会更新acc_0，gyr_0的值，即当要建立一个新的预积分的时候，这个基准直接从acc_0，gyr_0获取。


    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;//滑窗中图片的帧数,代表当前处理的这一帧在滑动窗口中的第几个。取值范围是在0到WINDOW_SIZE之间。
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    //用来对滑动窗口内所有特征点的管理
    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;//标志
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;

    //视觉测量残差.以三角化的特征点一个个进行添加残差。具体过程，假设第l个特征点（在代码中用feature_index表示），第一次被第i帧图像观察到（代码中用imu_i表示），那个这个特征点在第j帧图像中（代码中用imu_j表示）的残差,即
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];//滑动窗口内11帧的位姿，6自由度7变量表示 SIZE_POSE: 7
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];//滑动窗口11帧对应的速度,ba,bg,9自由度。SIZE_SPEEDBIAS: 9,当没有使用imu的时候，para_SpeedBias就没有加入待求变量中，并且把para_Pose[0]设置为常量。
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];//相机到IMU的外参变换矩阵，6自由度7变量表示 SIZE_POSE: 7,当我们有精确的相机到imu的外参时候，para_Ex_Pose也设置为常量。
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];// 滑窗内第一个时刻的相机到IMU的时钟差,如果使用同步的VI设备的话，para_Td也作为一个常量。
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;//边缘化的残差项。观察变量
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;

    //P是位置，Q是四元数字，V是速度
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    // 是否初始化线程 false则表示还未初始化线程数
    bool initThreadFlag;
};
