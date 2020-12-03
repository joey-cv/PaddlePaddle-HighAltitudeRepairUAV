/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "globalOpt.h"
#include "Factors.h"
//当该类已进行初始化，就同步开启了新线程optimize（），对两个结果不断进行优化
GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}
//GPS2XYZ函数是将GPS的经纬高坐标转换成当前的坐标系的函数
void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}
//将回调函数中的vio数据导入
void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;


    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}
//getGlobalOdom为获取融合后位姿函数。
void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}
//将回调函数中的gps数据导入
void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    newGPS = true;

}
/**融合算法的实现
 * 首先呢，判断是否有gps数据，整体的算法就是在ceres架构下的优化算法。
所以总体的步骤就是ceres优化的步骤，首先添加优化参数块（AddParameterBlock函数），参数为globalPoseMap中的6维位姿（代码中旋转用四元数表示，所以共7维）。
之后添加CostFunction，通过构建factor重载operator方法实现（具体实现需要学习ceres库）。该部分有两个factor，一个是位姿图优化，另一个则是利用gps进行位置约束。
将factor添加后，进行ceres求解，更新此时gps和vio间的坐标系转换参数，之后再利用updateGlobalPath函数更新位姿。
总而言之，VF的和GPS的融合也是一个优化框架下的松组合，利用GPS的位置约束，使得位姿图优化可以不依赖回环，这是一大优势。


（1）当有新的GPS信号到来时候，进行GPS与视觉惯性的融合
（2）建立ceres的problem
lossfunction 设置为Huberloss
addParameterBlock添加优化的变量 ,优化的变量是q_array以及t_array。即globalPoseMap保存下来的每帧图像的位姿信息。其中参数变量的多少是由localPoseMap来决定的。即VIO有多少个数据，全局也就有多少个。迭代器指向的first为时间，second为7变量的位姿。其中在添加q_array由于维度只有三维，因此增加了local_parameterization来进行约束。
接着开始添加残差项，总共有两个残差项分别是：vio factor以及gps factor
– vio factor：残差项的costfunction创建由 relativeRTError来提供。观测值由vio的结果提供。此时计算的是以i时刻作为参考，从i到j这两个时刻的位移值以及四元数的旋转值作为观测值传递进入代价函数中。 此时iPj代表了i到j的位移，iQj代表了i到j的四元数变换。添加残差项的时候，需要添加当前i时刻的位姿以及j时刻的位姿。即用观测值来估计i时刻的位姿以及j时刻的位姿。
– gps factor：残差项的costfunction创建由 TError来提供。观测值由Gps数据的结果提供。添加残差项的时候，只需要添加当前i时刻的位姿。
求解非线性优化方程
求解出来后，把t_array和q_array（即两个优化的变量）赋值给globalposeMap。并且根据最新解算出来的结果（即i=length-1时刻最新的结果），跟新GPS到vio这两个独立体系之间坐标转换关系。
 
 
 
 TError及RelativeRTError

直观上理解：
{0, 1，2，3，4, 5，6…}
要估计出这些时刻，每个时刻的位姿。
我有的是两个方面的观测值，一方面是GPS得到的每个时刻的位置（x,y,z）（并且GPS信号可以提供在该时刻得到这个位置的精度posAccuracy），没有累计误差，精度较低。另一方面是VIO得到的每个时刻的位置（x,y,z）以及对应的姿态四元数（w,x,y,z）,存在累计误差，短时间内精度较高。为了得到更好的一个融合结果，因此我们采用这个策略：观测值中，初始位置由GPS提供，并且vio观测值信任的是i到j时刻的位移以及姿态变化量。 并不信任vio得到的一个绝对位移量以及绝对的旋转姿态量。只信任短期的i到j的变化量，用这个变化量作为整个代价函数的观测值，进行求解。
因此两个残差项TError及RelativeRTError分别对应的就是GPS位置信号以及vio短时间内估计的i到j时刻的位姿变化量对最终结果的影响。迭代求解的过程中均采用了AutoDiffCostFunction自动求解微分来进行迭代。
（1）TError
TError（x,y,z,accuracy），最后一项是定位精度，可以由GPS系统提供。残差除了直接观测值与真值相减以外，还除了这个accuracy作为分母。意味着精度越高，accuracy越小，对结果的影响就越大。
（2）RelativeRTError
RelativeRTError（dx,dy,dz,dqw,dqx,dqy,dqz，t_var,q_var），最后两项是位移以及旋转之间的权重分配比例，并且为了使得与TError对应。在程序中，应该是根据经验把最后两项设置成一个常值，分别对应0.1以及0.01。残差的得到就根据实际值与观测值之间的偏差直接得出。

 * 
 **/
void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            //约束
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];//,优化的变量
            double q_array[length][4];//,优化的变量
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                //优化参数块
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    //RelativeRTError（dx,dy,dz,dqw,dqx,dqy,dqz，t_var,q_var），最后两项是位移以及旋转之间的权重分配比例，并且为了使得与TError对应。在程序中，应该是根据经验把最后两项设置成一个常值，分别对应0.1以及0.01。残差的得到就根据实际值与观测值之间的偏差直接得出。
                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */

                }
                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    //TError（x,y,z,accuracy）
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}

//updateGlobalPath顾名思义更新全局位姿函数。
void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}