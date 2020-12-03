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

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);//getGlobalOdom为获取融合后位姿函数。
	nav_msgs::Path global_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);//GPS2XYZ函数是将GPS的经纬高坐标转换成当前的坐标系的函数
	void optimize();//融合算法的实现
	void updateGlobalPath();//updateGlobalPath顾名思义更新全局位姿函数。

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;//保存着vio的位姿,参数变量的多少
	map<double, vector<double>> globalPoseMap;//保存着优化后的全局位姿
	map<double, vector<double>> GPSPositionMap;//保存着gps数据
	bool initGPS;//第一个GPS信号触发
	bool newGPS;//有新的GPS信号后触发
	GeographicLib::LocalCartesian geoConverter;//GPS经纬度信号转化为X,Y,Z用到的第三方库
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

};