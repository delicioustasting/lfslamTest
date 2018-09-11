/*************************************************************************
    > File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Created Time: 2015年07月18日 星期六 15时14分22秒
    > 说明：rgbd-slam教程所用到的基本函数（C风格）
 ************************************************************************/
# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

//typedef cv::Mat_<double> lf_intrinsic;

class LF_intrin
{
public:
	LF_intrin();
	LF_intrin(int e, int h);
	~LF_intrin();
private:
	int a;
	int b;
	//lf_intrinsic LF_H;
	//cv::Mat<double>(5,5) m;


};
/*
struct lytroillum_intrinsic
{
	cv::Mat Mat_<double>(5,5);
	double cx, cy, fx, fy, scale;
};
*/
// 函数接口
// image2PonitCloud 将rgb图转换为点云
void image2PointCloud( cv::Mat& rgb, cv::Mat& depth,
		CAMERA_INTRINSIC_PARAMETERS& camera, PointCloud::Ptr &cloud );

template<typename T_type1, typename T_type2>
void image2ColorDepthMap(cv::Mat& rgb, cv::Mat& depth, PointCloud::Ptr &cloud);

void stereoDepthMap(cv::Mat& depth, PointCloud::Ptr &cloud);

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );
/////// modified by jdy 20180816
