/*************************************************************************
    > File Name: detectFeatures.cpp
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > 特征提取与匹配
    > Created Time: 2015年07月18日 星期六 16时00分21秒
 ************************************************************************/

#include<iostream>
#include<string>
#include<fstream>
//#include "string.h"
#include "slamBase.h"
using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "feature_extract.h"

//#define SIFT_DETECTOR
#define ORB_DETECTOR
#define POSE_ESTI
#define TEST_CODES

Point3f pixel2cam ( const Point2d& p, const cv::Mat & lf_intrinsic, double depth);
// disparity label to depth
void disparityLabel2depth(const cv::Mat & disparityMap, cv::Mat & depthMap,
		const cv::Mat & lf_intrinsic, double labeluint, int labelnum,
		cv::Mat &rgb, PointCloud::Ptr &pointCloud);

void Depth2PointCloud(const cv::Mat & depthMap, const cv::Mat & rgb,
		PointCloud::Ptr &pointCloud, CAMERA_INTRINSIC_PARAMETERS& K_camera);

template <typename T_type1,typename T_type2>
void saveMat2txt(const cv::Mat & depthMap, std::string fileName);

template <typename T_type>
void readtxt2mat(cv::Mat & disparityMap, std::string fileName);


int main( int argc, char** argv )
{
    // 声明并从data文件夹里读取两个rgb与深度图
	/*
    cv::Mat rgb1 = cv::imread( "/home/jdy/project/pointXYZRGB/data/1.png");
    cv::Mat rgb2 = cv::imread( "/home/jdy/project/pointXYZRGB/data/2.png");
    cv::Mat depth1 = cv::imread( "/home/jdy/project/pointXYZRGB/data/1_depth.png", -1);
    cv::Mat depth2 = cv::imread( "/home/jdy/project/pointXYZRGB/data/2_depth.png", -1);
    */
    // B03
    cv::Mat rgb1 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0085.png");
    cv::Mat rgb2 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0086.png");
    cv::Mat rgb3 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0088.png");

    cv::Mat depth1 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0085_d.png", -1);
    cv::Mat depth2 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0086_d.png", -1);
    cv::Mat depth3 = cv::imread( "/home/jdy/project/pointXYZRGB/data/B03/IMG_0088_d.png", -1);


#ifdef ORB_DETECTOR
    std::vector<KeyPoint> keypoints_1;
    std::vector<KeyPoint> keypoints_2;
    std::vector< DMatch > matches;
    find_feature_matches_orb ( rgb1, rgb2, keypoints_1, keypoints_2, matches );
    cv::waitKey(2000);
    cout<<"一共找到computing hover expression了"<<matches.size() <<"组匹配点"<<endl;
#endif
#ifdef SIFT_DETECTOR
    std::vector<KeyPoint> keypoints_1;
       std::vector<KeyPoint> keypoints_2;
       std::vector< DMatch > matches;
       find_feature_matches_sift ( rgb1, rgb2, keypoints_1, keypoints_2, matches );
    ////////////////////////// sift feature detector ////////
    
    /////////////////// sift feature detector ////////
#endif
    /*
    //////////////////////////////////////////////////////
    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 为调用此函数准备必要的参数
    
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = 325.5;
    C.cy = 253.5;
    C.fx = 518.0;
    C.fy = 519.0;
    C.scale = 1000.0;

    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( kp2[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    cout<<"inliers: "<<inliers.rows<<endl;
    cout<<"R="<<rvec<<endl;
    cout<<"t="<<tvec<<endl;

    // 画出inliers匹配 
    vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::imwrite( "./data/inliers.png", imgMatches );
    cv::waitKey( 0 );
    ///////////////CMakeFilesCMakeFilesCMakeFiles//////////////////////////
    */
    ///////////// image to point clouds // jdy // 20180816


    /////////////////// disparity to depth
    //cv::Mat lf_intrinsic = cv::Mat::zeros(5,5,CV_32FC1);

    cv::Mat lf_intrinsic = (Mat_<double>(5,5) <<
    		0.0002696354775,0,0.0004057370983,0,-0.1291527956,
			0,0.0002571136427,0,0.000396613423,-0.08832032864,
			-0.0001850038529,0,0.001385227506,0,-0.4320961786,
			0,-0.0001747097584,0,0.001398433839,-0.3027616819,
			0,0,0,0,1);

    cout<<"lf_intrinsic"<<" "<<lf_intrinsic<<endl;
    /////////////////////// disparity read
    cv::Mat disparityMap1 = cv::Mat::zeros(depth1.rows, depth1.cols, CV_64FC1);
    cv::Mat depthMap1 = cv::Mat::zeros(depth1.rows, depth1.cols, CV_64FC1);

    readtxt2mat<double>(disparityMap1,
    		std::string("/home/jdy/project/lfslamVO/data/dataset/disparity_85.txt"));
    ////////////////////// disparity to depthMap && image to point clouds
    ////// first image
    double labeluint = 0.02;
    int labelnum = 75;
    PointCloud::Ptr pointCloud(new PointCloud);
    disparityLabel2depth(disparityMap1, depthMap1, lf_intrinsic, labeluint, labelnum,
    		rgb1, pointCloud);
    pcl::io::savePCDFileBinary(
      		"/home/jdy/project/pointXYZRGB/data/B03/pcloud_0085.pcd", *pointCloud);
    saveMat2txt<double, double>(depthMap1, std::string("depthMap_0085.txt"));

    ////// second image
    /////////////////////// disparity read
    cv::Mat disparityMap2 = cv::Mat::zeros(depth1.rows, depth1.cols, CV_64FC1);
    cv::Mat depthMap2 = cv::Mat::zeros(depth1.rows, depth1.cols, CV_64FC1);

    readtxt2mat<double>(disparityMap2,
       		std::string("/home/jdy/project/lfslamVO/data/dataset/disparity_86.txt"));
    ////////////////////// disparity to depthMap && image to point clouds

    PointCloud::Ptr pointCloud2(new PointCloud);
    disparityLabel2depth(disparityMap2, depthMap2, lf_intrinsic, labeluint, labelnum,
        		rgb1, pointCloud2);
    pcl::io::savePCDFileBinary(
    		"/home/jdy/project/pointXYZRGB/data/B03/pcloud_0086.pcd", *pointCloud);
    saveMat2txt<double, double>(depthMap2, std::string("depthMap_0086.txt"));

    /////////////////////////
#ifdef TEST_CODES
    PointCloud::Ptr pointCloud_test2(new PointCloud);
    image2ColorDepthMap<unsigned int,uchar>(rgb1, depth1, pointCloud_test2);
    pcl::io::savePCDFileBinary(
          		"/home/jdy/project/pointXYZRGB/data/B03/pcloud_0085_disparity.pcd", *pointCloud_test2);

    cv::Mat rgbtest = cv::imread(
    		"/home/jdy/project/slamtest/tex+depth/lowres_estimation/grape2/view_grape.jpg");
    cv::Mat depthtest = cv::imread(
    		"/home/jdy/project/slamtest/tex+depth/lowres_estimation/grape2/CVPR15_depth.png", -1);
    saveMat2txt<int, uchar>(depthtest, std::string("grape2DepthMap.txt"));

    PointCloud::Ptr pointCloud_test3(new PointCloud);
    image2ColorDepthMap<unsigned int, uchar>(rgbtest, depthtest, pointCloud_test3);
    pcl::io::savePCDFileBinary(
      		"/home/jdy/project/pointXYZRGB/data/B03/pcloud_smallduck_disparity.pcd", *pointCloud_test3);
#endif
    /////////////////////////
#ifdef TEST_CODES
    double minDepth, maxDepth;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(depthMap1, &minDepth, &maxDepth, &minLoc, &maxLoc);
    cv::Mat temp2 = cv::Mat::zeros(depthMap1.rows, depthMap1.cols, CV_8UC1);
    cout<<"maxDepth = "<<" "<<maxDepth<<endl;
    depthMap1.convertTo(temp2, CV_8U, 255/2, 0);
    saveMat2txt<double,double>(temp2, std::string("depthMapInt_0085.txt"));
    cv::imwrite("depthMapTest_0085.png", temp2);
    /*
        for (int i = 0; i < depthMap.rows -1; ++i)
        {
        	for (int j = 0; j < depthMap.cols-1; ++j)
        	{
        		double tempDouble = (depthMap.at<double>(i, j) / maxDepth)*255;
        		temp2.at<int>(i, j) = (int)tempDouble;
        	}
        }*/
#endif


    /////////////////////// end

    //Depth2PointCloud(depthMap, rgb1, pointCloud, K_camera);

    //pointCloud = image2PointCloud( rgb1, depth1, K_camera );
    /*
    string fileName = "/home/jdy/project/pointXYZRGB/data/B03/p_cloud_rgb";
    string fileNLast = ".pcd";
    string fileNumber = std::to_string(img_num);
    fileName += fileNumber;
    fileName += fileNLast;

    pcl::io::savePCDFileBinary(fileName, *pointCloud); */
    ///////////// image to point clouds // jdy // 20180816
#ifdef POSE_ESTI
    ///////////// SVD ICP edit by jdy 20180816
    vector<Point3f> pts1, pts2;

    for ( DMatch m:matches )
    {
        double d1 = depthMap1.ptr<double> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        double d2 = depthMap2.ptr<double> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        cv::Point3f p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, lf_intrinsic, d1 );
        cv::Point3f p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, lf_intrinsic, d2 );
        //float dd1 = float ( d1 ) /5000.0;
        //float dd2 = float ( d2 ) /5000.0;
        pts1.push_back ( p1 );
        pts2.push_back ( p2 );
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    pose_estimation_3d3d ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;
    ///////////// SVD ICP edit by jdy 20180816
#endif
    return 0;
}

//detectFeatures.cpp
Point3f pixel2cam ( const Point2d& p, const cv::Mat & lf_intrinsic,
		double depth)
{
	Point3f tempPoint;
	cv::Mat original_coord_in(5, 1, CV_64FC1);
				original_coord_in.at<double>(0,0)= 8.0;
				original_coord_in.at<double>(1,0)= 8.0;
				original_coord_in.at<double>(2,0)= p.x;
				original_coord_in.at<double>(3,0)= p.y;
				original_coord_in.at<double>(4,0)= 1.0;

				cv::Mat original_coord_out(5, 1, CV_64FC1);
				//cv::Mat reference_coord_out(5, 1, CV_64FC1);

				//reference_coord_out=lf_intrinsic*reference_coord_in;
				original_coord_out=lf_intrinsic*original_coord_in;
				double s = original_coord_out.at<double>(0,0);
				double t = original_coord_out.at<double>(1,0);
				double u = original_coord_out.at<double>(2,0);
				double v = original_coord_out.at<double>(4,0);


				tempPoint.z = depth;
				tempPoint.x = depth*(u - s)+s;
				tempPoint.y = depth*(v - t)+t;
    return tempPoint;
}

// disparity label to depth
void disparityLabel2depth(const cv::Mat & disparityMap, cv::Mat & depthMap,
		const cv::Mat & lf_intrinsic, double labeluint, int labelnum,
		cv::Mat &rgb, PointCloud::Ptr &pointCloud)
{
	for (int i = 0; i < disparityMap.rows; ++i)
		{
		for (int j = 0; j < disparityMap.cols; ++j)
		{
			int tempLabel = disparityMap.at<double>(i,j);

			double tempDisparity;
			// (8, 8) to (8, 15) disparity of view
			double diff_row = 7.0;
			tempDisparity = labeluint*diff_row*(tempLabel - labelnum/2.0);
			// necessary to be modified

			cv::Mat original_coord_in(5, 1, CV_64FC1);
			original_coord_in.at<double>(0,0)= 8.0;
			original_coord_in.at<double>(1,0)= 8.0;
			original_coord_in.at<double>(2,0)= i;
			original_coord_in.at<double>(3,0)= j;
			original_coord_in.at<double>(4,0)= 1.0;

			cv::Mat reference_coord_in(5, 1, CV_64FC1);
			reference_coord_in.at<double>(0,0)= 8.0;
			reference_coord_in.at<double>(1,0)= original_coord_in.at<double>(1,0) + diff_row;
			reference_coord_in.at<double>(2,0)= original_coord_in.at<double>(2,0);
			reference_coord_in.at<double>(3,0)= original_coord_in.at<double>(3,0)+tempDisparity;
			reference_coord_in.at<double>(4,0)= 1.0;

			cv::Mat original_coord_out(5, 1, CV_64FC1);
			cv::Mat reference_coord_out(5, 1, CV_64FC1);

			reference_coord_out=lf_intrinsic*reference_coord_in;
			original_coord_out=lf_intrinsic*original_coord_in;

			double delta_st = reference_coord_out.at<double>(1,0)- original_coord_out.at<double>(1,0);
			double delta_uv = reference_coord_out.at<double>(3,0)- original_coord_out.at<double>(3,0);
			if ( 1 && i == j)
			{
				cout<<"i"<<" "<<i<<""<<"j"<<" "<<j<<endl;
				cv::Mat tempMat;
				cv::transpose(original_coord_in, tempMat);
				cout<<"original_coord_in"<<" "<<tempMat<<endl;
				cv::transpose(original_coord_out, tempMat);
				cout<<"original_coord_out"<<" "<<tempMat<<endl;
				cv::transpose(reference_coord_out, tempMat);
				cout<<"reference_coord_out"<<" "<<tempMat<<endl;

				cout<<"delta_st"<<" "<<delta_st<<""<<"delta_uv"<<" "<<delta_uv<<endl;

			}

			double D = 1;
			double z;
			z = D/(1 - (delta_uv/delta_st));
			/////// 3D position computing
			double x = original_coord_out.at<double>(0,0) -
					z*(original_coord_out.at<double>(0,0) - original_coord_out.at<double>(2,0));
			double y = original_coord_out.at<double>(1,0) -
					z*(original_coord_out.at<double>(1,0) - original_coord_out.at<double>(3,0));
			//////
			if (z< 2 && z> 0.1)
			{
			 PointT p;
			 // 计算这个点的空间坐标
			 p.z = z;
			 p.x = x;
			 p.y = y;

			 // 从rgb图像中获取它的颜色
			 // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			 p.b = rgb.ptr<uchar>(i)[j*3];
			 p.g = rgb.ptr<uchar>(i)[j*3+1];
			 p.r = rgb.ptr<uchar>(i)[j*3+2];

			 // 把p加入到点云中
			 pointCloud->points.push_back( p );
			}
			/////

			///////
			/*
			if (delta_st*delta_uv >= 0.0)
			{
				z = D/(1 - (delta_uv/delta_st));
			}else
			{
				z = D/(1 - (delta_uv/delta_st)); // 当delta_uv同delta_st异号时，它俩的比值也是负的
				// 于是，此时的表达式应该是z = D/(1 + abs(delta_uv/delta_st))。两个负号相抵，就不加abs了。
			} */
			depthMap.at<double>(i,j) = z;

		}
		}
}


void Depth2PointCloud(const cv::Mat &depthMap, const cv::Mat &rgb,
		PointCloud::Ptr &pointCloud, CAMERA_INTRINSIC_PARAMETERS& K_camera)
{
	for (int m = 0; m < depthMap.rows; m++)
	            for (int n=0; n < depthMap.cols; n++)
	            {
	                // 获取深度图中(m,n)处的值
	                ushort d = depthMap.ptr<ushort>(m)[n];
	                // d 可能没有值，若如此，跳过此点
	                if (d == 0)
	                    continue;
	                // d 存在值，则向点云增加一个点
	                PointT p;

	                // 计算这个点的空间坐标
	                p.z = double(d) / K_camera.scale;
	                p.x = (n - K_camera.cx) * p.z / K_camera.fx;
	                p.y = (m - K_camera.cy) * p.z / K_camera.fy;

	                // 从rgb图像中获取它的颜色
	                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
	                p.b = rgb.ptr<uchar>(m)[n*3];
	                p.g = rgb.ptr<uchar>(m)[n*3+1];
	                p.r = rgb.ptr<uchar>(m)[n*3+2];

	                // 把p加入到点云中
	                pointCloud->points.push_back( p );
	            }
}

template <typename T_type1,typename T_type2>
void saveMat2txt(const cv::Mat & depthMap, std::string fileName)
{
	  std::ofstream openfile(fileName);
	    for (int i = 0; i < depthMap.rows; ++i)
	       {
	       	for (int j = 0; j < depthMap.cols; ++j)
	       	{
	    			//T_type temp_d_double = depthMap.ptr<T_type>(i)[j];
	       		T_type1 temp_d_double = depthMap.at<T_type2>(i, j);
	    			//int temp_d_int = (int)temp_d_double;
	    			openfile << " " << temp_d_double;
	    			//openfile << " " << temp_d_int;

	       	}
	    		openfile << "\n";
	       }
	    openfile.close();
}

template <typename T_type>
void readtxt2mat(cv::Mat & disparityMap, std::string fileName)
{
	std::ifstream fileRead(fileName);
	//std::fstream fileRead;
		//fileRead.open(fileName, ios::in);

	    /*
	    std::vector<double> temp_DataVector;
	    double tempData;
	    while (fileRead.eof() == false)
	    {
	    	tempData<<fileReadlfslamVOlfslamVO;
	    	temp_DataVector.pushback(tempData);
	    }*/
	    //double tempLine[271250];
	    //fileRead.read((char*)(&tempLine), depth1.cols*sizeof(long));
		T_type tempLine;
	    for (int i = 0; i < disparityMap.rows; ++i)
	    {
	    	for (int j = 0; j < disparityMap.cols; ++j)
	    	{
	    		if (!fileRead.eof())
	    		{

	    			fileRead>>tempLine;
	    		//fileRead.read((char*)(&tempLine), sizeof(long));
	    		disparityMap.at<T_type>(i,j) = tempLine;
	    		}
	    		else
	    			cout<<"wrong disparity data \n"<<endl;
	    	}

	    }
	    fileRead.close();
}
