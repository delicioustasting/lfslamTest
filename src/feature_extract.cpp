#include "feature_extract.h"
#define DRAW_FEATURE
//using namespace std;

void find_feature_matches_orb ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
		}
	}
	// 可视化， 显示关键点
	//cv::Mat imgShow;
	cv::Mat imgMatches;
	cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches,
			imgMatches);
	cv::imshow("matches", imgMatches);
	cv::imwrite("/home/jdy/project/pointXYZRGB/data/B03/matches.png", imgMatches);
	//cv::waitKey(0); //暂停等待一个按键
}

void find_feature_matches_sift ( const Mat& rgb1, const Mat& rgb2,
                            std::vector<KeyPoint>& kp1,
                            std::vector<KeyPoint>& kp2,
                            std::vector< DMatch >& matches )
{
	// 声明特征提取器与描述子提取器
	    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> _detector;
	    cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> _descriptor;

	    // 构建提取器，默认两者#include <opencv2/xfeatures2d/nonfree.hpp>都为sift
	    // 构建sift, surf之前要初始化nonfree模块
	    //cv::initModule_nonfree();
	    _detector = cv::xfeatures2d::SiftFeatureDetector::create();
	    _descriptor = cv::xfeatures2d::SiftDescriptorExtractor::create();


	    //vector< cv::KeyPoint > kp1, kp2; //关键点
	    _detector->detect( rgb1, kp1 );  //提取关键点
	    _detector->detect( rgb2, kp2 );

	    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;

	    // 可视化， 显示关键点
	    cv::Mat imgShow;
	    cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	    cv::imshow( "keypoints", imgShow );
	    cv::imwrite( "/home/jdy/project/pointXYZRGB/data/B03/keypoints.png", imgShow );
	    //cv::waitKey(0); //暂停等待一个按键

	    // 计算描述子
	    cv::Mat desp1, desp2;
	    _descriptor->compute( rgb1, kp1, desp1 );
	    _descriptor->compute( rgb2, kp2, desp2 );

	    // 匹配描述子
	    //vector< cv::DMatch > matches;
	    cv::FlannBasedMatcher matcher;
	    matcher.match( desp1, desp2, matches );
	    cout<<"Find total "<<matches.size()<<" matches."<<endl;

	    // 可视化：显示匹配的特征
	    cv::Mat imgMatches;
	    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
	    cv::imshow( "matches", imgMatches );
	    cv::imwrite( "/home/jdy/project/pointXYZRGB/data/B03/matches.png", imgMatches );
	    //cv::waitKey( 0 );

	    // 筛选匹配，把距离太大的去掉
	    // 这里使用的准则是去掉大于四倍最小距离的匹配
	    vector< cv::DMatch > goodMatches;
	    double minDis = 9999;
	    for ( size_t i=0; i<matches.size(); i++ )
	    {
	        if ( matches[i].distance < minDis )
	            minDis = matches[i].distance;
	    }

	    for ( size_t i=0; i<matches.size(); i++ )
	    {
	        if (matches[i].distance < 4*minDis)
	            goodMatches.push_back( matches[i] );
	    }

	    // 显示 good matches
	    cout<<"good matches="<<goodMatches.size()<<endl;
	    cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
	    cv::imshow( "good matches", imgMatches );
	    cv::imwrite( "/home/jdy/project/pointXYZRGB/data/B03/good_matches.png", imgMatches );
	    //cv::waitKey(0);
}
void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}
