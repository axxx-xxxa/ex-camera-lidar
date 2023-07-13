#ifndef LIDAR_CAMERA_CALIB_H
#define LIDAR_CAMERA_CALIB_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/affine.hpp>

#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/ccalib/multicalib.hpp"
#include "opencv2/ccalib/randpattern.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "common.h"

ros::Subscriber points_cloud_subscriber_;
ros::Subscriber selected_points_cloud_subscriber_;
ros::Subscriber right_points_cloud_subscriber_;

ros::Publisher rgb_cloud_pub_;
ros::Publisher init_cloud_pub_;
ros::Publisher final_cloud_pub_;
ros::Publisher final_image_pub_;
ros::Publisher right_cloud_pub_;

std_msgs::Header _velodyne_header;

std::string lidar_topic, camera_topic, right_lidar_topic;

std::string result_path, extrinsic_file;
bool save_img, save_pcl, save_points;
// 相机内参
double fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, k4_, s_;
int width_, height_;
cv::Mat camera_matrix_;
cv::Mat dist_coeffs_;
cv::Mat extrinsic_;
cv::Mat lastImg;

bool usefisheye_;
int camera_model_=0;

double xi_;

pcl::PointCloud<pcl::PointXYZI> last_raw_lidar_cloud;

double rightdis;
double leftdis;
double intensityth;
int g_nAlphaValuesSlider;
int g_calibClose;
int g_clash;
int Isclash;
int calibClose;
const int maxintensityth = 255;
double EuclMinPointsNum, EuclMaxPointsNum, EuclMinDistance;
double cameraplateYdis;
double lidarplateZdis;
bool useRobotSensor;
int imgCailbNums=0;
int imgCailbPoints=0;
int ScreenW,ScreenH;

float voxel_size_;
float down_sample_size_;

bool calibSucess=false;

float colors[] = {
  255, 0,   0,   // red        1
  0,   255, 0,   // green      2
  0,   0,   255, // blue       3
  255, 255, 0,   // yellow     4
  0,   255, 255, // light blue 5
  255, 0,   255, // magenta    6
  255, 255, 255, // white      7
  255, 128, 0,   // orange     8
  255, 153, 255, // pink       9
  51,  153, 255, //           10
  153, 102, 51,  //           11
  128, 51,  153, //           12
  153, 153, 51,  //           13
  163, 38,  51,  //           14
  204, 153, 102, //           15
  204, 224, 255, //           16
  128, 179, 255, //           17
  206, 255, 0,   //           18
  255, 204, 204, //           19
  204, 255, 153, //           20
};

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)    
)

class ReprojectionError
{
public:
  ReprojectionError(Eigen::Vector3d point_, Eigen::Vector2d observed_)
      : point(point_), observed(observed_)
  {

  }
  template<typename T>
  bool operator()(const T* const camera_r, const T* const camera_t, T* residuals) const
  {
    T pt1[3];
    pt1[0] = T(point.x());
    pt1[1] = T(point.y());
    pt1[2] = T(point.z());

    T pt2[3];
    ceres::AngleAxisRotatePoint(camera_r, pt1, pt2);


    pt2[0] = pt2[0] + camera_t[0];
    pt2[1] = pt2[1] + camera_t[1];
    pt2[2] = pt2[2] + camera_t[2];

    const T xp = T(fx_ * (pt2[0] / pt2[2]) + cx_);
    const T yp = T(fy_ * (pt2[1] / pt2[2]) + cy_);
    const T u = T(observed.x());
    const T v = T(observed.y());

    residuals[0] = u - xp;
    residuals[1] = v - yp;
    return true;
    }
  static ceres::CostFunction* Create(Eigen::Vector3d points, Eigen::Vector2d observed)
  {return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
      new ReprojectionError(points, observed)));}

private:
  Eigen::Vector3d point;
  Eigen::Vector2d observed;
};

cv::Point3f lidarPoints;
std::vector<cv::Point2f> cameraPoints;
std::vector<cv::Point3f> lidarPnPPoints;
std::vector<cv::Point2f> cameraPnPPoints;

boost::shared_ptr<boost::thread> solver_thread;

void solver_callback();

#endif
