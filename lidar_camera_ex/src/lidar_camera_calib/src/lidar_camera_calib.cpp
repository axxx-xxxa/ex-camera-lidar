#include "lidar_camera_calib.h"

template<typename _Tp>
void publishCloud(const ros::Publisher *in_publisher, const _Tp in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

template<typename _Tp>
std::vector<_Tp> convertMat2Vector(const cv::Mat &mat)
{
  return (std::vector<_Tp>)(mat.reshape(1, 1));
}

template<typename _Tp>
std::vector<_Tp> convertMat2Vector2(const cv::Mat &mat)
{
  return (std::vector<_Tp>)(mat.reshape(2, 2));
}

template<typename _Tp>
std::vector<_Tp> convertMat2Vector4(const cv::Mat &mat)
{
  return (std::vector<_Tp>)(mat.reshape(4, 4));
}

template<typename _Tp>
std::vector<_Tp> convertMat2Vector6(const cv::Mat &mat)
{
  return (std::vector<_Tp>)(mat.reshape(6, 6));
}

template<typename _Tp>
std::vector<_Tp> convertMat2Vector8(const cv::Mat &mat)
{
  return (std::vector<_Tp>)(mat.reshape(8, 8));
}

template<typename _Tp>
cv::Mat convertVector2Mat(std::vector<_Tp> v, int channels, int rows)
{
  cv::Mat mat = cv::Mat(v);
  cv::Mat dest = mat.reshape(channels, rows).clone();
  return dest;
}

bool checkFov(const cv::Point2d &p) 
{
  if(p.x > 0 && p.x < width_ && p.y > 0 && p.y < height_)
    return true;
  else
    return false;
}

//void on_Trackbar(int, void *){intensityth = g_nAlphaValuesSlider;}
void on_calbar(int , void *){calibClose = g_calibClose;}
void on_clashbar(int , void *){Isclash = g_clash;}

bool writeCalfile(std::string result_file_, cv::Mat R, cv::Mat t)
{
  std::ofstream outfile(result_file_, std::ios::out);

  if(!outfile.is_open())
  {
     return false;
  }
  outfile<<"ExtrinsicMat:"<<std::endl<<"  rows: 4"<<std::endl<<"  cols: 4"<<std::endl<<"  dt: d"<<std::endl
         <<"  data: ["<<R.at<double>(0)<<", "<<R.at<double>(1) <<", "<<R.at<double>(2) <<", "<<t.at<double>(0)<<","<<std::endl
         <<"         "<<R.at<double>(3)<<", "<<R.at<double>(4) <<", "<<R.at<double>(5) <<", "<<t.at<double>(1)<<","<<std::endl
         <<"         "<<R.at<double>(6)<<", "<<R.at<double>(7) <<", "<<R.at<double>(8) <<", "<<t.at<double>(2)<<","<<std::endl
         <<"         "<<          "0.0"<<", "<<           "0.0"<<", "<<           "0.0"<<", "<<"1.0"          <<"]"<<std::endl;
  outfile.close();
  return true;
}

bool loadCalibConfig(ros::NodeHandle nh)
{
  std::vector<double> camera_matrix;
  std::vector<double> dist_coeffs;
  std::vector<double> init_extrinsic;

  nh.param<std::string>("lidar_node",  lidar_topic, "");
  nh.param<std::string>("right_lidar_node",  right_lidar_topic, "");
  nh.param<std::string>("camera_node",   camera_topic, "");

  nh.param<std::vector<double>>("camera_matrix", camera_matrix, std::vector<double>());
  nh.param<std::vector<double>>("dist_coeffs", dist_coeffs, std::vector<double>());
  nh.param<std::vector<double>>("ExtrinsicMat/data", init_extrinsic, std::vector<double>());

  nh.param<int>("camera_model", camera_model_, 0);

  if(camera_model_==2)
  {
    if(camera_matrix.size()!=9 || dist_coeffs.size()!=4 || init_extrinsic.size()!=16)
    {
      ROS_INFO("\033[1;41m Failed to load settings file, param error !\033[0m");
      ros::shutdown();
    }
    else
    {
      ROS_INFO("Sucessfully load calib config file");
    }

    dist_coeffs_  = convertVector2Mat<double>(dist_coeffs, 4, 1);

    k1_ = dist_coeffs[0];
    k2_ = dist_coeffs[1];
    k3_ = dist_coeffs[2];
    k4_ = dist_coeffs[3];

    dist_coeffs_   = (cv::Mat_<double> ( 4,1 ) << k1_, k2_, k3_, k4_);
  }

  if(camera_model_==1)
  {
    if(camera_matrix.size()!=9 || dist_coeffs.size()!=5 || init_extrinsic.size()!=16)
    {
      ROS_INFO("\033[1;41m Failed to load settings file, param error !\033[0m");
      ros::shutdown();
    }
    else
    {
      ROS_INFO("Sucessfully load calib config file");
    }

    dist_coeffs_  = convertVector2Mat<double>(dist_coeffs, 5, 1);

    k1_ = dist_coeffs[0];
    k2_ = dist_coeffs[1];
    p1_ = dist_coeffs[2];
    p2_ = dist_coeffs[3];
    k3_ = dist_coeffs[4];      

    dist_coeffs_   = (cv::Mat_<double> ( 5,1 ) << k1_, k2_, p1_, p2_, k3_);
  }

  if(camera_model_==3)
  {
    if(camera_matrix.size()!=9 || dist_coeffs.size()!=4 || init_extrinsic.size()!=16)
    {
      ROS_INFO("\033[1;41m Failed to load settings file, param error !\033[0m");
      ros::shutdown();
    }
    else
    {
      ROS_INFO("Sucessfully load calib config file");
    }

    dist_coeffs_  = convertVector2Mat<double>(dist_coeffs, 4, 1);

    k1_ = dist_coeffs[0];
    k2_ = dist_coeffs[1];
    k3_ = dist_coeffs[2];
    k4_ = dist_coeffs[3];  

    dist_coeffs_   = (cv::Mat_<double> ( 4,1 ) << k1_, k2_, k3_, k4_);

    nh.param<double>("MEI_xi", xi_, 0.0);
  }

  camera_matrix_  = convertVector2Mat<double>(camera_matrix, 3, 3);

  fx_ = camera_matrix[0];
  cx_ = camera_matrix[2];
  fy_ = camera_matrix[4];
  cy_ = camera_matrix[5];

  camera_matrix_ = (cv::Mat_<double> ( 3,3 ) << fx_, 0.0, cx_,
                                                 0.0, fy_, cy_,
                                                 0.0, 0.0, 1.0);

  nh.getParam("useRobotSensor", useRobotSensor);
  nh.getParam("Cloud/leftdis", leftdis);
  nh.getParam("Cloud/rightdis", rightdis);
  nh.getParam("Cloud/intensityth", intensityth);
  nh.getParam("Cloud/EuclMinPointsNum", EuclMinPointsNum);
  nh.getParam("Cloud/EuclMaxPointsNum", EuclMaxPointsNum);
  nh.getParam("Cloud/EuclMinDistance", EuclMinDistance);
  nh.getParam("Image/Ydis", cameraplateYdis);
  nh.getParam("Image/ScreenW", ScreenW);
  nh.getParam("Image/ScreenH", ScreenH);
  nh.getParam("Cloud/Zdis", lidarplateZdis);

  nh.getParam("Voxel/size", voxel_size_);
  nh.getParam("Voxel/down_sample_size", down_sample_size_);

  nh.getParam("common/result_path", result_path);
  nh.getParam("common/extrinsic_file", extrinsic_file);
  nh.getParam("common/save_img", save_img);
  nh.getParam("common/save_pcl", save_pcl);
  nh.getParam("common/save_points", save_points);

  std::string time_q = std::to_string(ros::Time::now().toSec());
  time_q=time_q.replace(time_q.find("."),1,"_");

  //result_path = result_path +"/";

  cameraPnPPoints.clear();
  lidarPnPPoints.clear();

  cv::Point2f p2;
  cv::Point3f p3;

  std::string file_name = result_path  + "points.txt";
  std::ifstream ifs(file_name);
  std::string str_q;
  std::vector<double> lines;
  double lines_q;
  while(!ifs.eof())
  {
    getline(ifs, str_q);
		std::stringstream stringin(str_q);
		lines.clear();
		while (stringin >> lines_q) 
    {      
			lines.push_back(lines_q);
		}
    if(lines.size()>4)
    {
      p2.x = lines[0]; p2.y = lines[1]; 
      p3.x = lines[2]; p3.y = lines[3]; p3.z = lines[4]; 
      cameraPnPPoints.push_back(p2);
      lidarPnPPoints.push_back(p3);
    }
  }
  ifs.close();

  if(cameraPnPPoints.size()>0)
    cameraPnPPoints.pop_back();
  if(lidarPnPPoints.size()>0)
    lidarPnPPoints.pop_back();

  return true;
}

void saveImg(int name)
{
  std::string file_name = result_path + std::to_string(name) + ".jpg";
  cv::imwrite(file_name, lastImg);
}

void savepcl(int name)
{
  std::string file_name = result_path + std::to_string(name) + ".pcd";
  pcl::io::savePCDFile (file_name, last_raw_lidar_cloud); 
}

void savePoints(std::string name)
{
  std::string file_name = result_path + name + ".txt";
  std::ofstream outfile(file_name, std::ios::out);

  if(!outfile.is_open())
  {
     return;
  }

  int size_q = cameraPnPPoints.size();

  for(int i=0; i < size_q; i++)
  {
    outfile<<cameraPnPPoints[i].x<<" "<<cameraPnPPoints[i].y <<" "<<lidarPnPPoints[i].x<<" "<<lidarPnPPoints[i].y<<" "<<lidarPnPPoints[i].z<<std::endl;
  }

  outfile.close();
}

static void on_draw(int event, int x, int y, int flags, void* userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    cv::Point2d p;

    p.x = x * ((double)width_ /ScreenW);
    p.y = y * ((double)height_/ScreenH);

    //if(Isclash)
    {
      cameraPnPPoints.push_back(p);
      lidarPnPPoints.push_back(lidarPoints);
      std::string file_name = "points";
      savePoints(file_name);
      std::cout << "clib point.x:" << p.x<<" clib point.y:"<< p.y<< std::endl;
    }
  }
}

void pointTrans(pcl::PointCloud<pcl::PointXYZI> in_cloud,
                pcl::PointCloud<pcl::PointXYZI> &out_cloud,
                double xx_trans, double yy_trans, double zz_trans,
                double xx_rotation, double yy_rotation, double zz_rotation)
{
  out_cloud.points.clear();

  for (unsigned int i = 0; i < in_cloud.points.size(); i++)
  {
    double x, y, z;    
    pcl::PointXYZI ppcloud=in_cloud.points[i];
        
    pcl::PointXYZI sourPoint   = ppcloud;
    pcl::PointXYZI xTransPoint = ppcloud;
    pcl::PointXYZI yTransPoint = ppcloud;
    pcl::PointXYZI zTransPoint = ppcloud;

    x = sourPoint.x;/*X rotation*/
    y = sourPoint.y;
    z = sourPoint.z;
    xTransPoint.x = x;

    xTransPoint.y = y * std::cos(xx_rotation) - z * std::sin(xx_rotation);
    xTransPoint.z = y * std::sin(xx_rotation) + z * std::cos(xx_rotation);

    x = xTransPoint.x;/*Y rotation*/
    y = xTransPoint.y;
    z = xTransPoint.z;

    yTransPoint.x = x * std::cos(yy_rotation) + z * std::sin(yy_rotation);
    yTransPoint.y = y;
    yTransPoint.z = x * (-std::sin(yy_rotation)) + z * std::cos(yy_rotation);

    x = yTransPoint.x;/*Z rotation*/
    y = yTransPoint.y;
    z = yTransPoint.z;

    zTransPoint.x = x * std::cos(zz_rotation) + y * (-std::sin(zz_rotation));
    zTransPoint.y = x * std::sin(zz_rotation) + y * std::cos(zz_rotation);
    zTransPoint.z = z;

    ppcloud.x = zTransPoint.x + xx_trans;
    ppcloud.y = zTransPoint.y + yy_trans;
    ppcloud.z = zTransPoint.z + zz_trans;

    out_cloud.points.emplace_back(ppcloud);    
  }
}

void bundleAdjustment (const std::vector<cv::Point3f > points_3d, const std::vector< cv::Point2f > points_2d, cv::Mat& rvec, cv::Mat& t)
{
    assert(rvec.type() == CV_64F);
    assert(t.type() == CV_64F);

    double camera_rvec[3];
    camera_rvec[0] = rvec.at<double>(0,0); 
    camera_rvec[1] = rvec.at<double>(1,0);
    camera_rvec[2] = rvec.at<double>(2,0);
    
    double camera_t[3];
    camera_t[0] = t.at<double>(0,0);
    camera_t[1] = t.at<double>(1,0);
    camera_t[2] = t.at<double>(2,0);

    ceres::Problem problem;
    ceres::LossFunction* lossfunction = NULL;
    for(uint i = 0; i < points_3d.size(); i++)
    {
        Eigen::Vector3d p3d(points_3d[i].x, points_3d[i].y, points_3d[i].z);
        Eigen::Vector2d p2d(points_2d[i].x, points_2d[i].y);

        ceres::CostFunction* costfunction = ReprojectionError::Create(p3d, p2d);
        problem.AddResidualBlock(costfunction, lossfunction, camera_rvec, camera_t);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double quat[4];
    ceres::AngleAxisToQuaternion(camera_rvec, quat);
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);

    rvec.at<double>(0,0) = camera_rvec[0];
    rvec.at<double>(1,0) = camera_rvec[1];
    rvec.at<double>(2,0) = camera_rvec[2];
    t.at<double>(0,0) = camera_t[0];
    t.at<double>(1,0) = camera_t[1];
    t.at<double>(2,0) = camera_t[2];
}
bool hhhh=true;
void img_callback(const sensor_msgs::ImageConstPtr& image_msg)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
  cv::Mat image_ = cv_image->image;

  // cv::Mat image_ = cv::Mat(image_msg->height, image_msg->width, CV_8UC3, const_cast<uint8_t*>(&image_msg->data[0]), image_msg->step);

  cv::Mat curimage;

  if(image_.empty())
  {
    std::string msg = "image empty";
    ROS_ERROR_STREAM(msg.c_str());
    return;
  }

  if(!Isclash)
  {
    width_ = image_.cols;
    height_ = image_.rows;

    if(camera_model_==1)
    {
      //普通相机内外参标定
      cv::undistort(image_, curimage, camera_matrix_, dist_coeffs_);
    
      //订阅到的图片是去畸变之后的
      //curimage = image_;
    }    
    //鱼眼内外参标定    
    if(camera_model_==2)
    {
      cv::Mat opt_camera_matrix_;
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix_, dist_coeffs_, cv::Size(width_, height_), cv::Mat::eye(3, 3, CV_32FC1), opt_camera_matrix_);
      cv::Mat map1, map2;
      cv::fisheye::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(), camera_matrix_, cv::Size(width_, height_), CV_32FC1, map1, map2);
      cv::remap(image_, curimage, map1, map2, cv::INTER_LINEAR); 
    }
    //MEI 模型鱼眼内外参标定    
    if(camera_model_==3)
    {
      cv::Mat map11, map22;
      cv::Mat opt_camera_matrix_;         
      cv::omnidir::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, xi_, 
            cv::Mat::eye(3, 3, CV_32FC1), opt_camera_matrix_, cv::Size(width_, height_), CV_32FC1, map11, map22, cv::omnidir::RECTIFY_PERSPECTIVE);
      cv::remap(image_, curimage, map11, map22, cv::INTER_LINEAR); 

     if(hhhh)
     {
       hhhh=false;
      std::cout<<"1111111111"<<std::endl;
      std::cout<<camera_matrix_<<std::endl;      
      std::cout<<dist_coeffs_<<std::endl;            
      std::cout<<xi_<<std::endl;       
     } 
    
    }

    width_ = curimage.cols;
    height_ = curimage.rows;
    lastImg = curimage;
  }
  else
  {
    curimage = lastImg;
  }

  if(calibSucess)
  {
    pcl::PointCloud<pcl::PointXYZ> qq_left_cloud;
    pcl::PointXYZ qqpp;
    int max_val = 10;
    for(size_t i=0; i<last_raw_lidar_cloud.points.size(); i++)
    {
      if(camera_topic.find("ud_camera5")!=std::string::npos || camera_topic.find("ud_camera1")!=std::string::npos)
      {
        if(last_raw_lidar_cloud.points[i].x <=0)
        {
          continue;
        }
      }
      if(camera_topic.find("ud_camera6")!=std::string::npos || camera_topic.find("ud_camera2")!=std::string::npos)
      {
        if(last_raw_lidar_cloud.points[i].x >=0)
        {
          continue;
        }
      }
      if(camera_topic.find("ud_camera7")!=std::string::npos || camera_topic.find("ud_camera3")!=std::string::npos)
      {
        if(last_raw_lidar_cloud.points[i].y <=0)
        {
          continue;
        }
      }
      if(camera_topic.find("ud_camera8")!=std::string::npos || camera_topic.find("ud_camera4")!=std::string::npos)
      {
        if(last_raw_lidar_cloud.points[i].y >=0)
        {
          continue;
        }
      }

      float point_in_cam_x = last_raw_lidar_cloud.points[i].x * extrinsic_.at<double>(0, 0) + 
                             last_raw_lidar_cloud.points[i].y * extrinsic_.at<double>(0, 1) + 
                             last_raw_lidar_cloud.points[i].z * extrinsic_.at<double>(0, 2) + extrinsic_.at<double>(0, 3);

      float point_in_cam_y = last_raw_lidar_cloud.points[i].x * extrinsic_.at<double>(1, 0) + 
                             last_raw_lidar_cloud.points[i].y * extrinsic_.at<double>(1, 1) + 
                             last_raw_lidar_cloud.points[i].z * extrinsic_.at<double>(1, 2) + extrinsic_.at<double>(1, 3);

      float point_in_cam_z = last_raw_lidar_cloud.points[i].x * extrinsic_.at<double>(2, 0) + 
                             last_raw_lidar_cloud.points[i].y * extrinsic_.at<double>(2, 1) + 
                             last_raw_lidar_cloud.points[i].z * extrinsic_.at<double>(2, 2) + extrinsic_.at<double>(2, 3);

      int x = (int)((camera_matrix_.at<double>(0, 0) * point_in_cam_x + 
                     camera_matrix_.at<double>(0, 1) * point_in_cam_y + 
                     camera_matrix_.at<double>(0, 2) * point_in_cam_z) / point_in_cam_z);

      int y = (int)((camera_matrix_.at<double>(1, 0) * point_in_cam_x + 
                     camera_matrix_.at<double>(1, 1) * point_in_cam_y + 
                     camera_matrix_.at<double>(1, 2) * point_in_cam_z) / point_in_cam_z);
             
      if(x > 0 && x < width_ && y > 0 && y < height_)
      {
        float dist = (float)(std::sqrt(point_in_cam_x * point_in_cam_x + point_in_cam_y * point_in_cam_y + point_in_cam_z * point_in_cam_z));
        int red = std::min(255, (int)(255 * std::abs((dist - max_val) / max_val)));
        int green = std::min(255, (int)(255 * (1 - std::abs((dist - max_val) / max_val))));
        cv::circle(curimage, cv::Point(x,y), 1, cv::Scalar(0, green, red), -1);  
      }
      // if(x > 553 && x < 596 && y > 175 && y < 200) // cam4
      // if(x > 1330 && x < 1382 && y > 62 && y < 110) // cam3
      // if(x > 1542 && x < 1650 && y > 139 && y < 231) // cam2
      // if(x > 1185 && x < 1256 && y > 141 && y < 233) // cam5
      // if(x > 1164 && x < 1225 && y > 110 && y < 177) // cam6
      // if(x > 1068 && x < 1090 && y > 98 && y < 123) // cam7
      // if(x > 1206 && x < 1290 && y > 149 && y < 229) // cam8
      if(x > 391 && x < 423 && y > 187 && y < 212) // cam5m
      {
        qqpp.x=last_raw_lidar_cloud.points[i].x;
        qqpp.y=last_raw_lidar_cloud.points[i].y;
        qqpp.z=last_raw_lidar_cloud.points[i].z;   
        qq_left_cloud.points.emplace_back(qqpp);            
      }
    }
    publishCloud(&final_cloud_pub_, qq_left_cloud);
  }


  for(int i=0; i < cameraPnPPoints.size(); i ++)
  {
    cv::circle(curimage, cv::Point(cameraPnPPoints[i].x,cameraPnPPoints[i].y), 5, cv::Scalar(0, 0, 255), -1);  
    cv::putText(curimage, std::to_string(i), cameraPnPPoints[i], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255), 2, 8, 0);
  }

  cv::resize(curimage, curimage, cv::Size(ScreenW, ScreenH));
  cv::imshow("image", curimage);
  cv::waitKey(10);
  curimage.release();

}

void selected_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI> raw_lidar_cloud;
  if(useRobotSensor)
  {
    pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
    pcl::fromROSMsg(*in_cloud_msg, *tmpRobosenseCloudIn);

    for(size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
    {
      pcl::PointXYZI p;
      p.x = tmpRobosenseCloudIn->points[i].x;
      p.y = tmpRobosenseCloudIn->points[i].y;
      p.z = tmpRobosenseCloudIn->points[i].z;
      p.intensity = tmpRobosenseCloudIn->points[i].intensity;
      raw_lidar_cloud.push_back(p);
    }
  }
  else
  {
    pcl::fromROSMsg(*in_cloud_msg, raw_lidar_cloud);
  }

  cv::Point3f p_center(0, 0, 0);
  for(size_t j=0; j < raw_lidar_cloud.points.size(); j++)
  {
    p_center.x += raw_lidar_cloud.points[j].x;
    p_center.y += raw_lidar_cloud.points[j].y;
    p_center.z += raw_lidar_cloud.points[j].z;
  }

  p_center.x = p_center.x / raw_lidar_cloud.points.size();
  p_center.y = p_center.y / raw_lidar_cloud.points.size();
  p_center.z = p_center.z / raw_lidar_cloud.points.size();

  lidarPoints = p_center;

  std::cout << "lidar point.x:" << lidarPoints. x<<" "<<lidarPoints.y<<" "<<lidarPoints.z<< std::endl;

}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg)
{
  if(Isclash)
  {
    publishCloud(&init_cloud_pub_, last_raw_lidar_cloud);
    return;        
  }

  _velodyne_header = in_cloud_msg->header;
  _velodyne_header.frame_id="left_velodyne";

  pcl::PointCloud<pcl::PointXYZI> raw_lidar_cloud;
  if(useRobotSensor)
  {
    pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
    pcl::fromROSMsg(*in_cloud_msg, *tmpRobosenseCloudIn);

    for(size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
    {
      pcl::PointXYZI p;
      p.x = tmpRobosenseCloudIn->points[i].x;
      p.y = tmpRobosenseCloudIn->points[i].y;
      p.z = tmpRobosenseCloudIn->points[i].z;
      p.intensity = tmpRobosenseCloudIn->points[i].intensity;
      raw_lidar_cloud.push_back(p);
    }
  }
  else
  {
    pcl::fromROSMsg(*in_cloud_msg, raw_lidar_cloud);
  }

  if(raw_lidar_cloud.points.size()<=10)
  {
    std::string msg = "Unable to load pointcloud ";
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  last_raw_lidar_cloud = raw_lidar_cloud;

  publishCloud(&init_cloud_pub_, raw_lidar_cloud);
}

void right_lidar_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg)
{
  if(Isclash)
  {
    publishCloud(&init_cloud_pub_, last_raw_lidar_cloud);
    return;        
  }

  _velodyne_header = in_cloud_msg->header;
  _velodyne_header.frame_id="left_velodyne";

  pcl::PointCloud<pcl::PointXYZI> raw_lidar_cloud;
  pcl::PointCloud<pcl::PointXYZI> to_left_cloud;

  if(useRobotSensor)
  {
    pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
    pcl::fromROSMsg(*in_cloud_msg, *tmpRobosenseCloudIn);

    for(size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
    {
      pcl::PointXYZI p;
      p.x = tmpRobosenseCloudIn->points[i].x;
      p.y = tmpRobosenseCloudIn->points[i].y;
      p.z = tmpRobosenseCloudIn->points[i].z;
      p.intensity = tmpRobosenseCloudIn->points[i].intensity;
      raw_lidar_cloud.push_back(p);
    }
  }
  else
  {
    pcl::fromROSMsg(*in_cloud_msg, raw_lidar_cloud);
  }

  if(raw_lidar_cloud.points.size()<=10)
  {
    std::string msg = "Unable to load pointcloud ";
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  pointTrans(raw_lidar_cloud, to_left_cloud, -0.0657553, -1.25163, -0.0530008, 0.51104/ 180.0 * M_PI, 0.0921283/ 180.0 * M_PI, -4.03423/ 180.0 * M_PI);

  last_raw_lidar_cloud = to_left_cloud;

  publishCloud(&right_cloud_pub_, to_left_cloud);
  publishCloud(&init_cloud_pub_, to_left_cloud);
}

void solver_callback()
{
  static int couter=0;
  cv::Mat thread_img;  

  while(ros::ok)
  {
    //if(Isclash)
    {
      publishCloud(&init_cloud_pub_, last_raw_lidar_cloud); 
      thread_img=lastImg;
      if(thread_img.empty())
      {
        usleep(3000);
        continue;
      }
      cv::resize(thread_img, thread_img, cv::Size(ScreenW, ScreenH));
      cv::imshow("image", thread_img);
      cv::waitKey(10);
    }

    if(!calibClose)
    {
      usleep(3000);
      continue;
    }

    if(lidarPnPPoints.size()<=0 || cameraPnPPoints.size()<=0)
    {
      ROS_INFO("\033[1;41m------->not get the lidar and camera points: %d, %d\033[0m", lidarPnPPoints.size(), cameraPnPPoints.size());
      usleep(100000);
      continue;
    }

    if(lidarPnPPoints.size()!=cameraPnPPoints.size())
    {
      ROS_INFO("\033[1;41m------->lidar and camera points are no same: %d, %d\033[0m", lidarPnPPoints.size(), cameraPnPPoints.size());
      usleep(100000);
      continue;
    }

    if(lidarPnPPoints.size()<=4 || cameraPnPPoints.size()<=4)
    {
      ROS_INFO("\033[1;41m------->not get the lidar and camera points enough: %d, %d\033[0m", lidarPnPPoints.size(), cameraPnPPoints.size());
      usleep(100000);
      continue;
    }

    cv::Mat R, t;
    cv::solvePnP(lidarPnPPoints, cameraPnPPoints, camera_matrix_, dist_coeffs_, R, t, false, cv::SOLVEPNP_EPNP);
    cv::Mat RR,RRR;

    cv::Rodrigues(R, RRR);
    std::cout<<std::endl;
    std::cout<<"R: "<<RRR<<std::endl;
    std::cout<<"t: "<<t<<std::endl;
    std::cout<<std::endl;
    bundleAdjustment(lidarPnPPoints, cameraPnPPoints, R, t);
    cv::Rodrigues(R, RR);
    std::cout<<"R: "<<RR<<std::endl;
    std::cout<<"t: "<<t<<std::endl;

    std::string result_file = result_path + extrinsic_file;
    writeCalfile(result_file, RR, t);

    extrinsic_ = (cv::Mat_<double>(4,4) <<RR.at<double>(0),RR.at<double>(1),RR.at<double>(2),t.at<double>(0),
                                          RR.at<double>(3),RR.at<double>(4),RR.at<double>(5),t.at<double>(1),
                                          RR.at<double>(6),RR.at<double>(7),RR.at<double>(8),t.at<double>(2),
                                          0.0, 0.0, 0.0, 1.0);

    calibSucess = true;

    couter++;
    if(couter>=1)
      break;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "lidar_camera_calib");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  loadCalibConfig(nh);

  g_nAlphaValuesSlider = intensityth;

  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("image", on_draw, 0);
  //cv::createTrackbar("lidar 值", "image", &g_nAlphaValuesSlider, maxintensityth, on_Trackbar);
  cv::createTrackbar("捕获", "image", &g_clash, 1, on_clashbar);
  cv::createTrackbar("标定开关", "image", &g_calibClose, 1, on_calbar);

  //on_Trackbar(g_nAlphaValuesSlider, 0);
  on_calbar(g_calibClose, 0);
  on_clashbar(g_clash, 0);

  image_transport::Subscriber camera_subscriber_ = it.subscribe(camera_topic, 1, img_callback);
  points_cloud_subscriber_ = nh.subscribe(lidar_topic, 1, lidar_callback);
  right_points_cloud_subscriber_ = nh.subscribe(right_lidar_topic, 1, right_lidar_callback);  
  selected_points_cloud_subscriber_ = nh.subscribe("selected_pointcloud", 1, selected_callback);

  rgb_cloud_pub_      = nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
  init_cloud_pub_     = nh.advertise<sensor_msgs::PointCloud2>("init_cloud", 1);
  right_cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("right_init_cloud", 1);
  final_cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("final_cloud", 1);
  final_image_pub_    = nh.advertise<sensor_msgs::Image>("final_image", 1);

  solver_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&solver_callback)));

  ros::spin();
  return 0;
  
}