#pragma once

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem> // requires gcc version >= 8

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

class ParamServer {
public:
    ros::NodeHandle nh;

    std::string PROJECT_NAME;
    std::string pathKITTI;
    std::string targetSequence;
    std::vector<std::string> sequence_scan_names;
    std::vector<std::string> sequence_scan_paths;
    std::vector<std::string> sequence_label_paths;

    float VFOV;
    float HFOV;
    std::pair<float, float> FOV;

    // Omp 多线程核心数
    int kNumOmpCores;

    std::vector<float> remove_resolution_list;
    std::vector<float> revert_resolution_list;

    float rimg_color_min;
    float rimg_color_max;
    std::pair<float, float> kRangeColorAxis; // meter
    std::pair<float, float> kRangeColorAxisForDiff; // meter

    ParamServer();
};

// ##################### label ########################
extern std::unordered_map<int, std::string> label_name;
extern std::unordered_map<int, std::vector<int> > color_map;

struct SphericalPoint
{
    float az; // azimuth
    float el; // elevation
    float r; // radius
};

void initColorMap();

void readBin(const std::string &binPath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr);

void readLabels(const std::string &labelPath, std::vector<uint16_t> &labels);

void colorPointCloud(std::vector<uint16_t> &labels, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColoredPtr);

inline float rad2deg(float radians);
inline float deg2rad(float degrees);
SphericalPoint cart2sph(const pcl::PointXYZI & _cp);
std::pair<int, int> getRangeImgSize(std::pair<float, float> _fov, float _resize_ratio = 1);
cv::Mat scan2RangeImg(const pcl::PointCloud<pcl::PointXYZI>::Ptr &_scan,
                      std::pair<float, float> _fov,
                      std::pair<int, int> _rimg_size, int numOmpCores = 16);
cv::Mat scan2ColoredRangeImg(const pcl::PointCloud<pcl::PointXYZI>::Ptr &_scan,
                             std::pair<float, float> _fov,
                             std::pair<int, int> _rimg_size,
                             const std::vector<uint16_t> &labels, int numOmpCores = 16);
cv::Mat convertColorMappedImg (const cv::Mat &_src, std::pair<float, float> _caxis);
void pubRangeImg(cv::Mat& _rimg, image_transport::Publisher& _publiser, std::pair<float, float> _caxis);
void pubRangeImg(cv::Mat& _rimg, image_transport::Publisher& _publiser);
void publishPointcloud2FromPCLptr(const ros::Publisher& _scan_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _scan, std::string frameID = "map");
#endif