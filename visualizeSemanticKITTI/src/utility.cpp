#include "utility.h"

ParamServer::ParamServer() {
    nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "visualizeSemanticKITTI");
    nh.param<std::string>(PROJECT_NAME + "/pathKITTI", pathKITTI, "/media/xjp/T7/KITTI/sequences/");
    nh.param<std::string>(PROJECT_NAME + "/targetSequence", targetSequence, "00");

    for (auto &_entry: std::filesystem::directory_iterator(pathKITTI + targetSequence + "/velodyne/")) {
        sequence_scan_names.emplace_back(_entry.path().filename());
        sequence_scan_paths.emplace_back(_entry.path());
    }
    for (auto &_entry: std::filesystem::directory_iterator(pathKITTI + targetSequence + "/labels/")) {
        sequence_label_paths.emplace_back(_entry.path());
    }
    std::sort(sequence_scan_names.begin(), sequence_scan_names.end());
    std::sort(sequence_scan_paths.begin(), sequence_scan_paths.end());
    std::sort(sequence_label_paths.begin(), sequence_label_paths.end());
    ROS_INFO_STREAM("\033[1;32m Total : " << sequence_scan_paths.size() << " scans in the directory.\033[0m");

    // fov
    nh.param<float>(PROJECT_NAME + "/vfov", VFOV, 50.0);
    nh.param<float>(PROJECT_NAME + "/hfov", HFOV, 360.0);
    FOV = std::pair<float, float>(VFOV, HFOV);

    // faster
    nh.param<int>(PROJECT_NAME + "/num_omp_cores", kNumOmpCores, 4);

    // resolution
    nh.param<std::vector<float>>(PROJECT_NAME + "/remove_resolution_list", remove_resolution_list,
                                 std::vector<float>());
    nh.param<std::vector<float>>(PROJECT_NAME + "/revert_resolution_list", revert_resolution_list,
                                 std::vector<float>());

    nh.param<float>("removert/rimg_color_min", rimg_color_min, 0.0);
    nh.param<float>("removert/rimg_color_max", rimg_color_max, 10.0);
    kRangeColorAxis = std::pair<float, float>{rimg_color_min, rimg_color_max}; // meter
    kRangeColorAxisForDiff = std::pair<float, float>{0.0, 0.5}; // meter

}

// ##################### label ########################
std::unordered_map<int, std::string> label_name;
std::unordered_map<int, std::vector<int> > color_map;

/*!
 * @brief load color map of semanticKITTI
 */
void initColorMap() {
    label_name[0] = "unlabeled";
    label_name[1] = "outlier";
    label_name[10] = "car";
    label_name[11] = "bicycle";
    label_name[13] = "bus";
    label_name[15] = "motorcycle";
    label_name[16] = "on-rails";
    label_name[18] = "truck";
    label_name[20] = "other-vehicle";
    label_name[30] = "person";
    label_name[31] = "bicyclist";
    label_name[32] = "motorcyclist";
    label_name[40] = "road";
    label_name[44] = "parking";
    label_name[48] = "sidewalk";
    label_name[49] = "other-ground";
    label_name[50] = "building";
    label_name[51] = "fence";
    label_name[52] = "other-structure";
    label_name[60] = "lane-marking";
    label_name[70] = "vegetation";
    label_name[71] = "trunk";
    label_name[72] = "terrain";
    label_name[80] = "pole";
    label_name[81] = "traffic-sign";
    label_name[99] = "other-object";
    label_name[252] = "moving-car";
    label_name[253] = "moving-bicyclist";
    label_name[254] = "moving-person";
    label_name[255] = "moving-motorcyclist";
    label_name[256] = "moving-on-rails";
    label_name[257] = "moving-bus";
    label_name[258] = "moving-truck";
    label_name[259] = "moving-other-vehicle";


    color_map[0] = {0, 0, 0};
    color_map[1] = {0, 0, 255};
    color_map[10] = {245, 150, 100};
    color_map[11] = {245, 230, 100};
    color_map[13] = {250, 80, 100};
    color_map[15] = {150, 60, 30};
    color_map[16] = {255, 0, 0};
    color_map[18] = {180, 30, 80};
    color_map[20] = {255, 0, 0};
    color_map[30] = {30, 30, 255};
    color_map[31] = {200, 40, 255};
    color_map[32] = {90, 30, 150};
    color_map[40] = {255, 0, 255};
    color_map[44] = {255, 150, 255};
    color_map[48] = {75, 0, 75};
    color_map[49] = {75, 0, 175};
    color_map[50] = {0, 200, 255};
    color_map[51] = {50, 120, 255};
    color_map[52] = {0, 150, 255};
    color_map[60] = {170, 255, 150};
    color_map[70] = {0, 175, 0};
    color_map[71] = {0, 60, 135};
    color_map[72] = {80, 240, 150};
    color_map[80] = {150, 240, 255};
    color_map[81] = {0, 0, 255};
    color_map[99] = {255, 255, 50};
    color_map[252] = {245, 150, 100};
    color_map[256] = {255, 0, 0};
    color_map[253] = {200, 40, 255};
    color_map[254] = {30, 30, 255};
    color_map[255] = {90, 30, 150};
    color_map[257] = {250, 80, 100};
    color_map[258] = {180, 30, 80};
    color_map[259] = {255, 0, 0};
}

/*!
 * @brief read Bin files in KITTI sequence
 * @param binPath
 * @param cloudPtr
 */
void readBin(const std::string &binPath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr) {
    std::fstream input(binPath.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << binPath << '\n';
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int ii = 0; input.good() && !input.eof(); ii++) {
        pcl::PointXYZI point;

        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));

        cloudPtr->push_back(point);
    }
    input.close();
}

/*!
 * @brief read label of each points
 * @param labelPath
 * @param labels
 */
void readLabels(const std::string &labelPath, std::vector<uint16_t> &labels) {
    std::fstream input(labelPath.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << labelPath << '\n';
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    for (int i = 0; input.good() && !input.eof(); i++) {
        uint32_t data;
        input.read(reinterpret_cast<char *>(&data), sizeof(data));
        uint16_t label = data & 0xFFFF;
        uint16_t instance = data >> 16;
        labels.push_back(label);
    }
    input.close();
}

/*!
 * @brief color the point cloud according to the color map
 * @param labels
 * @param cloudPtr
 * @param cloudColoredPtr
 */
void colorPointCloud(std::vector<uint16_t> &labels, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColoredPtr) {
    for (int i = 0; i < labels.size(); ++i) {
        pcl::PointXYZRGB p;
        p.x = cloudPtr->points[i].x;
        p.y = cloudPtr->points[i].y;
        p.z = cloudPtr->points[i].z;
        p.r = color_map[labels[i]][2];
        p.g = color_map[labels[i]][1];
        p.b = color_map[labels[i]][0];
        cloudColoredPtr->points.push_back(p);
    }
}


inline float rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}

inline float deg2rad(float degrees) {
    return degrees * M_PI / 180.0;
}

/*!
 * @brief 根据FOV和缩放计算range image 的尺寸
 * @param _fov
 * @param _resize_ratio
 * @return
 */
std::pair<int, int> getRangeImgSize(const std::pair<float, float> _fov, const float _resize_ratio) {
    float V_FOV = _fov.first;
    float H_FOV = _fov.second;

    int NUM_RANGE_IMG_ROW = std::round(V_FOV * _resize_ratio);
    int NUM_RANGE_IMG_COL = std::round(H_FOV * _resize_ratio);

    std::pair<int, int> imgShape{NUM_RANGE_IMG_ROW, NUM_RANGE_IMG_COL};
    return imgShape;
}

/*!
 * @brief 投影到球面坐标系
 * @param _cp
 * @return
 */
SphericalPoint cart2sph(const pcl::PointXYZI &_cp) { // _cp means cartesian point

    //vstd::cout << "Cartesian Point [x, y, z]: [" << _cp.x << ", " << _cp.y << ", " << _cp.z << '\n';

    SphericalPoint sph_point{
            std::atan2(_cp.y, _cp.x),
            std::atan2(_cp.z, std::sqrt(_cp.x * _cp.x + _cp.y * _cp.y)),
            std::sqrt(_cp.x * _cp.x + _cp.y * _cp.y + _cp.z * _cp.z)
    };
    return sph_point;
}

/*!
 * @brief scan to range image
 * @param _scan
 * @param _fov  e.g., [vfov = 50 (upper 25, lower 25), hfov = 360]
 * @param _rimg_size
 * @return
 */
cv::Mat scan2RangeImg(const pcl::PointCloud<pcl::PointXYZI>::Ptr &_scan,
                      const std::pair<float, float> _fov,
                      const std::pair<int, int> _rimg_size, int numOmpCores) {
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;

    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;
    // cout << "rimg size is: [" << _rimg_size.first << ", " << _rimg_size.second << "]." << endl;

    // @ range image initizliation
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, 10000); // float matrix

    // @ points to range img
    int num_points = _scan->points.size();

    // openmp 多线程并行运算
#pragma omp parallel for num_threads(numOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx) {
        pcl::PointXYZI this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks.
        int lower_bound_row_idx{0};
        int lower_bound_col_idx{0};
        int upper_bound_row_idx{kNumRimgRow - 1};
        int upper_bound_col_idx{kNumRimgCol - 1};
        // 找到每个点在range image中的像素位置
        int pixel_idx_row = int(std::min(std::max(
                std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))),
                float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(
                std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))),
                float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section),
        // @ But, the resulting range image is acceptable (watching via Rviz),
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them,
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel,
        // #               so we can assume practically there are few race conditions.
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        if (curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col)) { // 取最近的range
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
        }
    }

    return rimg;
}

/*!
 * @brief scan to colored range image
 * @param _scan
 * @param _fov  e.g., [vfov = 50 (upper 25, lower 25), hfov = 360]
 * @param _rimg_size
 * @return
 */
cv::Mat scan2ColoredRangeImg(const pcl::PointCloud<pcl::PointXYZI>::Ptr &_scan,
                             const std::pair<float, float> _fov,
                             const std::pair<int, int> _rimg_size,
                             const std::vector<uint16_t> &labels, int numOmpCores) {
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;

    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;
    // cout << "rimg size is: [" << _rimg_size.first << ", " << _rimg_size.second << "]." << endl;

    // @ range image initizliation
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_8UC3, cv::Scalar::all(255)); // float matrix

    // @ points to range img
    int num_points = _scan->points.size();

    // openmp 多线程并行运算
#pragma omp parallel for num_threads(numOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx) {
        pcl::PointXYZI this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks.
        int lower_bound_row_idx{0};
        int lower_bound_col_idx{0};
        int upper_bound_row_idx{kNumRimgRow - 1};
        int upper_bound_col_idx{kNumRimgCol - 1};
        // 找到每个点在range image中的像素位置
        int pixel_idx_row = int(std::min(std::max(
                std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV / float(2.0))) / (kVFOV - float(0.0)))),
                float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(
                std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV / float(2.0))) / (kHFOV - float(0.0)))),
                float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section),
        // @ But, the resulting range image is acceptable (watching via Rviz),
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them,
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel,
        // #               so we can assume practically there are few race conditions.
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        rimg.at<cv::Vec3b>(pixel_idx_row, pixel_idx_col)[0] = color_map[labels[pt_idx]][0];
        rimg.at<cv::Vec3b>(pixel_idx_row, pixel_idx_col)[1] = color_map[labels[pt_idx]][1];
        rimg.at<cv::Vec3b>(pixel_idx_row, pixel_idx_col)[2] = color_map[labels[pt_idx]][2];
    }

    return rimg;
}

/*!
 * @brief 将float深度值转化为颜色
 * @param _src
 * @param _caxis
 * @return
 */
cv::Mat convertColorMappedImg(const cv::Mat &_src, std::pair<float, float> _caxis) {
    float min_color_val = _caxis.first;
    float max_color_val = _caxis.second;

    cv::Mat image_dst;
    image_dst = 255 * (_src - min_color_val) / (max_color_val - min_color_val);
    image_dst.convertTo(image_dst, CV_8UC1);

    cv::applyColorMap(image_dst, image_dst, cv::COLORMAP_JET);

    return image_dst;
}

/*!
 * @brief 将PCL点云转换成Pointcloud2并发布
 * @param _scan_publisher
 * @param _scan
 * @param frameID
 */
void
publishPointcloud2FromPCLptr(const ros::Publisher &_scan_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _scan,
                             std::string frameID) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*_scan, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = frameID;
    _scan_publisher.publish(tempCloud);
}

/*!
 * @brief 将range image可视化转换格式并发布
 * @param _rimg
 * @param _publiser
 * @param _caxis
 */
void pubRangeImg(cv::Mat &_rimg,
                 image_transport::Publisher &_publiser,
                 std::pair<float, float> _caxis) {
    cv::Mat scan_rimg_viz = convertColorMappedImg(_rimg, _caxis);
    sensor_msgs::ImagePtr rangeImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", scan_rimg_viz).toImageMsg();
    _publiser.publish(rangeImgMsg);
}

void pubRangeImg(cv::Mat& _rimg, image_transport::Publisher& _publiser){
    sensor_msgs::ImagePtr rangeImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _rimg).toImageMsg();
    _publiser.publish(rangeImgMsg);
}