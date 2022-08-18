//
// Created by xjp on 2022/8/17.
//

#include "utility.h"

class Visualizer : public ParamServer {
public:
    ros::Publisher coloredCloudPublisher;
    image_transport::Publisher rangeImgPublisher;
    image_transport::Publisher coloredRangeImgPublisher;

    Visualizer();

    void run();
};

Visualizer::Visualizer() {
    coloredCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/seg_pointcloud", 1);
    image_transport::ImageTransport it(nh);
    rangeImgPublisher = it.advertise("/scan_range_image", 1);
    coloredRangeImgPublisher = it.advertise("/scan_colored_range_image", 1);

    initColorMap();
}

void Visualizer::run() {
    for (int frame = 0; ros::ok() && frame < sequence_scan_paths.size(); ++frame) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColorPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<uint16_t> labels;

        readBin(sequence_scan_paths[frame], cloudPtr);
        readLabels(sequence_label_paths[frame], labels);
        colorPointCloud(labels, cloudPtr, cloudColorPtr);

        std::pair<int, int> imgShape = getRangeImgSize(FOV, remove_resolution_list[0]);
        cv::Mat scanRangeImg = scan2RangeImg(cloudPtr, FOV, imgShape, kNumOmpCores);
        cv::Mat scanColoredRangeImg = scan2ColoredRangeImg(cloudPtr, FOV, imgShape, labels, kNumOmpCores);

        publishPointcloud2FromPCLptr(coloredCloudPublisher, cloudColorPtr, "map");
        pubRangeImg(scanRangeImg, rangeImgPublisher, kRangeColorAxis);
        pubRangeImg(scanColoredRangeImg, coloredRangeImgPublisher);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualize");

    Visualizer v;
    v.run();

    return 0;
}