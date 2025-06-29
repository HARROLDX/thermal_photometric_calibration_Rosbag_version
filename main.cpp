#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>
#include "irPhotoCalib.h"
#include <cstdlib>
#include <time.h>
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <random>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;
using namespace cv;

// Config sets
const int k_frame_history = 10;
int k_div = 16;
float k_SP_threshold = 0.97;
bool k_calibrate_SP = true;
int k_max_corres_per_frame = 300;
int k_max_corres_per_history = 30;
int k_min_corres_per_history = 4;

void ReadCorrespondences(string csv_filename, 
                        vector<vector<vector<float>>>& all_intensity_history,
                        vector<vector<vector<float>>>& all_intensity_current,
                        vector<vector<int>>& all_history_frame_diffs,
                        vector<vector<vector<pair<int,int>>>>& all_pixels_history,
                        vector<vector<vector<pair<int,int>>>>& all_pixels_current,
                        vector<int>& all_isKFs) {
    all_intensity_history.clear();
    all_intensity_current.clear();
    all_history_frame_diffs.clear();
    all_pixels_history.clear();
    all_pixels_current.clear();
    all_isKFs.clear();

    ifstream c_file(csv_filename);
    if (!c_file.is_open()) {
        cerr << "Error: Could not open correspondence file: " << csv_filename << endl;
        return;
    }

    string value;
    int prev_CFID = 1;
    int columns = 10;
    int prev_HKFID = -1;
    vector<vector<float>> CFID_intensity_history, CFID_intensity_current;
    vector<vector<pair<int,int>>> CFID_pixels_history, CFID_pixels_current;
    vector<int> CFID_history_frame_diffs;
    vector<float> CFID_HKFID_intensity_history, CFID_HKFID_intensity_current;
    vector<pair<int,int>> CFID_HKFID_pixels_history, CFID_HKFID_pixels_current;
    bool done_all_lines = false;
    int x_history = 0;
    int x_current = 0;
    int CFID_is_KF = 0;

    while (c_file.good()) {
        for (int i = 0; i < columns - 1; i++) {
            getline(c_file, value, ',');
            if (value.empty()) {
                done_all_lines = true;
                break;
            }
            switch (i) {
                case 0:
                    {
                        int CFID = stoi(value);
                        if (CFID != prev_CFID) {
                            CFID_intensity_history.push_back(CFID_HKFID_intensity_history);
                            CFID_intensity_current.push_back(CFID_HKFID_intensity_current);
                            CFID_history_frame_diffs.push_back(prev_HKFID);
                            CFID_pixels_history.push_back(CFID_HKFID_pixels_history);
                            CFID_pixels_current.push_back(CFID_HKFID_pixels_current);
                            CFID_HKFID_intensity_history.clear();
                            CFID_HKFID_intensity_current.clear();
                            CFID_HKFID_pixels_history.clear();
                            CFID_HKFID_pixels_current.clear();
                            all_intensity_history.push_back(CFID_intensity_history);
                            all_intensity_current.push_back(CFID_intensity_current);
                            all_history_frame_diffs.push_back(CFID_history_frame_diffs);
                            all_pixels_history.push_back(CFID_pixels_history);
                            all_pixels_current.push_back(CFID_pixels_current);
                            all_isKFs.push_back(CFID_is_KF);
                            CFID_intensity_history.clear();
                            CFID_intensity_current.clear();
                            CFID_history_frame_diffs.clear();
                            CFID_pixels_history.clear();
                            CFID_pixels_current.clear();
                            prev_HKFID = -1;
                        }
                        prev_CFID = CFID;
                    }
                    break;
                case 1:
                    {
                        int HKFID = stoi(value);
                        if (prev_HKFID != HKFID && prev_HKFID != -1) {
                            CFID_intensity_history.push_back(CFID_HKFID_intensity_history);
                            CFID_intensity_current.push_back(CFID_HKFID_intensity_current);
                            CFID_history_frame_diffs.push_back(prev_HKFID);
                            CFID_pixels_history.push_back(CFID_HKFID_pixels_history);
                            CFID_pixels_current.push_back(CFID_HKFID_pixels_current);
                            CFID_HKFID_intensity_history.clear();
                            CFID_HKFID_intensity_current.clear();
                            CFID_HKFID_pixels_history.clear();
                            CFID_HKFID_pixels_current.clear();
                        }
                        prev_HKFID = HKFID;
                    }
                    break;
                case 2:
                    {
                        float o_history = stof(value);
                        CFID_HKFID_intensity_history.push_back(o_history);
                    }
                    break;
                case 3:
                    {
                        float o_current = stof(value);
                        CFID_HKFID_intensity_current.push_back(o_current);
                    }
                    break;
                case 4:
                    {
                        CFID_is_KF = stoi(value);
                        break;
                    }
                case 5:
                    {
                        x_history = (int)round(stof(value));
                        break;
                    }
                case 6:
                    {
                        int y_history = (int)round(stof(value));
                        CFID_HKFID_pixels_history.push_back(make_pair(x_history, y_history));
                    }
                    break;
                case 7:
                    {
                        x_current = (int)round(stof(value));
                        break;
                    }
                case 8:
                    {
                        int y_current = (int)round(stof(value));
                        CFID_HKFID_pixels_current.push_back(make_pair(x_current, y_current));
                    }
                    break;
            }
        }
        if (!done_all_lines) {
            getline(c_file, value);
        } else {
            CFID_intensity_history.push_back(CFID_HKFID_intensity_history);
            CFID_intensity_current.push_back(CFID_HKFID_intensity_current);
            CFID_history_frame_diffs.push_back(prev_HKFID);
            CFID_pixels_history.push_back(CFID_HKFID_pixels_history);
            CFID_pixels_current.push_back(CFID_HKFID_pixels_current);
            all_intensity_history.push_back(CFID_intensity_history);
            all_intensity_current.push_back(CFID_intensity_current);
            all_history_frame_diffs.push_back(CFID_history_frame_diffs);
            all_pixels_history.push_back(CFID_pixels_history);
            all_pixels_current.push_back(CFID_pixels_current);
            all_isKFs.push_back(CFID_is_KF);
            break;
        }
    }
    c_file.close();
    cout << "Loaded " << all_intensity_history.size() << " frames of correspondence data" << endl;
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg, Mat& gray_frame, vector<KeyPoint>& keypoints, Mat& descriptors) {
    Mat frame = cv::imdecode(Mat(img_msg->data), IMREAD_COLOR);
    if (frame.empty()) {
        cerr << "Error: Could not decode image at " << img_msg->header.stamp << endl;
        return;
    }
    cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    Ptr<ORB> orb = ORB::create();
    orb->detectAndCompute(gray_frame, noArray(), keypoints, descriptors);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "match_node");
    ros::NodeHandle nh;

    // 打开输入 bag 文件
    rosbag::Bag input_bag;
    string input_bag_path = "/home/xiajiawei/slam_data/NTU_thermal_loop1/NTU_thermal_loop1.bag";
    try {
        input_bag.open(input_bag_path, rosbag::bagmode::Read);
        cout << "Opened input bag: " << input_bag_path << endl;
    } catch (const rosbag::BagException& e) {
        cerr << "Error opening input bag: " << e.what() << endl;
        return -1;
    }

    // 创建输出 bag 文件
    rosbag::Bag output_bag;
    string output_bag_path = "/home/xiajiawei/slam_data/NTU_thermal_loop1/NTU_thermal_loop1_corrected.bag";
    try {
        output_bag.open(output_bag_path, rosbag::bagmode::Write);
        cout << "Created output bag: " << output_bag_path << endl;
    } catch (const rosbag::BagException& e) {
        cerr << "Error creating output bag: " << e.what() << endl;
        input_bag.close();
        return -1;
    }

    // 读取对应点
    vector<vector<vector<float>>> all_intensity_history, all_intensity_current;
    vector<vector<int>> all_history_frame_diffs;
    vector<int> all_isKFs;
    vector<vector<vector<pair<int,int>>>> all_pixels_history, all_pixels_current;
    string correspondence_path = "/home/xiajiawei/thermal_change/thermal_photometric_calibration/correspondence.txt";
    ReadCorrespondences(correspondence_path, all_intensity_history, all_intensity_current,
                       all_history_frame_diffs, all_pixels_history, all_pixels_current, all_isKFs);

    // 初始化 IRPhotoCalib
    IRPhotoCalib* calib = nullptr;
    Mat gray_frame;
    vector<KeyPoint> keypoints;
    Mat descriptors;
    int frame_counter = 0;

    // 遍历所有话题
    rosbag::View view(input_bag); 
    for (const rosbag::MessageInstance& m : view) {
        string topic = m.getTopic();

        // 处理热成像图像话题
        if (topic == "/thermal_cam/thermal_image/compressed") {
            sensor_msgs::CompressedImageConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();
            if (img_msg) {
                // 处理图像
                imageCallback(img_msg, gray_frame, keypoints, descriptors);
                if (gray_frame.empty()) {
                    cerr << "Skipping empty frame at " << img_msg->header.stamp << endl;
                    // 仍写入原始消息
                    output_bag.write(topic, img_msg->header.stamp, img_msg);
                    continue;
                }

                // 校正图像
                Mat corrected_frame;
                if (frame_counter >= 1 && frame_counter <= all_intensity_history.size()) {
                    if (!calib) {
                        cerr << "Error: calib is null at frame " << frame_counter << endl;
                        corrected_frame = gray_frame.clone();
                    } else if (all_intensity_history[frame_counter - 1].empty()) {
                        cout << "Warning: Empty correspondence for frame " << frame_counter << endl;
                        corrected_frame = gray_frame.clone();
                    } else {
                        TickMeter tm;
                        tm.start();
                        PTAB current_params = calib->ProcessCurrentFrame(
                            all_intensity_history[frame_counter - 1],
                            all_intensity_current[frame_counter - 1],
                            all_history_frame_diffs[frame_counter - 1],
                            all_pixels_history[frame_counter - 1],
                            all_pixels_current[frame_counter - 1],
                            (bool)all_isKFs[frame_counter - 1]);
                        corrected_frame = calib->getCorrectedImage(gray_frame, current_params);
                        tm.stop();
                        cout << "Processed frame " << frame_counter << " in " << tm.getTimeSec() << "s" << endl;
                    }
                } else {
                    corrected_frame = gray_frame.clone();
                    if (!calib) {
                        calib = new IRPhotoCalib(gray_frame.cols, gray_frame.rows, k_div,
                                                k_calibrate_SP, k_SP_threshold, true);
                        cout << "Initialized IRPhotoCalib for " << gray_frame.cols << "x" << gray_frame.rows << endl;
                    }
                }

                // 显示校正结果
                if (!corrected_frame.empty()) {
                    Mat res;
                    hconcat(gray_frame, corrected_frame, res);
                    imshow("Frame", res);
                    waitKey(1);
                } else {
                    cerr << "Warning: Corrected frame is empty at " << img_msg->header.stamp << endl;
                    corrected_frame = gray_frame.clone();
                }

                // 保存校正图像
                if (!corrected_frame.empty()) {
                    vector<uchar> buf;
                    vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 100};
                    cv::imencode(".jpg", corrected_frame, buf, params);

                    sensor_msgs::CompressedImage corrected_msg;
                    corrected_msg.header = img_msg->header;
                    corrected_msg.format = "jpeg";
                    corrected_msg.data.assign(buf.begin(), buf.end());

                    output_bag.write("/thermal_cam/thermal_image_corrected/compressed",
                                   corrected_msg.header.stamp, corrected_msg);
                    cout << "Saved corrected frame at " << corrected_msg.header.stamp << endl;
                }

                // 写入原始热成像消息
                output_bag.write(topic, img_msg->header.stamp, img_msg);
                frame_counter++;
            }
        } else {
            // 直接复制其他话题的消息
            output_bag.write(topic, m.getTime(), m);
        }
    }

    // 清理
    if (calib) {
        delete calib;
        calib = nullptr;
    }
    input_bag.close();
    output_bag.close();
    cv::destroyAllWindows();
    cout << "Finished processing. Output bag saved to " << output_bag_path << endl;

    return 0;
}
