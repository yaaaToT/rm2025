
#ifndef DESIGN_TRACKER_H
#define DESIGN_TRACKER_H
#include "../outpost/params.h"
#include "../kalmanfilter/KalmanFilter.h"
#include "eigen3/Eigen/Dense"
#include "Eigen/SVD"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <vector>
#include "opencv2/opencv.hpp"

class Trackers {
public:
    void TrackJudge(cv::Mat& src);

    int armor_lost_count;       //装甲板丢失帧数
    int armor_drop_count;       //装甲板掉帧帧数
    int armor_found_count;      //装甲板发现帧数

    float last_send_yaw;
    float last_send_pitch;
    float last_send_distance;

    int tracker_count;          //跟踪计数


    bool if_track;              // 是否跟踪标志
    bool if_armors_found;       // 是否发现装甲板
    bool if_first_frame;        // 是否为第一帧


};




#endif //DESIGN_TRACKER_H
