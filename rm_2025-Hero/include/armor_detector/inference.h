#ifndef INFERENCE_H
#define INFERENCE_H

//c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

//openvino
#include <openvino/openvino.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Core>

#include "armor.h"
#include "ArmorDetetion.h"

#define ROI_OUTPOST_XMIN 542
#define ROI_OUTPOST_YMIN 588
#define ROI_OUTPOST_WIDTH 416
#define ROI_OUTPOST_HEIGHT 416

#define ROI_OUTPOST_WITH_ROLL_XMIN 632
#define ROI_OUTPOST_WITH_ROLL_YMIN 608

// ArmorObject
    struct ArmorObject
    {
        //ArmorObject() {};
        //ArmorObject(const Armor& target_armors) {}
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
        int area;
        cv::Point2f apex[4];
        ArmorObject()
        {

        }
        std::vector<cv::Point2f>object_points;

        cv::Point2f center;

        // resolver
        Eigen::Affine3d pose;
        double distance;
        double distance_to_target_center;

    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    class ArmorDetector
    {
    public:
        int roi_outpost_xmin = ROI_OUTPOST_XMIN;
        int roi_outpost_ymin = ROI_OUTPOST_YMIN;
        ArmorDetector();
//        ~ArmorDetector();
        bool detect(cv::Mat &src, std::vector<ArmorObject>& objects, bool use_roi=false);

        bool initModel(std::string path);
        // vis: draw for debug
        void drawArmors(cv::Mat &drawing, std::vector<ArmorObject>& objects);

        cv::Point2f TargetCenter(const cv::Point2f points[]);

        cv::Point2f FourPoints(cv::Point2f points[]);

        cv::Point2f center;

        std::vector<ArmorObject> objects;

    private:
        int dw, dh;
        float rescale_ratio;



        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;

        std::string input_name;
        std::string output_name;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;
    };


#endif // INFERENCE_H
