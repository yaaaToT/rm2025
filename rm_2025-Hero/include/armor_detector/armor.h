
#ifndef DESIGN_ARMOR_H
#define DESIGN_ARMOR_H

#include"../debug.h"
#include "opencv2/opencv.hpp"
#include"eigen3/Eigen/Eigen"


struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width  = (cv::norm(p[0] - p[1])+cv::norm(p[2]-p[3])) / 2;

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    cv::Point2f top, bottom;
    double      length, width;
    float       tilt_angle;
};

struct Armor : public Light {

    cv::Rect_<float> rect;
            int cls;
            int color;
            float prob;
            std::vector<cv::Point2f> pts;
            int area;
            cv::Point2f apex[4];

    Armor()=default;
    Armor(const Light& l1,const Light& l2)
    {
        if(l1.center.x<l2.center.x)
        {
            left_light = l1;
            right_light = l2;
        }
        else
        {
            left_light = l2;
            right_light = l1;
        }
        center = (left_light.center + right_light.center) /2;

        // 顶点顺序：左下角为0，顺时针旋转
//        armor_points.push_back(left_light.bottom);
//        armor_points.push_back(left_light.top);
//        armor_points.push_back(right_light.top);
//        armor_points.push_back(right_light.bottom);

        armor_points.push_back(left_light.bottom);
        armor_points.push_back(left_light.top);
        armor_points.push_back(right_light.bottom);
        armor_points.push_back(right_light.top);



        //顶点顺序：左上角为0，顺时针旋转
//        armor_points.push_back(left_light.top);
//        armor_points.push_back(right_light.top);
//        armor_points.push_back(right_light.bottom);
//        armor_points.push_back(left_light.bottom);


        // 延长灯条
        left_light_= extendLight(left_light);
        right_light_= extendLight(right_light);

        cv::Point2f left_light_points[4],right_light_points[4];
        left_light_.points(left_light_points);
        right_light_.points(right_light_points);
        std::vector<cv::Point2f> left_light_points_,right_light_points_;
        sortLightPoints(left_light_points,left_light_points_);
        sortLightPoints(right_light_points,right_light_points_);

        //最后状态： 左上角为0 按顺时针排序
//        armor_points_.push_back(left_light_points_[1]);
//        armor_points_.push_back(right_light_points_[0]);
//        armor_points_.push_back(right_light_points_[3]);
//        armor_points_.push_back(left_light_points_[2]);

          armor_points_.push_back(left_light_points_[1]);
          armor_points_.push_back(right_light_points_[0]);
          armor_points_.push_back(right_light_points_[3]);
          armor_points_.push_back(left_light_points_[2]);

    }

    Light extendLight(const Light & light);
    void sortLightPoints(cv::Point2f points[4],std::vector<cv::Point2f>& new_points);
    // before
    Light                    left_light, right_light;
    cv::Point2f              center;
    std::vector<cv::Point2f> armor_points;
    // final
    Light left_light_,right_light_;
    std::vector<cv::Point2f> armor_points_;

    std::string number;      // 数字结果
    float confidence;        // 置信度
    std::string classfication_result;
    cv::Mat   number_img;       // 48*48的二值化数字图像
    ArmorType type;

    // resolver
    Eigen::Affine3d pose;
    double distance;
    double distance_to_image_center;

    int score;      // 击打评分
};


//******************************************************************************************


static bool status = false;

class ArmorBlob{
    public:
        ArmorBlob(){
            corners = std::vector<cv::Point2f>(4);
        }
        double confidence;
        cv::Rect rect;
        std::vector<cv::Point2f> corners;
        int _class;
        double angle;
        double x, y, z;
        bool is_big_armor;
        // 重载小于号，用于set，按(x,y,z)到(0,0,0)的距离排序
        bool operator < (const ArmorBlob& a) const
        {
            return x * x + y * y + z * z < a.x * a.x + a.y * a.y + a.z * a.z;
        }

    };

    typedef std::vector<ArmorBlob> ArmorBlobs;


class ArmorFinder {
    public:
        bool judgeArmor(const ArmorBlob&);
        bool matchTwoLightBar(const cv::RotatedRect&, const cv::RotatedRect&);
        bool getArmor(const cv::RotatedRect&, const cv::RotatedRect&, ArmorBlob& armor_g);
        cv::Rect getScaleArmorToRoi(const cv::Rect&);
        std::vector<int> getExtreme(const ArmorBlob&);
    private:
        inline float getAngle(const cv::RotatedRect&);
        inline bool checkAngleDiff(const cv::RotatedRect& l, const cv::RotatedRect& r);
        inline bool checkHeightDiff(const cv::RotatedRect& l, const cv::RotatedRect& r);
        inline bool checkHeightMatch(const cv::RotatedRect& l, const cv::RotatedRect& r);
        inline bool checkHorizontalDistance(const cv::RotatedRect& l, const cv::RotatedRect& r);
        inline bool checkDislocation(const cv::RotatedRect& l, const cv::RotatedRect& r);

    };


#endif //DESIGN_ARMOR_H
