#include "../../include/armor_detector/inference.h"
#include "../../include/armor_detector/ArmorDetetion.h"
#include "fstream"

ArmorDetetion::ArmorDetetion()
{
    net_ = cv::dnn::readNetFromONNX(onnx_path);
    std::ifstream label_file(label_path);
    std::string   line;
    while (std::getline(label_file, line))
    {
        class_names_.push_back(line);
    }
}

bool ArmorDetetion::FindLight(cv::Mat& src)
{
    cv::Mat src_ = src.clone();

    cv::namedWindow("armor",cv::WINDOW_NORMAL);
    cv::resizeWindow("armor",720,480);


    // 每次处理的是一帧中的图像，每次寻找前注意清空
    lights_.clear();

    std::vector<std::vector<cv::Point2i>> contours;
    std::vector<cv::Vec4i>                hierarchy;
    findContours(src_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours)
    {
        cv::RotatedRect rrect = minAreaRect(contour);
        auto            light = Light(rrect);
        //        灯条筛选
        if (IsLight(light))
        {
            lights_.push_back(light);
        }
    }

    if (lights_.size() == 0)
    {
        std::cout << "lights empty" << std::endl;
        return false;
    }
    else
    {
        for (auto light_ : lights_)
        {
            cv::Point2f light_points[4];
            light_.points(light_points);
            for (size_t i = 0; i < 4; i++)
            {
                line(this->src_, light_points[i], light_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
            }
            //imshow("light", this->src_);
//            std::cout << "light numbers: " << lights_.size() << std::endl;
        }
        return true;
    }
}

bool ArmorDetetion::IsLight(const Light& light)
{

    float ratio = light.width / light.length;
    // std::cout<<"ratio="<<ratio<<std::endl;

    // 基准比例 5.1/9=0.56
    bool ratio_ok = 0.03f < ratio && ratio < 0.65f;

    bool angle_ok = light.tilt_angle < 65.0f;
    // std::cout<<"light.tilt_angle="<<light.tilt_angle<<std::endl;
    bool is_light = ratio_ok && angle_ok;
//
//    if (1)
//    {
//        cv::Mat light_frame(640, 512, CV_8UC3, cv::Scalar(0, 0, 0));
//        putText(light_frame, "ratio :" + std::to_string(ratio), cv::Point(4, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
//        putText(light_frame, "light.tilt_angle :" + std::to_string(light.tilt_angle), cv::Point(4, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
//        imshow("light_frame",light_frame);
//    }

    return is_light;
}

bool ArmorDetetion::FindArmor(cv::Mat& src)
{
    // 每次处理的是一帧中的图像，每次寻找前注意清空
    armors_.clear();
    target_armors.clear();

    for (auto light_1 = lights_.begin(); light_1 != lights_.end(); light_1++)
    {
        for (auto light_2 = light_1 + 1; light_2 != lights_.end(); light_2++)
        {
            auto armor = Armor(*light_1, *light_2);
            if (IsArmor(armor))
            {
                armors_.emplace_back(armor);
            }
        }
    }

    if (armors_.size() == 0)
    {
        std::cout << "not find armor " << std::endl;
        return false;
    }
    else
    {
        for (size_t i = 0; i < armors_.size(); i++)
        {
            for (uint8_t j = 0; j < 4; j++)
            {
                line(this->src_, armors_[i].armor_points_[j], armors_[i].armor_points_[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
                //cv::putText(this->src_, "" + std::to_string(j), armors_[i].armor_points_[j], 4, 1, cv::Scalar(0, 255, 0));
            }
            imshow("armor", this->src_);
        }
        return true;
    }
}

bool ArmorDetetion::IsArmor(Armor& armor)
{
    Light light_1 = armor.left_light_;
    Light light_2 = armor.right_light_;

    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
    // std::cout << "light_length_ratio =" << light_length_ratio << std::endl;

    // 0.8
    bool light_ratio_ok = light_length_ratio > 0.5;
//     std::cout << "light_ratio " << light_length_ratio << std::endl;

    float avg_light_length = (light_1.length + light_2.length) / 2;

//     std::cout<<"avg_light_length ="<<avg_light_length<<std::endl;

    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;

//     std::cout<<"center_distance = "<<center_distance<<std::endl;

    // 0.5 4.0
    bool center_distance_ok = (0.5 < center_distance && center_distance < 5.0);

    cv::Point2f diff  = light_1.center - light_2.center;
    float       angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;

    // 8.0
    bool angle_ok = angle < 14.0;
//         std::cout<<"angle="<<angle<<std::endl;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    // std::cout<<"is_amor =="<<is_armor <<std::endl;

    //根据装甲板长宽比判断装甲板类型
 //   armor.type = center_distance > 3.2 ? BIG : SMALL;
    //    std::cout<<"center_distence: "<<center_distance<<std::endl;
    if (1)
//    {
//        cv::Mat armor_frame(640, 512, CV_8UC3, cv::Scalar(0, 0, 0));
//        putText(armor_frame, "light_length_ratio :" + std::to_string(light_length_ratio), cv::Point(4, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
//        putText(armor_frame, "center_distance :" + std::to_string(center_distance), cv::Point(4, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
//        putText(armor_frame, "angle :" + std::to_string(angle), cv::Point(4, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
//        imshow("armor_frame",armor_frame);
//    }
    return is_armor;
}

void ArmorDetetion::Digital_recognition(const cv::Mat& src, std::vector<Armor> armors, std::vector<Armor>& target_armors)
{

    extractNumbers(src, armors_);
    classify(armors_, target_armors);
}

// 压根用不到src
void ArmorDetetion::extractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
    cv::Mat src_show = src.clone();
    for (auto& armor : armors)
    {
        cv::Point2f affinePoints[4];
        double      indentation_left_x = 0, indentation_right_x = 0, indentation_left_y = 0, indentation_right_y = 0;
        if (armor.type == BIG)
        {
            indentation_left_x  = 0.29 * fabs(armor.armor_points_[1].x - armor.armor_points_[0].x);
            indentation_right_x = 0.29 * fabs(armor.armor_points_[1].x - armor.armor_points_[0].x);

            indentation_left_y  = 0.29 * (armor.armor_points_[2].y - armor.armor_points_[3].y);
            indentation_right_y = 0.29 * (armor.armor_points_[2].y - armor.armor_points_[3].y);
            affinePoints[0]     = cv::Point2f(armor.armor_points_[0].x + indentation_left_x, armor.armor_points_[0].y + indentation_left_y);
            affinePoints[1]     = cv::Point2f(armor.armor_points_[1].x - indentation_right_x, armor.armor_points_[1].y - indentation_right_y);
            affinePoints[2]     = cv::Point2f(armor.armor_points_[2].x - indentation_right_x, armor.armor_points_[2].y - indentation_right_y);
            affinePoints[3]     = cv::Point2f(armor.armor_points_[3].x + indentation_left_x, armor.armor_points_[3].y + indentation_left_y);
        }

        if (armor.type == SMALL)
        {
            indentation_left_x  = 0.132  * fabs(armor.armor_points_[1].x - armor.armor_points_[0].x);
            indentation_right_x = 0.132 * fabs(armor.armor_points_[1].x - armor.armor_points_[0].x);

            indentation_left_y  = 0.132 * (armor.armor_points_[2].y - armor.armor_points_[3].y);
            indentation_right_y = 0.132 * (armor.armor_points_[2].y - armor.armor_points_[3].y);
            affinePoints[0]     = cv::Point2f(armor.armor_points_[0].x + indentation_left_x, armor.armor_points_[0].y + indentation_left_y);
            affinePoints[1]     = cv::Point2f(armor.armor_points_[1].x - indentation_right_x, armor.armor_points_[1].y - indentation_right_y);
            affinePoints[2]     = cv::Point2f(armor.armor_points_[2].x - indentation_right_x, armor.armor_points_[2].y - indentation_right_y);
            affinePoints[3]     = cv::Point2f(armor.armor_points_[3].x + indentation_left_x, armor.armor_points_[3].y + indentation_left_y);
        }

        cv::Point2f affinePoints2[4] = { cv::Point2f(0, 0), cv::Point2f(48, 0), cv::Point2f(48, 48), cv::Point2f(0, 48) };

        cv::Mat affine_trans;
        //透视变换
        affine_trans = getPerspectiveTransform(affinePoints, affinePoints2);
        cv::Mat number_image;
        //仿射变换
        cv::warpPerspective(src, number_image, affine_trans, cv::Size(48, 48), cv::INTER_CUBIC);
        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        //        cv::threshold(number_image, number_image, 80, 240, cv::THRESH_BINARY);
        armor.number_img = number_image;
    }
//    std::cout << "armor numbers out: " << armors.size() << std::endl;
}

void ArmorDetetion::classify(std::vector<Armor>& armors, std::vector<Armor>& target_armors)
{

    for (auto& armor : armors)
    {
        cv::Mat image = armor.number_img.clone();
        // 先进行归一化
        image = image / 255.0;

        cv::Mat blob;
        // 20,28
        cv::dnn::blobFromImage(image, blob, 1.0, cv::Size(20, 28));

        // 输入
        net_.setInput(blob);
        // 分类预测
        cv::Mat outputs = net_.forward();

        float   max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double    confidence;
        cv::Point class_id_point;
        // 概率最高值
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number     = class_names_[label_id];
        std::stringstream result_ss;
        result_ss << armor.number << ": " << std::fixed << std::setprecision(1) << armor.confidence * 100.0 << "%";
        armor.classfication_result = result_ss.str();

        // std::cout << armor.classfication_result << std::endl;

        cv::imshow("11", armor.number_img);

        if (confidence * 100.0 > 80 & class_names_[label_id] != "negative")
        {
            target_armors.push_back(armor);
            // std::cout << "target in num: " << target_armors.size() << std::endl;
            cv::imshow("abc", armor.number_img);
        }
    }
}

cv::Point2f TargetCenter(const std::vector<cv::Point2f>& points)
{
    cv::Point2f center(0,0);
    for(const auto&point:points)
    {
        center.x+=point.x;
        center.y+=point.y;
    }
    center.x/=points.size();
    center.y/=points.size();

    return center;
}


bool ArmorDetetion::Target_Confirm(cv::Mat& src, std::vector<Armor>& target_armors)
{
    cv::Point2f center=TargetCenter(confirm_armor.armor_points);

    // std::cout << "target: " << target_armors.size() << std::endl;
    if (target_armors.size() == 0)
    {
        std::cout << "don't have armor with number" << std::endl;
        return false;
    }

    else if (target_armors.size() == 1)
    {
        this->confirm_armor = target_armors[0];

        putText(this->src_, confirm_armor.classfication_result, cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238),
                1);
        if (confirm_armor.type == SMALL)
        {
            putText(this->src_, "small_armor", cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
            for (size_t k = 0; k < 4; k++)
            {
                line(src, confirm_armor.armor_points[k], confirm_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                //cv::putText(src, "" + std::to_string(k), confirm_armor.armor_points[k], 4, 1, cv::Scalar(0, 255, 255));

            }
            cv::circle(src,center,3,cv::Scalar(0,255,0),-1);
            cv::circle(src,center,10,cv::Scalar(0,255,0),1,3);
        }
        else if (confirm_armor.type == BIG)
        {
            putText(this->src_, "big_armor", cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
            for (size_t k = 0; k < 4; k++)
            {
                line(src, confirm_armor.armor_points[k], confirm_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
            }
        }
        imshow("confirm_armor", this->src_);
        return true;
    }
    else
    {
        for (size_t i = 0; i < target_armors.size(); i++)
        {
            // this->confirm_armor = target_armors[i];

            putText(this->src_, target_armors[i].classfication_result, cv::Point2f(target_armors[i].armor_points[0].x, target_armors[i].armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(165, 155, 238), 1);
            if (target_armors[i].type == SMALL)
            {
                putText(this->src_, "small_armor", cv::Point2f(target_armors[i].armor_points[0].x, target_armors[i].armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238),
                        1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armors[i].armor_points[k], target_armors[i].armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                    //cv::putText(src, "" + std::to_string(k), target_armors[i].armor_points[k], 4, 1, cv::Scalar(0, 255, 255));
                    cv::circle(src,center,3,cv::Scalar(0,255,0),-1);
                }
            }
            else if (target_armors[i].type == BIG)
            {
                putText(this->src_, "big_armor", cv::Point2f(target_armors[i].armor_points[0].x, target_armors[i].armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armors[i].armor_points[k], target_armors[i].armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                    cv::circle(src,center,3,cv::Scalar(0,255,0),-1);
                }
            }
        }
        imshow("confirm_armor", this->src_);
        return true;
    }
}
//*************************************************************************


void Detector::setParams(const Params_ToVideo &params_to_video, const Params_ToSerialPort &params_to_serial_port)
{
        // send params to detector thread
        _detector_thread_params.frame_pp = params_to_video.frame_pp;
}



void Detector::startDetect(cv::Mat &src/*const Params_ToDecter &param, Uart *SerialPort_*/)
{
    cv::Mat drawing=cv::Mat();
    roi_accelerator=new ROIAccelerator();
    target_chooser=new TargetChooser();
    auto solver =new PoseSolver();
    auto bumper = new Bumper();                // 缓冲器
    int last_frame_class = 0;                      // 上一次选中的目标
    int this_frame_class = 0;
    bool right_clicked = false;                   // 用于检测是否右击
    bool last_right_clicked = false;

    // 声明模型
    auto armorDetector = new ArmorDetector();
//            auto greenDetector = new GreenLightDetector();
    //        auto greenDetector = new YOLOv10Detector("./model/0715_v10_416n_openvino_model/0715_v10_416n.xml", 0.20);
    std::string network_path = "/home/yaaa/rm-Hero/params/best_06_02.xml";
    armorDetector->initModel(network_path);

//    cv::namedWindow("frame",0);

    // 记录时间
    auto start=std::chrono::steady_clock::now();   // 真实使用

    TimeSystem::time_zero=std::chrono::steady_clock::now();
    GroundChassisData recv_data(SerialParam::recv_data);
    sleep(1);

    // TODO 新增
    double delta_t=0;
    bool first=false; // 操作手是否第一次右击，也就是一个上升沿

    // 前哨站中心点的二维投影坐标
    cv::Point2f center_pixel=cv::Point2f(0,0);

    const std::string target_mode_str[4] = {"NOT_GET_TARGET", "CONTINOUS_GET_TARGET", "LOST_BUMP", "DETECT_BUMP"};

    enum COLOR{I_AM_BLUE, I_AM_RED};

    while(/*!(*_detector_thread_params.frame_pp)->mat->empty()*/true)
    {
    const Image &image=**_detector_thread_params.frame_pp;

    const cv::Mat &frame=*image.mat;


    // StateParam::state = OUTPOST;
    // 默认不射击
    SerialParam::send_data.shootStatus = 0;

    // 前哨战击打模式
    std::vector<ArmorObject> objects;
    bool getTarget = false;
    std::vector<cv::Rect2f> greenObjects;
    // 是前哨战模式 或者使用固定ROI，则会进行ROI识别
    // 输出frame的尺寸

    if(abs(recv_data.roll)>500)
    {
        armorDetector->roi_outpost_xmin=ROI_OUTPOST_WITH_ROLL_XMIN;
        armorDetector->roi_outpost_ymin=ROI_OUTPOST_WITH_ROLL_YMIN;
    }
    else{
        armorDetector->roi_outpost_xmin=ROI_OUTPOST_XMIN;
        armorDetector->roi_outpost_ymin=ROI_OUTPOST_YMIN;
    }

    cv::Mat img=frame(cv::Rect(armorDetector->roi_outpost_xmin,armorDetector->roi_outpost_ymin,ROI_OUTPOST_WIDTH,ROI_OUTPOST_HEIGHT));
//    getTarget=armorDetector->detect(img,objects,true);

//    armorDetector->drawArmors(drawing,objects);

    cv::rectangle(drawing,cv::Rect(ROI_OUTPOST_XMIN,ROI_OUTPOST_YMIN,ROI_OUTPOST_WIDTH,ROI_OUTPOST_HEIGHT),cv::Scalar(0,0,255),2);

    ArmorBlobs armors;
    ArmorBlob armor_g;

    for(ArmorObject object:objects){
        // 按照点的顺序加入  顺序不一样

        armor_g.corners[0]=object.apex[0];
        armor_g.corners[1]=object.apex[3];
        armor_g.corners[2]=object.apex[2];
        armor_g.corners[3]=object.apex[1];
        armor_g.confidence=object.prob;

        //armor_g._class=classifier.predict(*image.mat,armor_g.corners,1).first;

        armor_g.rect=cv::boundingRect(armor_g.corners);

        armors.emplace_back(armor_g);
    }

    ArmorBlobs armor_targets;

    bool getCenter=false;

    for(int i=0;i<armors.size();i++)
    {
        if(armors[i]._class==7||armors[i]._class==2)
            armor_targets.emplace_back(armors[i]);
    }// 所有6号
    // 两次时间差
    delta_t=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count()/1000.0f; // m

    start=std::chrono::steady_clock::now();

    // 前哨战模式
//    center_pixel = solver->outpostMode(armor_targets, delta_t / 1000, recv_data, SerialPort_, getTarget);
    // break;

    cv::Scalar color(10,10,255);
//    putText(drawing,"center",cv::Point2f(center_pixel.x,center_pixel.y),cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(0,0,255),4,cv::LINE_8);
//    circle(drawing,center_pixel,10,color,-1);

    imshow("frame",drawing);
    cv::waitKey(1);


}
    cv::destroyAllWindows();
}

Detector::Detector()
{

}











