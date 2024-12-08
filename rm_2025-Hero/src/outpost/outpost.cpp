#include "../../include/armor_detector/armor.h"
#include "../../include/outpost/outpost.h"
#include "../../include/outpost/params.h"
#include "../../include/Tracker/Predictor.h"
#include "../../include/SerialPort/SerialPort.h"

#include<iostream>
#include<thread>
#include "opencv2/opencv.hpp"

// BA优化的状态量
Eigen::Matrix<double,8,1>angle_state_vector;

PoseSolver::PoseSolver(){

    predictor=new Predictor();  //创建一个预测器

    send_shoot_time=std::chrono::steady_clock::now();

    // 设置相机内外参数
    setCameraMatrix(CameraParam::fx,CameraParam::fy,CameraParam::u0,CameraParam::v0);

    //设置相机畸变
    setDistortionCoefficients(CameraParam::k1,CameraParam::k2,CameraParam::p1,CameraParam::p2,CameraParam::k3);

    //云台坐标系在世界坐标系下的位姿
    gimbal_to_world=Sophus::SE3d(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));

    // 老车这里是5
    double DEG_TO_ARC = 0.0174532925199433;

    double degree_yaw = CameraParam::yaw;
    double degree_pitch = CameraParam::pitch;
    double degree_roll = CameraParam::roll;

    Eigen::Vector3d euler_angle(degree_pitch * DEG_TO_ARC, degree_roll * DEG_TO_ARC, degree_yaw * DEG_TO_ARC);
    Eigen::Matrix3d rotation_matrix;

    // 固定的旋转角度，旋转固定的角度
    //roll
    rotation_matrix=Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY());
    camera_to_gimbal=Sophus::SE3d(rotation_matrix,Eigen::Vector3d(CameraParam::camera_trans_x,CameraParam::camera_trans_y,CameraParam::camera_trans_z));
}

//设置相机内参

void PoseSolver::setCameraMatrix(double fx, double fy, double u0, double v0){
    camera_matrix=cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0]=fx;
    camera_matrix.ptr<double>(0)[2]=u0;
    camera_matrix.ptr<double>(1)[1]=fy;
    camera_matrix.ptr<double>(1)[2]=v0;
    camera_matrix.ptr<double>(2)[2]=1.0f;
}

// 设置畸变系数矩阵
void PoseSolver::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3){
    distortion_coefficients=cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0]=k_1;
    distortion_coefficients.ptr<double>(1)[0]=k_2;
    distortion_coefficients.ptr<double>(2)[0]=p_1;
    distortion_coefficients.ptr<double>(3)[0]=p_2;
    distortion_coefficients.ptr<double>(4)[0]=k_3;
}

void PoseSolver::setimu(float pitch, float yaw, float roll){
    Eigen::Matrix3d rotation_matrix3;

    //云台坐标系 在 世界坐标系 中的位姿

    rotation_matrix3=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
                     Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitX())*
                     Eigen::AngleAxisd(-roll,Eigen::Vector3d::UnitY());
    gimbal_to_world=Sophus::SE3(rotation_matrix3,Eigen::Vector3d(0,0,0));
}

float PoseSolver::cosineLaw(float a, float b, float c){
    double value = fmin(fmax((a*a+b*b-c*c)/(2*a*b),-1),1);
    return acos(value)/M_PI*180;
}

// 更新时间
void PoseSolver::update_delta_t(double &delta_time){
    delta_t=delta_time;
}

/**
 * @brief
 *
 * @param armor 指定的要解算位姿的装甲板
 * @param imu_data 当前的imu数据，包括pitch、yaw、roll，都是相对于世界坐标系的数据
 *                 pitch、roll数据是根据实际得到的，yaw轴数据是相当于初始上电时的基准来的
 *                 pitch 当前云台与水平面的夹角
 */

Sophus::SE3d PoseSolver::solveArmor(ArmorBlob&armor,const GroundChassisData &imu_data){
    // 装甲板的位姿
    const static Sophus::SE3d armor_pose(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));

    double width=sqrt((armor.corners[0].x-armor.corners[1].x)*(armor.corners[0].x-armor.corners[1].x)+
            (armor.corners[0].y-armor.corners[1].y)*(armor.corners[0].y-armor.corners[1].y));
    double height=sqrt((armor.corners[2].x-armor.corners[1].x)*(armor.corners[2].x-armor.corners[1].x)+
            (armor.corners[2].y-armor.corners[1].y)*(armor.corners[2].y-armor.corners[1].y));
    std::vector<cv::Point2f>corners_center;

    for(int i=0;i<4;i++){
        corners_center.push_back((armor.corners[i]+armor.corners[(i+1)%4])/2);
    }
    double ratio=width/height;
    if(armor._class==1||armor._class==GlobalParam::BIGID||(StateParam::state==AUTOAIM&&armor._class==6&&0))
    { // 或者等于基地大装甲
        // large armor
        armor.is_big_armor=true;
        cv::solvePnP(points_large_3d,armor.corners,camera_matrix,distortion_coefficients,
                     rvec,tvec,false,cv::SOLVEPNP_IPPE_SQUARE);
    }else{
        armor.is_big_armor=false;
        //small armor
        cv::solvePnP(points_small_3d,armor.corners,camera_matrix,distortion_coefficients,
                     rvec,tvec,false,cv::SOLVEPNP_IPPE_SQUARE);
    }
    Eigen::Matrix3d R;
    R<<1,0,0,
       0,0,1,
       0,-1,0;
    Sophus::SE3d transRight(R,Sophus::Vector3d(0,0,0));
    cv2eigen(tvec,e_T); // 平移向量没有问题

    // 旋转
    Rodrigues(rvec,m_R);  // 旋转顺序没有改变
    cv2eigen(m_R,e_R);

    yaw=atan2(-m_R.at<double>(2,0),sqrt(pow(m_R.at<double>(2,1),2)+pow(m_R.at<double>(2,2),2)))/M_PI*180;
    armor.angle=abs(yaw);

    armor_to_camera=transRight*Sophus::SE3d(e_R,e_T);

    float rcv_yaw=imu_data.yaw/100.0f*M_PI/180.0f;
    float rcv_pitch=imu_data.pitch/100.0f*M_PI/180.0f;
    float rcv_roll=imu_data.roll/100.0f*M_PI/180.0f;

    setimu(rcv_pitch,rcv_yaw,rcv_roll);

    // 从 相机坐标系坐标 转换成 世界坐标系 坐标
    camera_to_world=gimbal_to_world*camera_to_gimbal;
    // 从 装甲板坐标 转换到 云台坐标系 坐标
    armor_to_gimbal=camera_to_gimbal*armor_to_camera;
    // 从 装甲板坐标 转换到 世界坐标系 坐标
    armor_to_world=camera_to_world*armor_to_camera;

    Eigen::Vector3d z=armor_to_world*Eigen::Vector3d(0,0,1); // 世界坐标系下z轴

    double armor_yaw=atan2(z[1],z[0])/M_PI*180;

    //装甲板在云台坐标系下的坐标
    const auto &a=armor_to_gimbal;
    cv::Point3d t={a.translation()[0],
                   a.translation()[1],
                   a.translation()[2]};
    // 更换坐标系之后的坐标
    cv::Point3d trans={armor_to_world.translation()[0],
                       armor_to_world.translation()[1],
                       armor_to_world.translation()[2]};
    // 世界坐标系中的坐标
    armor.x=trans.x; // 装甲板的坐标定义
    armor.y=trans.y;
    armor.z=trans.z;

    return armor_to_world;

}

static int offset_x=432+416/2;
static int offset_y=408+416/2;

GimbalPose PoseSolver::solveGimbalPose(cv::Point3d shootTarget){
    shootTarget.y=(shootTarget.y==0?1e-6:shootTarget.y);
    GimbalPose gimbalPose;
    gimbalPose.yaw=atan(-shootTarget.x/shootTarget.y);
    double distance=sqrt(shootTarget.x*shootTarget.x+shootTarget.y*shootTarget.y); // 此处为平面距离
    gimbalPose.pitch=atan(shootTarget.z/distance);

    double theta=gimbalPose.pitch;
    double delta_z;
    double k1=0.47*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
    double center_distance=distance;  //距离
    double gravity=9.8;
    double flyTime;
    for(int i=0;i<100;i++)
    {
        // 计算炮弹的飞行时间
        flyTime=(pow(2.718281828,k1*center_distance)-1)/(k1*speed*cos(theta));
        delta_z=shootTarget.z-speed*sin(theta)*flyTime/cos(theta)+
                0.5*gravity*flyTime*flyTime/cos(theta)/cos(theta);
        if(fabs(delta_z)<0.000001)
            break;
        theta-=delta_z/(-(speed*flyTime)/pow(cos(theta),2)+
                        gravity*flyTime*flyTime/(speed*speed)*sin(theta)/pow(cos(theta),3));
    }

    gimbalPose.pitch=(theta/M_PI*180)*100;
    return gimbalPose;

}

bool PoseSolver::getPoseInCamera(std::vector<ArmorBlob> &armors, double delta_t, const GroundChassisData imu_data,
                                 Uart *SerialPort_, int &this_frame_class, int &last_frame_class){
    static int last_pitch,last_yaw;

    if(armors.size()<1)
        return false;   //没有目标

    for(ArmorBlob &a:armors)
        solveArmor(a,imu_data);
    ArmorBlobs candidates;

    right_clicked=imu_data.right_clicked; // 接收到发送的右击数据，右击重置中心装甲板
    if(last_right_clicked==0&&right_clicked==1)
        first=true;
    if(first){ // 按下右键时瞄准中心装甲
        first=false;
        // 按照距离中心的距离来进行排序，找到最靠近中心的装甲板
        if(StateParam::state==AUTOAIM_WITH_ROI){
            // 远距离
            sort(armors.begin(),armors.end(),[](const ArmorBlob &a,const ArmorBlob &b)->bool{
                const cv::Rect &r1=a.rect;
                const cv::Rect &r2=b.rect;
                return abs(r1.x + r1.y + r1.height / 2 + r1.width / 2 - offset_x - offset_y) <
                        abs(r2.x + r2.height / 2 - offset_x + r2.y + r2.width / 2 - 1280 / 2 - offset_y);
            });
        }else {
            // 近距离辅瞄
            sort(armors.begin(),armors.end(),[](const ArmorBlob &a,const ArmorBlob &b)->bool{
                const cv::Rect &r1=a.rect;
                const cv::Rect &r2=b.rect;
                return abs(r1.x+r1.y+r1.height/2+r1.width/2-1024/2-1280/2)<
                       abs(r2.x+r2.height/2-1024/2+r2.y+r2.width/2-1280/2);
            });
        }
        // 找出距离中心最近的装甲板
        armor=armors.at(0);
        // 最中心的装甲板优先级最高，操作手瞄准的是最高优先级的装甲板 top_pri 就是操作手开启辅瞄时指定的跟踪目标(最靠近中心的部分)
        top_pri=armor._class;   // 设定优先级
     }

    //中心最近的装甲板
    int target=chooseArmor(armors);

    // 选中的目标
    SerialParam::send_data.num=target; // 当前辅瞄选定的目标

    //筛选出 优先级最高的 类别
    for(const auto &a:armors){
        if(a._class==target)
            candidates.push_back(a);
    }

    // 没有符合的装甲板，可能是调帧，减少错误
    if(candidates.size()<1)
        return false;

    if(candidates.size()>1){
        sort(candidates.begin(),candidates.end(),[&](const ArmorBlob &a,const ArmorBlob &b) {
            return calcDiff(a,last_armor)<calcDiff(b,last_armor);
        });
    }
    // 选择和上一帧识别的装甲板距离最近的那一个装甲板
    armor=candidates.at(0);

    // candidates 都是装甲板和上一帧锁定的装甲板类别一致的
    // 都更新 last_armor
    if(target==last_armor._class){
        // 按照和上一帧出现的装甲板距离进行降序排序(相当于一个追踪的效果)

        // 调帧缓冲
        if(lost_cnt<20&&calcDiff(armor,last_armor)>0.3){
            // 如果距离太远就认为进入掉帧，掉帧之后仍选取上一次的位置作为当前位置
            armor=last_armor;
            lost_cnt++;
        }else{
            lost_cnt=0;
            armor=candidates.at(0);     // 更新最近的
        }

        bool has_two_armor=false;
        cv::Point3d armor1=cv::Point3d(candidates[0].x,candidates[0].y,candidates[0].z);
        cv::Point3d armor2=cv::Point3d(0,0,0);
        if(candidates.size()>=2){
            has_two_armor=true;
            armor2=cv::Point3d(candidates[1].x,candidates[1].y,candidates[1].z);
        }

        float delta_time=delta_t; // 类型转换
        Angle_t tmp_angle;
        if(false){

            tmp_angle=predictor->Predict(armor1,armor2,has_two_armor,0,imu_data,delta_time);
            // pitch 已经计算过，增加补偿即可，yaw要重新计算

            double tmp_pitch=0;
            // 得到的pitch和yaw都是角度
            // 5m左右

            if(tmp_angle.distance<4)                   //小于4m
                tmp_pitch=(tmp_angle.pitch+1)*180+120;
            else if(tmp_angle.distance<5.5)            // 小于5.5m
                tmp_pitch=(tmp_angle.pitch+1)*180+140;
            else                                       // 大于5.5m,6m以上
                tmp_pitch=(tmp_angle.pitch+1)*180+150;

            SerialParam::send_data.pitch=tmp_pitch;

            double tmp_yaw=tmp_angle.yaw*100;
            tmp_yaw+=-0.0*100;          // TODO: 进行补偿
            while(abs(tmp_yaw-imu_data.yaw)>9000){
                if(tmp_yaw-imu_data.yaw>=9000)
                    tmp_yaw-=18000;
                else
                    tmp_yaw+=18000;
            }
            SerialParam::send_data.yaw=tmp_yaw;

            SerialParam::send_data.shootStatus=1;

            last_armor=armor; // 上一次识别到的armor
            return true;      // 返回跟踪
        }

        // 没有预测，只有跟随
        armor1.y=(armor1.y==0?1e-6:armor1.y);
        tmp_angle.yaw=atan(-armor1.x/armor.y);
        tmp_angle.distance=sqrt(armor1.x*armor1.x+armor1.y*armor1.y+armor1.z*armor1.z);
        tmp_angle.pitch=atan(armor1.z/tmp_angle.distance);

        double theta=tmp_angle.pitch;
        double delta_z;
        double k1_c=0.47;

        double k1=k1_c*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
        double center_distance=tmp_angle.distance;    // 距离
        if(candidates[0].is_big_armor) center_distance=center_distance/1.08;
        double gravity=9.8;
        for(int i=0;i<100;i++){
            // 计算炮弹飞行时间
            double t=(pow(2.718281828,k1*center_distance)-1)/(k1*speed*cos(theta));
            delta_z=armor1.z-speed*sin(theta)*t/cos(theta)+
                    0.5*gravity*t*t/cos(theta)/cos(theta);
            if(fabs(delta_z)<0.000001)
                break;
            theta-=delta_z/(-(speed*t)/pow(cos(theta),2)+
                            gravity*t*t/(speed*speed)*sin(theta)/pow(cos(theta),3));

        }
        double tmp_pitch=(theta/M_PI*180)*180;

        if(GlobalParam::SOCKET){
            outpostPoseDataFrame.x=imu_data.yaw/100.;
            outpostPoseDataFrame.y=tmp_angle.distance;
            outpostPoseDataFrame.z=SerialParam::recv_data.yaw/100.;

            outpostPoseDataFrame.pitch=armor.angle;
            outpostPoseDataFrame.yaw=imu_data.yaw;
            outpostPoseDataFrame.roll=imu_data.roll;

            udpsender->send(outpostPoseDataFrame);
        }

        SerialParam::send_data.shootStatus=1; // 实际击打

        SerialParam::send_data.pitch=tmp_pitch;

        double tmp_yaw=tmp_angle.yaw/M_PI*180*100;
        tmp_yaw+=0*100;         // TODO: 进行补偿
        while(abs(tmp_yaw-imu_data.yaw)>9000){
            if(tmp_yaw-imu_data.yaw>=9000)
                tmp_yaw-=18000;
            else
                tmp_yaw+=18000;
        }
        SerialParam::send_data.yaw=tmp_yaw;

        double filter_ratio_yaw=0.3;
        double filter_ratio_pitch=0.7;

        bool useFilter=!first;
        if(abs(SerialParam::send_data.yaw-last_yaw)>300||abs(SerialParam::send_data.pitch-last_pitch)>100){
            useFilter=false;
        }
        if(useFilter){
            SerialParam::send_data.yaw=last_yaw*filter_ratio_yaw+SerialParam::send_data.yaw*(1-filter_ratio_yaw);
            SerialParam::send_data.pitch=last_pitch*filter_ratio_pitch+SerialParam::send_data.pitch*(1-filter_ratio_pitch);
        }
        last_yaw=SerialParam::send_data.yaw;
        last_pitch=SerialParam::send_data.pitch;
        last_armor=armor; // 上一次识别到的armor
        return true;
    }else{
        // 掉帧缓冲
        if(lost_cnt<20)
            lost_cnt++;
        else{
            lost_cnt=0;
            predictor->resetPredictor();
        }
        last_armor=armor; // 上一次识别到的armor
        first=false;
        return false;
    }


}


cv::Point2f PoseSolver::antiTop(std::vector<ArmorBlob> &armors, double delta_t, const GroundChassisData &imu_data,
                                                            Uart *SerialPort_, bool &getCenter){
    //DLOG(WARNING) << "  >>>>>>>>>>>>>>>            IN ANTITOP            <<<<<<<<<<<<<<<<" << std::endl;
    static int frameCount=0;
    static int lostInShootZone=0;
    getCenter=false;
    for(ArmorBlob &a:armors)
        solveArmor(a,imu_data);

    ArmorBlobs candidates;
    if(right_clicked == 0) {
                getCenter = false;
                last_right_clicked = right_clicked;
                return cv::Point2f (0,0);
            }
    if (last_right_clicked == 0 && right_clicked == 1)
                first = true;
    last_right_clicked = right_clicked;
    if (first) { // 只要第一次按右键就清空
                first = false;
                armors_set.clear();
                //DLOG(WARNING) << ">>>>>>>>>>>init ANTITOP<<<<<<<<<<<<<" << std::endl;
                timeInZone.clear();
                time.clear();
                getCenter = false;
                return cv::Point2f(0,0);
            }
    if (last_right_clicked == 0 && right_clicked == 1)
                first = true;
            last_right_clicked = right_clicked;
            if (first) { // 只要第一次按右键就清空
                first = false;
                armors_set.clear();
                //DLOG(WARNING) << ">>>>>>>>>>>init ANTITOP<<<<<<<<<<<<<" <<std::endl;
                timeInZone.clear();
                time.clear();
                getCenter = false;
                return cv::Point2f(0,0);
            }
            if (armors.size() < 1) {
                getCenter = false;
                return cv::Point2f(0, 0);
            }

            if (first) { // 按下右键时瞄准中心装甲
                        // 按照距离中心的距离来进行排序，找到最靠近中心的装甲板

                        // 近距离辅瞄
                        sort(armors.begin(), armors.end(), [](const ArmorBlob &a, const ArmorBlob &b) -> bool {
                            const cv::Rect &r1 = a.rect;
                            const cv::Rect &r2 = b.rect;
                            return abs(r1.x + r1.y + r1.height / 2 + r1.width / 2 - 1024 / 2 - 1280 / 2) <
                                   abs(r2.x + r2.height / 2 - 1024 / 2 + r2.y + r2.width / 2 - 1280 / 2);
                        });
                        // 找出距离中心最近的装甲板
                        armor = armors.at(0);
                        // 最中心的装甲板优先级最高, 操作手瞄准的是最高优先级的装甲板 top_pri 就是操作手开启辅瞄时指定的跟踪目标（最靠近中心的部分）
                        top_pri = armor._class;     // 设定优先级
                    }
                    // 中心最近的装甲板
                    // DLOG(WARNING) << "class: " << top_pri << std::endl;
                    // 选择装甲板(优先选‘操作手右击离中心最近的’，其次‘选上一次选中的装甲板’，然后‘选择英雄’，最后选择‘步兵’    优先操作手的选项
                    int target = chooseArmor(armors);
                    // 选中的目标
//                    SerialParam::send_data.num = target; // 当前辅瞄选定的目标
//                    DLOG(WARNING)<<"target: "<<target<<endl;
                    // 筛选出 优先级最高的 类别
                    for (const auto &a: armors) {
                        if (a._class == target)
                            candidates.push_back(a);
                    }

                    // 没有符合的装甲板， 可能是掉帧, 减少错
                    if (candidates.size() < 1) {
            //            getCenter = false;

                        return cv::Point2f(0, 0);
                    }

                    //DLOG(INFO)<<"armors set:" <<armors_set.size()<<endl;

                    if(armors_set.size()>5) {
                        getCenter=true;
                    }
                    else getCenter= false;
                    //DLOG(INFO) << "armor size: " << candidates.size();
                    if (candidates.size() > 1) {
            //            sort(candidates.begin(), candidates.end(), [&](const ArmorBlob &a, const ArmorBlob &b) {
            //                return calcDiff(a, last_armor) < calcDiff(b, last_armor);
            //            });
                        // DLOG(INFO) << "diff: " << calcDiff(candidates.at(0), last_armor) << " " << calcDiff(candidates.at(1), last_armor);
                        // 找出y最小的装甲板
                        sort(candidates.begin(), candidates.end(), [](const ArmorBlob &a, const ArmorBlob &b) {
                            return a.y < b.y;
                        });
                    }

                    // 选择和上一帧识别的装甲板距离最近的那唯一一个装甲板
                    armor = candidates.at(0);

                    //DLOG(WARNING)<<"armor_id: "<< armor._class<<endl;
                    // candidates 都是装甲板和上一帧锁定的装甲板类别一致的
                    // 都更新 last_armor

                    if(target==last_armor._class){
                        // 按照和上一帧出现的装甲板距离进行降序排序（相当于一个追踪的效果）

                        //掉帧缓冲
                        if(lost_cnt<20&&calcDiff(armor,last_armor)>0.3){
                            armor=last_armor;
                            lost_cnt++;
                        }else{
                            lost_cnt=0;
                            armor=candidates.at(0);
                        }
                        bool has_two_armor=false;
                        cv::Point3d armor1=cv::Point3d(candidates[0].x,candidates[0].y,candidates[0].z);

                        cv::Point3d armor2 = cv::Point3d(0, 0, 0);
                        if (candidates.size() >= 2) {
                           has_two_armor = true;
                           armor2 = cv::Point3d(candidates[1].x, candidates[1].y, candidates[1].z);
                         }
                        float delta_time=delta_t;  // 类型转换
                        Angle_t tmp_angle;
                        // 如果candidates[0]的y比shootcenter高超过5cm，不加入
                                    static cv::Point3d shootCenter = cv::Point3d(0, 0, 0);
                        //            if (candidates[0].y - shootCenter.y > 0.08 && norm(shootCenter) != 0 && armors_set.size()>25 && candidates.size() == 1) {
                        //                getCenter = false;
                        //                return Point2f(0, 0);
                        //            }
                         //DLOG(WARNING)<<"center y diff: "<<candidates[0].y - shootCenter.y<<std::endl;
                         if(has_two_armor) {
                             //DLOG(WARNING)<<"center y diff of armor 2:"<<candidates[1].y-shootCenter.y<<std::endl;
                         }
                         armors_set.push_back(candidates[0]);
                         // 获取现在的时间戳
                         auto now = std::chrono::steady_clock::now();
                         // 获取armors中前20帧近距离的装甲板，取平均值获得中心点，当多于20帧时删除最远的
                          frameCount++;
                           //DLOG(INFO)<<"armors_set size: "<<armors_set.size()<<std::endl;
                           if(armors_set.size()>31){
                               //通过迭代器找到最大元素，删除该元素
                               auto max=max_element(armors_set.begin(),armors_set.end());
                               //找到最后一个元素。删除该元素
                               armors_set.erase(armors_set.end());
                               armors_set.erase(max);
                           }
                           //DLOG(WARNING) << "              OUT ANTITOP" << std::endl;
                           for(auto a:armors_set){
                               shootCenter+=cv::Point3d(a.x,a.y,a.z);
                           }
                           if(armors_set.size()!=0)
                               shootCenter=shootCenter/(int)armors_set.size();
                           // 显示shootCenter

                           /////////////////////弹道公式
                           // 没有预测，只有跟随

                           shootCenter.y=(shootCenter.y==0?1e-6:shootCenter.y);
                           tmp_angle.yaw=atan(-shootCenter.x/shootCenter.y);
                           tmp_angle.distance=sqrt(shootCenter.x*shootCenter.x+shootCenter.y*shootCenter.y);
                           tmp_angle.pitch=atan(shootCenter.z/tmp_angle.distance);

                           double theta=tmp_angle.pitch;
                           double delta_z;
                           double k1=0.47*1.169*(2*M_PI*0.02125*0.02125)/2/0.041;
                           double center_distance=tmp_angle.distance/0.9;   //距离
                           double gravity=9.8;
                           double flyTime;
                           for(int i=0;i<100;i++){
                               //计算炮弹飞行时间
                               flyTime=(pow(2.718281828,k1*center_distance)-1)/(k1*speed*cos(theta));
                               delta_z=shootCenter.z-speed*sin(theta)*flyTime/cos(theta)+
                                       0.5*gravity*flyTime*flyTime/cos(theta)/cos(theta);
                               if(fabs(delta_z)<0.000001)
                                   break;
                               theta-=delta_z/(-(speed*flyTime)/pow(cos(theta),2)+
                                               gravity*flyTime*flyTime/(speed*speed)*sin(theta)/pow(cos(theta),3));

                           }

                           double tmp_pitch=(theta/M_PI*180)*100;

                           //DLOG(WARNING) << "           timeInZone size:" << timeInZone.size() << std::endl;
                           static bool markLast;// 用于记录是否标记了上一个的时间
                           if(armors_set.size()>=30){
                               //使用重投影到平面内平面距离判定击打
                               cv::Point2f shootCenter2D=reproject(shootCenter);
                               cv::Point2f armor2D=reproject(cv::Point3d(candidates[0].x,candidates[0].y,candidates[0].z));

                               double p=0.9;
                               //if(abs(imu_data.gain_roll>500)) p=0.7;
                               double distance = p * fabsf(shootCenter2D.x - armor2D.x) + (1 - p) * fabsf(shootCenter2D.y - armor2D.y);
                               //DLOG(WARNING)<<"shoot distance: "<<distance<<std::endl;

                               if(lostInShootZone>=3&&!markLast){
                                   std::chrono::steady_clock::duration sum=std::chrono::steady_clock::duration::zero();
                                   for(const auto &timestamp: time){
                                       sum+=timestamp.time_since_epoch();
                                   }
                                   if(time.size()){
                                       std::chrono::steady_clock::duration average=sum/time.size();
                                       auto average_ms=std::chrono::duration_cast<std::chrono::milliseconds>(average);
                                       timeInZone.push_back(average_ms);
                                   }
                                   time.clear();
                                   markLast=true;
                               }
                               // 当装甲板的位置与center距离小于0.05m认为进入了击打范围
                               //                DLOG(WARNING) << "distance: " << distance << endl;
                               if(distance<5.0){
                                   //DLOG(WARNING) << "IN SHOOT ZONE!!!!" << std::endl;

                                   markLast = false;
                                   lostInShootZone = 0;
                                   // 将当前时间加入time
                                   time.push_back(now);
                                   // 拟合一圈开始，识别出了四个装甲板，开始计算时间差
                                   if (timeInZone.size() >= 3) {
                                       // 计算timeInZone相邻时间之间差值的平均值
                                       std::chrono::steady_clock::duration sum = std::chrono::steady_clock::duration::zero();


                                        sum = timeInZone[timeInZone.size()-1] - timeInZone[timeInZone.size()-3];
                                        std::chrono::steady_clock::duration average = sum / 2; // 只用最近两个周期的时间进行拟合
                                        auto average_ms = std::chrono::duration_cast<std::chrono::milliseconds>(average);
                                        // 设置延迟击打
                                        SerialParam::send_data.shootStatus = 0;
                                        // 休眠时间，下两个装甲板转过来的时间提前一个发弹延迟的时间
                                        bool shootable = 1;
                                        if(average_ms.count()>950) shootable=0; // 可能存在掉帧导致记录了两个装甲板的时间
                                        //DLOG(WARNING) << "average time: " << average_ms.count() << "; tmp_time: " << tmp_time
                                                                         //<< "; flyTime: " << flyTime * 1000 << "; user bias: "<< (int)SerialParam::recv_data.user_time_bias * 25<< std::endl;
                                        int i=0, sleep_time=-1;

                                        double timeBias;
                                        if(SerialParam::recv_data.outpost_state==0) timeBias=time_bias;
                                        else timeBias=time_bias_inverse;



                                        while (sleep_time <0) {
                                               sleep_time = (int) (1.0 * i * average_ms.count()  - 1. * tmp_time - flyTime * 1000 + timeBias + (int)SerialParam::recv_data.user_time_bias * 25);
                                               i++;
                                         }
                                        // 设置打弹
                                        if(shootable)
                                          std::thread([this, sleep_time, SerialPort_]() {
                                            // 延迟击打
                                          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
                                          SerialParam::send_data.shootStatus = 1;
                                          SerialPort_->Inpdata(&SerialParam::send_data);
                                          //DLOG(INFO) << "                shoot";
                                            }).detach();

                                    }
                               }
                               // 距离大于0.2认为离开了击打范围
                               else if(distance>25.0){
                                   lostInShootZone++;
                               }
                           }
                           // DLOG(INFO) << armor1.z;
                                       //DLOG(INFO) << "distance : " << tmp_angle.distance;
                                       // 跟随功能

                                       SerialParam::send_data.shootStatus = 0; // 实际击打

                                       SerialParam::send_data.pitch = tmp_pitch;

                                       double tmp_yaw = tmp_angle.yaw / M_PI * 180 * 100;
                                       tmp_yaw += 0.8 * 100;       // TODO: 进行补偿
                                       while (abs(tmp_yaw - imu_data.yaw) > 9000) {
                                           if (tmp_yaw - imu_data.yaw >= 9000)
                                               tmp_yaw -= 18000;
                                           else
                                               tmp_yaw += 18000;
                                       }
                                       SerialParam::send_data.yaw = tmp_yaw;


                                       //DLOG(INFO) << "                                        send yaw: " << SerialParam::send_data.yaw
                                                  //<< "  send pitch: " << SerialParam::send_data.pitch;
                                       //DLOG(INFO) << "                                        recv yaw: " << SerialParam::recv_data.yaw
                                                  //<< "  recv pitch: " << SerialParam::recv_data.pitch;

                                       last_armor = armor; // 上一次识别到的armor
                                       return reproject(shootCenter);
                                   } else {
                                       getCenter = false;
                                       // 掉帧缓冲
                                       if (lost_cnt < 20)
                                           lost_cnt++;
                                       else {
                                           lost_cnt = 0;
                                           predictor->resetPredictor();
                                       }

                                       last_armor = armor; // 上一次识别到的armor
                                       return cv::Point2f(0, 0);
                                   }

        return cv::Point2f(0,0);
}

float PoseSolver::calcDiff(const ArmorBlob &a, const ArmorBlob &b){
    return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}

//公用的函数
void PoseSolver::clearCircle()
{
    predictor->resetPredictor();
    outpost.clear();      //清空中心区域
    outpost_center.clear();
    filtered_pitch.clear();      // 清空pitch稳定
}

void PoseSolver::clearSentinel(){
    sentinel.clear();
}

// 瞄准跟踪的装甲板
int PoseSolver::chooseArmor(const std::vector<ArmorBlob> &armors){
    bool has_same_class=false;
    bool has_hero=false;
    bool has_sentry=false;
    bool has_engineer=false;
    for(const auto&a:armors){
        if(a._class==6) //优先选择最高级 相当于操作手指定的类别
            return 6;

        if(a._class==7)
            return 7;
        std::cout<<a._class<<" "<<last_armor._class<<std::endl;
        if(a._class==last_armor._class) //其次选择和上一帧识别的类别一样的装甲板
            has_same_class=true;

        switch(a._class){
            case 1:
                has_hero=true;
                break;
            case 2:
                has_engineer=true;
                break;
            case 3:
            case 4:
            case 5:
                has_sentry=true;
                break;
        }
    }

    // 没有操作手指定的目标时
    if(has_same_class)              //选择和上一次一样的类别
        return last_armor._class;
    else if(has_hero)               // 优先选择英雄
        return 1;
    else if(has_sentry)             //其次选择步兵
    {
        double d=10;
        int c=0;
        for(const auto&a:armors)    //选择距离在 10m以内的步兵
        {
            double distance=sqrt(a.x*a.x+a.z*a.z+a.y*a.y);
            if((a._class==3||a._class==4||a._class==5)&&d>distance){
                d=a.x*a.x+a.z*a.z;
                c=a._class;
            }
        }
        return c;
    } else if(has_engineer)        //最后再选择工程
        return 2;
    return 0;
}

cv::Point2f PoseSolver::reproject(cv::Point3f center){
    // 将center转成eigen
    Eigen::Vector3d e_center;
    e_center<<center.x,center.y,center.z;

    e_center=armor_to_world.inverse()*e_center;
    std::vector<cv::Point3f>center3D;
    center3D.push_back(cv::Point3f(e_center[0],e_center[1],e_center[2]));
    std::vector<cv::Point2f>reprojected_points;
    cv::projectPoints(center3D,rvec,tvec,camera_matrix,distortion_coefficients,reprojected_points);
    return reprojected_points[0];
}


static int shoot_count=0;

cv::Point2f PoseSolver::outpostMode(std::vector<ArmorBlob>&candidates,double delta_t,const GroundChassisData &imu_data,
                                    Uart *SerialPort_,bool &getTarget){
    cv::Point2f center_pixel=cv::Point2f(0,0);

    if(candidates.size()<1)
        return center_pixel;
    for (ArmorBlob &a:candidates)
        solveArmor(a,imu_data);

    right_clicked=imu_data.right_clicked;

    if(last_right_clicked==0&&right_clicked==1)
        first=true;

    // 第一次右击
    if(first)
    {
        outpost.clear();
        outpost_center.clear();
        filtered_pitch.clear();      // 清空pitch
        first=false;
        shoot_count=0;
    }

    last_right_clicked=right_clicked;

    for(const auto &a:candidates) // 首先估计中心
    {
        if(outpost.getSize()==outpost.size()){            // 当没满的时候重新对准
            int outpost_arr_cursize=outpost.size();       // 当前
            center=outpost.getMeanOfNearestPoints(std::min(33,outpost_arr_cursize),outpost_arr_cursize>3?3:0);
            // 获取最近的一定的装甲板
            // 在靠近中心一定范围内才会取更新中心
            bool flag=abs(imu_data.roll)<550?abs(a.angle)<10:true;

            if(sqrt((a.x-center.x)*(a.x-center.x)+(a.y-center.y)*(a.y-center.y)+(a.z-center.z)*(a.z-center.z))<0.1&&flag){
                // 距离在一定范围内都加入
                outpost_center.update({a.x,a.y,a.z});  // 随后就是只选取其中的一部分进行，不断选择离我最近的一段始终靠近
                outpost.update({a.x,a.y,a.z});
            }
        }else
            outpost.update({a.x,a.y,a.z});
    }
    double w=0.4*360;

    sort(candidates.begin(),candidates.end(),[&](const ArmorBlob &a,const ArmorBlob &b){return calcDiff(a,last_armor)<calcDiff(b,last_armor);});

    int outpost_arr_cursize=outpost.size();       // 当前

    // 最初瞄向最近的那个位置

    cv::Point3d minimum=outpost.getMeanOfNearestPoints(std::min(33, outpost_arr_cursize), outpost_arr_cursize>3 ? 3:0);

    center.x = minimum.x;
    center.y = minimum.y;
    center.z = minimum.z;

    // 当已经经过一轮之后

    if(outpost.getSize()==outpost.size())
    {
        getTarget=true;
        if(outpost_center.size()<10)
            center=outpost.getMeanOfNearestPoints(std::min(33, outpost_arr_cursize), outpost_arr_cursize>3 ? 3:0);
        else
            center=outpost_center.getMetric()/outpost_center.size();

        // 距离
        double distance = sqrt(center.x * center.x + center.y * center.y);

        // 计算时间
        // speed按15.5
        double time = distance / (speed*cos((imu_data.pitch/100.0) * M_PI / 180));

        // 默不射击
        SerialParam::send_data.shootStatus = 0;

        for(const auto &a:candidates){
            double armor_outpost_distance=sqrt((center.x - a.x) * (center.x - a.x) + (center.y - a.y) * (center.y - a.y) + (center.z - a.z) * (center.z - a.z));
            bool flag = abs(imu_data.roll) < 550 ? abs(armor.angle) < 3 : true;

            if(armor_outpost_distance<0.065&&(outpost_center.size()>5)&&flag){

                if(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()- send_shoot_time).count()/1000>500)
                    shoot_count+=1;

                send_shoot_time=std::chrono::steady_clock::now();

                // 休眠时间
                // double tmp_time=430;
                double tmp_time = (120 / w - time) * 1000 + OutpostParam::time_bias + (int)imu_data.user_time_bias * 25; // ms 还要考虑到机械这方面控制的时间

                if(abs(imu_data.roll) > 550)
                    tmp_time += 30;

//                thread([this, a, tmp_time, w, SerialPort_]()
//                {
//                    int sleep_time = tmp_time;
//                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
//                    SerialParam::send_data.shootStatus = 1;
//                    // 暂时不打蛋
//                    SerialPort_->writeData(&SerialParam::send_data);
//                    //DLOG(INFO) << "                shoot"; }).detach();
//                    break;

               }

        }

    }


        double tmp_yaw = ((atan((-center.x) / (center.y)) / M_PI * 180)) * 100;

        // 5m 左右偏置  yaw -0.2度
        //             pitch + 1.2或1.1度
        // 3m 左右可以用
        double distance_op = sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

        // yaw 的偏置没问题
        while (abs(tmp_yaw - imu_data.yaw) > 9000)        {
            if (tmp_yaw - imu_data.yaw >= 9000)
                tmp_yaw -= 18000;
            else
                tmp_yaw += 18000;
        }
        SerialParam::send_data.yaw = tmp_yaw;
        // 二维地图上的距离
        double center_distance = sqrt(center.x * center.x + center.y * center.y);
        // DLOG(WARNING)

        // 考虑空气阻力， 计算pitch
        double theta = atan(center.z / center_distance);
        double delta_z;

        // R = 42.50 mm, m = 41 g
                // 首先计算空气阻力系数 K
                double k1 = 0.47 * 1.169 * (2 * M_PI * 0.02125 * 0.02125) / 2 / 0.041;
                // 使用迭代法求解炮弹的发射角度
                // v是炮弹的发射速度，英雄默认 15 即可
                // 根据炮弹的初速度、发射角度、空气阻力系数，计算炮弹的飞行轨迹
                // 灯钩
                for (int i = 0; i < 100; i++){
                    // 计算炮弹的飞行时间
                    double t = (pow(2.718281828, k1 * center_distance) - 1) / (k1 * speed * cos(theta));
                    delta_z = center.z - speed * sin(theta) * t / cos(theta) + 4.9 * t * t / cos(theta) / cos(theta);
                    if (fabs(delta_z) < 0.000001)
                        break;
                    theta -= delta_z / (-(speed * t) / pow(cos(theta), 2) + 9.8 * t * t / (speed * speed) * sin(theta) / pow(cos(theta), 3));
                }

                short tmp_pitch = (theta / M_PI * 180) *100;

                filtered_pitch.update(tmp_pitch);
                SerialParam::send_data.pitch = filtered_pitch.getMetric() / filtered_pitch.size();  // 平均值

                last_armor = armor;

                // 绘制前哨站中心
                if(GlobalParam::DEBUG_MODE)
                {
                    // 世界坐标系  の  前哨站中心点
                    cv::Mat center_matrix = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));
                    center_matrix.ptr<double>(0)[0] = center.x;
                    center_matrix.ptr<double>(1)[0] = center.y;
                    center_matrix.ptr<double>(2)[0] = center.z;

                    Eigen::Vector3d e_center;
                    cv::cv2eigen(center_matrix,e_center);

                    // 初始位置矩阵SE3 平移位置
                    // 世界坐标系中的位置
                    Sophus::SE3d center_pose(Eigen::Matrix3d::Identity(), // 世界坐标系下的三维坐标
                                             e_center);

                    // 对于rotation 转置 == 求逆
//                    Eigen::Matrix3d rotation = camera_to_world.rotation_matrix().inverse();

                    // world_to_camera 世界坐标系在相机坐标系的位置
//                   Sophus::SE3d world_to_camera(rotation, Eigen::Vector3d(-camera_to_world.translation()[0],
//                                                                                      -camera_to_world.translation()[1],
//                                                                                      -camera_to_world.translation()[2]));

                    // 注意在相机坐标系中进行了转换，所以注意切换坐标系回去

                    // 得到相机坐标系中的前哨站中心三维坐标
//                    Sophus::SE3d center_in_camera = world_to_camera * center_pose;

                    /*cv::Mat T_matrix;
                    cv::eigen2cv(center_in_camera.translation(), T_matrix);

                    double temp = T_matrix.ptr<double>(0)[2];                // x = x
                    T_matrix.ptr<double>(0)[2] = T_matrix.ptr<double>(0)[1]; // z = y
                    T_matrix.ptr<double>(0)[1] = -temp;   */                   // y = -z

                    // 利用相机内参转换
                    // 相机内参 x 相机坐标系下的坐标
//                    center_matrix = camera_matrix * T_matrix;
                    // std::cout << "---------" <<std::endl;
                    // std::cout << center_matrix <<std::endl;

                    // uv 坐标
                    double u = center_matrix.at<double>(0, 0) / center_matrix.at<double>(2, 0);
                    double v = center_matrix.at<double>(1, 0) / center_matrix.at<double>(2, 0);

                    // DLOG(WARNING)<< "二维像素坐标" << u <<" , " <<v << std::endl;
                    // std::cout<<" Pixel: " << u <<" "<< v << std::endl;
                    return cv::Point2f(u, v);

             }

        return center_pixel;
}
