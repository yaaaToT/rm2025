
#include "../../include/Resolver/Resolver.h"

Resolver::Resolver() {
    this->CameraMatrix_ = (cv::Mat_<double>(3, 3) << 1150, 0, 660, 0, 1150, 512, 0, 0, 1);
    this->DistCoeffs_ = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
}

void Resolver::DistanceMeasurer(Armor &armor)
{
    SetWorldPoints(armor.type);
    SolvePNP(armor);
    DistanceToCenter(armor);
}

void Resolver::Dl_DistanceMeasurer(ArmorObject &object)
{
    Dl_SetWorldPoints();
    Dl_SolvePNP(object);
    Dl_DisToCenter(object);

}


void Resolver::SetWorldPoints(ArmorType armor_type)
{
    if(armor_type==SMALL)
    {
        target_width=133;
        target_height=55;
    }
    if(armor_type==BIG)
    {
        target_width=228;
        target_height=55;
    }
    points_in_world.clear();
    // 右手坐标系 左上开始逆时针
    points_in_world.push_back(cv::Point3d (-target_width/2.0,target_height/2.0,0));
    points_in_world.push_back(cv::Point3d(-target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, target_height / 2.0, 0));

}

void Resolver::Dl_SetWorldPoints()
{
    target_width=133;
    target_height=55;

    points_in_world.clear();

    points_in_world.push_back(cv::Point3d (-target_width/2.0,target_height/2.0,0));
    points_in_world.push_back(cv::Point3d(-target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, target_height / 2.0, 0));


}

void Resolver::SolvePNP(Armor& armor)
{
    solvePnP(points_in_world,armor.armor_points,CameraMatrix_,DistCoeffs_,rotate_mat,trans_mat,false,cv::SOLVEPNP_AP3P);

    // 设置平移部分
    armor.pose.translation().x()=trans_mat.ptr<double>(0)[0];
    armor.pose.translation().y()=trans_mat.ptr<double>(0)[1];
    armor.pose.translation().z()=trans_mat.ptr<double>(0)[2];

    // 旋转向量到旋转矩阵的转化    3*1/1*3 -> 3*3
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotate_mat, rotation_matrix);

    Eigen::Matrix3d tf2_rotation_matrix;
    tf2_rotation_matrix << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
            rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);
    // 设置旋转部分
    armor.pose.linear() = tf2_rotation_matrix;
    armor.distance= sqrtf(pow(trans_mat.ptr<double>(0)[0],2)+
                             pow(trans_mat.ptr<double>(0)[1],2)+
                             pow(trans_mat.ptr<double>(0)[2],2));
}

void Resolver::Dl_SolvePNP(ArmorObject&object)
{
    auto armorDetector=new ArmorDetector();
//    std::vector<cv::Point2f> object_points;
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[0]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[1]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[3]));
    object.object_points.push_back(armorDetector->FourPoints(&object.apex[2]));

    solvePnP(points_in_world,object.object_points,CameraMatrix_,DistCoeffs_,rotate_mat,trans_mat,false,cv::SOLVEPNP_AP3P);

    //std::cout<<"points:"<<object_points[0].x<<","<<object.object_points[0].y<<std::endl;

    object.pose.translation().x()=trans_mat.ptr<double>(0)[0];
    object.pose.translation().y()=trans_mat.ptr<double>(0)[1];
    object.pose.translation().z()=trans_mat.ptr<double>(0)[2];

    // 旋转向量到旋转矩阵的转化    3*1/1*3 -> 3*3
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotate_mat, rotation_matrix);

    Eigen::Matrix3d tf2_rotation_matrix;
    tf2_rotation_matrix << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
            rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);
    // 设置旋转部分
    object.pose.linear() = tf2_rotation_matrix;
    object.distance= sqrtf(pow(trans_mat.ptr<double>(0)[0],2)+
                             pow(trans_mat.ptr<double>(0)[1],2)+
                             pow(trans_mat.ptr<double>(0)[2],2));


}

void Resolver::DistanceToCenter(Armor& armor)
{
    armor.distance_to_image_center=cv::norm(armor.center-cv::Point2f(image_width/2,image_height/2));
}

void Resolver::Dl_DisToCenter(ArmorObject &object)
{
    auto detector=new ArmorDetector();
    object.center=detector->TargetCenter(object.apex);
   object.distance_to_target_center=cv::norm(object.center-cv::Point2f(image_width/2,image_height/2));
   std::cout<<"center:"<<object.center.x<<","<<object.center.y<<std::endl;
   std::cout<<"distance:"<<object.distance_to_target_center<<std::endl;

}

void Resolver::Dl_AimTarget(std::vector<ArmorObject> &objects)
{
//    ArmorObject object;

    ArmorObject temp=objects[0];

//    ArmorObject temp;

    for(auto&object:objects)
    {
        if(object.distance_to_target_center<temp.distance_to_target_center)
        {
            temp=object;
        }

    }

//    object.distance_to_target_center=temp.distance;

    send_distance=temp.distance;

//    send_distance=object.distance;

//    std::cout<<"distance:"<<object.distance_to_target_center<<std::endl;

    init();



    if(send_distance < 1500)
    {
        send_yaw=(-atan(temp.pose.translation().x()/temp.distance)-0.013f)*180/CV_PI;

        send_pitch=Transform(temp.pose.translation().z(),(temp.pose.translation().y() -0.0f)) * 180/CV_PI;

    }
    else if(send_distance >= 1500 && send_distance < 2000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 5.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2000 && send_distance < 2500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 8.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2500 && send_distance < 3000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 10.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3000 && send_distance < 3500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 11.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3500 && send_distance < 4000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 13.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4000 && send_distance < 4500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 15.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4500 && send_distance < 5000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 17.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 5000 && send_distance < 5500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 19.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 5500 && send_distance < 6000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 21.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 6000 && send_distance < 6500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 23.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 6500 && send_distance < 7000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 25.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 7000 && send_distance < 7500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 27.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 7500 && send_distance < 8000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 29.0f)) * 180 / CV_PI;
    }

}


// 距离击打
void Resolver::AimTraget(std::vector<Armor>& armors)
{
    Armor temp = armors[0];
    for(auto& armor : armors)
    {
        if(armor.distance_to_image_center<temp.distance_to_image_center)
        {
            temp=armor;
        }
    }

    send_distance=temp.distance;

    init();

//    send_yaw=(-atan(temp.pose.translation().x()/temp.distance))*180/CV_PI;
//
//    send_pitch=Transform(temp.pose.translation().z(),(temp.pose.translation().y())) * 180/CV_PI;

//    send_pitch=() * 180/CV_PI;

    if(send_distance < 1500)
    {
        send_yaw=(-atan(temp.pose.translation().x()/temp.distance)-0.013f)*180/CV_PI;

        send_pitch=Transform(temp.pose.translation().z(),(temp.pose.translation().y() -0.0f)) * 180/CV_PI;

    }
    else if(send_distance >= 1500 && send_distance < 2000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 5.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2000 && send_distance < 2500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 8.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 2500 && send_distance < 3000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 10.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3000 && send_distance < 3500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 11.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 3500 && send_distance < 4000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 13.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4000 && send_distance < 4500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 15.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 4500 && send_distance < 5000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 17.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 5000 && send_distance < 5500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 19.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 5500 && send_distance < 6000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 21.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 6000 && send_distance < 6500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 23.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 6500 && send_distance < 7000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 25.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 7000 && send_distance < 7500)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 27.0f)) * 180 / CV_PI;
    }
    else if(send_distance >= 7500 && send_distance < 8000)
    {
        send_yaw = (-atan(temp.pose.translation().x() / temp.distance) - 0.013f) * 180 / CV_PI;

        send_pitch = Transform(temp.pose.translation().z(), (temp.pose.translation().y() + 29.0f)) * 180 / CV_PI;
    }

}
