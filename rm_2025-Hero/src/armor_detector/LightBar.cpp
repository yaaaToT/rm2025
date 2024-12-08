//#include "../../include/armor_detector/LightBar.h"

//bool LightBarFinder::findLightBarBlobs(const cv::Mat &frame, LightBarBlobs &lightBarBlobs){
//    cv::Mat mat,hsv,green,white;
//    std::vector<cv::Mat>channels;
//    std::vector<std::vector<cv::Point>> contours;
//    split(frame, channels);

//    // 101: 自己是蓝色
//    SerialParam::recv_data.color = 1;   // 写明自己是红色
//    SerialParam::recv_data.color = 101;

////    if(SerialParam::recv_data.color==101){
////        cv::subtract(channels[2],channels[0],mat);    // BGR R-B  突出红色
////    }
////    else{
//        cv::subtract(channels[0],channels[2],mat);    // BGR B-R 突出蓝色
////    }

//    threshold(mat, mat, DetectorParam::thresh, 255, cv::THRESH_BINARY);
//    // extract green
//    subtract(channels[1], channels[2], green);
//    subtract(green, channels[0], green);
//    threshold(green, green, 100, 255, cv::THRESH_BINARY_INV);
//    // merge
//    bitwise_and(mat, green, mat);
//    // bitwise_or(mat, white, mat);
//    morphologyEx(mat, mat, cv::MORPH_CLOSE, kernel1);
//    // morphologyEx(mat, mat, MORPH_OPEN, kernel2);

//    if(GlobalParam::SHOW_THRESH){
//        imshow("thresh", mat);
//        // imshow("white", white);
//        cv::waitKey(1);
//    }

//    findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//    for(const auto& contour: contours){
//        const cv::RotatedRect& rrect = minAreaRect(contour);
//            if(isValidLightBarBlob(rrect)){
//                lightBarBlobs.emplace_back(rrect);
//            }
//    }

//    sort(lightBarBlobs.begin(), lightBarBlobs.end(), [](const cv::RotatedRect& a, const cv::RotatedRect& b)->bool {
//          if(a.center.x != b.center.x) return a.center.x < b.center.x;
//          return a.center.y > b.center.y;
//     });
//        return lightBarBlobs.size() >= 2;

//}

//bool LightBarFinder::isValidLightBarBlob(const cv::RotatedRect &rrect){
//    if(
//         checkAspectRatio(rrect.size.aspectRatio()) &&
//         checkArea(rrect.size.area()) &&
//         checkAngle(rrect)
//      ){
//            return true;
//        }
//        // DLOG(INFO) << "not lightbar: " << rrect.size.aspectRatio() << " " << rrect.size.area() << " " << rrect.angle;
//        return false;
//}

//bool LightBarFinder::checkAspectRatio(double ratio){
//    // return true;
//    return ratio <= 10 && ratio >= 2.5/2 || ratio <= 2./2.5 && ratio >= 1./10;
//    // return ratio <= 10 && ratio >= 1./10;
//}

//bool LightBarFinder::checkArea(double area) {
//    return area >= 20 && area < 400*100;
//}

//bool LightBarFinder::checkAngle(const cv::RotatedRect& rrect) {
//    double angle = rrect.angle;
//    return rrect.size.width < rrect.size.height ? angle <= 30 : angle >= 60;
//}

//LightBarFinder::LightBarFinder(){
//    kernel1 = getStructuringElement(0, cv::Size(5, 5));
//    // kernel1 = getStructuringElement(0, Size(11, 11));
//    kernel2 = getStructuringElement(0, cv::Size(5, 5));
//}
