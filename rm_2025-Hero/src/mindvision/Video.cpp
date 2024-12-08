#include "../../include/mindvision/VideoCapture.h"

//void NativeVideo::open(){
//    std::string filename="/home/yaaa/1733403228000_cokotools.mp4";

//    this->video.open(filename);

//    if(!this->video.isOpened()){
//        return;
//    }
//}

//void NativeVideo::startCapture(Params_ToVideo& params){

//    // params in
//    _video_thread_params.video = this->video;
//    _video_thread_params.__this = this;
//    // params out
//    _video_thread_params.frame_pp = params.frame_pp;
// //    int id = 0;
// //    constexpr int size = 10;
// //    Image frame[size];
// //    for(auto& m: frame) m.mat = new cv::Mat(cv::Size(1280, 1024), CV_32FC3);
// //    this->video >> *frame[id].mat;
// //    *_video_thread_params.frame_pp = &frame[id];
// //    id = (id+1) % size;
//    cv::Mat hsv;
//    double alpha = 1, beta = 30;
//    while(/*!(*_video_thread_params.frame_pp)->mat->empty()*/true){

//        video>>hsv;

//        cv::imshow("video",hsv);

//        if(cv::waitKey(30)==27); break;
// //         _video_thread_params.video >> *(frame[id].mat);
// //         (*_video_thread_params.frame_pp)->mat = frame[id].mat;
// //         id = (id+1) % size;
// //         usleep(10000);
//    }
//}

//NativeVideo::NativeVideo() {

//}

//NativeVideo::~NativeVideo() {
//    video.release();
// //    writer.release();
//    cv::destroyAllWindows();
//}
