//#include "../../include/mindvision/VideoSaver.h"

//VideoSaver::VideoSaver(){
//    std::string filename="/home/yaaa/1733403228000_cokotools.mp4";

//    std::string save_path=filename.substr(0,filename.rfind('/')+1);
//    FILE* fp=popen(("ls -l"+save_path+" |grep '\\<"+("blue")+"' |grep^- | wc -l").c_str(),"r");
//    std::fscanf(fp,"%d",&id);
//    pclose(fp);
//    writer = cv::VideoWriter(save_path + DetectorParam::color + std::to_string(id) + ".mp4", cv::VideoWriter::fourcc('m','p','4','v'), 210.2, cv::Size(1280, 1024));

//}

//VideoSaver::~VideoSaver(){
//    writer.release();
//}

//void VideoSaver::SaveVideo(Image** frame_p){
//    sleep(1);
//    const cv::Mat& frame = *(**frame_p).mat;
//    while(!frame.empty()){
//        writer.write(frame);
//    }
//}
