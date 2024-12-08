#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#include"../outpost/params.h"
#include "opencv2/opencv.hpp"
#include "../SerialPort/SerialPort.h"

void *saveFrameToNative(void *params_p);
void *getFrameFromPicture(void*params_p);


enum CameraType
{
    mindvison,
    Video,
    Picture,
};

// Image 包含了图像、时间戳以及imu数据       追求时钟同步
    class Image
    {
    public:
        cv::Mat* mat;                                           // 图像
        std::chrono::steady_clock::time_point time_stamp;   // 时间戳
        GroundChassisData imu_data;                            // imu_data
    };

    struct Params_ToVideo
    {
            cv::VideoCapture video;        // 相机
            Image **frame_pp;              // Image
            void *__this;
            cv::VideoWriter writer;            // 写入

            Params_ToVideo()
            {
                frame_pp = (Image **)malloc(sizeof(Image *));
                *frame_pp = new Image();
            }
            ~Params_ToVideo(){
                free(*frame_pp);
                free(frame_pp);
            }
    };

    class VideoCapture
        {
        public:
            VideoCapture();
            ~VideoCapture();
            virtual void open() = 0;
            virtual void startCapture(Params_ToVideo &) = 0;
            void startSave(Params_ToVideo &params_to_video);
            void chooseCameraType(VideoCapture *&);

        protected:
            double rate{};
            int _id;
            uint16_t height;
            uint16_t width;
            uint16_t offset_x;
            uint16_t offset_y;

            pthread_t threadID{};
            pthread_t threadID2{};

            Params_ToVideo _video_thread_params;

            cv::VideoWriter writer;

            HostComputerData *_serial_port;
            GroundChassisData *_data_read;
        };

    class NativeVideo : public VideoCapture
        {
        public:
            explicit NativeVideo();
            ~NativeVideo();
            void open() override;
            void startCapture(Params_ToVideo &params) override;

        private:
            cv::VideoCapture video;
        };


#endif // VIDEOCAPTURE_H
