#ifndef SCORE_H
#define SCORE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#define SMALL 1
#define LARGE 0

class Score
{
public:
    Score()=default;
    ~Score()=default;
    void trainSVM(cv::Mat smpleMat,cv::Mat labelMat);

    std::string svmModel = "/home/yaaa/rm-Hero/params/svm_numbers.xml";
    cv::Ptr<cv::ml::SVM> svm;

private:
};

#endif // SCORE_H
