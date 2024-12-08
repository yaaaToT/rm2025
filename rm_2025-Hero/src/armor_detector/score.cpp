#include "../../include/armor_detector/score.h"

void Score::trainSVM(cv::Mat sampleMat, cv::Mat labelMat) {
        // create SVM
        svm = cv::ml::SVM::create();
        // set params
        svm->setType(cv::ml::SVM::C_SVC);
        svm->setKernel(cv::ml::SVM::LINEAR);
        svm->setC(1);

        svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 5000, 1e-7));
        // train
        cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(sampleMat, cv::ml::ROW_SAMPLE, labelMat);
        svm->train(trainData);
        svm->save(svmModel);
    }
