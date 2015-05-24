#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <iostream>

#include <stdio.h>

#define WIN_W 3
#define WIN_H 3




#define SIFT 1
#define SURF 2
#define CUSTOM 3

class  FeatureDetector{
    
    public:
        FeatureDetector(int type);
        void extractFeatures(cv::Mat *image);
        void drawAndSave(char *file);
        cv::Mat getDescriptors();
        std::vector  <cv::KeyPoint> getKeyPoints();
        
    private:
        void extractSIFTFeatures();
        void extractCustomFeatures();
        int type;
        cv::Mat * input_image;
        std::vector  <cv::KeyPoint> keypoints;
        cv::Mat descriptor;
        
    
};
