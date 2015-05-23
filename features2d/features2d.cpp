#include "features2d.hpp"

FeatureDetector::FeatureDetector(int mtype){
        type=mtype;
    
    
    
    
}

void FeatureDetector::extractFeatures(cv::Mat * image)
{
    input_image=image;
    switch(type)
    {
     case SIFT:
        extractSIFTFeatures();
        break;
     default:
        break;
        
        
    }
}

void FeatureDetector::extractSIFTFeatures()
{
    cv::SiftFeatureDetector detector;
    detector.detect(*input_image, keypoints );
    cv::SiftDescriptorExtractor extractor;
    extractor.compute(*input_image, keypoints,descriptor );
    std::cout<<descriptor.rows<<'x'<<descriptor.cols<<std::endl;
    std::cout<<keypoints.size();
    
    
}

void  FeatureDetector::drawAndSave(char *file)
{
     cv::Mat output;
     cv::drawKeypoints(*input_image, keypoints, output);
     cv::imwrite(file, output); 
    
}
