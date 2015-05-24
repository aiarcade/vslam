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
    case CUSTOM:
        extractCustomFeatures();
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
    
    
}

void FeatureDetector::extractCustomFeatures()
{
    
    int max=40;
    int count=0;
    for(int row = 0; row < input_image->rows; ++row) {
        for(int col = 0; col < input_image->cols; ++col) {
            int value=input_image->at<uchar>(row,col);
            if(value>40&&value<45)
            {
                    keypoints.push_back(cv::KeyPoint((float)row,(float)col,3.0));
                    descriptor.push_back((float)value);
                    count++;
                    if(count>max) break;
            } 
          
        }
         if(count>max) break;
    }
    
    
    
}


void  FeatureDetector::drawAndSave(char *file)
{
     cv::Mat output;
     cv::drawKeypoints(*input_image, keypoints, output);
     cv::imwrite(file, output); 
    
}

cv::Mat  FeatureDetector::getDescriptors()
{
    
        return descriptor;
    
 }
 
 
 std::vector  <cv::KeyPoint> FeatureDetector::getKeyPoints()
{
    
        return keypoints;
    
 }
