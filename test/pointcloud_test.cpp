
#include "features2d.hpp"
#include "matchlib.hpp"


#include "opencv2/calib3d/calib3d.hpp"


using namespace std;





int main( int argc, char** argv )
{
    if( argc != 3)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    cv::Mat input_image_r;
    cv::Mat input_image_l;
    cv::Mat output_image;
    input_image_r = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);   
    input_image_l = cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
    FeatureDetector *fd_r=new FeatureDetector(1);
    fd_r->extractFeatures(&input_image_r);
   
    FeatureDetector *fd_l=new FeatureDetector(1);
    fd_l->extractFeatures(&input_image_l);
   
    Matcher * matcher= new Matcher(1);
    matcher->match(fd_r->getDescriptors(),fd_l->getDescriptors());
    std::vector<cv::DMatch> matches=matcher->getAllMatches();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    keypoints1=fd_l->getKeyPoints();
    keypoints2=fd_r->getKeyPoints();
  
 
    std::vector<cv::Point2f> points1, points2;
    
    for (std::vector<cv::DMatch>::
         const_iterator it= matches.begin();
         it!= matches.end(); ++it) {
        
        // Get the position of left keypoints
        float x=  keypoints1[it->queryIdx].pt.x;
        float y=   keypoints1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));
        
        // Get the position of right keypoints
        x=  keypoints2[it->trainIdx].pt.x;
        y=  keypoints2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }
    std::cout << points1.size() << " " << points2.size() << std::endl; 
    cv::Mat F = cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2),CV_FM_RANSAC);
    std::cout<<F<<std::endl;
    cv::Mat H1(4,4, input_image_l.type());
    cv::Mat H2(4,4, input_image_l.type());
    cv::stereoRectifyUncalibrated(points1, points2, F, input_image_l.size(), H1, H2);
    
    cv::Mat rectified1(input_image_l.size(), input_image_l.type());
    cv::warpPerspective(input_image_l, rectified1, H1, input_image_l.size());
    cv::imwrite("rectified1.jpg", rectified1);
  
    cv::Mat rectified2(input_image_r.size(), input_image_r.type());
    cv::warpPerspective(input_image_r, rectified2, H2, input_image_r.size());
    cv::imwrite("rectified2.jpg", rectified2);
    
    return 0;
}
