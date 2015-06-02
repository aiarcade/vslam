
#include "features2d.hpp"
#include "matchlib.hpp"


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"



using namespace std;

static void saveXYZ(const char* filename, const cv::Mat mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    cout<<mat.rows;
    for(int x = 0; x < mat.cols; x++)
    {
       
           
            float w=mat.at<float>(3,x);
            fprintf(fp, "%f %f %f\n", mat.at<float>(0,x)/w, mat.at<float>(1,x)/w, mat.at<float>(2,x)/w);
        
    }
    fclose(fp);
}




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
    cv::Mat P0 = cv::Mat::zeros(3, 4, CV_32F);
    P0.at<float>(0,0) = 718.856 ;
    P0.at<float>(0,2) = 607.1928;
    P0.at<float>(1,1)=718.856;
    P0.at<float>(1,2)=185.2157;
    P0.at<float>(2,2)=1.0;
    
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);
    P1.at<float>(0,0)=718.856;
    P1.at<float>(0,2)=607.1928; 
    P1.at<float>(0,3)=-386.1448; 
    P1.at<float>(1,1)=718.856; 
    P1.at<float>(1,2)=185.2157; 
    P1.at<float>(2,2)=1.0;
    
    cv::Mat pnts3D;
    cv::triangulatePoints(P0,P1,points1,points2,pnts3D);
    cout<<"3d point size"<<pnts3D.size()<<endl;
    saveXYZ("out.pcd",pnts3D);
    
    for( size_t m = 0; m < matches.size(); m++ )
    {
        
       
       
    
    }
    //cv::Mat K(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
    //cv::Mat R(3,3,cv::DataType<float>::type); // rotation matrix
    //cv::Mat T(4,1,cv::DataType<float>::type); // translation vector
    
    //cv::decomposeProjectionMatrix(P0, K, R, T);
    
    
    
   
    //cv::Mat distCoeffs(4, 1, CV_64FC1);
    //cv::Mat rvec(3, 1, CV_64FC1);
    //cv::Mat tvec(3, 1, CV_64FC1);
    //cv::Mat d(3, 3, CV_64FC1);
    
    //cv::Mat new3dpoints;
    
    //new3dpoints=pnts3D.rowRange(0,3).clone();
    
   
    //cout << "cam matrix"<<K<<endl;
   
    //cv::transpose(new3dpoints,new3dpoints);
     //cout<<"new size"<<new3dpoints.size()<<endl;
    //cv::solvePnPRansac(new3dpoints,points1,K, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
    //cout<<rvec<<endl;

    //cv::Mat rvecR(3,1,cv::DataType<double>::type);
    //cv::Rodrigues(rvec,rvecR);
    //std::vector<cv::Point2f> projectedPoints;
    //cv::projectPoints(new3dpoints,rvecR,tvec, K,distCoeffs, projectedPoints);
    //for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    //{
        //std::cout << "Image point: " << points1[i] << " Projected to " << projectedPoints[i] << std::endl;
    //}
    return 0;
}
