
#include "features2d.hpp"
#include "matchlib.hpp"


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <string>

#define DESC_TYPE 1
#define MATCHER 1
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
    if( argc != 2)
    {
     cout <<" Usage: depth_test dir" << endl;
     return -1;
    }

    cv::Mat pr_image,r_image;
    cv::Mat pl_image,l_image;
    cv::Mat output_image;
    
    string dir=argv[1];
    
    char base_name[256];
    sprintf(base_name,"%06d.png",0);
    std::string pleft_img_file_name  = dir + "/image_0/" + base_name;
    std::string pright_img_file_name = dir + "/image_1/" + base_name;
    
    pr_image = cv::imread(pleft_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);   
    pl_image = cv::imread(pright_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
     
    FeatureDetector *pfd_r=new FeatureDetector(1);
    pfd_r->extractFeatures(&pr_image);
    FeatureDetector *pfd_l=new FeatureDetector(1);
    pfd_l->extractFeatures(&pl_image);
    
    Matcher * pmatcher= new Matcher(1);
    pmatcher->match(pfd_r->getDescriptors(),pfd_l->getDescriptors());
    std::vector<cv::DMatch> pmatches=pmatcher->getAllMatches();
    
    
    
    for (int i=1; i<10; i++) {
        
        
       
        sprintf(base_name,"%06d.png",i);
        std::string left_img_file_name  = dir + "/image_0/" + base_name;
        std::string right_img_file_name = dir + "/image_1/" + base_name;
       
            

        
        r_image = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);   
        l_image = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        
        FeatureDetector *fd_r=new FeatureDetector(1);
        fd_r->extractFeatures(&input_image_r);
   
        FeatureDetector *fd_l=new FeatureDetector(1);
        fd_l->extractFeatures(&input_image_l);
   
        Matcher * matcher= new Matcher(1);
        matcher->match(fd_r->getDescriptors(),fd_l->getDescriptors());
        std::vector<cv::DMatch> matches=matcher->getAllMatches();
        
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
