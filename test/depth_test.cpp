#include "features.hpp"
#include "matchlib.hpp"
#include "csvwriter.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"





#define DESC_TYPE 1
#define MATCHER 1
#define NO_IMAGES 1101
#define OUT_FILE "/home/mahesh/out/sift01.csv"

using namespace std;

cv::Mat P0 = cv::Mat::zeros(3, 4, CV_32F);
cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);

std::vector<cv::Point2f> points1, points2;


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

cv::Mat computeCameraPos(cv::Mat points3d)
{
    
    
    cv::Mat K(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
    cv::Mat R(3,3,cv::DataType<float>::type); // rotation matrix
    cv::Mat T(4,1,cv::DataType<float>::type); // translation vector
    
    cv::decomposeProjectionMatrix(P0, K, R, T);
    
      
   
    cv::Mat distCoeffs(4, 1, CV_64FC1);
    cv::Mat rvec(3, 1, CV_64FC1);
    cv::Mat tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    
    cv::Mat new3dpoints;
    
    new3dpoints=points3d.rowRange(0,3).clone();
    
   
    
   
    cv::transpose(new3dpoints,new3dpoints);
    cout<<"new size"<<new3dpoints.size()<<endl;
    cv::solvePnP(new3dpoints,points1,K, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
    

    cv::Mat mR;
    cv::Rodrigues(rvec, mR); // R is 3x3

    mR = mR.t();  // rotation of inverse
    tvec = -mR * tvec; // translation of inverse

    cv::Mat Tp(4, 4, mR.type()); // T is 4x4
    Tp( cv::Range(0,3), cv::Range(0,3) ) = mR * 1; // copies R into T
    Tp( cv::Range(0,3), cv::Range(3,4) ) = tvec * 1; // copies tvec into T
    // fill the last row of T (NOTE: depending on your types, use float or double)
    double *p = Tp.ptr<double>(3);
    p[0] = p[1] = p[2] = 0; p[3] = 1;
    
    cout<<"campos "<<Tp<<endl;
    
    return Tp;

}

cv::Mat estimate3Dpoints(std::vector<cv::DMatch> matches,std::vector<cv::KeyPoint> keypoints1,
                    std::vector<cv::KeyPoint> keypoints2)
{
    
    points1.clear();
    points2.clear();
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
    //std::cout << points1.size() << " " << points2.size() << std::endl; 
    
    
    cv::Mat pnts3D;
    cv::triangulatePoints(P0,P1,points1,points2,pnts3D);
    cout<<"Estimated size "<<pnts3D.size();
    return pnts3D;
    
    
    
}
string doubleToString(double dbl)
{
    std::ostringstream strs;
    strs << dbl;
    std::string str = strs.str();
    return str;
}


int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: depth_test dir" << endl;
     return -1;
    }
    
    CSVWriter rst_writer;
    

    cv::Mat pr_image,r_image;
    cv::Mat pl_image,l_image;
    cv::Mat output_image;
    
    string dir=argv[1];
    
    P0.at<float>(0,0) = 718.856 ;
    P0.at<float>(0,2) = 607.1928;
    P0.at<float>(1,1)=718.856;
    P0.at<float>(1,2)=185.2157;
    P0.at<float>(2,2)=1.0;
    
    
    P1.at<float>(0,0)=718.856;
    P1.at<float>(0,2)=607.1928; 
    P1.at<float>(0,3)=-386.1448; 
    P1.at<float>(1,1)=718.856; 
    P1.at<float>(1,2)=185.2157; 
    P1.at<float>(2,2)=1.0;
    
    cv::Mat pcamPos;
    cv::Mat camPos;
    
    
    
    char base_name[256];
    sprintf(base_name,"%06d.png",0);
    string pleft_img_file_name  = dir + "/image_0/" + base_name;
    string pright_img_file_name = dir + "/image_1/" + base_name;
    
    //pr_image = cv::imread(pleft_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);   
    //pl_image = cv::imread(pright_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
     
    //FeatureDetector *pfd_r=new FeatureDetector(1);
    //pfd_r->extractFeatures(&pr_image);
    //FeatureDetector *pfd_l=new FeatureDetector(1);
    //pfd_l->extractFeatures(&pl_image);
    
    //Matcher * pmatcher= new Matcher(1);
    //pmatcher->match(pfd_r->getDescriptors(),pfd_l->getDescriptors());
    //std::vector<cv::DMatch> pmatches=pmatcher->getAllMatches();
    
    //std::vector<cv::KeyPoint> pl_keypoints, pr_keypoints;
    //pl_keypoints=pfd_l->getKeyPoints();
    //pr_keypoints=pfd_r->getKeyPoints();
    //cv::Mat p_pnts3D=estimate3Dpoints(pmatches,pl_keypoints,pr_keypoints);
    rst_writer.openFile(OUT_FILE," ",1);
    for (int i=0; i<NO_IMAGES; i++) {
        
        
       
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir + "/image_0/" + base_name;
        string right_img_file_name = dir + "/image_1/" + base_name;
        
        cout << "Position estimation on image"<<left_img_file_name<<endl;    

        
        r_image = cv::imread(left_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);   
        l_image = cv::imread(right_img_file_name,CV_LOAD_IMAGE_GRAYSCALE);
        
        FeatureDetector *fd_r=new FeatureDetector(1);
        fd_r->extractFeatures(&r_image);
   
        FeatureDetector *fd_l=new FeatureDetector(1);
        fd_l->extractFeatures(&l_image);
   
        Matcher * matcher= new Matcher(1);
        matcher->match(fd_r->getDescriptors(),fd_l->getDescriptors());
                
        std::vector<cv::DMatch> matches=matcher->getAllMatches();
        std::vector<cv::KeyPoint> l_keypoints, r_keypoints;
        l_keypoints=fd_l->getKeyPoints();
        r_keypoints=fd_r->getKeyPoints();
        
        cv::Mat pnts3D=estimate3Dpoints(matches,l_keypoints,r_keypoints);
        cout<<"keyl:"<< l_keypoints.size()<<" keyr:"<< r_keypoints.size()<<" matches"<<matches.size()<<" 3d size"<<pnts3D.size()<<endl;
    
        cv::Mat camPos=computeCameraPos(pnts3D);
        //A NaN
        if(camPos.at<double>(0,0)!=camPos.at<double>(0,0))
        {
            cout<<"Invalid campos Nan"<<endl;
            camPos=pcamPos.clone();
        }
        
        vector<string>result;
        result.push_back(doubleToString(camPos.at<double>(0,0)));
        result.push_back(doubleToString(camPos.at<double>(0,1)));
        result.push_back(doubleToString(camPos.at<double>(0,2)));
        result.push_back(doubleToString(camPos.at<double>(0,3)));
        result.push_back(doubleToString(camPos.at<double>(1,0)));
        result.push_back(doubleToString(camPos.at<double>(1,1)));
        result.push_back(doubleToString(camPos.at<double>(1,2)));
        result.push_back(doubleToString(camPos.at<double>(1,3)));
        result.push_back(doubleToString(camPos.at<double>(2,0)));
        result.push_back(doubleToString(camPos.at<double>(2,1)));
        result.push_back(doubleToString(camPos.at<double>(2,2)));
        result.push_back(doubleToString(camPos.at<double>(2,3)));
        rst_writer.writeLine(result);
        
        pcamPos=camPos.clone();
        
       
        //Matcher * ll_matcher= new Matcher(1);
        //ll_matcher->match(fd_l->getDescriptors(),pfd_l->getDescriptors());
        
        //std::vector<cv::DMatch> ll_matches=ll_matcher->getAllMatches();
        
        
        //Matcher * rr_matcher= new Matcher(1);
        //rr_matcher->match(fd_r->getDescriptors(),pfd_r->getDescriptors());
        
        //std::vector<cv::DMatch> rr_matches=rr_matcher->getAllMatches();
            
        
        
    
    
    
    }
   
    
    return 0;
}
