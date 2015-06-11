#include "features.hpp"
#include "matchlib.hpp"
#include "csvwriter.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/video/tracking.hpp"





#define DESC_TYPE 1
#define MATCHER 1
#define NO_IMAGES 1101
#define OUT_FILE "/home/mahesh/out/sift01.csv"

using namespace std;

cv::Mat P0 = cv::Mat::zeros(3, 4, CV_32F);
cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);

//std::vector<cv::Point2f> points1, points2;

cv::KalmanFilter KF(4, 4, 0);


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

cv::Mat transformationVectorToMatrix (cv::Mat tvec,cv::Mat rvec) {

  // extract parameters
  double rx = rvec.at<double>(0,0);
  double ry = rvec.at<double>(0,1);
  double rz = rvec.at<double>(0,2);
  double tx = tvec.at<double>(0,0);
  double ty = tvec.at<double>(0,1);
  double tz = tvec.at<double>(0,2);

  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  double cy = cos(ry);
  double sz = sin(rz);
  double cz = cos(rz);

  // compute transformation
  cv::Mat Tr(4,4,CV_64FC1);
  Tr.at<double>(0,0) = +cy*cz;          Tr.at<double>(0,1) = -cy*sz;          Tr.at<double>(0,2) = +sy;    Tr.at<double>(0,3) = tx;
  Tr.at<double>(1,0) = +sx*sy*cz+cx*sz; Tr.at<double>(1,1) = -sx*sy*sz+cx*cz; Tr.at<double>(1,2) = -sx*cy; Tr.at<double>(1,3) = ty;
  Tr.at<double>(2,0) = -cx*sy*cz+sx*sz; Tr.at<double>(2,1) = +cx*sy*sz+sx*cz; Tr.at<double>(2,2) = +cx*cy; Tr.at<double>(2,3) = tz;
  Tr.at<double>(3,0) = 0;               Tr.at<double>(3,1) = 0;               Tr.at<double>(3,2) = 0;      Tr.at<double>(3,3) = 1;
  return Tr;
}



cv::Mat computeCameraPos(cv::Mat points3d,std::vector<cv::Point2f> nl_points)
{
    
    
    cv::Mat K(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
    cv::Mat R(3,3,cv::DataType<float>::type); // rotation matrix
    cv::Mat T(4,1,cv::DataType<float>::type); // translation vector
    
    cv::decomposeProjectionMatrix(P0, K, R, T);
    
      
   
    cv::Mat distCoeffs(5, 1, CV_64FC1);
    
    //-3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 -7.233722e-02
    distCoeffs.at<float>(1,0)=-0.3728755;
    distCoeffs.at<float>(2,0)=0.2037299;
    distCoeffs.at<float>(3,0)=.002219027;
    distCoeffs.at<float>(4,0)=.001383707;
    distCoeffs.at<float>(5,0)=-.07233722;
    
    
    cv::Mat rvec(3, 1, CV_64FC1);
    cv::Mat tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    
    cv::Mat new3dpoints;
    
    new3dpoints=points3d.rowRange(0,3).clone().t();
    
    //cout<<"org mat"<<points3d(cv::Range(0,4), cv::Range(0,3))<<endl;
    //cout<<"tran mat"<<points3d.t()(cv::Range(0,4), cv::Range(0,3));
    
    points3d=points3d.t();
    
    for (int vi = 0; vi < points3d.rows; vi++)
    {
       new3dpoints.at<float>(vi,0)=new3dpoints.at<float>(vi,0)/points3d.at<float>(vi,3);
       new3dpoints.at<float>(vi,1)=new3dpoints.at<float>(vi,1)/points3d.at<float>(vi,3);
       new3dpoints.at<float>(vi,2)=new3dpoints.at<float>(vi,2)/points3d.at<float>(vi,3);
    }
    
    
    //cout<<"org mat"<<new3dpoints(cv::Range(0,4), cv::Range(0,3))<<endl;
    //cv::convertPointsHomogeneous(points4d,euclidian);
   
    //cout<<euclidian<<endl;
   
    
    
    //cv::transpose(new3dpoints,new3dpoints);
    //cout<<"new size"<<new3dpoints.size()<<endl;
   

   
    cv::solvePnPRansac(new3dpoints,nl_points,K, distCoeffs, rvec, tvec, false);
    

    cv::Mat mR;
    cv::Rodrigues(rvec, mR); // R is 3x3
    
    
    
    
    

    mR = mR.t();  // rotation of inverse
    tvec = -mR * tvec; // translation of inverse

    cv::Mat Tp(4, 4, mR.type()); // T is 4x4
    Tp( cv::Range(0,3), cv::Range(0,3) ) = mR * 1; // copies R into T
    Tp( cv::Range(0,3), cv::Range(3,4) ) = tvec * 1; // copies tvec into T
    
    double *p = Tp.ptr<double>(3);
    p[0] = p[1] = p[2] = 0; p[3] = 1;
    
    cout<<"campos from cv"<<Tp<<endl;
    Tp= transformationVectorToMatrix (tvec,rvec);
    cout<<"campos from direct"<<Tp<<endl;
    return Tp;

}

void KalmanCorrection(cv::Mat campos)
{
    
    
    
    
    
    
}

void cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose)
{
    pose = cv::Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));  
    float norm2 = (float)norm(H.col(1));  
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    cv::Mat p1 = H.col(0);       // Pointer to first column of H
    cv::Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

    p1 = H.col(1);           // Pointer to second column of H
    p2 = pose.col(1);        // Pointer to second column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

    p1 = pose.col(0);
    p2 = pose.col(1);

    cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    cv::Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

    pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}

void estimate3Dpoints(std::vector<cv::DMatch> matches,std::vector<cv::KeyPoint> keypoints1,std::vector<cv::KeyPoint> keypoints2,
                        std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2, cv::Mat &pnts3D)
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
    
     //cv::Mat H =cv::findHomography(points1,points2,CV_RANSAC );
     //cv::Mat pos;
     //cameraPoseFromHomography(H,pos);
     //cout<<"Homographic pos"<<pos<<endl;
    
    
    cv::triangulatePoints(P0,P1,points1,points2,pnts3D);
    //cout<<"Estimated size "<<pnts3D.size();
    
    
    
    
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
    char base_name[256];

    cv::Mat nl_image,r_image;
    cv::Mat l_image;
    cv::Mat output_image;
    cv::Mat pos=cv::Mat::eye(4,4,CV_64F);
    string dir=argv[1];
    /*
    P_rect_00: 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 
    *   `      0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 
    *          0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
    */
    P0.at<float>(0,0) = 721.5377 ;
    P0.at<float>(0,2) = 609.5593;
    P0.at<float>(1,1)=721.5377;
    P0.at<float>(1,2)=172.8540;
    P0.at<float>(2,2)=1.0;
    /* 
    P_rect_01: 7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02 
                0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 
                0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
    */
    P1.at<float>(0,0)=721.5377;
    P1.at<float>(0,2)=609.5593; 
    P1.at<float>(0,3)=-387.5744; 
    P1.at<float>(1,1)=721.5377; 
    P1.at<float>(1,2)=172.8540; 
    P1.at<float>(2,2)=1.0;
    
    cv::Mat pcamPos;
    cv::Mat camPos;
    
    std::vector<cv::Point2f> pr_points,pl_points,nl_points;
    
   
    rst_writer.openFile(OUT_FILE," ",1);
    
    for (int i=0; i<NO_IMAGES-1; i++) {
        
        
       
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
   
        cv::Mat pr_desc=fd_r->getDescriptors();
        cv::Mat pl_desc=fd_l->getDescriptors();
   
        Matcher * matcher= new Matcher(1);
        matcher->match(pl_desc,pr_desc);
                
        std::vector<cv::DMatch> matches=matcher->getAllMatches();
        std::vector<cv::KeyPoint> l_keypoints, r_keypoints,nl_keypoints;
        l_keypoints=fd_l->getKeyPoints();
        r_keypoints=fd_r->getKeyPoints();
        
         
       
        cv::Mat p_pnts3D;
        estimate3Dpoints(matches,l_keypoints,r_keypoints,pl_points,pr_points,p_pnts3D);
        
        
        
        sprintf(base_name,"%06d.png",i+1);
        string nleft_img_file_name  = dir + "/image_0/" + base_name;
        
        nl_image = cv::imread(nleft_img_file_name ,CV_LOAD_IMAGE_GRAYSCALE); 
        
        FeatureDetector *fd_nl=new FeatureDetector(1);
        fd_nl->extractFeatures(&nl_image);
        
        nl_keypoints=fd_nl->getKeyPoints();
        
        
        Matcher * pn_matcher= new Matcher(1);
        pn_matcher->match(pl_desc,fd_nl->getDescriptors());
        nl_points.clear();
        std::vector<cv::DMatch> pn_matches=pn_matcher->getAllMatches();
        
        for (std::vector<cv::DMatch>::
         const_iterator it= pn_matches.begin();
         it!= pn_matches.end(); ++it) {
        
            
            
        // Get the position of left keypoints
            float x= nl_keypoints[it->queryIdx].pt.x;
            float y= nl_keypoints[it->queryIdx].pt.y;
            nl_points.push_back(cv::Point2f(x,y));
        
        }
        
        cout<<"3D point size "<<p_pnts3D.size()<< "2d size "<<nl_points.size()<<endl;
        cv::Mat camPos=computeCameraPos(p_pnts3D,nl_points);
        ////A NaN
        if(camPos.at<double>(0,0)!=camPos.at<double>(0,0))
        {
            cout<<"Invalid campos Nan"<<endl;
            camPos=pcamPos.clone();
        }
        pos=pos*camPos.inv();
        //cout<<pos<<endl;
        
        vector<string>result;
        result.push_back(doubleToString(pos.at<double>(0,0)));
        result.push_back(doubleToString(pos.at<double>(0,1)));
        result.push_back(doubleToString(pos.at<double>(0,2)));
        result.push_back(doubleToString(pos.at<double>(0,3)));
        result.push_back(doubleToString(pos.at<double>(1,0)));
        result.push_back(doubleToString(pos.at<double>(1,1)));
        result.push_back(doubleToString(pos.at<double>(1,2)));
        result.push_back(doubleToString(pos.at<double>(1,3)));
        result.push_back(doubleToString(pos.at<double>(2,0)));
        result.push_back(doubleToString(pos.at<double>(2,1)));
        result.push_back(doubleToString(pos.at<double>(2,2)));
        result.push_back(doubleToString(pos.at<double>(2,3)));
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
