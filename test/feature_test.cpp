
#include "features2d.hpp"
#include "matchlib.hpp"

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
    FeatureDetector *fd_r=new FeatureDetector(3);
    fd_r->extractFeatures(&input_image_r);
   
    FeatureDetector *fd_l=new FeatureDetector(3);
    fd_l->extractFeatures(&input_image_l);
   
    Matcher * matcher= new Matcher(1);
    matcher->match(fd_r->getDescriptors(),fd_l->getDescriptors());
    cv::drawMatches( input_image_r,fd_r->getKeyPoints(),input_image_l,fd_l->getKeyPoints(),
               matcher->getMatches(),output_image,cv::Scalar::all(-1),cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    cv::imwrite("match.jpg", output_image); 
    return 0;
}
