
#include "features2d.hpp"


using namespace std;

int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    cv::Mat input_image;
    cv::Mat output_image;
    input_image = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    FeatureDetector *fd=new FeatureDetector(1);
    fd->extractFeatures(&input_image);
    fd->drawAndSave("out.jpg");
    
    return 0;
}
