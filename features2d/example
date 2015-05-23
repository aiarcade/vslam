#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <stdio.h>

#define WIN_W 3
#define WIN_H 3

using namespace cv;
using namespace std;

//Min of a matrix
float minOfBlock(Mat  tile)
{
    float min=300;
    int value=0;
    for(int row = 0; row < tile.rows; ++row) {
        for(int col = 0; col < tile.cols; ++col) {
            value=tile.at<uchar>(row,col);
            if(value<min) min=value; 
          
        }
    }
    return min;    
    
}
//Max of a matrix
float maxOfBlock(Mat  tile)
{
    float max=300;
    int value=0;
    for(int row = 0; row < tile.rows; ++row) {
        for(int col = 0; col < tile.cols; ++col) {
            value=tile.at<uchar>(row,col);
            if(value>max) max=value; 
          
        }
    }
    return max;    
    
}
//Median of a matrix
float medianOfBlock(Mat  tile)
{
    cv::Mat image_sort = cv::Mat::zeros(tile.rows, tile.cols, tile.type()); // allocated memory
    tile.copyTo(image_sort); // copy data in image_sorted
    std::sort(image_sort.data, image_sort.dataend); // call std::sort
    cv::Mat vectorized = image_sort.reshape(1, 1); // reshaped your WxH matrix into a 1x(W*H) vector
    return image_sort.at<uchar>(tile.rows/2,tile.cols/2);    
   
    
}


int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat input_image;
    Mat output_image;
    input_image = imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if(! input_image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cout << "Size is " << input_image.rows << "x" << input_image.cols<<endl;
    
       
    Mat padded_image;
    int padding = 3;
    copyMakeBorder(input_image,padded_image,WIN_H/2,WIN_H/2,WIN_W/2,WIN_W/2,BORDER_CONSTANT,Scalar(0));
    
    Mat tile;
    
    output_image.create(input_image.rows , input_image.cols , input_image.type()); 
    
    for(int row = (WIN_W/2)-1 ; row <  padded_image.rows; ++row) {

        for(int col = (WIN_H/2)-1 ; col <  padded_image.cols; ++col) {
          
           tile =  padded_image(cv::Range(row, min(row + WIN_W, padded_image.rows)), 
                cv::Range(col, min(col + WIN_H,  padded_image.cols)));
            
           output_image.at<uchar>(row-(WIN_W/2)+1,col-(WIN_H/2)+1)=medianOfBlock(tile);
           
        }
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", output_image);                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
