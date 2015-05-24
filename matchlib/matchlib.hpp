#include <stdio.h>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#define FLANN 1

class Matcher
{
    public:
        Matcher(int type);
        void match(cv::Mat descriptors_1, cv::Mat descriptors_2);
        std::vector< cv::DMatch > getMatches();
    private:
        std::vector< cv::DMatch > matches;
        std::vector< cv::DMatch > good_matches;
        void flannMatch(cv::Mat descriptors_1, cv::Mat descriptors_2); 
        int type;
        





};
