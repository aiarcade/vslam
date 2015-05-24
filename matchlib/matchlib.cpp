

#include "matchlib.hpp"

Matcher::Matcher(int mtype)
{
    type=mtype;
    
}

void Matcher::match(cv::Mat descriptors_1, cv::Mat descriptors_2 )
{
     switch(type)
    {
     case FLANN:
        flannMatch(descriptors_1,descriptors_2 );
        break;
     default:
        break; 
        
        
    }
}

void Matcher::flannMatch(cv::Mat descriptors_1, cv::Mat descriptors_2)
{
     double max_dist=0 ; double min_dist=100; 
     cv::FlannBasedMatcher matcher;
     matcher.match( descriptors_1, descriptors_2, matches );
     for( int i = 0; i < descriptors_1.rows; i++ )
    {   double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
        { good_matches.push_back(matches[i]); }
    }
}

std::vector< cv::DMatch > Matcher::getMatches()
{
    return  good_matches;
    
}


