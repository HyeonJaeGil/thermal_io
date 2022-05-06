#include "lsd_detector.h"

LsdDetector::LsdDetector(){
    bd = cv::line_descriptor::LSDDetector::createLSDDetector();
    lines.clear();

}


void LsdDetector::DetectLineFeature(cv::Mat& img_in){
    mask = cv::Mat::ones( img_in.size(), CV_8UC1 );
    bd->detect( img_in, lines, 2, 1, mask );

    /* draw lines extracted from octave 0 */
   if( img_in.channels() == 1 )
     cvtColor( img_in, img_in, cv::COLOR_GRAY2BGR );
   for ( size_t i = 0; i < lines.size(); i++ )
   {
     cv::line_descriptor::KeyLine kl = lines[i];
     if( kl.octave == 0)
     {
       /* get a random color */
       int R = ( rand() % (int) ( 255 + 1 ) );
       int G = ( rand() % (int) ( 255 + 1 ) );
       int B = ( rand() % (int) ( 255 + 1 ) );
 
       /* get extremes of line */
       cv::Point pt1 = cv::Point2f( kl.startPointX, kl.startPointY );
       cv::Point pt2 = cv::Point2f( kl.endPointX, kl.endPointY );
 
       /* draw line */
       line( img_in, pt1, pt2, cv::Scalar( B, G, R ), 3 );
     }
 
   }
}