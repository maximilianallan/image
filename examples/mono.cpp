#include "../image/image.hpp"

int main(int argc, char **argv){

  boost::shared_ptr<cv::Mat> i(new cv::Mat(100,100,CV_8UC3));
  sv::StereoFrame mi(i);
  //StereoImage<unsigned char,3> si(i);

  return 0;

}
