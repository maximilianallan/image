#include "../image/image.hpp"

int main(int argc, char **argv){

  cv::Mat i = cv::Mat::zeros(100,100,CV_8UC3);

  MonocularImage<unsigned char,3> mi(i);
  //StereoImage<unsigned char,3> si(i);

  return 0;

}
