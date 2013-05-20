#include <cv.h>

template<typename PixelType, int Channels>
class __InnerImage {

public:

  cv::Mat frame_;
  PixelType *frame_data_;

  template<typename U, int V>
  friend class Image;

protected:

  explicit __InnerImage(cv::Mat &frame):frame_(frame){
    frame_data_ = (PixelType *)frame_.data;
  }


};


template<typename PixelType, int Channels>
class Image {

public:

  //typedef Image<PixelType,Channels>::Pixel_ Pixel;
  struct Pixel {
    PixelType data_[Channels];
  };

  //typedef Pixel_ Pixel;

  explicit Image(cv::Mat &frame):image_data_(frame){}

  virtual Pixel operator()(const int r, const int c) const = 0;
    
protected:
  
  __InnerImage<PixelType,Channels> image_data_;

};

template<typename PixelType, int Channels>
class MonocularImage : public Image<PixelType,Channels> {

public:
  explicit MonocularImage(cv::Mat &frame):Image<PixelType,Channels>(frame){}

  //typedef typename Image<PixelType,Channels>::Pixel_ Pixel;
  
  virtual typename Image<PixelType,Channels>::Pixel operator()(const int r, const int c) const {
    typename Image<PixelType,Channels>::Pixel px;
    memcpy(this->image_data_.frame_data_,px.data_,Channels*sizeof(PixelType));
    return px;
  }

};
/*

template<typename PixelType, int Channels>
class StereoImage : public Image<PixelType,Channels> {

public:
  
  explicit StereoImage(cv::Mat &stereo_frame) : Image(frame){}
  //explicit StereoImage(cv::Mat &left_frame,right_frame):

  virtual Pixel operator()(const int r, const int c) {}

private:
};
*/
