#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include <cv.h>
#include <boost/shared_ptr.hpp>

namespace tcv {

  template<typename PixelType, int Channels>
  class __InnerImage {
    
  public:
    
    template<typename U, int V>
    friend class Image;
    template<typename U, int V>
    friend class MonocularImage;
    template<typename U, int V>
    friend class StereoImage;

  protected:

    boost::shared_ptr<cv::Mat> frame_;
    PixelType *frame_data_;

    explicit __InnerImage(boost::shared_ptr<cv::Mat> &frame):frame_(frame){
      frame_data_ = (PixelType *)frame_->data;
    }

  };

  template<typename PixelType, int Channels>
  class Image {

  public:

    struct Pixel_ {
      PixelType data_[Channels];
    };

    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    explicit Image(boost::shared_ptr<cv::Mat> im) : image_data_(im) {}

    virtual Pixel operator()(const int r, const int c) const = 0;
    virtual boost::shared_ptr<cv::Mat> &Mat() = 0;
    virtual int rows() const = 0;
    virtual int cols() const = 0;

  protected:
  
    __InnerImage<PixelType,Channels> image_data_;

  };

  template<typename PixelType, int Channels>
  class MonocularImage : public Image<PixelType,Channels> {

  public:
    
    explicit MonocularImage(boost::shared_ptr<cv::Mat> frame) : Image<PixelType,Channels>(frame) {}

    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;
    //typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    virtual Pixel operator()(const int r, const int c) const {
      Pixel px;
      const int index = (r*this->image_data_.frame_->cols + c)*Channels;
      memcpy(&this->image_data_.frame_data_[index],px.data_,Channels*sizeof(PixelType));
      return px;
    }

    virtual boost::shared_ptr<cv::Mat> &Mat(){
      return this->image_data_.frame_;
    }

    virtual int rows() const {
      return this->image_data_.frame_->rows;
    }

    virtual int cols() const {
      return this->image_data_.frame_->cols;
    }

  };

  /*
  template<typename PixelType, int Channels>
  class StereoImage : public Image<PixelType,Channels> {

  public:
  
    explicit StereoImage(cv::Mat &stereo_frame) : Image(frame){}
    //explicit StereoImage(cv::Mat &left_frame,right_frame): TODO
    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    //typedef typename Image<PixelType,Channels>::Pixel_ Pixel;
  

  private:
  };*/
}

#endif
