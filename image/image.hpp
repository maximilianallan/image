#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include <cv.h>
#include <boost/shared_ptr.hpp>

namespace sv {

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
    //remove this from here and put in StereoImage as __InnerImage instance
    boost::shared_ptr<cv::Mat> classified_; 
    boost::shared_ptr<cv::Mat> disparity_;
    boost::shared_ptr<cv::Mat> point_cloud_;

    PixelType *frame_data_;


    explicit __InnerImage(boost::shared_ptr<cv::Mat> frame):frame_(frame){
      frame_data_ = (PixelType *)frame_->data;
      classified_.reset(new cv::Mat(frame->size(),CV_8UC1));
      disparity_.reset(new cv::Mat(frame->size(),CV_16SC1));
      point_cloud_.reset(new cv::Mat(frame->size(),CV_32FC3));
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

    PixelType *FrameData() { return image_data_.frame_data_; }

    virtual Pixel operator()(const int r, const int c) const = 0;
    virtual cv::Mat &Mat() = 0;
    virtual boost::shared_ptr<cv::Mat> ClassifiedImage(){return image_data_.classified_;}
    virtual boost::shared_ptr<cv::Mat> PtrToMat(){return image_data_.frame_;}
    virtual boost::shared_ptr<cv::Mat> PtrToDisparity(){return image_data_.disparity_;}
    virtual boost::shared_ptr<cv::Mat> PtrToPointCloud(){return image_data_.point_cloud_;}
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

    virtual cv::Mat &Mat(){ return *(this->image_data_.frame_); }

    virtual int rows() const { return this->image_data_.frame_->rows; }

    virtual int cols() const { return this->image_data_.frame_->cols; }

  };


  template<typename PixelType, int Channels>
  class StereoImage : public Image<PixelType,Channels> {

  public:

    explicit StereoImage(boost::shared_ptr<cv::Mat> stereo_frame) : Image(stereo_frame){}
    //explicit StereoImage(cv::Mat &left_frame,right_frame): TODO
    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    //typedef typename Image<PixelType,Channels>::Pixel_ Pixel;


    virtual Pixel operator()(const int r, const int c) const {
      Pixel px;
      const int index = (r*this->image_data_.frame_->cols + c)*Channels;
      memcpy(&this->image_data_.frame_data_[index],px.data_,Channels*sizeof(PixelType));
      return px;
    }

    virtual cv::Mat &Mat(){ return *(this->image_data_.frame_); }

    cv::Mat LeftMat(){ std::cout << "LeftMat() ==> " << (int)this->image_data_.frame_->data << "\n"; return (*(this->image_data_.frame_))(cv::Range::all(),cv::Range(0,cols()/2)); }
    cv::Mat RightMat(){ return (*(this->image_data_.frame_))(cv::Range::all(),cv::Range(cols()/2,cols())); }

    virtual int rows() const { return this->image_data_.frame_->rows; }

    virtual int cols() const { return this->image_data_.frame_->cols; }

  };

  typedef Image<unsigned char, 3> Frame;
  typedef MonocularImage<unsigned char, 3> MonoFrame;
  typedef StereoImage<unsigned char, 3> StereoFrame;

}

#endif
