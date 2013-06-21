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

    void Reset(const cv::Size size) { frame_.reset(new cv::Mat(size,CV_MAKETYPE(cv::DataDepth<PixelType>::value,Channels) )); }
    cv::Size Size() const { return frame_ == 0x0 ?  cv::Size(0,0) : frame_->size(); }
    PixelType *frame_data_;

    __InnerImage() {}
    explicit __InnerImage(boost::shared_ptr<cv::Mat> frame):frame_(frame){
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

    //explicit Image(boost::shared_ptr<cv::Mat> im) : image_data_(im) {}

    virtual PixelType *FrameData() = 0;//{ return image_data_.frame_data_; }
    virtual Pixel operator()(const int r, const int c) const = 0;
    virtual cv::Mat &Mat() = 0;
    //virtual boost::shared_ptr<cv::Mat> ClassifiedImage(){return boost::shared_ptr<cv::Mat>(new cv::Mat);}
    virtual boost::shared_ptr<cv::Mat> PtrToMat() = 0; //{return image_data_.frame_;}
    virtual boost::shared_ptr<cv::Mat> PtrToClassificationMap() = 0;
    virtual cv::Mat &ClassificationMap() = 0;
    virtual int rows() const = 0;
    virtual int cols() const = 0;

  //protected:

    //__InnerImage<PixelType,Channels> image_data_;

  };

  template<typename PixelType, int Channels>
  class MonocularImage : public Image<PixelType,Channels> {

  public:

    explicit MonocularImage(boost::shared_ptr<cv::Mat> frame) {} //: Image<PixelType,Channels>() {}

    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;

    virtual Pixel operator()(const int r, const int c) const;
    virtual PixelType *FrameData() { return image_data_.frame_->data; }
    virtual cv::Mat &Mat(){ return *(this->image_data_.frame_); }
    virtual boost::shared_ptr<cv::Mat> PtrToMat() { return this->image_data_.frame_; }
    virtual boost::shared_ptr<cv::Mat> PtrToClassificationMap();
    virtual cv::Mat &ClassificationMap() { return *(classification_map_data_.frame_); }
    virtual int rows() const { return this->image_data_.frame_->rows; }

    virtual int cols() const { return this->image_data_.frame_->cols; }

  protected:

    __InnerImage<PixelType,Channels> image_data_;
    __InnerImage<unsigned char,1> classification_map_data_;

  };

  template<typename PixelType, int Channels>
  boost::shared_ptr<cv::Mat> MonocularImage<PixelType,Channels>::PtrToClassificationMap() {
    if(classification_map_data_.frame_ == 0x0) classification_map_data_.Reset(image_data_.Size());
    return classification_map_data_.frame_;
  }

  template<typename PixelType, int Channels>
  class StereoImage : public Image<PixelType,Channels> {

  public:


    //TODO SPLIT TEH DATA
    explicit StereoImage(boost::shared_ptr<cv::Mat> stereo_frame) ; //: Image(stereo_frame){}
    //explicit StereoImage(cv::Mat &left_frame,right_frame): TODO
    
    typedef typename Image<PixelType,Channels>::Pixel_ Pixel;


    virtual Pixel operator()(const int r, const int c) const;

    virtual cv::Mat &Mat(){ return *(this->left_image_data_.frame_); }

    cv::Mat &LeftMat(){ return *(this->left_image_data_.frame_); }
    cv::Mat &RightMat(){ return *(this->right_image_data_.frame_); }
    virtual PixelType *FrameData() { return left_image_data_.frame_->data; }
    virtual boost::shared_ptr<cv::Mat> PtrToMat() { return this->left_image_data_.frame_; }
    boost::shared_ptr<cv::Mat> PtrToDisparityMap();
    boost::shared_ptr<cv::Mat> PtrToPointCloud();
    virtual boost::shared_ptr<cv::Mat> PtrToClassificationMap();
    virtual cv::Mat &ClassificationMap() { return *(classification_map_data_.frame_); }
    virtual int rows() const { return this->left_image_data_.frame_->rows; }
    virtual int cols() const { return this->left_image_data_.frame_->cols; }

  protected:

    __InnerImage<PixelType,Channels> left_image_data_;
    __InnerImage<PixelType,Channels> right_image_data_;
    __InnerImage<float,3> point_cloud_data_;
    __InnerImage<short,1> disparity_map_data_;
    __InnerImage<unsigned char,1> classification_map_data_;

  };

  typedef Image<unsigned char, 3> Frame;
  typedef MonocularImage<unsigned char, 3> MonoFrame;
  typedef StereoImage<unsigned char, 3> StereoFrame;


  template<typename PixelType, int Channels>
  StereoImage<PixelType,Channels>::StereoImage(boost::shared_ptr<cv::Mat> stereo_frame){
    const int width = stereo_frame->cols/2;
    const cv::Size size(width,stereo_frame->rows);
    
    left_image_data_.Reset(size); 
    
    (*stereo_frame)(cv::Range::all(),cv::Range(0,width)).copyTo(*left_image_data_.frame_);
    right_image_data_.Reset(cv::Size(0,0));
    (*stereo_frame)(cv::Range::all(),cv::Range(width,2*width)).copyTo(*right_image_data_.frame_);
  }

  template<typename PixelType, int Channels>
  boost::shared_ptr<cv::Mat> StereoImage<PixelType,Channels>::PtrToDisparityMap() {
    if(disparity_map_data_.frame_ == 0x0) disparity_map_data_.Reset(left_image_data_.Size());
    return disparity_map_data_.frame_;
  }

  template<typename PixelType, int Channels>
  boost::shared_ptr<cv::Mat> StereoImage<PixelType,Channels>::PtrToClassificationMap() {
    if(classification_map_data_.frame_ == 0x0) classification_map_data_.Reset(left_image_data_.Size());
    return classification_map_data_.frame_;
  }

  template<typename PixelType, int Channels>
  boost::shared_ptr<cv::Mat> StereoImage<PixelType,Channels>::PtrToPointCloud() {
    if(point_cloud_data_.frame_ == 0x0) point_cloud_data_.Reset(left_image_data_.Size());
    return point_cloud_data_.frame_;
  }

  template<typename PixelType, int Channels>
  typename StereoImage<PixelType,Channels>::Pixel StereoImage<PixelType,Channels>::operator()(const int r, const int c) const  {
    Pixel px;
    const int index = (r*this->left_image_data_.frame_->cols + c)*Channels;
    memcpy(&this->left_image_data_.frame_data_[index],px.data_,Channels*sizeof(PixelType));
    return px;
  }


  template<typename PixelType, int Channels>
  typename MonocularImage<PixelType,Channels>::Pixel MonocularImage<PixelType,Channels>::operator()(const int r, const int c) const {
    Pixel px;
    const int index = (r*this->image_data_.frame_->cols + c)*Channels;
    memcpy(&this->image_data_.frame_data_[index],px.data_,Channels*sizeof(PixelType));
    return px;
  }

}

#endif
