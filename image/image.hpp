template<typename PixelType, int Channels>
class __InnerImage {

public:
  cv::Mat frame_;
  PixelType *frame_data_;
  friend class Image;

protected:

  explicit InnerImage_(cv::Mat &frame):frame_(frame){
    frame_data_ = (PixelType *)frame_.data;
  }

};

template<typename PixelType, int Channels>
class Image {

public:

  struct Pixel_ {
    PixelType data_[Channels];
  };

  typedef Pixel_ Pixel;

  virtual Pixel operator()(const int r, const int c);
    
protected:
  
  __InnerImage image_data_;

};

template<typename PixelType, int Channels>
class MonocularImage : Image<PixelType,Channels> {

public:
  explicit MonocularImage(cv::Mat &frame):Image(frame){}
  virtual Pixel operator()(const int r, const int c){
    Pixel r;
    memcpy(image_data_.frame_data_,r.data_,Channels*sizeof(PixelType));
    return r;
  }

};


template<typename PixelType, int Channels>
class StereoImage : Image<PixelType,Channels> {

public:
  
  explicit StereoImage(cv::Mat &stereo_frame) : Image(frame){}
  //explicit StereoImage(cv::Mat &left_frame,right_frame):

private:
};
