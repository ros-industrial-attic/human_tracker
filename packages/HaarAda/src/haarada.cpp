/**
 * @file /src/haarada.cpp
 *
 * @brief Ros communication central!
 *
 * @date Nov 2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "HaarAda/haarada.hpp"
#include <stdlib.h>
#include <algorithm>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace HaarAda {

/*****************************************************************************
** Implementation
*****************************************************************************/
  HaarAdaClassifier::HaarAdaClassifier()
  {
    classifier_filename_ = "UnNamedHaarAda.xml";
    HaarAdaPrior_        = exp(2);// default prior
    loaded = false;
    init();
  }

  HaarAdaClassifier::HaarAdaClassifier(string filename)
  {  
    classifier_filename_ = filename;
    HaarAdaPrior_        = exp(2);// default prior
    loaded = false;
    init();
  }

  void HaarAdaClassifier::detect(vector<Rect> &R_in, 
				 vector<int>  &L_in,
				 Mat &I_in, 
				 vector<Rect> &R_out, 
				 vector<int> &L_out,
				 bool label_all)
  {
    int count =0;
    Mat HF(1,num_filters_,CV_32F);

    R_out.clear();
    L_out.clear();
    if(!loaded) return;
    for(unsigned int i=0;i<R_in.size();i++){
      float result = 0;
      if(R_in[i].width > 2 && R_in[i].height > 2){
//	setImageRoi(R_in[i],I_in);
//	int rtn = haar_features(HF);
        setImageROI_fast(R_in[i],I_in);
        int rtn = haar_features_fast(HF);
	if(rtn== 1){
	  //	  result = HAC_.predict(HF);
	  Mat missing;
	  Range slice     = Range::all();
	  bool raw_mode   = false;
	  bool return_sum = true;
	  result = HAC_.predict(HF,missing,slice,raw_mode,return_sum);
	}
        else{
	  ROS_ERROR("WHY O WHY");
          result = 0;
	}
      }
      if(result>0 || label_all == true){
	R_out.push_back(R_in[i]);
	if(label_all) L_out.push_back(result); // apply the label
	if(!label_all)L_out.push_back(L_in[i]); // give the same label it came in with to allow eval
	if(result>0) count++;
      }
    }
  }

  int HaarAdaClassifier::addToTraining(vector<Rect> R_in, vector<int> L_in, Mat I_in)
  {
    Mat HF(1,num_filters_,CV_32F);
    for(unsigned int i = 0; i<R_in.size(); i++){
      if(R_in[i].width < 2 || R_in[i].height <2){
	// do nothing with really small rois
      }
      else if(numSamples_<maxSamples_){// not too many samples already
	setImageROI_fast(R_in[i],I_in);
	int rtn = haar_features_fast(HF);
	if(rtn){
	  for(int j=0;j<num_filters_;j++){// copy the subset of samples 
	    trainingSamples_.at<float>(numSamples_,j) = HF.at<float>(0,j);
	  }
	  if(L_in[i] <= 0 ){
	    trainingLabels_.at<int>(numSamples_,0) = 0;//classes: 0,1  not -1,1 
	  }
	  else{
	    trainingLabels_.at<int>(numSamples_,0) = L_in[i];
	  }
	  numSamples_++;
	}// end if successful compute feature
      }// end of if not too many samples already
    }// end each roi
    return(numSamples_);
  }// end addToTraining

  void HaarAdaClassifier::setImageRoi(Rect R_in, Mat I_in)
  {
    int xmin = R_in.x;
    int xmax = xmin+R_in.width;
    int ymin = R_in.y;
    int ymax = ymin+R_in.height;
    Mat Cropped_Image = I_in(Range(ymin,ymax),Range(xmin,xmax)); // extract ROI
    resize(Cropped_Image,Scaled_Image, Size(16,16), 0, 0,INTER_AREA ); // resize to 16x16
    Scaled_Image.convertTo(Bit8_Image,CV_8UC1); // convert to  8 bit gray scale
    equalizeHist(Bit8_Image, Equal_Gray_Image); // do histogram normalization
    Equal_Gray_Image.convertTo(Image4Haar,CV_16SC1); // convert to  16 bit gray scale
  }

  void HaarAdaClassifier::setImageROI_fast(Rect R_in, Mat I_in)
  {
    cv::resize(I_in(R_in), haar16x16, Size(16,16), 0, 0, CV_INTER_AREA);
  }

  void HaarAdaClassifier::train(string filename)
  {
    CvBoostParams bparams = CvBoostParams();
    ROS_ERROR("HaarAdaPrior_ = %f",HaarAdaPrior_);
    float priorFloat[] = { 1.0, HaarAdaPrior_ };  // preliminary priors based on ROC)
    bparams.priors = &priorFloat[0];
    bparams.use_surrogates = false;
    bparams.weak_count = 100;
    Mat HF(1,num_filters_,CV_32F);

    // copy sub matrix for training 
    Mat VarIdx;
    Mat Features(numSamples_,num_filters_,CV_32F);
    for(int i=0;i<numSamples_;i++){
      for(int j=0;j<num_filters_;j++){
	Features.at<float>(i,j) = trainingSamples_.at<float>(i,j);
      }
    }
    Mat Responses(numSamples_,1,CV_32S);
    for(int i=0; i<numSamples_;i++){
      if(trainingLabels_.at<int>(i,0) == -1) trainingLabels_.at<int>(i,0) = 0;//classes: 0,1  not -1,1 
      Responses.at<int>(i,0) = trainingLabels_.at<int>(i,0);
    }
    Mat vIdx,sIdx,vtyp,mdm;
    HAC_.train(Features, CV_ROW_SAMPLE, Responses,vIdx,sIdx,vtyp,mdm,bparams,false);
    loaded = true;// ok to use now
    ROS_ERROR("saving trained classifier to %s",filename.c_str());
    HAC_.save(filename.c_str());
    // Determine Recall Statistics
    int num_TP = 0;
    int num_FP = 0;
    int num_people = 0;
    int num_neg = 0;
    int num_TN = 0;
    int num_FN = 0;
    for(int i=0;i<Features.rows;i++){
      float result = HAC_.predict(Features.row(i));
      if(Responses.at<int>(i,0) == 1) num_people++;
      if(Responses.at<int>(i,0) != 1) num_neg++;
      if(result==1 && Responses.at<int>(i,0) == 1) num_TP++; // true pos
      else if(result==1 && Responses.at<int>(i,0) == 0) num_FP++; // false pos
      else if(result==0 && Responses.at<int>(i,0) == 0) num_TN++; // true neg
      else if(result==0 && Responses.at<int>(i,0) == 1) num_FN++; // false neg
    }
    float percent = (float)num_TP/(float)num_people*100.0;
    ROS_ERROR("Recall = %6.2f%c",percent,'%');
    percent = (float)num_FP/(float)num_neg*100.0;
    ROS_ERROR("False Positives = %6.2f%c",percent,'%');
  }

  void HaarAdaClassifier::setMaxSamples(int n)
  {
    maxSamples_ = n;
    numSamples_ = 0;
    trainingSamples_.create(maxSamples_,num_filters_,CV_32FC1);
    trainingLabels_.create(maxSamples_,1,CV_32SC1);
  }  
  void HaarAdaClassifier::init()
  {
    num_filters_ = 174;
    setMaxSamples(350); // Default to running in detect with very little memory for training
    AvePosTrainingImg_.create(64,64,CV_64F);
    
    // images used by haar_features() set here since known size and type to avoid realloc
    map.create(16,16,CV_16SC1);
    WaveletResponse.create(16,16,CV_16SC1);

    // image of setImageRoi() set here since known size and type to avoid realloc
    Scaled_Image.create(16,16,CV_32FC1);
    Bit8_Image.create(16,16,CV_8UC1); 
    Equal_Gray_Image.create(16,16,CV_8UC1);
    Image4Haar.create(16,16,CV_16SC1);

    // faster haar
    haar16x16.create(16, 16, CV_8U);
    haar8x8.create(8, 8, CV_8U);
    haar4x4.create(4, 4, CV_8U);
  }

  void HaarAdaClassifier::load(string filename)
  {
    HAC_.load(filename.c_str());
    loaded = true;
  }

  int HaarAdaClassifier::test()
  {
    return(1);
  }

  void  HaarAdaClassifier::computeIntegralImage(Mat image_in)
  {
    cv::integral(image_in, integralImage_, CV_32F );
  }

  // TODO find out what this function is for
  void HaarAdaClassifier::alpha_map(int idx)
  {
    int i, j, rowWidth, lPower, lPower2, type;

    type = idx % 3;  // find type (horz, vert, diag)

    if (idx >= 174)
    {  // in 16x16, there are 225 2x2s
      rowWidth = 15;
      idx = idx - 174;
      lPower = 2;
      lPower2 = 1;
    }
    else if (idx >= 27)
    {  // in 16x16, there are 49 4x4s
      rowWidth = 7;
      idx = idx - 27;
      lPower = 4;
      lPower2 = 2;
    }
    else
    {  // in 16x16, there are 9 8x8s
      rowWidth = 3;
      lPower = 8;
      lPower2 = 4;
    }  // if elseif else

    int i1 = (idx / 3) % rowWidth;
    int j1 = (idx / 3) / rowWidth;

    
    map = cv::Mat::zeros(map.rows,map.cols,CV_16SC1);

    for (i = i1 * lPower2; i < i1 * lPower2 + lPower; i++)
    {  // width/x
      for (j = j1 * lPower2; j < j1 * lPower2 + lPower; j++)
      {  // height/y
        if (type == 0)
        {  // horz type
          if (i >= i1 * lPower2 + lPower2)
          {
	    map.at<short int>(i,j) = 1;
          }
          else
          {
	    map.at<short int>(i,j) = -1;
          }
        }
        else if (type == 1)
        {  // vert type
          if (j >= j1 * lPower2 + lPower2)
          {
	    map.at<short int>(i,j) = 1;
          }
          else
          {
	    map.at<short int>(i,j) = -1;
          }
        }
        else
        {  // diag type (type == 2)
          if (((i >= i1 * lPower2 + lPower2) && (j >= j1 * lPower2 + lPower2))
              || ((i < i1 * lPower2 + lPower2) && (j < j1 * lPower2 + lPower2)))
          {
	    map.at<short int>(i,j) = 1;
          }
          else
          {
	    map.at<short int>(i,j) = -1;
          }
        }  // if
      }  // for j
    }  // for i
  }  // alpha map


  int HaarAdaClassifier::haar_features(Mat &HF)
  {
    if(Image4Haar.rows != 16 || Image4Haar.cols !=16)
      {
	ROS_ERROR("Wrong sz Input4Haar haar_response %dX%d ",Image4Haar.rows,Image4Haar.cols);
	return(0);
      }
    
    for (int k = 0; k < num_filters_; k++) // do each feature
      {
	alpha_map(k);  // generate the map,
	//print_map(k,map);
	multiply(map, Image4Haar, WaveletResponse);  // elementwise multiplication
	HF.at<float>(0,k) = abs(sum(WaveletResponse)[0]);
      } 
    return(1);
  }// haar_features

  void HaarAdaClassifier::print_map(int k)
  {
    printf("Map %d :\n",k);
    alpha_map(k);
    for(int i=0;i<map.rows;i++){
      for(int j=0;j<map.cols;j++){
	printf("%3d ",map.at<short int>(i,j));
      }
      printf("\n");
    }
  }

  int HaarAdaClassifier::haar_features_fast(Mat &HF)
  {
    int jl, idx, i1, j1;

    cv::resize(haar16x16, haar8x8, cv::Size(8,8), 0, 0, CV_INTER_AREA);
    cv::resize(haar8x8, haar4x4, cv::Size(4,4), 0, 0, CV_INTER_AREA);

    float* hf_ptr = HF.ptr<float>(0);

    if(haar16x16.rows != 16 || haar16x16.cols !=16)
      {
  ROS_ERROR("Wrong sz Input4Haar haar_response %dX%d ",haar16x16.rows,haar16x16.cols);
  return(0);
      }


    // Largest features are first (8x8s)
    for (jl = 0; jl < 27; jl += 3)
    {  // in 16x16 template, there are 27 8x8s (9 positions, 3 types, windows may overlap by 50%)
      idx = jl;
      i1 = (idx / 3) % 3;  // "x" value of our location in map
      j1 = (idx / 3) / 3;  // "y" value of our location in map
      hf_ptr[jl] = 16 * (-haar4x4.at<uint8_t>(j1,i1) + haar4x4.at<uint8_t>(j1,i1+1)
                      -haar4x4.at<uint8_t>(j1 +1 ,i1) + haar4x4.at<uint8_t>(j1 + 1,i1 + 1));

      hf_ptr[jl+1] = 16 * (-haar4x4.at<uint8_t>(j1,i1) - haar4x4.at<uint8_t>(j1,i1+1)
                         +haar4x4.at<uint8_t>(j1 +1 ,i1) + haar4x4.at<uint8_t>(j1 + 1,i1 + 1));

      hf_ptr[jl+2] = 16 * (haar4x4.at<uint8_t>(j1,i1) - haar4x4.at<uint8_t>(j1,i1+1)
                        -haar4x4.at<uint8_t>(j1 +1 ,i1) + haar4x4.at<uint8_t>(j1 + 1,i1 + 1));
    }  // for

    // Medium features are next (4x4s)
    for (jl = 27; jl < 174; jl += 3)
    {  // in 16x16 template, there are 147 4x4s (49 positions, 3 types, windows may overlap by 50%)
      idx = jl - 27;
      i1 = (idx / 3) % 7;  // "x" value of our location in map
      j1 = (idx / 3) / 7;  // "y" value of our location in map
      hf_ptr[jl] = 4 * (-haar8x8.at<uint8_t>(j1,i1) + haar8x8.at<uint8_t>(j1,i1+1)
                      -haar8x8.at<uint8_t>(j1+1 ,i1) + haar8x8.at<uint8_t>(j1 + 1,i1 + 1));

      hf_ptr[jl+1] = 4 * (-haar8x8.at<uint8_t>(j1,i1) - haar8x8.at<uint8_t>(j1,i1+1)
                         +haar8x8.at<uint8_t>(j1 +1 ,i1) + haar8x8.at<uint8_t>(j1 + 1,i1 + 1));

      hf_ptr[jl+2] = 4 * (haar8x8.at<uint8_t>(j1,i1) - haar8x8.at<uint8_t>(j1,i1+1)
                        -haar8x8.at<uint8_t>(j1 +1 ,i1) + haar8x8.at<uint8_t>(j1 + 1,i1 + 1));
    }  // for

    HF = abs(HF);

//    // Smallest features are last (2x2s)
//    for (jl = 174; jl < 849; jl += 3)
//    {  // in 16x16 template, there are 675 2x2s (225 positions, 3 types, windows may overlap by 50%)
//      idx = jl - 174;
//      i1 = ((idx / 3) % 15);  // Lookup x location
//      j1 = ((idx / 3) / 15);  // Lookup y location
//      hf_ptr[jl] = -haar16x16.at<uint8_t>(j1,i1) + haar16x16.at<uint8_t>(j1,i1+1)
//                      -haar16x16.at<uint8_t>(j1 +1 ,i1) + haar16x16.at<uint8_t>(j1 + 1,i1 + 1);
//
//      hf_ptr[jl+1] = -haar16x16.at<uint8_t>(j1,i1) - haar16x16.at<uint8_t>(j1,i1+1)
//                         +haar16x16.at<uint8_t>(j1 +1 ,i1) + haar16x16.at<uint8_t>(j1 + 1,i1 + 1);
//
//      hf_ptr[jl+2] = haar16x16.at<uint8_t>(j1,i1) - haar16x16.at<uint8_t>(j1,i1+1)
//                        -haar16x16.at<uint8_t>(j1 +1 ,i1) + haar16x16.at<uint8_t>(j1 + 1,i1 + 1);
//    }
    return 1;

  }  // haar_features_fast

} // namespace HaarAda
