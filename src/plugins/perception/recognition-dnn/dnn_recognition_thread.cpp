#include "dnn_recognition_thread.h"
#include <iostream>
#include <tf/types.h>
#include <interfaces/DnnRecognitionInterface.h>
#include <dlfcn.h>
#include <pthread.h>
#include <string>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iterator>
#include <algorithm>
#define CFG_PREFIX "/plugins/dnn_recognition/"

using namespace tiny_dnn;
using namespace activation;

using namespace fawkes;
using namespace std;

/** @class DnnRecognitionThread "dnn_recognition_thread.h"
 * Thread to recognize images based on trained cnns
 * @author Daniel Habering
 */


/** Constructor. */
DnnRecognitionThread::DnnRecognitionThread()
  : Thread("DnnRecognitionThread", Thread::OPMODE_WAITFORWAKEUP),
    VisionAspect(VisionAspect::CYCLIC),
    ConfigurationChangeHandler(CFG_PREFIX),
    fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER,"dnn_recognition")
{
	
}

void DnnRecognitionThread::finalize() {
	blackboard->close(mps_rec_if_);
}
	

void DnnRecognitionThread::init(){
      	
	mps_rec_if_ = blackboard->open_for_writing<DnnRecognitionInterface>("/DnnRecognition");

	std::string prefix = CFG_PREFIX;

}

template <typename Activation>
double rescale(double x) {
  Activation a(1);
  return 100.0 * (x - a.scale().first) / (a.scale().second - a.scale().first);
}

void DnnRecognitionThread::convert_image(const std::string &imagefilename,
                   double minv,
                   double maxv,
                   int w,
                   int h,
                   vec_t &data) {
  try{
  	image<> img(imagefilename, image_type::grayscale);
	image<> resized = resize_image(img, w, h);

  	// mnist dataset is "white on black", so negate required
  	std::transform(
    		resized.begin(), resized.end(), std::back_inserter(data),
    		[=](uint8_t c) { return (255 - c) * (maxv - minv) / 255.0 + minv; });

  } catch(const nn_error& e){
	logger->log_error(name(),e.what());
	throw(e);
	return;
  }
}

void DnnRecognitionThread::recognize_image(const std::string &dictionary, const std::string &src_filename) {
  tiny_dnn::network<tiny_dnn::sequential> nn;
  try{
  	nn.load(dictionary);
  } catch(const nn_error& e){
	logger->log_error(name(), e.what());
	return;
  }


  // convert imagefile to vec_t
  tiny_dnn::vec_t data;
  try{
  	convert_image(src_filename, -1.0, 1.0, 32, 32, data);
  } catch(const nn_error& e){
	return;
  }
  // recognize
  auto res = nn.predict_max_value(data);
  label_t labels = nn.predict_label(data);

  mps_rec_if_->set_final(true);
  mps_rec_if_->set_recognition_result(std::to_string(labels).c_str());
  mps_rec_if_->set_highest_score(res);
  mps_rec_if_->write();
}


void DnnRecognitionThread::loop(){
   	
   	while ( ! mps_rec_if_->msgq_empty() ) {
     			
		if ( mps_rec_if_->msgq_first_is<DnnRecognitionInterface::RecognitionMessage>() ) {
      			DnnRecognitionInterface::RecognitionMessage *m = mps_rec_if_->msgq_first<DnnRecognitionInterface::RecognitionMessage>();
					

 			logger->log_info(name(), "Received Recognition Message");
		
			mps_rec_if_->set_final(false);
			mps_rec_if_->write();

 			logger->log_info(name(), "Start Recognition");
			recognize_image(m->network_path(), m->image_path());
	
    		
    		}	
   	 	else {
    			logger->log_warn(name(), "Unknown message received");

    		}
    			
		mps_rec_if_->msgq_pop();
	}
}

