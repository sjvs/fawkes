#include <iostream>
#include <thread> 
#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/export.hpp> 
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include "tiny_dnn/tiny_dnn.h"
#include "caffe.pb.h"

using namespace std;
using namespace boost::filesystem;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::core;

struct Dataset{
	vec_t data;
	std::string path;
	label_t label;
};

static void construct_net(network<sequential>& nn) {
    // connection table, see Table 1 in [LeCun1998]
#define O true
#define X false
/*    static const bool tbl[] = {
        O, X, X, X, O, O, O, X, X, O, O, O, O, X, O, O,
        O, O, X, X, X, O, O, O, X, X, O, O, O, O, X, O,
        O, O, O, X, X, X, O, O, O, X, X, O, X, O, O, O,
        X, O, O, O, X, X, O, O, O, O, X, X, O, X, O, O,
        X, X, O, O, O, X, X, O, O, O, O, X, O, O, X, O,
        X, X, X, O, O, O, X, X, O, O, O, O, X, O, O, O
    };*/
#undef O
#undef X


    // by default will use backend_t::tiny_dnn unless you compiled
    // with -DUSE_AVX=ON and your device supports AVX intrinsics
    core::backend_t backend_type = core::backend_t::avx;

    // construct nets
    //
    // C : convolution
    // S : sub-sampling
    // F : fully connected
  using fc = tiny_dnn::layers::fc;
  using conv = tiny_dnn::layers::conv;
  using ave_pool = tiny_dnn::layers::ave_pool;
  using max_pool = tiny_dnn::layers::max_pool;
//  using tanh = tiny_dnn::activation::tanh;
  using leaky_relu = tiny_dnn::activation::leaky_relu;
  using softmax = tiny_dnn::activation::softmax;
//  using tiny_dnn::core::connection_table;
//  using padding = tiny_dnn::padding;


  nn << conv(64,64,5,1,6, padding::same, true, 1,1,backend_type)
     << leaky_relu()
     << ave_pool(64,64,6,2)
     << leaky_relu()
     << conv(32,32,5,6,12, padding::same, true, 1, 1,backend_type)
     << leaky_relu()
     << max_pool(32,32,12,2)
     << leaky_relu()
     << conv(16,16,3,12,24,padding::same, true, 1,1,backend_type)
     << leaky_relu()
     << fc(16*16*24,4096,true,backend_type)
     << leaky_relu()
     << ave_pool(64,64,1,2) 
     << leaky_relu()
     << dropout_layer(32*32,0.3)
     << leaky_relu()
//     << fc(1024,512,true,backend_type)
//     << leaky_relu()
     << fc(1024,10,true,backend_type)
     << softmax();
}

// convert image to vec_t
bool convert_image(const std::string& imagefilename,
                   double scale,
                   int w,
                   int h,
                   Dataset& data)
{
    auto img = cv::imread(imagefilename);
    if (img.data == nullptr) {
	    std::cout << "Can not open " << imagefilename << std::endl;
	    return false; // cannot open, or it's not an image
    }

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(w, h));
    std::vector<float> input_vec;
    for(int i = resized.channels()-1; i > -1;i--){
	    for(int j = 0; j < resized.cols; ++j){
		    for(int k = 0; k < resized.rows; k++){
			input_vec.push_back(resized.at<cv::Vec3b>(k,j)[i]);
		    }
	    }
    }
    vec_t d(input_vec.begin(),input_vec.end());
    data.data = d;
    return true;
}

// convert all images found in directory to vec_t
void convert_images(const std::string& directory,
                    double scale,
                    int w,
                    int h,
                    std::vector<Dataset>& training_set)
{
    std::vector<std::string> label_mapping;
    path p(directory);

     for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (is_directory(i->path())) //we eliminate directories
        {
   		BOOST_FOREACH(const path& p, std::make_pair(directory_iterator(i->path().string()), directory_iterator())) {
        		if (is_directory(p)) continue;
			
			std::vector<std::string> parts;
			boost::split(parts,p.string(), [](char c){return c == '/';});
			
			Dataset data;

			std::string label_string = parts[parts.size()-2];	
			label_t label = 9999;
			for(unsigned int i = 0; i < label_mapping.size(); ++i){
				if(label_mapping[i] == label_string){
					label = i;
					break;
				}
			}
			if(label == 9999){
				label_mapping.push_back(label_string);
				label = label_mapping.size()-1;
			}
				
			if(convert_image(p.string(), scale, w, h, data)){
				data.label = label;
				data.path = p.string();
				training_set.push_back(data);
			}
    		}
      
	}
        else
            continue;
    }
    std::vector<int> indices;
    for (unsigned int i = 0; i< training_set.size(); ++i){

	indices.push_back(i);
    }

    std::random_shuffle(indices.begin(), indices.end());

    std::vector<Dataset> training_set2;
    for(unsigned int i = 0; i < indices.size(); ++i){
	training_set2.push_back(training_set[indices[i]]);
    }
    training_set = training_set2;


    std::cout << "Training set:  " << training_set.size() << endl;

}


void extract_features(const std::string& datadir, 
				const std::string& model_file, 
				const std::string& trained_file, 
				std::vector<Dataset>& training_set, 
				bool overwrite = false){

    auto nn = tiny_dnn::create_net_from_caffe_prototxt(model_file);
    tiny_dnn::reload_weight_from_caffe_protobinary(trained_file,nn.get());
    int width = (*nn)[0]->in_data_shape()[0].width_;
    int height = (*nn)[0]->in_data_shape()[0].height_;

    convert_images(datadir,1,width,height,training_set);

    std::vector<std::string> parts;
    for(unsigned int i = 0; i < training_set.size(); ++i){
	//Check if file is there
	boost::split(parts, training_set[i].path, [](char c){return c == '.';});
	std::string feature_file;
	for(unsigned int i = 0; i < parts.size()-1;++i){
		feature_file += parts[i] + ".";
	}
	feature_file += "features";
	
	std::ifstream in(feature_file.c_str(),ios::binary);
	bool successfully_loaded = false;
	if(in.good()&& !overwrite){
		vec_t loaded;
		try{
			boost::archive::binary_iarchive ia(in);	
			ia >> loaded;
			in.close();

			std::cout << "loaded dim: " << loaded.size() << std::endl;
			vec_t loaded_vec(loaded.begin(),loaded.end());
			successfully_loaded = true;
			training_set[i].data = loaded;
		}catch (...){
			std::cout << "Invalid input file " << feature_file << std::endl;
		}

	}
	if(!successfully_loaded){
		vec_t result = nn->predict(training_set[i].data);
		training_set[i].data=result;
		std::cout << "Result dim: " << result.size() << std::endl;
		std::ofstream out(feature_file.c_str());
		stringstream ss;
		boost::archive::binary_oarchive oa(ss);
		oa << result;
		out << ss.str();
		out.close();
	}
        std::cout << i << "/" << training_set.size() << std::endl;	
    }

    std::cout << "Training set size: " << training_set.size() << std::endl;

}


static void train_lenet(const std::string& data_dir_path, 	
				const std::string& test_dir_path, 
				const std::string& caffe_prototxt_path, 
				const std::string& caffemodel_path, 
				const std::string& output_path,bool overwrite) {
    // specify loss-function and learning strategy
    
    network<sequential> nn;
    adagrad optimizer;

    construct_net(nn);

    std::cout << "load models..." << std::endl;

    // load MNIST dataset
    std::vector<label_t> train_labels;
    std::vector<vec_t> train_images;
    std::vector<label_t> test_labels;
    std::vector<vec_t> test_images;

    std::vector<Dataset> training_set;
    std::vector<Dataset> test_set;

    extract_features(data_dir_path,caffe_prototxt_path,caffemodel_path,training_set,overwrite);
    extract_features(test_dir_path,caffe_prototxt_path,caffemodel_path,test_set,overwrite);

//    convert_images(data_dir_path,1,64,64,training_set);
//    convert_images(test_dir_path,1,64,64,test_set);
    std::cout << "start training, images: " << training_set.size() << ", " << test_set.size() << std::endl;

    for(unsigned int i = 0; i < training_set.size(); ++i){
	train_labels.push_back(training_set[i].label);
	train_images.push_back(training_set[i].data);
	
	std::cout << training_set[i].label << "          " << training_set[i].path << std::endl;
    }

    std::cout << "Test set..." << std::endl;

    for(unsigned int i = 0; i < test_set.size(); ++i){
	test_labels.push_back(test_set[i].label);
	test_images.push_back(test_set[i].data);
	
	std::cout << test_set[i].label << "          " << test_set[i].path << std::endl;
    }


    progress_display disp(static_cast<unsigned long>(train_images.size()));
    timer t;
    int minibatch_size = 5;
    int num_epochs = 30;
    double learning_rate = 0.1;

    optimizer.alpha *= static_cast<tiny_dnn::float_t>(learning_rate);


    // create callback
    auto on_enumerate_epoch = [&](){
        std::cout << t.elapsed() << "s elapsed." << std::endl;
        tiny_dnn::result res = nn.test(test_images,test_labels);
        std::cout << res.num_success << "/" << res.num_total << std::endl;
        
	disp.restart(static_cast<unsigned long>(train_images.size()));
        t.restart();
    };

    auto on_enumerate_minibatch = [&](){
        disp += minibatch_size;
    };

    // training
    nn.train<cross_entropy>(optimizer, train_images, train_labels, minibatch_size, num_epochs,
             on_enumerate_minibatch, on_enumerate_epoch);

    std::cout << "end training." << std::endl;

    // test and show results
//    nn.test(test_images, test_labels).print_detail(std::cout);

    // save network model & trained weights
    nn.save(output_path);
}

int main(int argc, char **argv) {
    if (argc != 7) {
        std::cerr << "Usage : " << argv[0]
                  << " path_to_data path_to_test_data path_to_caffe_prototxt path_to_caffemodel output_path" << std::endl;
        return -1;
    }
    bool overwrite;
    std::string arg(argv[6]);
    if(arg == "true"){
	    overwrite = true;
    }
    else{
	    overwrite = false;
    }
    train_lenet(argv[1],argv[2],argv[3],argv[4], argv[5],overwrite);
    return 0;
}
