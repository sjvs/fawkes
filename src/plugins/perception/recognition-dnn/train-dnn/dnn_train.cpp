#include <iostream>
#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "tiny_dnn/tiny_dnn.h"

using namespace std;
using namespace boost::filesystem;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::core;

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
  using tanh = tiny_dnn::activation::tanh;

  using tiny_dnn::core::connection_table;
  using padding = tiny_dnn::padding;


/*    nn << conv(128, 128, 5, 1, 6,   // C1, 1@32x32-in, 6@28x28-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(15376,6)
     << tanh()
     << conv(128,128,5,6,8,padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(14400,8)
     << tanh()
     << max_pool(128,128,8,2,backend_type)
     << tanh()
     << conv(64,64,5,8,16,padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(3136,16)
     << tanh()
     << ave_pool(64, 64, 16, 2)   // S2, 6@28x28-in, 6@14x14-out
     << tanh()
     << conv(32, 32, 3, 16, 16,   // C3, 6@14x14-in, 16@10x10-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(676,16)
     << tanh()
     << ave_pool(32, 32, 16, 2)  // S4, 16@10x10-in, 16@5x5-out
     << tanh()
     << fc(16384,65536)
     << tanh()
     << conv(64, 64, 3, 16, 120,   // C5, 16@5x5-in, 120@1x1-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(121,120)
     << tanh()
     << max_pool(64,64,120,2)
     << tanh()
     << conv(32,32,5,120,60, padding::same,true,1,1,backend_type)
     << batch_normalization_layer(1024,60)
     << tanh()
     << fc(61440, 61440, true, backend_type)  // F6, 120-in, 10-out
     << tanh()
     << fc(61140, 10, true, backen_type)
     << batch_normalization_layer(10,1)
     << tanh();
*/
    nn << conv(32, 32, 3, 1, 6,   // C1, 1@32x32-in, 6@28x28-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(1024,6)
     << tanh()
     << conv(32,32,3,6,8,padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(1024,8)
     << tanh()
     << max_pool(32,32,8,2,backend_type)
     << tanh()
     << conv(16,16,5,8,16,padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(256,16)
     << tanh()
     << ave_pool(16, 16, 16, 2)   // S2, 6@28x28-in, 6@14x14-out
     << tanh()
     << conv(8, 8, 3, 16, 16,   // C3, 6@14x14-in, 16@10x10-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(64,16)
     << tanh()
    // << ave_pool(32, 32, 16, 2)  // S4, 16@10x10-in, 16@5x5-out
    // << tanh()
     << conv(8, 8, 3, 16, 120,   // C5, 16@5x5-in, 120@1x1-out
             padding::same, true, 1, 1, backend_type)
     << batch_normalization_layer(64,120)
     << tanh()
     << fc(7680, 10, true, backend_type)  // F6, 120-in, 10-out
     << batch_normalization_layer(10,1)
     << tanh();

/*nn << conv(32, 32, 5, 1, 6,   // C1, 1@32x32-in, 6@28x28-out
             padding::same, true, 1, 1, backend_type)
     << tanh()
     << ave_pool(28, 28, 6, 2)   // S2, 6@28x28-in, 6@14x14-out
     << tanh()
     << conv(14, 14, 5, 6, 16,   // C3, 6@14x14-in, 16@10x10-out
             connection_table(tbl, 6, 16),
             padding::same, true, 1, 1, backend_type)
     << tanh()
     << ave_pool(10, 10, 16, 2)  // S4, 16@10x10-in, 16@5x5-out
     << tanh()
     << conv(5, 5, 5, 16, 120,   // C5, 16@5x5-in, 120@1x1-out
             padding::same, true, 1, 1, backend_type)
     << tanh()
     << fc(120, 10, true, backend_type)  // F6, 120-in, 10-out
     << tanh();
*/
}

// convert image to vec_t
bool convert_image(const std::string& imagefilename,
                   double scale,
                   int w,
                   int h,
                   std::vector<vec_t>& data)
{
    auto img = cv::imread(imagefilename, cv::IMREAD_GRAYSCALE);
    if (img.data == nullptr) return false; // cannot open, or it's not an image

    cv::Mat_<uint8_t> resized;
    cv::resize(img, resized, cv::Size(w, h));
    vec_t d;

    std::transform(resized.begin(), resized.end(), std::back_inserter(d),
                   [=](uint8_t c) { return c * scale; });
    data.push_back(d);
    return true;
}

// convert all images found in directory to vec_t
void convert_images(const std::string& directory,
                    double scale,
                    int w,
                    int h,
                    std::vector<vec_t>& data,
		    std::vector<label_t>& labels)
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
				labels.push_back(label);
			}
    		}
      
	}
        else
            continue;
    }
    std::vector<int> indices;
    for (unsigned int i = 0; i< labels.size(); ++i){

	indices.push_back(i);
    }

    std::random_shuffle(indices.begin(), indices.end());

    std::vector<vec_t> images2;
    std::vector<label_t> labels2;
    for(unsigned int i = 0; i < indices.size(); ++i){
	images2.push_back(data[indices[i]]);
	labels2.push_back(labels[indices[i]]);
    }
    data = images2;
    labels = labels2;

    std::cout << "Label count: " << label_mapping.size() << endl;

    for(unsigned int i = 0; i < labels.size(); ++i){
	cout << labels[i] << endl;

    }
}

static void train_lenet(const std::string& data_dir_path, const std::string& test_dir_path, const std::string& output_path) {
    #ifdef CNN_SINGLE_THREAD
    std::cout << "Single Thread\n";
    #else
     std::cout << "Threads: " << CNN_TASK_SIZE << endl;	
    #endif

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
/*    parse_mnist_labels(data_dir_path + "/train-labels.idx1-ubyte",
                       &train_labels);
  parse_mnist_images(data_dir_path + "/train-images.idx3-ubyte",
                      &train_images, -1.0, 1.0, 2, 2);
   parse_mnist_labels(data_dir_path + "/t10k-labels.idx1-ubyte",
                       &test_labels);
    parse_mnist_images(data_dir_path + "/t10k-images.idx3-ubyte",
                       &test_images, -1.0, 1.0, 2, 2);
*/
    convert_images(data_dir_path,1.0,32,32,train_images,train_labels);
    convert_images(test_dir_path,1.0,32,32,test_images,test_labels);

    std::cout << "start training, images: " << train_images.size() << ", " << train_labels.size() << std::endl;

    progress_display disp(static_cast<unsigned long>(train_images.size()));
    timer t;
    int minibatch_size = 16;
    int num_epochs = 30;
    double learning_rate = 1;

  //  optimizer.alpha *= static_cast<tiny_dnn::float_t>(2.4);
    optimizer.alpha *=
    	std::min(tiny_dnn::float_t(1),
             static_cast<tiny_dnn::float_t>(sqrt(minibatch_size) * learning_rate));


    // create callback
    auto on_enumerate_epoch = [&](){
        std::cout << t.elapsed() << "s elapsed." << std::endl;
        tiny_dnn::result res = nn.test(test_images, test_labels);
        std::cout << res.num_success << "/" << res.num_total << std::endl;
        
	disp.restart(static_cast<unsigned long>(train_images.size()));
        t.restart();
    };

    auto on_enumerate_minibatch = [&](){
        disp += minibatch_size;
    };

    // training
    nn.train<mse>(optimizer, train_images, train_labels, minibatch_size, num_epochs,
             on_enumerate_minibatch, on_enumerate_epoch);

    std::cout << "end training." << std::endl;

    // test and show results
//    nn.test(test_images, test_labels).print_detail(std::cout);

    // save network model & trained weights
    nn.save(output_path);
}

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cerr << "Usage : " << argv[0]
                  << " path_to_data (example:../data) path_to_test_data output_path" << std::endl;
        return -1;
    }
    train_lenet(argv[1],argv[2],argv[3]);
    return 0;
}
