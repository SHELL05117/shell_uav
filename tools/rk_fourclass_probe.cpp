// Isolated real-NPU raw-head check for the unmodified incoming 4-class model.
#include <rknn_api.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
static void require(int result,const char*stage){if(result<0)throw std::runtime_error(std::string(stage)+":"+std::to_string(result));}
static std::vector<char> read(const std::string&path){
    std::ifstream f(path,std::ios::binary|std::ios::ate);
    if(!f||f.tellg()<=0)throw std::runtime_error("Cannot read "+path);
    std::vector<char>b(static_cast<size_t>(f.tellg()));f.seekg(0);f.read(b.data(),b.size());
    if(!f)throw std::runtime_error("Read failed "+path);return b;
}
int main(int argc,char**argv){
    if(argc!=5){std::cerr<<"probe model.rknn input_dir manifest output_dir\n";return 2;}
    rknn_context context=0;
    try{
        auto model=read(argv[1]);require(rknn_init(&context,model.data(),model.size(),0),"init");
        rknn_input_output_num count{};require(rknn_query(context,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)),"count");
        if(count.n_input!=1||count.n_output!=3)throw std::runtime_error("Unexpected IO count");
        std::vector<rknn_tensor_attr>attrs(3);
        for(unsigned i=0;i<3;++i){
            auto&a=attrs[i];a.index=i;require(rknn_query(context,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a)),"output_attr");
            unsigned side=52u>>i;
            if(a.n_dims!=5||a.dims[0]!=9||a.dims[1]!=side||a.dims[2]!=side||a.dims[3]!=3||a.dims[4]!=1)
                throw std::runtime_error("Unverified head layout");
        }
        std::ifstream list(argv[3]);if(!list)throw std::runtime_error("Missing manifest");
        std::ofstream timing(std::string(argv[4])+"/timings.tsv");timing<<"stem\tset_run_get_ms\n";
        std::string stem;unsigned images=0;
        while(std::getline(list,stem)){
            if(!stem.empty()&&stem.back()=='\r')stem.pop_back();
            if(stem.empty()||stem.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
                throw std::runtime_error("Invalid input basename");
            auto rgb=read(std::string(argv[2])+"/"+stem+".rgb");
            if(rgb.size()!=416*416*3)throw std::runtime_error("Wrong RGB input size");
            rknn_input input{};input.index=0;input.buf=rgb.data();input.size=rgb.size();input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;
            auto start=std::chrono::steady_clock::now();
            require(rknn_inputs_set(context,1,&input),"inputs_set");require(rknn_run(context,nullptr),"run");
            rknn_output output[3]{};for(unsigned i=0;i<3;++i){output[i].index=i;output[i].want_float=1;}
            require(rknn_outputs_get(context,3,output,nullptr),"outputs_get");
            double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            bool valid=true;
            for(unsigned i=0;i<3;++i){
                if(output[i].size!=attrs[i].n_elems*sizeof(float)){valid=false;break;}
                std::ofstream f(std::string(argv[4])+"/"+stem+"_"+std::to_string(i)+".f32",std::ios::binary);
                f.write(static_cast<const char*>(output[i].buf),output[i].size);valid=valid&&bool(f);
            }
            require(rknn_outputs_release(context,3,output),"release");if(!valid)throw std::runtime_error("Output byte count/write mismatch");
            timing<<stem<<'\t'<<ms<<'\n';std::cout<<++images<<' '<<stem<<' '<<ms<<" ms; float_bytes=292032,73008,18252"<<std::endl;
        }
        rknn_destroy(context);std::cout<<"completed="<<images<<std::endl;return images?0:1;
    }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;if(context)rknn_destroy(context);return 1;}
}
