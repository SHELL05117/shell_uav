// Query a model using the already running NPU; no ROS/control interfaces.
#include <rknn_api.h>
#include <fstream>
#include <iostream>
#include <vector>
static void attr(rknn_context context, rknn_query_cmd command, unsigned index) {
    rknn_tensor_attr value{}; value.index=index;
    int result=rknn_query(context,command,&value,sizeof(value));
    if (result<0) throw result;
    std::cout<<(command==RKNN_QUERY_INPUT_ATTR?"input":"output")<<" index="<<index
        <<" name="<<value.name<<" n_dims="<<value.n_dims<<" dims=";
    for (unsigned i=0;i<value.n_dims;++i) std::cout<<(i?",":"")<<value.dims[i];
    std::cout<<" fmt="<<value.fmt<<" type="<<value.type<<" qnt_type="<<value.qnt_type
        <<" zp="<<value.zp<<" scale="<<value.scale<<" n_elems="<<value.n_elems
        <<" size="<<value.size<<std::endl;
}
int main(int argc,char**argv) {
    if(argc!=2)return 2;
    std::ifstream file(argv[1],std::ios::binary|std::ios::ate);
    if(!file||file.tellg()<=0)return 2;
    std::vector<char> data(static_cast<size_t>(file.tellg()));
    file.seekg(0); file.read(data.data(),data.size());
    rknn_context context=0;
    int result=rknn_init(&context,data.data(),data.size(),0);
    if(result<0){std::cerr<<"init="<<result<<std::endl;return 1;}
    try {
        rknn_input_output_num count{};
        result=rknn_query(context,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count));
        if(result<0)throw result;
        for(unsigned i=0;i<count.n_input;++i)attr(context,RKNN_QUERY_INPUT_ATTR,i);
        for(unsigned i=0;i<count.n_output;++i)attr(context,RKNN_QUERY_OUTPUT_ATTR,i);
    } catch(int error) {std::cerr<<"query="<<error<<std::endl;rknn_destroy(context);return 1;}
    rknn_destroy(context);
}
