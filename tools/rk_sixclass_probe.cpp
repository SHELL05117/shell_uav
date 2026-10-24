// Isolated RKNN 1.7.5 hardware probe. No ROS or actuator interfaces.
// Build against the board's official rknn_api.h and librknn_api.so.
#include <rknn_api.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static void ok(int ret, const char* stage) {
    if (ret < 0) throw std::runtime_error(std::string(stage)+": "+std::to_string(ret));
}
static std::vector<char> read(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open "+path);
    auto n = f.tellg();
    if (n <= 0) throw std::runtime_error("empty "+path);
    std::vector<char> b(static_cast<size_t>(n));
    f.seekg(0); f.read(b.data(), b.size());
    if (!f) throw std::runtime_error("read failed "+path);
    return b;
}
int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: probe model.rknn input_dir manifest.txt existing_output_dir\n";
        return 2;
    }
    rknn_context ctx = 0;
    try {
        auto model = read(argv[1]);
        ok(rknn_init(&ctx, model.data(), model.size(), 0), "init");
        rknn_sdk_version version{};
        ok(rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)), "version");
        std::cout << "api=" << version.api_version << " driver=" << version.drv_version << std::endl;
        rknn_input_output_num num{};
        ok(rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &num, sizeof(num)), "io_num");
        if (num.n_input != 1 || num.n_output != 3) throw std::runtime_error("expected one input, three outputs");
        rknn_tensor_attr input{};
        ok(rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input)), "input_attr");
        if (input.n_elems != 640*640*3) throw std::runtime_error("expected 640 RGB input");
        std::ofstream meta(std::string(argv[4])+"/tensors.tsv");
        if (!meta) throw std::runtime_error("cannot write tensor metadata");
        meta << "index\tname\tfmt\ttype\tzp\tscale\telements\tdims_api_order\n";
        std::vector<rknn_tensor_attr> attrs(3);
        for (unsigned i=0; i<3; ++i) {
            auto& a=attrs[i]; a.index=i;
            ok(rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &a, sizeof(a)), "output_attr");
            const unsigned side=80u>>i;
            // The legacy API reports dimensions in reverse order (W,H,C,N).
            if (a.n_dims!=4 || a.dims[0]!=side || a.dims[1]!=side || a.dims[2]!=33 || a.dims[3]!=1 || a.fmt!=RKNN_TENSOR_NCHW)
                throw std::runtime_error("unexpected six-class head layout");
            meta<<i<<'\t'<<a.name<<'\t'<<a.fmt<<'\t'<<a.type<<'\t'<<a.zp<<'\t'<<a.scale<<'\t'<<a.n_elems<<'\t';
            for (unsigned j=0;j<a.n_dims;++j) meta<<(j?",":"")<<a.dims[j];
            meta<<'\n';
            std::cout<<"head "<<i<<" channels=33 side="<<side<<" elements="<<a.n_elems<<std::endl;
        }
        meta.close();
        std::ifstream list(argv[3]);
        if (!list) throw std::runtime_error("cannot read manifest");
        std::ofstream timings(std::string(argv[4])+"/timings.tsv");
        if (!timings) throw std::runtime_error("cannot write timings");
        timings<<"stem\tset_run_get_ms\n";
        std::string stem; unsigned count=0;
        while (std::getline(list,stem)) {
            if (stem.empty() || stem.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
                throw std::runtime_error("unsafe manifest stem");
            auto rgb=read(std::string(argv[2])+"/"+stem+".rgb");
            if (rgb.size()!=640*640*3) throw std::runtime_error("incorrect RGB byte count");
            rknn_input in{}; in.index=0; in.buf=rgb.data(); in.size=rgb.size();
            in.type=RKNN_TENSOR_UINT8; in.fmt=RKNN_TENSOR_NHWC; in.pass_through=0;
            auto start=std::chrono::steady_clock::now();
            ok(rknn_inputs_set(ctx,1,&in), "inputs_set");
            ok(rknn_run(ctx,nullptr), "run");
            rknn_output out[3]{};
            for(unsigned i=0;i<3;++i) { out[i].index=i; out[i].want_float=1; }
            ok(rknn_outputs_get(ctx,3,out,nullptr), "outputs_get");
            auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            bool wrote=true;
            for (unsigned i=0;i<3;++i) {
                if (out[i].size!=attrs[i].n_elems*sizeof(float)) { wrote=false; break; }
                std::ofstream f(std::string(argv[4])+"/"+stem+"_"+std::to_string(i)+".f32",std::ios::binary);
                f.write(static_cast<char*>(out[i].buf),out[i].size);
                if (!f) { wrote=false; break; }
            }
            ok(rknn_outputs_release(ctx,3,out), "outputs_release");
            if (!wrote) throw std::runtime_error("unexpected output byte count or write failure");
            timings<<stem<<'\t'<<ms<<'\n';
            std::cout<<++count<<" "<<stem<<" "<<ms<<" ms"<<std::endl;
        }
        if (!count) throw std::runtime_error("empty manifest");
        ok(rknn_destroy(ctx), "destroy"); ctx=0;
        std::cout<<"completed="<<count<<std::endl;
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        if (ctx) rknn_destroy(ctx);
        return 1;
    }
}
