基于WSL-Ubuntu18.04LTS的gem5搭建与DNNMark benchmark流程
1.Proxy

##clash

alias clash='export HOSTIP=$(cat /etc/resolv.conf | grep "nameserver" | cut -f 2 -d " ") && export http_proxy="http://$HOSTIP:7890" && export https_proxy="https://$HOSTIP:7890" && export all_proxy="socks5://$HOSTIP:7890" && export ALL_PROXY="socks5://HOSTIP:7890"'

alias unclash='unset HOSTIP && unset http_proxy && unset https_proxy && unset all_proxy && unset ALL_PROXY'

2.更新
sudo apt update
sudo apt upgrade

3.基本环境(如果是22LTS，不需要安装python)
sudo apt install build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev python libboost-all-dev pkg-config

4.改git Proxy
git config --global http.https://github.com.proxy http://$HOSTIP:7890
git config --global https.https://github.com.proxy https://$HOSTIP:7890

git config --global http.proxy 'socks5://$HOSTIP:7890'
git config --global https.proxy 'socks5://$HOSTIP:7890'

5.拉仓库
git clone https://github.com/gem5/gem5
git clone https://github.com/gem5/gem5-resources

6.装CMake
sudo apt install CMake

7.build
python3 `which scons` build/X86/gem5.opt -j 17

8.test
build/X86/gem5.opt configs/learning_gem5/part1/simple.py

9.安装cuda和cudnn
sudo bash cuda_10.0.130_410.48_linux.run

##cuda

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.0/lib64
export PATH=$PATH:/usr/local/cuda-10.0/bin

nvcc -V

tar -xvf cudnn-10.0-linux-x64-v7.6.5.32.tgz

sudo cp cuda/include/cudnn.h /usr/local/cuda-10.0/include/
sudo cp cuda/lib64/libcudnn* /usr/local/cuda-10.0/lib64/
sudo chmod a+r /usr/local/cuda-10.0/include/cudnn.h
sudo chmod a+r /usr/local/cuda-10.0/lib64/libcudnn*

##cudnn

export CUDNN_ROOT=/usr/local/cuda-10.0

cat /usr/local/cuda-10.0/include/cudnn.h | grep CUDNN_MAJOR -A 2

10.安装docker
sudo apt install docker.io

11.benchmark
cd src/gpu/DNNMark
sudo docker run --rm -v ${PWD}:${PWD} -w ${PWD} -u $UID:$GID ghcr.io/gem5/gcn-gpu ./setup.sh HIP
sudo docker run --rm -v ${PWD}:${PWD} -w ${PWD}/build -u $UID:$GID ghcr.io/gem5/gcn-gpu make

sudo docker run --rm -v ${PWD}:${PWD} -v${PWD}/cachefiles:/root/.cache/miopen/2.9.0 -w ${PWD} ghcr.io/gem5/gcn-gpu python3 generate_cachefiles.py cachefiles.csv --gfx-version=gfx801 --num-cus=4
g++ -std=c++0x generate_rand_data.cpp -o generate_rand_data
./generate_rand_data

sudo docker run --rm -v ${PWD}:${PWD} -w ${PWD} -u $UID:$GID ghcr.io/gem5/gcn-gpu scons -sQ -j$(nproc) build/GCN3_X86/gem5.opt

sudo docker run --rm -v ${PWD}:${PWD} -v ${PWD}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0 -w ${PWD} ghcr.io/gem5/gcn-gpu gem5/build/GCN3_X86/gem5.opt gem5/configs/example/apu_se.py -n3 --benchmark-root=gem5-resources/src/gpu/DNNMark/build/benchmarks/test_fwd_softmax -cdnnmark_test_fwd_softmax --options="-config gem5-resources/src/gpu/DNNMark/config_example/softmax_config.dnnmark -mmap gem5-resources/src/gpu/DNNMark/mmap.bin"