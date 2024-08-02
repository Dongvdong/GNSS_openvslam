


# 编译问题

1 修改opencv路径

#设置opencv安装路径
set(CMAKE_PREFIX_PATH "/home/dongdong/1sorftware/1work/opencv/opencv455/install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

2 如果装了conda 注意编译时候注销了conda环境


3 G2O编译时候参数注意，和SLAM阶段对应参数设置
-DBUILD_WITH_MARCH_NATIVE=OFF （默认教程是on 但是slam14讲代码用不了好像，段错误，所以关了，这里也跟着关闭）


# 编译指令
https://www.cnblogs.com/gooutlook/p/14238636.html

mkdir build && cd build
cmake \
    -DBUILD_WITH_MARCH_NATIVE=ON \
    -DUSE_PANGOLIN_VIEWER=ON \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=DBoW2 \
    -DBUILD_TESTS=OFF \
    ..
make -j4

对应g2o编译 -DBUILD_WITH_MARCH_NATIVE=ON 不然导致可视化图有问题
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DBUILD_WITH_MARCH_NATIVE=ON \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    ..
make -j12
make install



# 运行指令
cd  /build

#建图
./run_image_slam \
-v /home/dongdong/2project/1salm/GNSS_openvslam/config/orb_vocab.dbow2 \
-i /home/dongdong/2project/0data/NWPU/FHY_img \
-c /home/dongdong/2project/0data/NWPU/FHY_config/FHY_config.yaml  \
--map-db /home/dongdong/2project/0data/NWPU/Map.msg


#重定位
./run_image_localization \
-v /home/dongdong/2project/1salm/GNSS_openvslam/config/orb_vocab.dbow2 \
-i /home/dongdong/2project/0data/NWPU/FHY_img \
-c /home/dongdong/2project/0data/NWPU/FHY_config/FHY_config.yaml  \
--map-db /home/dongdong/2project/0data/NWPU/Map.msg

