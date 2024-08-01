





1 修改opencv路径

#设置opencv安装路径
set(CMAKE_PREFIX_PATH "/home/dongdong/1sorftware/1work/opencv/opencv455/install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



2 G2O编译时候参数注意，和SLAM阶段对应
教程是on
-DBUILD_WITH_MARCH_NATIVE=OFF 
