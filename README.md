1 修改opencv路径

#设置opencv安装路径
set(CMAKE_PREFIX_PATH "/home/dongdong/1sorftware/1work/opencv/opencv455/install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

2 如果有conda环境，编译前先注销环境
