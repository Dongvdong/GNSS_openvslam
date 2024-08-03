#include "util/image_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/gnss_src/API_GNSS_ENU_TXT_YAML.cc" // 自己的类

#include "openvslam/system.h"
#include "openvslam/config.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

using namespace std;
 



#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

// void mono_tracking(const std::shared_ptr<openvslam::config>& cfg,
//                    const std::string& vocab_file_path, const std::string& image_dir_path, const std::string& mask_img_path,
//                    const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
//                    const bool eval_log, const std::string& map_db_path, const std::string& GNSS_path,const std::string& path_config) {
void mono_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string config_file_path, 
                   const unsigned int frame_skip,
                   const bool no_sleep, 
                   const bool auto_term,
                   const bool eval_log) {
    
    // 获取GNSS信息
    std::string Config_PATH = Get_YAML(config_file_path ,"Config_PATH");
    std::string vocab_file_path = Get_YAML(config_file_path ,"vocab_file_path");// 绝对路径
    std::string map_db_path = Config_PATH + Get_YAML(config_file_path ,"map_db_path");
    std::string image_dir_path = Config_PATH + Get_YAML(config_file_path ,"image_dir_path");
    std::string gnss_path = Config_PATH + Get_YAML(config_file_path ,"GNSS_PSTH");
    std::string gnss_use = Get_YAML(config_file_path ,"GNSS_USE");
    std::string mask_img_path = Get_YAML(config_file_path ,"mask_img_path");

    
    std::cout<<  "config_file_path : "<< config_file_path <<  std::endl;
    std::cout<<  "image_dir_path : "<< image_dir_path <<  std::endl;
    std::cout<<  "Config_PATH : "<< Config_PATH <<  std::endl;
    std::cout<<  "gnss_path : "<<gnss_path <<  std::endl;
    std::cout<<  "gnss_use : "<<gnss_use <<  std::endl;
    std::cout<<  "vocab_file_path : "<<vocab_file_path <<  std::endl;
    std::cout<<  "map_db_path : "<<map_db_path <<  std::endl;
    std::cout<<  "mask_img_path : "<<mask_img_path <<  std::endl;



    // load the mask image
    cv::Mat mask ;
    if( mask_img_path=="none.jpg"){
        mask = cv::Mat{} ;
    }
    else{
        mask =cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    }
     std::cout<<  "加载图像: " <<  std::endl;
    //  加载图像
    const image_sequence sequence(image_dir_path, cfg->camera_->fps_);
    const auto frames = sequence.get_frames();

    std::vector<gnss_data> gnss_Lists ;

    if(gnss_use=="0"){
        
        int N = frames.size();
        for(int i=0;i<N;i++){
            gnss_data gnss_Lists_i(-1.0,-1.0,-1.0,-1.0);
            gnss_Lists.push_back(gnss_Lists_i) ;
        }
    }
    else{




        std::cout<<  "加载GNSS参考点: " <<  std::endl;
        // 1 获取参考点
        
        gnss_data gnss_data_int0(-1,-1,-1,-1);
        Get_GNSS_INTI_YAML(config_file_path,gnss_data_int0);

        std::cout  << fixed << setprecision(10)<< endl;
        std::cout << "原点 纬度: " << gnss_data_int0.lat << endl;
        std::cout << "原点 经度: " << gnss_data_int0.lon << endl;
        std::cout << "原点 高度: " << gnss_data_int0.high << endl;
        
        // 2 当前点的经纬度和高度，作为局部坐标系的原点
        double origin_latitude =  gnss_data_int0.lat;   // 纬度
        double origin_longitude = gnss_data_int0.lon; // 经度
        double origin_height = gnss_data_int0.high;   // 高度
        // 转化为enu，并设置原点
        GeographicLib::LocalCartesian geoConverter;
        geoConverter.Reset(origin_latitude, origin_longitude, origin_height);
        
        // 转化为ecef，使用WGS84椭球模型
        GeographicLib::Geocentric wgs84(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());//  6378137  298.257223563LL
        
        //3 获取数据点
        GNSS_TextFileReader reader(gnss_path, ' '); // 读取路径 分隔符号
        


        if (reader.readFile()) {
            //reader.printData();
            gnss_Lists =  reader.get_gnss_List();
            for(int i=0;i<gnss_Lists.size();i++){
                
                gnss_Lists[i].Set_orinGnss(gnss_data_int0.lat,gnss_data_int0.lon,gnss_data_int0.high);// 设置原始GNSS点

                double target_latitude = gnss_Lists[i].lat;
                double target_longitude = gnss_Lists[i].lon;
                double target_height = gnss_Lists[i].high;


                // gnss转化为enu
                double x, y, z;
                geoConverter.Forward(target_latitude, target_longitude, target_height, x, y, z);

                gnss_Lists[i].x=x;
                gnss_Lists[i].y=y;
                gnss_Lists[i].z=z;


                //  WGS84 gnss转化为ecef
                wgs84.Forward(target_latitude, target_longitude, target_height, x, y, z);
                gnss_Lists[i].ecef_x=x;
                gnss_Lists[i].ecef_y=y;
                gnss_Lists[i].ecef_z=z;      

                gnss_data gnss_List_i=gnss_Lists[i];
                
                // cout << fixed << setprecision(10)<< endl;
                // cout<< "编号 " << i 
                // << " 时间戳 "<< gnss_List_i.time
                // << " \n纬度 " << gnss_List_i.lat 
                // << " 经度 " << gnss_List_i.lon
                // << " 高度 "<<  gnss_List_i.high 
                // << " \nenu-x " << gnss_List_i.x 
                // << " enu-y " << gnss_List_i.y
                // << " enu-z "<<  gnss_List_i.z 
                // << " \necef_x " << gnss_List_i.ecef_x
                // << " ecef_y " << gnss_List_i.ecef_y
                // << " ecef_z "<<  gnss_List_i.ecef_z
                // << endl;
            }
        }

    }

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    #ifdef USE_PANGOLIN_VIEWER
        pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
    #elif USE_SOCKET_PUBLISHER
        socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
    #endif

    std::vector<double> track_times;
    track_times.reserve(frames.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {

            
            const auto& frame = frames.at(i);
            const auto img = cv::imread(frame.img_path_, cv::IMREAD_UNCHANGED);

            gnss_data img_gnss = gnss_Lists[i];

            cout<< i <<"   frame.img_path_ "<< frame.img_path_  <<  "  GNSS时间戳  "  << std::to_string(img_gnss.time) << endl;

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame_gnss(img,  frame.timestamp_, mask, gnss_Lists );// frame.timestamp_ 只是控制帧率
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    //auto gnss_path = op.add<popl::Value<std::string>>("p", "gnss", "store gnss", "");


    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    // if (help->is_set()) {
    //     std::cerr << op << std::endl;
    //     return EXIT_FAILURE;
    // }
    // if (!vocab_file_path->is_set() || !img_dir_path->is_set() || !config_file_path->is_set()) {
    //     std::cerr << "invalid arguments" << std::endl;
    //     std::cerr << std::endl;
    //     std::cerr << op << std::endl;
    //     return EXIT_FAILURE;
    // }

    if (!config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // // 获取文路径
    // std::vector<std::string> parts = splitString(config_file_path->value() , '/');
    // std::string config_file_dir ;
    // for (int i = 0; i < parts.size() - 2; ++i) {
    //     config_file_dir += parts[i] + '/';
    // }


    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
         
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        // mono_tracking(cfg, vocab_file_path->value(), img_dir_path->value(), mask_img_path->value(),
        //               frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
        //               eval_log->is_set(), map_db_path->value(),config_file_path->value());

        mono_tracking(cfg, 
                config_file_path->value(),
                frame_skip->value(), 
                no_sleep->is_set(), 
                auto_term->is_set(),
                eval_log->is_set());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
