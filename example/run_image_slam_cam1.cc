#include "util/image_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>
// 打印精度控制
#include <iostream>
#include <iomanip>


using namespace std;

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif



void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<string> &vTimestamps, vector<vector<double>> &GPSDataList)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/FHY_config/FHY_gps.txt";
 
 
    // load image list
    FILE* file;
    file = std::fopen(strPathTimeFile.c_str() , "r"); //.c_str()
    if(file == NULL){
        printf("cannot find file: %s  \n", strPathTimeFile.c_str());
         
        return ;         
    }
 
    char timeshara_c[30];
    char lat_c[30], lon_c[30], alt_c[30];
 
 
 
 
    vector<string> imageNameTimeList;
 
    //vector<vector<string>> GPSDataList;
 
 
    // 按照string 读取出来 容易保存超高精度
    while (fscanf(file, "%s %s %s %s", &timeshara_c, &lat_c, &lon_c, &alt_c) != EOF)
    ////%lf之间应该有逗号，因为没有逗号只能读第一个数。用&是因为要把数存到对应数组元素的地址中去。\n是换行读取
    //while (fscanf(file, "%lf %lf %lf %lf", &timesharap_l, &lat_l, &lon_l, &alt_l) != EOF)
    {
 
 
        string timesharap_s;
        string lat_s, lon_s, alt_s;
 
 
        timesharap_s=(string(timeshara_c)); //stod
        lat_s=(string(lat_c));
        lon_s=(string(lon_c));
        alt_s=(string(alt_c));
 
         
        double timesharap_d;
        double lat_d, lon_d, alt_d;
 
 
 
        timesharap_d=std::stod(timesharap_s);
        lat_d=std::stod(lat_s);
        lon_d=std::stod(lon_s);
        alt_d=std::stod(alt_s);
 
 
 
        imageNameTimeList.push_back(timesharap_s);
 
 
 
        vTimestamps.push_back(timesharap_s);

        //cout<<"timesharap_s  "<<timesharap_s<< "   timesharap_d  "<<timesharap_d<<endl;
 
        vector<double> gpsdata_i;
        gpsdata_i.push_back(timesharap_d);
        gpsdata_i.push_back(lat_d);
        gpsdata_i.push_back(lon_d);
        gpsdata_i.push_back(alt_d);
        GPSDataList.push_back(gpsdata_i);
 
        std::cout<< fixed << setprecision(18)   <<"timesharap_string  "<<timesharap_s<< "   timesharap_double18  "<<timesharap_d << ' '<<  setprecision(18)  <<lat_s<< " " << lon_s <<" "<<alt_s<<std::endl;
        
    }
    std::fclose(file);
 
 
    string strPrefixLeft = strPathToSequence + "/left/";
 
    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
 
    for(int i=0; i<nTimes; i++)
    {
        //stringstream ss;
        //ss << setfill('0') << setw(10) << i;
 
        vstrImageFilenames[i] = strPrefixLeft + imageNameTimeList[i] + ".png";

        //cout<< vstrImageFilenames[i] <<endl;
    }
}

    vector<string> vstrImageFilenames;  // 图像路径
    vector<double> vTimestamps;    // 时间戳  也就是图像名字
    vector<vector<double>> GPSDataList;// 保存GPS数据列表

//==============================跟踪进程=========================================
void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, 
                   const std::string& vocab_file_path, 
                   const std::string& image_dir_path, 
                   const std::string& mask_img_path, 
                   const unsigned int frame_skip, 
                   const bool no_sleep, const bool auto_term, 
                   const bool eval_log, const std::string& map_db_path, 
                   std::string &data_path,
                   vector<string> &vstrImageFilenames, 
                   vector<string> &vTimestamps,        
                   vector<vector<double>> &GPSDataList 
                   ) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    
    
    //const image_sequence sequence(image_dir_path, cfg->camera_->fps_);
    //const auto frames = sequence.get_frames();

    int nImages = vstrImageFilenames.size();


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
    track_times.reserve(nImages);

    // run the SLAM in another thread
    std::thread thread([&]() {

        string path= data_path+"/openvslam_outdata/slam_xyzgps.txt";
        std::ofstream ofs(path, std::ios::out);
        if (!ofs.is_open()) {
            spdlog::critical("cannot create a file at {}", path);
            throw std::runtime_error("cannot create a file at " + path);
        }


        for (unsigned int i = 0; i < nImages; ++i) {
            //const auto& frame = frames.at(i);

            double timesharpi_fps=(1.0 / cfg->camera_->fps_) * i; // 根据帧率控制读取离线图像的速度
            double timesharpi1_fps=(1.0 / cfg->camera_->fps_) * (i+1);


            const auto img = cv::imread(vstrImageFilenames[i], cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                //=======================识别当前帧================
                // 这里的位姿并没有经历过局部和全局优化
                //double img_time_sharp=std::stod(vTimestamps[i]);//  time will lost  no img name
                double img_time_sharp=i;
                 
                Eigen::Matrix4d cam_pose_cw  = SLAM.feed_monocular_frame(img, img_time_sharp , mask);

                Eigen::Matrix4d cam_pose_wc = cam_pose_cw.inverse();

                Eigen::Matrix3d rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                Eigen::Vector3d trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                Eigen::Quaterniond quat_wc = Eigen::Quaterniond(rot_wc);

                cout << fixed <<  std::setprecision(4)
                    //<< vTimestamps[i] << " "
                    << i  << " "
                    << std::setprecision(9)
                    << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                    //<< quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
                    << GPSDataList[i][1] << " " << GPSDataList[i][2] << " " << GPSDataList[i][3] << std::endl;


                ofs <<  fixed << std::setprecision(4)
                            //<< vTimestamps[i]  << " "
                            << i  << " "
                            << std::setprecision(9)
                            << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                            << GPSDataList[i][1] << " " << GPSDataList[i][2] << " " << GPSDataList[i][3] << std::endl;
                            


                 
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < nImages - 1) {
                //// 根据帧率控制读取离线图像的速度
                const auto wait_time = timesharpi1_fps - (timesharpi_fps+ track_time);
                if (0.0 < wait_time) {
                    //std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
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


        ofs.close();



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

    if (1) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory(data_path+"/openvslam_outdata/frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory(data_path+"/openvslam_outdata/keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs(data_path+"/openvslam_outdata/track_times.txt", std::ios::out);
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


//========================主函数==============================

int main(int argc, char* argv[]) {


    #ifdef USE_STACK_TRACE_LOGGER
        google::InitGoogleLogging(argv[0]);
        google::InstallFailureSignalHandler();
    #endif

    // create options
    //# 参数输入有顺序 别搞错位了
    //popl::OptionParser op("Allowed options");
    //auto help = op.add<popl::Switch>("h", "help", "produce help message");

    // auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    // auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    // auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    // auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    // auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    // auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    // auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    // auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    // auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    // auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");


    string vocab_file_path = argv[1];
    string data_dir_path = argv[2];
    std::string img_dir_path=data_dir_path+"/left/";
    std::string map_db_path =data_dir_path+"/openvslam_outdata/"+argv[4];
    string config_file_path = argv[3];

    string mask_img_path = "";
    unsigned int frame_skip = 1;
    bool no_sleep = 0 ;// 睡眠等待处理完在加载下一张离线图像
    bool auto_term = 0;
    bool debug_mode = 1;
    bool eval_log = 1;


    if (vocab_file_path=="" || img_dir_path=="" || config_file_path=="") {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        //std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    #ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStart("slam.prof");
    #endif

    //=================加载图像数据 gps时间戳==========================
  
    // Retrieve paths to images
    vector<string> vstrImageFilenames;  // 图像路径
    vector<string> vTimestamps_string;    // 时间戳  也就是图像名字
    vector<vector<double>> GPSDataList;// 保存GPS数据列表

    // 根据时间戳列表 找图像加载
    LoadImages(data_dir_path, vstrImageFilenames, vTimestamps_string, GPSDataList);
 
    



    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {

        mono_tracking(cfg, vocab_file_path,
                img_dir_path, 
                mask_img_path,
                frame_skip, 
                no_sleep, 
                auto_term,
                eval_log, 
                map_db_path,
                data_dir_path,
                vstrImageFilenames,
                vTimestamps_string,
                GPSDataList
                );
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

    #ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStop();
    #endif

    return EXIT_SUCCESS;
}
