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

#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;


#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void mono_localization(const std::shared_ptr<openvslam::config>& cfg,
                       const std::string& vocab_file_path, const std::string& image_dir_path, const std::string& mask_img_path,
                       const std::string& map_db_path, const bool mapping,
                       const unsigned int frame_skip, const bool no_sleep, const bool auto_term) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    const image_sequence sequence(image_dir_path, cfg->camera_->fps_);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // load the prebuilt map
    SLAM.load_map_database(map_db_path);
    // startup the SLAM process (it does not need initialization of a map)
    SLAM.startup(false);
    // select to activate the mapping module or not
    if (mapping) {
        SLAM.enable_mapping_module();
    }
    else {
        SLAM.disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
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

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!img.empty() && (i % frame_skip == 0)) {

                 //要缩小图像，通常使用CV_INTER_AREA插值效果最好，而要放大图像，通常使用CV_INTER_CUBIC（慢）或CV_INTER_LINEAR（更快但看起来还可以）效果最好。
                cv::Size dsize = cv::Size(1920, 1080);
                cv::Mat imgResize;
                cv::resize(img, imgResize, dsize, 0, 0, INTER_AREA);

                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame(imgResize, frame.timestamp_, mask);
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
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "path to a prebuilt map database");
    auto mapping = op.add<popl::Switch>("", "mapping", "perform mapping as well as localization");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
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
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !img_dir_path->is_set()
        || !config_file_path->is_set() || !map_db_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

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

    // run localization
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_localization(cfg, vocab_file_path->value(), img_dir_path->value(), mask_img_path->value(),
                          map_db_path->value(), mapping->is_set(),
                          frame_skip->value(), no_sleep->is_set(), auto_term->is_set());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
