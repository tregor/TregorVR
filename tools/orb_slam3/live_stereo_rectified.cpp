#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

#include "System.h"
#include "Tracking.h"

namespace {

struct Args {
    std::string vocab;
    std::string settings;
    std::string rectify;
    // Индексы V4L2: video2 = левая (cam0), video0 = правая (cam1), как ov9281_stereo_record.launch.
    // Числа надёжнее в WSL, чем путь /dev/video* (меньше срабатывает «can't capture by name»).
    std::string device_left = "2";
    std::string device_right = "0";
    int width = 800;
    int height = 600;
    int fps = 20;
    std::string fourcc = "MJPG";
    bool auto_levels = false;
    double auto_levels_tail = 0.02;
    bool uvc_auto = false;
    bool viewer = true;
    bool show = true;
};

struct RectifyData {
    int width = 0;
    int height = 0;
    std::string model;
    cv::Mat k_left;
    cv::Mat d_left;
    cv::Mat k_right;
    cv::Mat d_right;
    cv::Mat t_c1_c2_raw;
    cv::Mat map_left_1;
    cv::Mat map_left_2;
    cv::Mat map_right_1;
    cv::Mat map_right_2;
    double baseline_m = 0.0;
};

std::string Usage() {
    return
        "Usage: orbslam3_stereo_live "
        "--vocab ORBvoc.txt "
        "--settings rectified_orbslam3.yaml "
        "--rectify rectify_input.yaml "
        "[--device-left 2] [--device-right 0] "
        "[--width 800] [--height 600] [--fps 20] [--fourcc MJPG] "
        "[--auto-levels 0] [--auto-levels-tail 0.02] [--uvc-auto 0] "
        "[--viewer 1] [--show 1]";
}

bool IsNumber(const std::string &value) {
    if (value.empty()) {
        return false;
    }
    char *end = nullptr;
    std::strtol(value.c_str(), &end, 10);
    return end != nullptr && *end == '\0';
}

int ParseInt(const std::string &name, const std::string &value) {
    try {
        return std::stoi(value);
    } catch (const std::exception &) {
        throw std::runtime_error("Invalid integer for " + name + ": " + value);
    }
}

double ParseDouble(const std::string &name, const std::string &value) {
    try {
        return std::stod(value);
    } catch (const std::exception &) {
        throw std::runtime_error("Invalid number for " + name + ": " + value);
    }
}

bool ParseBool01(const std::string &name, const std::string &value) {
    if (value == "0") {
        return false;
    }
    if (value == "1") {
        return true;
    }
    throw std::runtime_error("Expected 0 or 1 for " + name + ", got: " + value);
}

Args ParseArgs(int argc, char **argv) {
    Args args;
    std::map<std::string, std::string *> string_flags = {
        {"--vocab", &args.vocab},
        {"--settings", &args.settings},
        {"--rectify", &args.rectify},
        {"--device-left", &args.device_left},
        {"--device-right", &args.device_right},
        {"--fourcc", &args.fourcc},
    };

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--help" || flag == "-h") {
            throw std::runtime_error(Usage());
        }
        if (flag == "--auto-levels") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            args.auto_levels = ParseBool01(flag, argv[++i]);
            continue;
        }
        if (flag == "--auto-levels-tail") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            args.auto_levels_tail = ParseDouble(flag, argv[++i]);
            continue;
        }
        if (flag == "--uvc-auto") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            args.uvc_auto = ParseBool01(flag, argv[++i]);
            continue;
        }
        if (flag == "--width" || flag == "--height" || flag == "--fps" ||
            flag == "--viewer" || flag == "--show") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + flag);
            }
            const std::string value = argv[++i];
            if (flag == "--width") {
                args.width = ParseInt(flag, value);
            } else if (flag == "--height") {
                args.height = ParseInt(flag, value);
            } else if (flag == "--fps") {
                args.fps = ParseInt(flag, value);
            } else if (flag == "--viewer") {
                args.viewer = ParseBool01(flag, value);
            } else {
                args.show = ParseBool01(flag, value);
            }
            continue;
        }

        const auto it = string_flags.find(flag);
        if (it == string_flags.end()) {
            throw std::runtime_error("Unknown argument: " + flag + "\n" + Usage());
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value for " + flag);
        }
        *it->second = argv[++i];
    }

    if (args.vocab.empty() || args.settings.empty() || args.rectify.empty()) {
        throw std::runtime_error("Missing required arguments.\n" + Usage());
    }

    return args;
}

cv::VideoCapture OpenCapture(const std::string &device) {
    if (IsNumber(device)) {
        const int index = ParseInt("device index", device);
#ifdef _WIN32
        cv::VideoCapture cap(index, cv::CAP_MSMF);
        if (cap.isOpened()) {
            return cap;
        }
        return cv::VideoCapture(index);
#else
        return cv::VideoCapture(index);
#endif
    }
#ifdef _WIN32
    return cv::VideoCapture(device);
#else
    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (cap.isOpened()) {
        return cap;
    }
    return cv::VideoCapture(device);
#endif
}

void ConfigureCapture(cv::VideoCapture &cap, int width, int height, int fps, const std::string &fourcc) {
    if (!fourcc.empty() && fourcc.size() == 4) {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
            fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    if (fps > 0) {
        cap.set(cv::CAP_PROP_FPS, fps);
    }
    cap.set(cv::CAP_PROP_BUFFERSIZE, 2);
}

void TryUvcAutoModes(cv::VideoCapture &cap, const char *tag) {
    // V4L2 через OpenCV: часто 0.75 = автоэкспозиция (зависит от драйвера).
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
    cap.set(cv::CAP_PROP_AUTO_WB, 1);
    std::cout << tag << " UVC: AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
              << " AUTO_WB=" << cap.get(cv::CAP_PROP_AUTO_WB) << std::endl;
}

RectifyData LoadRectifyData(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open rectify file: " + path);
    }

    RectifyData data;
    fs["Camera.width"] >> data.width;
    fs["Camera.height"] >> data.height;
    fs["Camera.model"] >> data.model;
    fs["Stereo.baseline_m"] >> data.baseline_m;
    fs["LEFT.K"] >> data.k_left;
    fs["LEFT.D"] >> data.d_left;
    fs["RIGHT.K"] >> data.k_right;
    fs["RIGHT.D"] >> data.d_right;
    fs["Stereo.T_c1_c2_raw"] >> data.t_c1_c2_raw;

    if (data.width <= 0 || data.height <= 0 || data.k_left.empty() || data.k_right.empty() ||
        data.d_left.empty() || data.d_right.empty() || data.t_c1_c2_raw.empty()) {
        throw std::runtime_error("Rectify file is missing required matrices");
    }

    cv::Mat r = data.t_c1_c2_raw(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t = data.t_c1_c2_raw(cv::Rect(3, 0, 1, 3)).clone();
    cv::Size image_size(data.width, data.height);

    cv::Mat r1, r2, p1, p2, q;
    if (data.model == "equidistant") {
        cv::fisheye::stereoRectify(
            data.k_left,
            data.d_left,
            data.k_right,
            data.d_right,
            image_size,
            r,
            t,
            r1,
            r2,
            p1,
            p2,
            q,
            cv::fisheye::CALIB_ZERO_DISPARITY,
            image_size,
            0.0,
            1.0);
        cv::fisheye::initUndistortRectifyMap(
            data.k_left, data.d_left, r1, p1, image_size, CV_16SC2, data.map_left_1, data.map_left_2);
        cv::fisheye::initUndistortRectifyMap(
            data.k_right, data.d_right, r2, p2, image_size, CV_16SC2, data.map_right_1, data.map_right_2);
    } else if (data.model == "radtan") {
        cv::stereoRectify(
            data.k_left,
            data.d_left,
            data.k_right,
            data.d_right,
            image_size,
            r,
            t,
            r1,
            r2,
            p1,
            p2,
            q,
            cv::CALIB_ZERO_DISPARITY,
            0.0,
            image_size);
        cv::initUndistortRectifyMap(
            data.k_left, data.d_left, r1, p1, image_size, CV_16SC2, data.map_left_1, data.map_left_2);
        cv::initUndistortRectifyMap(
            data.k_right, data.d_right, r2, p2, image_size, CV_16SC2, data.map_right_1, data.map_right_2);
    } else {
        throw std::runtime_error("Unsupported Camera.model in rectify file: " + data.model);
    }

    return data;
}

std::string TrackingStateName(int state) {
    switch (state) {
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: return "SYSTEM_NOT_READY";
        case ORB_SLAM3::Tracking::NO_IMAGES_YET: return "NO_IMAGES_YET";
        case ORB_SLAM3::Tracking::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case ORB_SLAM3::Tracking::OK: return "OK";
        case ORB_SLAM3::Tracking::RECENTLY_LOST: return "RECENTLY_LOST";
        case ORB_SLAM3::Tracking::LOST: return "LOST";
        case ORB_SLAM3::Tracking::OK_KLT: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

bool HasUsablePose(int state) {
    return state == ORB_SLAM3::Tracking::OK ||
           state == ORB_SLAM3::Tracking::RECENTLY_LOST ||
           state == ORB_SLAM3::Tracking::OK_KLT;
}

cv::Mat ToGray(const cv::Mat &image) {
    if (image.channels() == 1) {
        return image;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

void AutoContrastBrightnessGlobal(cv::Mat &gray_u8, double tail_frac) {
    if (gray_u8.empty() || gray_u8.type() != CV_8UC1) {
        return;
    }
    double t = tail_frac;
    if (t <= 0.0 || t >= 0.25) {
        t = 0.02;
    }
    const int n = gray_u8.rows * gray_u8.cols;
    int hist[256] = {};
    const uchar *pix = gray_u8.ptr<uchar>();
    for (int i = 0; i < n; ++i) {
        ++hist[pix[i]];
    }
    const int cut = std::max(1, static_cast<int>(std::floor(static_cast<double>(n) * t)));
    int lo = 0;
    int hi = 255;
    {
        int acc = 0;
        for (int i = 0; i < 256; ++i) {
            acc += hist[i];
            if (acc >= cut) {
                lo = i;
                break;
            }
        }
        acc = 0;
        for (int i = 255; i >= 0; --i) {
            acc += hist[i];
            if (acc >= cut) {
                hi = i;
                break;
            }
        }
    }
    if (hi <= lo) {
        return;
    }
    const double scale = 255.0 / static_cast<double>(hi - lo);
    const double shift = -static_cast<double>(lo) * scale;
    gray_u8.convertTo(gray_u8, CV_8U, scale, shift);
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const Args args = ParseArgs(argc, argv);
        const RectifyData rectify = LoadRectifyData(args.rectify);

        cv::VideoCapture cap_left = OpenCapture(args.device_left);
        cv::VideoCapture cap_right = OpenCapture(args.device_right);
        if (!cap_left.isOpened()) {
            throw std::runtime_error(
                "Failed to open left device: " + args.device_left +
                " (v4l2-ctl --list-devices; usbipd; group 'video'; для DECXIN: left=2 → /dev/video2, right=0 → /dev/video0)");
        }
        if (!cap_right.isOpened()) {
            throw std::runtime_error(
                "Failed to open right device: " + args.device_right +
                " (не подставляй 1 или 3 — это второй узел той же камеры; правая линза — индекс 0 при типичной нумерации)");
        }

        ConfigureCapture(cap_left, args.width, args.height, args.fps, args.fourcc);
        ConfigureCapture(cap_right, args.width, args.height, args.fps, args.fourcc);
        if (args.uvc_auto) {
            TryUvcAutoModes(cap_left, "left");
            TryUvcAutoModes(cap_right, "right");
        }

        ORB_SLAM3::System slam(
            args.vocab,
            args.settings,
            ORB_SLAM3::System::STEREO,
            args.viewer);

        if (args.show) {
            cv::namedWindow("rectified stereo", cv::WINDOW_NORMAL);
        }

        const auto t0 = std::chrono::steady_clock::now();
        std::size_t frame_id = 0;
        std::size_t consecutive_failures = 0;

        std::cout << "Starting live stereo ORB-SLAM3" << std::endl;
        std::cout << "Left=" << args.device_left
                  << " Right=" << args.device_right
                  << " width=" << rectify.width
                  << " height=" << rectify.height
                  << " baseline_m=" << std::fixed << std::setprecision(6) << rectify.baseline_m
                  << " auto_levels=" << (args.auto_levels ? 1 : 0)
                  << " auto_tail=" << args.auto_levels_tail
                  << " uvc_auto=" << (args.uvc_auto ? 1 : 0)
                  << std::endl;

        for (;;) {
            cv::Mat raw_left;
            cv::Mat raw_right;
            const bool ok_left = cap_left.read(raw_left);
            const bool ok_right = cap_right.read(raw_right);
            if (!ok_left || !ok_right || raw_left.empty() || raw_right.empty()) {
                ++consecutive_failures;
                if (consecutive_failures > 40) {
                    throw std::runtime_error("Too many empty frames from cameras");
                }
                continue;
            }
            consecutive_failures = 0;

            if (raw_left.cols != rectify.width || raw_left.rows != rectify.height) {
                cv::resize(raw_left, raw_left, cv::Size(rectify.width, rectify.height), 0.0, 0.0, cv::INTER_AREA);
            }
            if (raw_right.cols != rectify.width || raw_right.rows != rectify.height) {
                cv::resize(raw_right, raw_right, cv::Size(rectify.width, rectify.height), 0.0, 0.0, cv::INTER_AREA);
            }

            cv::Mat rect_left;
            cv::Mat rect_right;
            cv::remap(raw_left, rect_left, rectify.map_left_1, rectify.map_left_2, cv::INTER_LINEAR);
            cv::remap(raw_right, rect_right, rectify.map_right_1, rectify.map_right_2, cv::INTER_LINEAR);

            cv::Mat gray_left = ToGray(rect_left);
            cv::Mat gray_right = ToGray(rect_right);
            cv::Mat slam_left = gray_left;
            cv::Mat slam_right = gray_right;
            if (args.auto_levels) {
                slam_left = gray_left.clone();
                slam_right = gray_right.clone();
                AutoContrastBrightnessGlobal(slam_left, args.auto_levels_tail);
                AutoContrastBrightnessGlobal(slam_right, args.auto_levels_tail);
            }

            const double timestamp =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            Sophus::SE3f t_cw = slam.TrackStereo(slam_left, slam_right, timestamp);
            const int state = slam.GetTrackingState();

            if ((frame_id % 15U) == 0U) {
                std::cout << "frame=" << frame_id
                          << " state=" << TrackingStateName(state)
                          << " pose_valid=" << (HasUsablePose(state) ? 1 : 0);
                if (HasUsablePose(state)) {
                    const Eigen::Vector3f translation = t_cw.translation();
                    std::cout << " t_cw=[" << translation.x() << ", "
                              << translation.y() << ", "
                              << translation.z() << "]";
                }
                std::cout << std::endl;
            }

            if (args.show) {
                cv::Mat combo;
                if (args.auto_levels) {
                    cv::Mat vis_left;
                    cv::Mat vis_right;
                    cv::cvtColor(slam_left, vis_left, cv::COLOR_GRAY2BGR);
                    cv::cvtColor(slam_right, vis_right, cv::COLOR_GRAY2BGR);
                    cv::hconcat(vis_left, vis_right, combo);
                } else {
                    cv::hconcat(rect_left, rect_right, combo);
                }
                cv::imshow("rectified stereo", combo);
                const int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 27) {
                    break;
                }
            }

            ++frame_id;
        }

        slam.Shutdown();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
