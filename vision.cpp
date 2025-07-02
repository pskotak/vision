#include <iostream>
#include "vision.h"

namespace vision {
    
const std::string d455_serial_number = "105322251084";
    
uint32_t FrameCnt = 0;
std::atomic<bool> NewD455(false);
cv::Mat depth_image16;
cv::Mat RGB_image;
uint16_t raw_depth[D455W*D455H];

rs2::pipeline d455_pipe;
rs2::frameset d455_frames;
    
void Init() {
    std::cout << "Vision module init" << std::endl;
    NewD455 = false;
    depth_image16 = cv::Mat(cv::Size(D455H,D455W),CV_16SC1);
    RGB_image = cv::Mat(cv::Size(D455H,D455W),CV_8UC3);
    
    // rs2::pipeline d455_pipe;
    rs2::config d455_cfg;
    d455_cfg.enable_device(d455_serial_number);
    d455_cfg.enable_stream(RS2_STREAM_COLOR, D455W, D455H, RS2_FORMAT_BGR8, D455FrameRate);
    d455_cfg.enable_stream(RS2_STREAM_DEPTH, D455W, D455H, RS2_FORMAT_Z16, D455FrameRate);

    d455_pipe.start(d455_cfg);
    // rs2::frameset d455_frames;
}

void Frame() {
    d455_frames = d455_pipe.wait_for_frames();
    rs2::video_frame color_frame = d455_frames.get_color_frame();
    rs2::depth_frame depth_frame = d455_frames.get_depth_frame();
    //if (color_frame && depth_frame) {
    if ((color_frame && depth_frame && NewD455 == false)) {
        RGB_image = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        memcpy(&raw_depth,depth_frame.get_data(),D455W*D455H);
        cv::Mat temp_depth_image = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()),CV_16SC1,(void *) depth_frame.get_data(),cv::Mat::AUTO_STEP);
        temp_depth_image.copyTo(depth_image16);
        NewD455 = true;
    }
}

} // end namespace
