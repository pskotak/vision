#include <iostream>
#include <unistd.h>

#include "vision.h"

int main(int argc, char **argv) {
    bool QuitProgram = false;
    cv::Mat processed_frame;
    
    std::cout << "Test vision." << std::endl;
    
#if 1
    vision::Init();
    
    while (!QuitProgram) {
        vision::Frame();
        if (vision::NewD455) {
            cv::imshow("RGB",vision::RGB_image);
            
            vision::depth_image16.convertTo(processed_frame,CV_8U,255.0 / D455depth_color_max,0.0);
            cv::applyColorMap(processed_frame,processed_frame,cv::COLORMAP_JET);
            cv::imshow("Depth",processed_frame);
            
            vision::NewD455 = false;
        }
        
        int key = cv::waitKey(1); // Wait for a key press for 1ms
        if (key == 27) { // ESC key
            QuitProgram = true;
        }
        
        usleep(100000);
    }
#else
    const std::string d455_serial_number = "105322251084";
    
    rs2::pipeline d455_pipe;
    rs2::config d455_cfg;
    d455_cfg.enable_device(d455_serial_number);
    d455_cfg.enable_stream(RS2_STREAM_COLOR, D455W, D455H, RS2_FORMAT_BGR8, 15);
    d455_cfg.enable_stream(RS2_STREAM_DEPTH, D455W, D455H, RS2_FORMAT_Z16, 15);

    d455_pipe.start(d455_cfg);
    rs2::frameset d455_frames;
#endif
    return 0;
}
