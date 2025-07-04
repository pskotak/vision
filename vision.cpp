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

//rs2::points points;
uint32_t StartPcRow = 0;
uint32_t EndPcRow = D455H;
uint32_t IgnoreFromLeft = 120;
std::vector<T3Dpoint> PointCloud;
std::vector<TScanPoint> ScPoints;
std::vector<float> Distances;

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

// Hi @zsert3
// points frame behaves like the depth frame it was created from: It contains the array of stacked rows.
// ++ will bring you to the next point to the right, unless you reach of of row, then you will jump to start of next row. 
// Some points are going to be (0,0,0). 
// We keep those to allow easy navigation within the point-cloud grid (you can access point corresponding to pixel (i, j) using v[j * width() +i])
void Frame() {
    rs2::pointcloud pc;
    rs2::points points;
    const rs2::vertex *pc_vertices;
    rs2::vertex v;
    T3Dpoint P;
    TScanPoint sP;
    
    d455_frames = d455_pipe.wait_for_frames();
    rs2::video_frame color_frame = d455_frames.get_color_frame();
    rs2::depth_frame depth_frame = d455_frames.get_depth_frame();
    //if (color_frame && depth_frame) {
    if ((color_frame && depth_frame && NewD455 == false)) {
        RGB_image = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        memcpy(&raw_depth,depth_frame.get_data(),D455W*D455H);        
        cv::Mat temp_depth_image = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()),CV_16SC1,(void *) depth_frame.get_data(),cv::Mat::AUTO_STEP);
        points = pc.calculate(depth_frame);
        temp_depth_image.copyTo(depth_image16);
        pc_vertices = points.get_vertices();
        PointCloud.clear();
        
        //int pointcnt = 0;
        for (int row=StartPcRow;row<EndPcRow;row++) {
            for (int col=IgnoreFromLeft;col<D455W;col++) {
                v = pc_vertices[row*D455W+col];
                if ((v.x != 0) && (v.y != 0) && (v.z != 0)) {
                    //pointcnt++;
                    P.x=v.x; P.y=v.y; P.z=v.z;
                    PointCloud.push_back(P);
                }
            }
        }

        ScPoints.clear(); Distances.clear();
        for (int col=IgnoreFromLeft;col<D455W;col++) {
            float mindist = 1e6;
            //float minx = 1e6, miny = 1e6; minz = 1e6;
            sP.dist = 0.0;
            for (int row=StartPcRow;row<EndPcRow;row++) {
                v = pc_vertices[row*D455W+col];
                if ((v.x != 0) && (v.y != 0) && (v.z != 0)) {
                    float dist = sqrt((v.x*v.x) + (v.z*v.z));
                    if (dist < mindist) {
                        mindist = dist;
                        sP.x = v.x; sP.y = v.z; sP.z = v.z; sP.dist = mindist;
                    }
                }
            }
            ScPoints.push_back(sP);
            Distances.push_back(mindist);
        }

        //std::cout << pointcnt << std::endl;
        NewD455 = true;
    }
}

} // end namespace
