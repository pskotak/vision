#ifndef VISION_H
#define VISION_H

#include <atomic>
#include <vector>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

namespace vision {

#define D455W 848
#define D455H 480

#define D455depth_color_max 3000.0f
#define D455FrameRate 30

extern cv::Mat depth_image16;
extern cv::Mat RGB_image;

extern std::atomic<bool> NewD455;

extern uint32_t StartPcRow;
extern uint32_t EndPcRow;
extern uint32_t IgnoreFromLeft;
typedef struct {
    float x,y,z;
} T3Dpoint;

typedef struct {
    float x,y,z,dist;
} TScanPoint;

extern std::vector<T3Dpoint> PointCloud;
extern std::vector<TScanPoint> ScPoints;
extern std::vector<float> Distances;

extern void Init();
extern void Frame();

} // end namespace

#endif
