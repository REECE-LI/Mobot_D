//
// Created by L2248 on 2025/6/5.
//

#ifndef GCODE_H
#define GCODE_H
#include <vector>



struct GCode_t {
    float x ; // X coordinate
    float y; // Y coordinate
    bool isPen; // Pen state
};

struct Point_t {
    float x;
    float y;
};

struct Lut_t {
    Point_t drawPoint; // Draw point
    Point_t targetPoint;

    float deltaX;
    float deltaY;
};

#define LUT_SIZE 255



extern const GCode_t testGcode[];
extern const size_t testGcodeSize;  // 声明数组的大小

extern const Lut_t Lut[255];


Point_t compensatePoint(Point_t point);

#endif //GCODE_H
