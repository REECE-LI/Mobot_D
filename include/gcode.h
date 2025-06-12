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

struct GCodeArray {
    const GCode_t* data;
    size_t size;
};



extern const GCode_t testGcode[];
extern const size_t testGcodeSize;  // 声明数组的大小

// extern const GCode_t gcode_0_fix[];
// extern const size_t gcode0Size;
// extern const GCode_t gcode_1_fix[];
// extern const size_t gcode1Size;
// extern const GCode_t gcode_2_fix[];
// extern const size_t gcode2Size;
// extern const GCode_t gcode_3_fix[];
// extern const size_t gcode3Size;
// extern const GCode_t gcode_4_fix[];
// extern const size_t gcode4Size;
// extern const GCode_t gcode_5_fix[];
// extern const size_t gcode5Size;
// extern const GCode_t gcode_6_fix[];
// extern const size_t gcode6Size;
// extern const GCode_t gcode_7_fix[];
// extern const size_t gcode7Size;
// extern const GCode_t gcode_8_fix[];
// extern const size_t gcode8Size;
// extern const GCode_t gcode_9_fix[];
// extern const size_t gcode9Size;
// extern const GCode_t gcode_10_fix[];
// extern const size_t gcode10Size;
// extern const GCode_t gcode_11_fix[];
// extern const size_t gcode11Size;
// extern const GCode_t gcode_12_fix[];
// extern const size_t gcode12Size;
// extern const GCode_t gcode_13_fix[];
// extern const size_t gcode13Size;
// extern const GCode_t gcode_14_fix[];
// extern const size_t gcode14Size;
// extern const GCode_t gcode_15_fix[];
// extern const size_t gcode15Size;
// extern const GCode_t gcode_16_fix[];
// extern const size_t gcode16Size;
// extern const GCode_t gcode_17_fix[];
// extern const size_t gcode17Size;
// extern const GCode_t gcode_18_fix[];
// extern const size_t gcode18Size;
// extern const GCode_t gcode_19_fix[];
// extern const size_t gcode19Size;
// extern const GCode_t gcode_20_fix[];
// extern const size_t gcode20Size;
// extern const GCode_t gcode_21_fix[];
// extern const size_t gcode21Size;
// extern const GCode_t gcode_22_fix[];
// extern const size_t gcode22Size;
// extern const GCode_t gcode_23_fix[];
// extern const size_t gcode23Size;
// extern const GCode_t gcode_24_fix[];
// extern const size_t gcode24Size;
// extern const GCode_t gcode_25_fix[];
// extern const size_t gcode25Size;
// extern const GCode_t gcode_26_fix[];
// extern const size_t gcode26Size;
// extern const GCode_t gcode_27_fix[];
// extern const size_t gcode27Size;
// extern const GCode_t gcode_28_fix[];
// extern const size_t gcode28Size;
// extern const GCode_t gcode_29_fix[];
// extern const size_t gcode29Size;
// extern const GCode_t gcode_30_fix[];
// extern const size_t gcode30Size;
// extern const GCode_t gcode_31_fix[];
// extern const size_t gcode31Size;
// extern const GCode_t gcode_32_fix[];
// extern const size_t gcode32Size;
// extern const GCode_t gcode_33_fix[];
// extern const size_t gcode33Size;
// extern const GCode_t gcode_34_fix[];
// extern const size_t gcode34Size;
// extern const GCode_t gcode_35_fix[];
// extern const size_t gcode35Size;
// extern const GCode_t gcode_36_fix[];
// extern const size_t gcode36Size;
// extern const GCode_t gcode_37_fix[];
// extern const size_t gcode37Size;
// extern const GCode_t gcode_38_fix[];
// extern const size_t gcode38Size;
// extern const GCode_t gcode_39_fix[];
// extern const size_t gcode39Size;
// extern const GCode_t gcode_40_fix[];
// extern const size_t gcode40Size;
// extern const GCode_t gcode_41_fix[];
// extern const size_t gcode41Size;
// extern const GCode_t gcode_42_fix[];
// extern const size_t gcode42Size;
// extern const GCode_t gcode_43_fix[];
// extern const size_t gcode43Size;
// extern const GCode_t gcode_44_fix[];
// extern const size_t gcode44Size;
// extern const GCode_t gcode_45_fix[];
// extern const size_t gcode45Size;
// extern const GCode_t gcode_46_fix[];
// extern const size_t gcode46Size;
// extern const GCode_t gcode_47_fix[];
// extern const size_t gcode47Size;
// extern const GCode_t gcode_48_fix[];
// extern const size_t gcode48Size;
// extern const GCode_t gcode_49_fix[];
// extern const size_t gcode49Size;
// extern const GCode_t gcode_50_fix[];
// extern const size_t gcode50Size;
// extern const GCode_t gcode_51_fix[];
// extern const size_t gcode51Size;
// extern const GCode_t gcode_52_fix[];
// extern const size_t gcode52Size;
// extern const GCode_t gcode_53_fix[];
// extern const size_t gcode53Size;
// extern const GCode_t gcode_54_fix[];
// extern const size_t gcode54Size;
// extern const GCode_t gcode_55_fix[];
// extern const size_t gcode55Size;
// extern const GCode_t gcode_56_fix[];
// extern const size_t gcode56Size;
// extern const GCode_t gcode_57_fix[];
// extern const size_t gcode57Size;
// extern const GCode_t gcode_58_fix[];
// extern const size_t gcode58Size;
// extern const GCode_t gcode_59_fix[];
// extern const size_t gcode59Size;
// extern const GCode_t gcode_60_fix[];
// extern const size_t gcode60Size;
// extern const GCode_t gcode_61_fix[];
// extern const size_t gcode61Size;
// extern const GCode_t gcode_62_fix[];
// extern const size_t gcode62Size;
// extern const GCode_t gcode_63_fix[];
// extern const size_t gcode63Size;
// extern const GCode_t gcode_64_fix[];
// extern const size_t gcode64Size;
// extern const GCode_t gcode_65_fix[];
// extern const size_t gcode65Size;
// extern const GCode_t gcode_66_fix[];
// extern const size_t gcode66Size;
// extern const GCode_t gcode_67_fix[];
// extern const size_t gcode67Size;
// extern const GCode_t gcode_68_fix[];
// extern const size_t gcode68Size;
// extern const GCode_t gcode_69_fix[];
// extern const size_t gcode69Size;
// extern const GCode_t gcode_70_fix[];
// extern const size_t gcode70Size;
// extern const GCode_t gcode_71_fix[];
// extern const size_t gcode71Size;
// extern const GCode_t gcode_72_fix[];
// extern const size_t gcode72Size;
// extern const GCode_t gcode_73_fix[];
// extern const size_t gcode73Size;
// extern const GCode_t gcode_74_fix[];
// extern const size_t gcode74Size;
// extern const GCode_t gcode_75_fix[];
// extern const size_t gcode75Size;
// extern const GCode_t gcode_76_fix[];
// extern const size_t gcode76Size;
// extern const GCode_t gcode_77_fix[];
// extern const size_t gcode77Size;
// extern const GCode_t gcode_78_fix[];
// extern const size_t gcode78Size;
// extern const GCode_t gcode_79_fix[];
// extern const size_t gcode79Size;
// extern const GCode_t gcode_80_fix[];
// extern const size_t gcode80Size;
// extern const GCode_t gcode_81_fix[];
// extern const size_t gcode81Size;
// extern const GCode_t gcode_82_fix[];
// extern const size_t gcode82Size;
// extern const GCode_t gcode_83_fix[];
// extern const size_t gcode83Size;
// extern const GCode_t gcode_84_fix[];
// extern const size_t gcode84Size;
// extern const GCode_t gcode_85_fix[];
// extern const size_t gcode85Size;
// extern const GCode_t gcode_86_fix[];
// extern const size_t gcode86Size;
// extern const GCode_t gcode_87_fix[];
// extern const size_t gcode87Size;
// extern const GCode_t gcode_88_fix[];
// extern const size_t gcode88Size;
// extern const GCode_t gcode_89_fix[];
// extern const size_t gcode89Size;
// extern const GCode_t gcode_90_fix[];
// extern const size_t gcode90Size;




extern const GCodeArray allGcodes[];




extern const Lut_t Lut[255];


Point_t compensatePoint(Point_t point);

#endif //GCODE_H
