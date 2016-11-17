//
//  Disparity_data.h
//  VerizonDisparityMap
//
//  Created by Madhusudan Govindraju on 11/16/16.
//  Copyright © 2016 Madhusudan Govindraju. All rights reserved.
//

#ifndef Disparity_data_h
#define Disparity_data_h

class Disparity_data
{
    
//private:
    
    //the members are declared public to reduce the implementation time else the class variables should all be private with set and get methods to access them.
public:
    cv::Mat pointCloud;
    cv::Mat leftOriginal;
    cv::Mat rightOriginal;
    

    // setting some global variables for plotting in OpenGL
    double focalLength = 300.0;
    double baseline = 100.0;
    int numDisparity = 16*7;
    float oPscale = 8.0f; // how we want to scale the output
    float xscale = oPscale / pointCloud.cols;
    float yscale = oPscale / pointCloud.rows;
    float dscale = oPscale / ((focalLength*baseline)/ numDisparity);
    
    // multiple of window size and 16 pixels
    int m_NumDisparities = 16*7;
    // the window size which gave the best disparity, if lesser, more noise, in case window size is more the disparity smoothes a lot.
    int m_SADWindowSize = 7;
    int m_UniquenessRatio = 0;
    int m_MinDisparity = 0;
    int m_P1 = 8 * 3 * m_SADWindowSize * m_SADWindowSize;
    int m_P2 = 32 * 3 * m_SADWindowSize * m_SADWindowSize;
    int m_Disp12MaxDiff = 100000;
    int m_SpeckleWindowSize = 0;
    int m_Mode = cv::StereoSGBM::MODE_SGBM_3WAY;
    double m_LambdaValue = 8000.0;
    double m_SigmaColor = 1.5;
    bool m_Downscale = false;
    bool m_QMatSet = false;
    int m_CalibrationImagesFilename = NULL;
    
    cv::Mat m_Disparity;
    cv::Mat flippedDisp;
    cv::Rect roi;
    
    //vectors to store the 3D points
    std::vector<float> Xpoints;
    std::vector<float> Ypoints;
    std::vector<float> Zpoints;
    std::vector<cv::Vec3f> colour;
//public:
    Disparity_data();
    void set_afterPointCloud();
    void set_leftOriginal(cv::Mat Img);
    void set_rightOriginal(cv::Mat Img);
    void set_pointCloud(cv::Mat pc);
    void set_Xpoints(std::vector<float> points);
    void set_Ypoints(std::vector<float> points);
    void set_Zpoints(std::vector<float> points);
    void set_Colour(std::vector<cv::Vec3f> points);
    cv::Mat get_leftOriginal();
    cv::Mat get_rightOriginal();
    cv::Mat get_pointCloud();
    std::vector<float> get_Xpoints();
    std::vector<float> get_Ypoints();
    std::vector<float> get_Zpoints();
    std::vector<cv::Vec3f> get_Colour();
    
}d;

void Disparity_data::set_afterPointCloud()
{
    // setting some global variables for OpenGL
    d.focalLength = 300.0;
    d.baseline = 100.0;
    d.numDisparity = 16*7;
    d.oPscale = 8.0f;
    d.xscale = oPscale / pointCloud.cols;
    d.yscale = oPscale / pointCloud.rows;
    d.dscale = oPscale / ((focalLength*baseline)/ numDisparity);
}

void Disparity_data::set_leftOriginal(cv::Mat leftImg)
{
    leftOriginal = leftImg;
}
void Disparity_data::set_rightOriginal(cv::Mat rightImg)
{
    rightOriginal = rightImg;
}

void Disparity_data::set_pointCloud(cv::Mat pc)
{
    pointCloud = pc;
}
void Disparity_data::set_Xpoints(std::vector<float> points)
{
    Xpoints = points;
}
void Disparity_data::set_Ypoints(std::vector<float> points)
{
    Ypoints = points;
}
void Disparity_data::set_Zpoints(std::vector<float> points)
{
    Zpoints = points;
}
void Disparity_data::set_Colour(std::vector<cv::Vec3f> points)
{
    colour = points;
}
cv::Mat Disparity_data::get_leftOriginal()
{
    return leftOriginal;
}
cv::Mat Disparity_data::get_rightOriginal()
{
    return rightOriginal ;
}

cv::Mat Disparity_data::get_pointCloud()
{
    return pointCloud;
}
std::vector<float> Disparity_data::get_Xpoints()
{
    return Xpoints ;
}
std::vector<float> Disparity_data::get_Ypoints()
{
    return Ypoints ;
}
std::vector<float> Disparity_data::get_Zpoints()
{
    return Zpoints ;
}
std::vector<cv::Vec3f> Disparity_data::get_Colour()
{
    return colour;
}





#endif /* Disparity_data_h */
