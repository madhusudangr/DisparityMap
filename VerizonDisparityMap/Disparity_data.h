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
    double baseline = 97.0;
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
//    Disparity_data();
//    void set_afterPointCloud();
//    void set_leftOriginal(cv::Mat Img);
//    void set_rightOriginal(cv::Mat Img);
//    void set_pointCloud(cv::Mat pc);
}d;

//void Disparity_data::set_afterPointCloud()
//{
//    // setting some global variables for OpenGL
//    focalLength = 300.0;
//    baseline = 97.0;
//    numDisparity = 16*7;
//    whscale = 8.0f; // width/height scale
//    xscale = whscale / pointCloud.cols; // downscale image
//    yscale = whscale / pointCloud.rows; // downscale image
//    cscale = 1.0f / 255.0f; // convert from 0-255 to 0-1
//    dscale = whscale / ((focalLength*baseline)/ numDisparity);
//    doffset = -6.0f;
//}
//
//void Disparity_data::set_leftOriginal(cv::Mat leftImg)
//{
//    leftOriginal = leftImg;
//}
//void Disparity_data::set_rightOriginal(cv::Mat rightImg)
//{
//    rightOriginal = rightImg;
//}
//
//void Disparity_data::set_pointCloud(cv::Mat pc)
//{
//    pointCloud = pc;
//}



#endif /* Disparity_data_h */
