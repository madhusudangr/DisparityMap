//
//  main.cpp
//  VerizonDisparityMap
//
//  Created by Madhusudan Govindraju on 11/9/16.
//  Copyright © 2016 Madhusudan Govindraju. All rights reserved.
//
//     Program to calculate the disparity map from sereo images
//


#include <iostream>
#include "main.h"

void my_computeDisparityMap()
{
    
    cv::Mat left_grey, right_grey;
    cv::Mat left_disp, right_disp, filtered_disp; // 16S
    
    d.leftOriginal = cv::imread( "im2.png");
    d.rightOriginal = cv::imread( "im6.png");
    
    // get greyscale images
    cv::cvtColor(d.leftOriginal, left_grey, CV_BGR2GRAY);
    cv::cvtColor(d.rightOriginal, right_grey, CV_BGR2GRAY);
    
    // scale down the image
    if (d.m_Downscale)
    {
        cv::resize(left_grey, left_grey, cv::Size(), 0.5, 0.5);
        cv::resize(right_grey, right_grey, cv::Size(), 0.5, 0.5);
    }
    
    // compute left disparity map using stereo correspondence algorithm (Semi-Global Block Matching or SGBM algorithm)
    cv::Ptr<cv::StereoSGBM> left_sbm = cv::StereoSGBM::create(d.m_MinDisparity, d.m_NumDisparities, d.m_SADWindowSize);
    left_sbm->setUniquenessRatio(d.m_UniquenessRatio);
    left_sbm->setDisp12MaxDiff(d.m_Disp12MaxDiff);
    left_sbm->setSpeckleWindowSize(d.m_SpeckleWindowSize);
    left_sbm->setP1(d.m_P1);
    left_sbm->setP2(d.m_P2);
    left_sbm->setMode(d.m_Mode);
    left_sbm->compute(left_grey, right_grey, left_disp);
    
    // compute right disparity map
    cv::Ptr<cv::StereoMatcher> right_sbm = cv::ximgproc::createRightMatcher(left_sbm);
    //m_RightRegionOfInterest = _computeRegionOfInterest(m_RightOriginal.size(), right_sbm);
    right_sbm->compute(right_grey, left_grey, right_disp);
    
    // applying Global Smoothness
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter = cv::ximgproc::createDisparityWLSFilterGeneric(true);
    filter->setDepthDiscontinuityRadius((int)ceil(0.5*d.m_SADWindowSize));
    filter->setLambda(d.m_LambdaValue);
    filter->setSigmaColor(d.m_SigmaColor);
    
    // compute filtered disparity map
    filter->filter(left_disp, left_grey, filtered_disp, right_disp);
    
    // convert filtered disparity map from 16 bit short to 8 bit unsigned char and normalize values
    double minVal, maxVal;
    cv::minMaxLoc(filtered_disp, &minVal, &maxVal);
    filtered_disp.convertTo(d.m_Disparity, CV_8UC1, 255 / (maxVal - minVal));
    //cv::imshow("Disparity",d.m_Disparity);
    imwrite("SGBM_sample.png", d.m_Disparity); //save the disparity for further use
}



void my_create_pointCloud()
{

    //Now creating a Camera Matrix and using that to calculate the pointCloud
    double principalPointLeftX = d.leftOriginal.cols * 0.5; //the principal point is the image center
    double principalPointLeftY = d.rightOriginal.rows * 0.5;
    double principalPointRightX = principalPointLeftX;
    
    cv::Mat Qmatrix(cv::Size(4,4), CV_64F);
    Qmatrix.at<double>(0, 0) = 1.0;
    Qmatrix.at<double>(0, 1) = 0.0;
    Qmatrix.at<double>(0, 2) = 0.0;
    Qmatrix.at<double>(0, 3) = -principalPointLeftX;
    Qmatrix.at<double>(1, 0) = 0.0;
    Qmatrix.at<double>(1, 1) = 1.0;
    Qmatrix.at<double>(1, 2) = 0.0;
    Qmatrix.at<double>(1, 3) = -principalPointLeftY;
    Qmatrix.at<double>(2, 0) = 0.0;
    Qmatrix.at<double>(2, 1) = 0.0;
    Qmatrix.at<double>(2, 2) = 0.0;
    Qmatrix.at<double>(2, 3) = d.focalLength;
    Qmatrix.at<double>(3, 0) = 0.0;
    Qmatrix.at<double>(3, 1) = 0.0;
    Qmatrix.at<double>(3, 2) = 1.0 / d.baseline;
    Qmatrix.at<double>(3, 3) = (principalPointLeftX - principalPointRightX) / d.baseline;// (epipolar plane conversion)

    
    //whie projecting the image we should keep in mind  min/max row & column bounds for the template and blocks
    cv::Size sz = d.leftOriginal.size();
    int rowMin = (d.m_NumDisparities-1)+(d.m_SADWindowSize/2);
    int rowMax = sz.width-(d.m_SADWindowSize/2);
    int colMin = (d.m_SADWindowSize/2);
    int colMax = sz.height-(d.m_SADWindowSize/2);
    cv::Rect troi (rowMin, colMin, rowMax - rowMin, colMax - colMin);
    d.roi = troi;
    
    //so we have to estimate point cloud for the roi, shouldnt go outside, else the point cloud will have splits, we will be searching for matches outside the search space.
    cv::flip(d.m_Disparity(troi), d.flippedDisp, 0);
    reprojectImageTo3D(d.flippedDisp, d.pointCloud, Qmatrix, false, CV_32F);
    //cv::reprojectImageTo3D(d.m_Disparity, d.pointCloud, Qmatrix,false,CV_32F);
    
    //after getting the point cloud we set some scaling variables to help scale the plot being rendered in OpenGL
    d.oPscale = 20.0f;
    d.xscale = d.oPscale / d.pointCloud.cols;
    d.yscale = d.oPscale / d.pointCloud.rows;
    d.dscale = d.oPscale / ((d.focalLength*d.baseline)/ d.numDisparity);
}

/* Handler for window-repaint event. Called back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {

    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    
    // Render the point cloud
    glLoadIdentity();                  // Reset the model-view matrix
    glTranslatef(-0.0f, -1.0f, -60.0f);  // Move view out of the screen(-30),  down(-1)
    glRotatef(angle,0.0f, 1.0f, 0.0f);  // Rotate about the (,1,0)-axis
    
    glPointSize(1);
    glBegin(GL_POINTS);           // Begin drawing the pyramid with 4 triangles

    for(int i=0;i<d.Xpoints.size();i++)
        {
            cv::Vec3f c = d.colour[i];
            glColor3f(c[2], c[1], c[0]);
            glVertex3f(d.Xpoints[i], d.Ypoints[i], d.Zpoints[i]);
        }
    glEnd();
    
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
    
    // Update the rotational angle after each refresh [NEW]
    angle += 0.09f;
}

void render3D_pointCloud(int argc,char** argv)
{
    //Convert the 3d Disparity Points to an array of positions so that I can display it in OpenGL
    cv::Mat newScale_leftImage;
    //cv::imshow("Original Image",d.leftOriginal);
    cv::waitKey(0);
    d.leftOriginal.convertTo(newScale_leftImage, CV_64FC3 ,1.0/255.0);//the colour scale
    for (int y = 0; y < d.pointCloud.rows; ++y)
    {
        for (int x = 0; x < d.pointCloud.cols; ++x)
        {
            // get point position
            cv::Vec3f pos = d.pointCloud.at<cv::Vec3f>(y, x);
            
            // skip this point if depth is unknown (+-infinity)
            if (pos[2] == -INFINITY || pos[2] == INFINITY)
            {
                continue;
            }
            
             //in matlab the array axis is facing down, in openGL the axis is facing up
            cv::Vec3f imgcolor = d.leftOriginal(d.roi).at<cv::Vec3b>(d.leftOriginal.rows - y - 1, x);

            // convert colors from 0-255 to 0-1
            float r = imgcolor[2] * 1.0f / 255.0f;
            float g = imgcolor[1] * 1.0f / 255.0f;
            float b = imgcolor[0] * 1.0f / 255.0f;
            
            imgcolor[2] = r;
            imgcolor[1] = g;
            imgcolor[0] = b;
            
            float posx = (pos[0] * d.xscale);
            float posy = (pos[1] * d.yscale);
            float posz = (pos[2] * d.dscale);
            d.Xpoints.push_back(posx);
            d.Ypoints.push_back(posy);
            d.Zpoints.push_back(posz);
            d.colour.push_back(imgcolor);
        }
    }
    
    //initialize the OPEN GL
    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
    glutInitWindowSize(640, 480);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutCreateWindow(title);          // Create window with the given title
    glutDisplayFunc(display);       // Register callback handler for window re-paint event
    glutReshapeFunc(reshape);       // Register callback handler for window re-size event
    initGL();                       // Our own OpenGL initialization
    glutTimerFunc(0, timer, 0);     // First timer call immediately [NEW]
    glutMainLoop();                 // Enter the infinite event-processing loop
    
}

int main(int argc, char** argv)
{
    //steps
    //Disparity_data d;
    //1. Compute Disparity Map
    my_computeDisparityMap();
    //2. Calculate the point cloud
    my_create_pointCloud();
    //3. show the points cloud in OpenGL
    render3D_pointCloud(argc, argv);

    return 1;
}








