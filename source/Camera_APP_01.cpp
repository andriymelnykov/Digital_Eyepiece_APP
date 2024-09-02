// Copyright 2024, Andriy Melnykov
// https://github.com/andriymelnykov/Digital_Eyepiece_APP
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)

// This file contains the 'main' function. Program execution begins and ends there.
//

#define _CRT_SECURE_NO_WARNINGS

//#include <cstdlib>

//#include <windows.h>
//#include <shellapi.h>
//HWND hwnd = NULL;

//#include <stdio.h>
//#include <stdlib.h>

//#include <vector>
#include <thread>


#include <fitsio.h>

#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs = std::filesystem;


#include <time.h>

#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/photo/photo.hpp"


#include "camera_functions.h"

#include "opencv2/reg/map.hpp"
#include "opencv2/reg/mapper.hpp"
#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mapperpyramid.hpp"


#define AI_NOISEREDUCTION true // if true, AI noise reducition code included

#if AI_NOISEREDUCTION
#include "fdeep/fdeep.hpp"
#endif /* AI_NOISEREDUCTION */


using namespace std;
using namespace cv;
using namespace reg;

long image_size; // , image_size_v, image_size_f;

#define exposure_threshold 100000   //50000 //in µs, threshold for video mode switching

ofstream logfile, picfile;    //logfile with debug information, picfile with saved picture information

int debug_flag;

double t_cycle_old;   // for cycle time measurement and fps
double t_cycle;
double t_delta;

long exposure_time, exposure_time_v, exposure_time_f;
long gain, gain_v, gain_f;
long WB_R, WB_R_v, WB_R_f;
long WB_B, WB_B_v, WB_B_f;
long offset, offset_v, offset_f;
int highspeed_v;
long bandwidth; // , bandwidth_v, bandwidth_f;
long monobin; // , monobin_v, monobin_f;
int banding_filter_flag;
int banding_filter_strength;
float banding_filter_threshold;
long target_temperature;
int bin; // , bin_v, bin_f;
int image_bytes; // , image_bytes_v, image_bytes_f;
int image_flip;
int image_rotation;
int dark_v_hotpixel_flag, dark_v_subtract_flag;
int dark_f_hotpixel_flag, dark_f_subtract_flag;
int flat_v_flag, flat_f_flag;
char dark_v_filename[80] = "dark_v.fits";
char dark_f_filename[80] = "dark_f.fits";
char flat_filename[80] = "flat.fits";
float flat_inv_factor;
int cooler_activation;
int display_height;
int background_comp_flag, noise_reduction_flag;
float filter_strength_1;
float filter_strength_2;
float black_level_value;
int circular_mask_background_flag;
float circular_mask_background_size;
int circular_mask_background_show;
int circular_mask_flag;
int enhance_stars_flag;
int star_blob_radius;
float star_blob_strength;
int highlight_protection_flag;
float init_gamma;
float WBcorr_R, WBcorr_G, WBcorr_B;

int color_correction_flag;
float CC11, CC12, CC13;  //color correction matrix
float CC21, CC22, CC23;
float CC31, CC32, CC33;

float aR, bR, cR;  //dual band colors for R
float aG, bG, cG;  //dual band colors for G
float aB, bB, cB;  //dual band colors for B
double focusing_zoom_value;
double display_zoom_value;

int GUI_flag;

int AI_noise_flag;
int AI_noise_frames;
char AI_noise_model_filename[80];

int eyepiece_display_flag;
int eyepiece_display_X_pixels;
int eyepiece_display_Y_pixels;
float eyepiece_display_X_mm;
float eyepiece_display_Y_mm;
float interpupillary_distance_mm;
int eyepiece_display_rotation;
int second_display_X;
int second_display_Y;


int focusing_flag;


int asi_connected_cameras;
int asi_num_controls;
ASI_CAMERA_INFO** asi_camera_info;
ASI_CONTROL_CAPS** asi_control_caps;
unsigned char* asi_image;
//ASI_EXPOSURE_STATUS asi_exp_status;
int camera_image_width, camera_image_height;


unsigned char* dark_v_image;
unsigned char* dark_f_image;

Mat dark_v_32sc1;
Mat dark_f_32sc1;
int num_hotpixel_v, num_hotpixel_f;
int* hotpixel_list_v;  //list of hot pixels with coordinates and neighbors for interpolation
int* hotpixel_list_f;  //list of hot pixels with coordinates and neighbors for interpolation
int32_t hotpixel_threshold = 5000;
double dark_v_mean = 0, dark_f_mean = 0;

unsigned char* flat_image;
Mat flat_32fc3;
Mat flat_inv_32fc3;

int cam;
int key;
int monobin_k; // , monobin_k_v, monobin_k_f;

int state;
int old_state;
int frames_stacked; // number of stacked frames in foto mode
int color_palette;
int new_picture = 0;
int hist_show_state = 0;

float gamma;
float gamma_dark;
//#define LUT_size 1000
#define LUT_size 65536
float LUT_in[LUT_size];
float LUT_out[LUT_size];
float LUT_max_y;
float LUT_dark_out[LUT_size];  // this LUT for highlight protection algorithm
float LUT_dark_max_y;


Mat RAW_image;    // direct from camera, copy for histogram plot
Mat final_image;  // final image before resize, with all processing
Mat stack_image;   // only stack with darks/flats, before further processing
//Mat stars_image;  // working image for bright star "blobs"
Mat sum_image;   // sum image for stacking
Mat first_image;
Mat display_image;  // resized image for display
Ptr<Map> mapPtr; //current affine map, used for image registration
Ptr<Map> mapPtr_old; //previos affine map, used as initial estimation for next image registration

//Control keys
/*
#define key_exit (int)'x'   // exit
#define key_mode (int)'m'    //mode change foto, video
#define key_plus (int)'+'    //gain +
#define key_minus (int)'-'   //gain -
#define key_palette (int)'p'    //palette change foto, video
#define key_save_image (int)'s'
#define key_focusing (int)'f'  //focusing zoom
/**/
int key_exit;       //(int)'x'   // exit
int key_mode;       //(int)'m'   //mode change foto, video
int key_plus;       //(int)'+'   //gain +
int key_minus;      //(int)'-'   //gain -
int key_palette;    //(int)'p'   //palette change foto, video
int key_save_image; //(int)'s'   //save images
int key_focusing;   //(int)'f'   //focusing zoom
int key_histogram;   //(int)'h'   //show histogram


#define palette_rgb 0 //for color palettes
#define palette_duo 1
//#define palette_halpha 2




//---------------- GUI
struct Button {
    cv::Rect rect;
    std::string text;
    bool pressed;
};

std::vector<Button> buttons;


void drawButton(cv::Mat& img, const Button& button) {
    cv::Scalar color = button.pressed ? cv::Scalar(1, 1, 1) : cv::Scalar(0.4, 0.4, 0.4);
    cv::rectangle(img, button.rect, color, 2);
    //cv::rectangle(img, Rect(button.rect.x, button.rect.y, button.rect.width, button.rect.height), color, 2);
    //cv::rectangle(img, button.rect, cv::Scalar(0, 0, 0), 2);
    int baseline = 0;
    //cout << button.rect.width << endl;
    double text_scale = (double)button.rect.width / 490.0;
    cv::Size textSize = cv::getTextSize(button.text, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
    cv::Point textOrg(button.rect.x + (button.rect.width - textSize.width) / 2, button.rect.y + (button.rect.height + textSize.height) / 2);
    cv::putText(img, button.text, textOrg, cv::FONT_HERSHEY_SIMPLEX, text_scale, color, 1);
}


void initButtons(std::vector<Button>& buttons) {

    // Initialize buttons
    buttons.clear();
    int k = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            Button btn;
            //btn.rect = cv::Rect(j * buttonWidth, originalHeight + i * buttonHeight, buttonWidth, buttonHeight);
            btn.rect = cv::Rect(0, 0, 1, 1);
            //btn.text = "Button " + std::to_string(i * 2 + j + 1);
            if (k == 0) btn.text = "-";
            if (k == 1) btn.text = "+";
            if (k == 2) btn.text = "Palette change";
            if (k == 3) btn.text = "Mode change";
            if (k == 4) btn.text = "Focus zoom";
            if (k == 5) btn.text = "RAW histogram";
            if (k == 6) btn.text = "Save picture";
            if (k == 7) btn.text = "Exit";
            btn.pressed = false;
            buttons.push_back(btn);
            k++;
        }
    }

}

cv::Mat addButtonField(cv::Mat& img, std::vector<Button>& buttons) {
    int originalHeight = img.rows;
    int originalWidth = img.cols;
    int buttonFieldHeight = originalHeight / (double)3.5;

    // Create new image with additional button field space
    cv::Mat newImg(originalHeight + buttonFieldHeight, originalWidth, img.type());
    img.copyTo(newImg(cv::Rect(0, 0, originalWidth, originalHeight)));

    // Define button attributes
    int buttonWidth = originalWidth / 2;
    int buttonHeight = buttonFieldHeight / 4;

    // Initialize buttons
    //buttons.clear();
    int k = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            //Button btn;
            //btn.rect = cv::Rect(j * buttonWidth, originalHeight + i * buttonHeight, buttonWidth, buttonHeight);
            buttons[k].rect = cv::Rect(j * buttonWidth + 5, originalHeight + i * buttonHeight + 5, buttonWidth - 5, buttonHeight - 5);
            //btn.text = "Button " + std::to_string(i * 2 + j + 1);
            //btn.pressed = false;
            //buttons.push_back(btn);
            k++;
        }
    }

    // Draw buttons
    for (const Button& button : buttons) {
        drawButton(newImg, button);
    }

    return newImg;
}


void onMouse(int event, int x, int y, int, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    std::vector<Button>* buttons = reinterpret_cast<std::vector<Button>*>(userdata);
    for (size_t i = 0; i < buttons->size(); ++i) {
        if (buttons->at(i).rect.contains(cv::Point(x, y))) {
            //buttons->at(i).pressed = !buttons->at(i).pressed;
            buttons->at(i).pressed = true;
            //std::cout << "Button " << (i + 1) << " pressed status: " << buttons->at(i).pressed << std::endl;
            break;
        }
    }
}
//---------------- GUI






void compute_LUT(float gamma) {

    if (debug_flag == 1) {
        cout << "Computing LUT" << endl;
        logfile << "Computing LUT" << endl;
    }

    if (gamma < 1.0)
        gamma_dark = gamma;
    else if (gamma < 15)
        gamma_dark = 0.286 * gamma + 0.714;
    else
        gamma_dark = 0.118 * gamma + 3.235;

    //gamma_dark = gamma / 10;


    //cout << "gamma: " << gamma << " " << gamma_dark << endl;

    LUT_max_y = atan(gamma);
    LUT_dark_max_y = atan(tan(1.0 * LUT_max_y) / gamma * gamma_dark);
    //LUT_dark_max_y = atan(gamma_dark);
    //LUT_max_y = asinh(gamma);
    for (int i = 0; i < LUT_size; i++) {
        LUT_in[i] = i / (float)(LUT_size);
        LUT_out[i] = atan(LUT_in[i] * gamma) / LUT_max_y;
        LUT_dark_out[i] = atan(tan( LUT_in[i] * LUT_max_y) / gamma * gamma_dark ) / LUT_dark_max_y;
        


        //LUT_dark_out[i] = atan(LUT_in[i] * gamma_dark) / LUT_dark_max_y;
        //LUT_out[i] = asinh(LUT_in[i] * gamma) / LUT_max_y;
        //LUT_out[i] = LUT_in[i];  // no stretch
        //LUT_out[i] = gamma * LUT_in[i];  // simple liniear

        //sRGB gamma
        /*
        if (LUT_out[i] < 0.0031308)
            LUT_out[i] = 12.92 * LUT_out[i];
        else
            LUT_out[i] = 1.055 * pow(LUT_out[i], (1 / 2.4))  - 0.055;  /**/
        
        //LUT_out[i] = LUT_out[i] * 2;
        //LUT_out[i] = asinh(LUT_out[i] * gamma) / LUT_max_y;

    }
    
    //printf("lut %f %f %f\n %f %f %f\n", LUT_in[0], LUT_in[1], LUT_in[LUT_size - 1], LUT_out[0], LUT_out[1], LUT_out[LUT_size - 1]);

    //Show LUT
    /*
    Mat LUTImage(500, 500, CV_8UC1, Scalar(0));
    
    for (int i = 1; i < LUT_size; i++)
    {
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500*LUT_out[i - 1])),
                       Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_out[i])),
                       Scalar(255), 1, 8, 0);
        line(LUTImage, Point(cvRound(500 * LUT_in[i - 1]), cvRound(500 - 500 * LUT_dark_out[i - 1])),
            Point(cvRound(500 * LUT_in[i]), cvRound(500 - 500 * LUT_dark_out[i])),
            Scalar(255), 1, 8, 0);
        
    }
    imshow("LUT", LUTImage);
    /**/
}



float LookUpTable(float x, float LookUp_in[], float LookUp_out[], unsigned int LookUp_length) {

    // old linear interpolation
    /*
    float out = 0;

    //liniear look-up table
    if (x <= LookUp_in[0]) out = LookUp_out[0];
    else  if (x >= LookUp_in[LookUp_length - 1]) out = LookUp_out[LookUp_length - 1];
    else {
        unsigned int i = 0;
        while (x >= LookUp_in[i + 1]) i++;
        out = (float)LookUp_out[i] + ((float)LookUp_out[i + 1] - (float)LookUp_out[i]) * ((float)x - (float)LookUp_in[i]) / ((float)LookUp_in[i + 1] - (float)LookUp_in[i]);
    }

    return out;
    /**/

    // faster linear interpolation
    /*
    float index = x * (LookUp_length - 1);

    unsigned int indexLower = static_cast<unsigned int>(index);
    unsigned int indexUpper = indexLower + 1;

    //if (indexUpper >= 1000) {
    //    indexUpper = 999; // Ensure we don't go out of bounds
    //}
    if (x <= 0) return LookUp_out[0];
    else  if (x >= 1) return LookUp_out[LookUp_length - 1];
    else {
        float weightUpper = index - indexLower;
        float weightLower = 1.0f - weightUpper;

        return (weightLower * LookUp_out[indexLower]) + (weightUpper * LookUp_out[indexUpper]);
        //return (LookUp_out[indexLower]);
    }
    /**/

    // fast without linear interpolation
    /**/
    float index = x * (LookUp_length - 1);

    int indexLower = static_cast<unsigned int>(index);

    if (indexLower < 0) return LookUp_out[0];
    else if (indexLower >= LookUp_length) return LookUp_out[LookUp_length - 1];
    else return (1.0*LookUp_out[indexLower]);
    /**/
    

}




void dualband_colors(Mat& image) {

    if (debug_flag == 1) {
        cout << "Apply dual-band palette" << endl;
        logfile << "Apply dual-band palette" << endl;
    }

    // Dual-Band colors

    // old slow algorithm
    /*
    vector<Mat> bgr_planes_t1;
    vector<Mat> bgr_planes_t2;
    split(image, bgr_planes_t1);
    split(image, bgr_planes_t2);
    //bgr_planes_t[0] = bgr_planes_t[0] + bgr_planes_t[1];  //B
    //bgr_planes_t[1] = bgr_planes_t[2] * 2 + bgr_planes_t[0] * 0.2;  //G
    //bgr_planes_t[2] = bgr_planes_t[2] * 2.5;  //R
     
    //printf("%f %f %f \n", aR, bR, cR);
    //printf("%f %f %f \n", aG, bG, cG);
    //printf("%f %f %f \n", aB, bB, cB);
    bgr_planes_t2[2] = bgr_planes_t1[2] * aR + bgr_planes_t1[1] * bR + bgr_planes_t1[0] * cR;  //R
    bgr_planes_t2[1] = bgr_planes_t1[2] * aG + bgr_planes_t1[1] * bG + bgr_planes_t1[0] * cG;  //G
    bgr_planes_t2[0] = bgr_planes_t1[2] * aB + bgr_planes_t1[1] * bB + bgr_planes_t1[0] * cB;  //B

    merge(bgr_planes_t2, image);
    /**/

    // new fast algorithm
    /**/
    if (image.isContinuous()) // check, if gaps in memory
    //if (false)
    {
        // using point arithmetics
        int nrows = image.rows;
        int ncols = image.cols;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        float* p1 = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            float B = *p1;
            p1++;
            float G = *p1;
            p1++;
            float R = *p1;
            p1++;

            *p = R * aB + G * bB + B * cB;  //B
            p++;
            *p = R * aG + G * bG + B * cG;  //G
            p++;
            *p = R * aR + G * bR + B * cR;  //R
            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            float B = (*it)[0];
            float G = (*it)[1];
            float R = (*it)[2];

            (*it)[2] = R * aR + G * bR + B * cR;  //R
            (*it)[1] = R * aG + G * bG + B * cG;  //G
            (*it)[0] = R * aB + G * bB + B * cB;  //B
        }
    }
    /**/
}




void color_correction(Mat& image) {

    if (debug_flag == 1) {
        cout << "Apply color correction matrix" << endl;
        logfile << "Apply color correction matrix" << endl;
    }

    // new fast algorithm
    /**/
    if (image.isContinuous()) // check, if gaps in memory
        //if (false)
    {
        // using point arithmetics
        int nrows = image.rows;
        int ncols = image.cols;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        float* p1 = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            float B = *p1;
            p1++;
            float G = *p1;
            p1++;
            float R = *p1;
            p1++;

            *p = R * CC31 + G * CC32 + B * CC33;  //B
            p++;
            *p = R * CC21 + G * CC22 + B * CC23;  //G
            p++;
            *p = R * CC11 + G * CC12 + B * CC13;  //R
            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            float B = (*it)[0];
            float G = (*it)[1];
            float R = (*it)[2];

            (*it)[2] = R * CC11 + G * CC12 + B * CC13;  //R
            (*it)[1] = R * CC21 + G * CC22 + B * CC23;  //G
            (*it)[0] = R * CC31 + G * CC32 + B * CC33;  //B
        }
    }
    /**/
}






void WB_correction(Mat& image, float WBcorr_R, float WBcorr_G, float WBcorr_B) {

    if (debug_flag == 1) {
        cout << "Apply WB correction" << endl;
        logfile << "Apply WB correction" << endl;
    }

    // WB correction for RGB

    // old algorithm
    /*
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    
    bgr_planes[2] = bgr_planes[2] * WBcorr_R;  //R
    bgr_planes[1] = bgr_planes[1] * WBcorr_G;  //G
    bgr_planes[0] = bgr_planes[0] * WBcorr_B;  //B

    merge(bgr_planes, image);
    /**/

    // new fast algorithm
    /**/
    if (image.isContinuous()) // check, if gaps in memory
    //if (false)
    {
        // using point arithmetics
        int nrows = image.rows;
        int ncols = image.cols;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            *p = *p * WBcorr_B;  //B
            p++;
            *p = *p * WBcorr_G;  //G
            p++;
            *p = *p * WBcorr_R;  //R
            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            (*it)[2] = (*it)[2] * WBcorr_R;  //R
            (*it)[1] = (*it)[1] * WBcorr_G;  //G
            (*it)[0] = (*it)[0] * WBcorr_B;  //B
        }
    }
    /**/
}




void gamma_correction(Mat& image, float gamma) {
    // Gamma correction
    //image = gamma * image;



    
    if (debug_flag == 1) {
        cout << "Apply stretch" << endl;
        logfile << "Apply stretch" << endl;
    }

    // old algorithm
    /*
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            bgr_planes[0].at<float>(i, j) = LookUpTable(bgr_planes[0].at<float>(i, j), LUT_in, LUT_out, LUT_size);
            bgr_planes[1].at<float>(i, j) = LookUpTable(bgr_planes[1].at<float>(i, j), LUT_in, LUT_out, LUT_size);
            bgr_planes[2].at<float>(i, j) = LookUpTable(bgr_planes[2].at<float>(i, j), LUT_in, LUT_out, LUT_size);
        }
    merge(bgr_planes, image);
    /**/

    // fast algorithm
    /**/

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_out[0];
            else if (indexLower >= LUT_size) *p = LUT_out[LUT_size - 1];
                 else *p = (1.0 * LUT_out[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_out, LUT_size);
            (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_out, LUT_size);
            (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_out, LUT_size);
        }
    }
    /**/
}



void gamma_dark_correction(Mat& image, float gamma) {
    // Gamma correction
    //image = 0.1 * image;



    /**/
    if (debug_flag == 1) {
        cout << "Apply dark stretch" << endl;
        logfile << "Apply dark stretch" << endl;
    }

    if (image.isContinuous()) // check, if gaps in memory
    {
        // using point arithmetics
        int channels = image.channels();
        int nrows = image.rows;
        int ncols = image.cols * channels;

        //cout << "continuous" << endl;
        float* p = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            //*p++ = LookUpTable(*p, LUT_in, LUT_out, LUT_size);

            float index = *p * (LUT_size - 1);

            int indexLower = static_cast<unsigned int>(index);

            if (indexLower < 0) *p = LUT_dark_out[0];
            else if (indexLower >= LUT_size) *p = LUT_dark_out[LUT_size - 1];
            else *p = (1.0 * LUT_dark_out[indexLower]);

            p++;
        }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            (*it)[0] = LookUpTable((*it)[0], LUT_in, LUT_dark_out, LUT_size);
            (*it)[1] = LookUpTable((*it)[1], LUT_in, LUT_dark_out, LUT_size);
            (*it)[2] = LookUpTable((*it)[2], LUT_in, LUT_dark_out, LUT_size);
        }
    }
    /**/
}





void black_level(Mat& image, float b_level) {
    
    if (debug_flag == 1) {
        cout << "Apply black level correction" << endl;
        logfile << "Apply black level correction" << endl;
    }
    
    // Black level correction, histogram threshold from left - b_level
    
    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 300.0 / image.rows;
    resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    //Scalar m = mean(image2);
    if (circular_mask_background_flag == 1)  {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC3, Scalar(2, 2, 2));
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    vector<Mat> bgr_planes;
    split(image2, bgr_planes);

    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());
    
    int r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (r_offs == 0)
            if (r_hist.at<float>(i) > b_level)
                r_offs = i;
        if (g_offs == 0)
            if (g_hist.at<float>(i) > b_level)
                g_offs = i;
        if (b_offs == 0)
            if (b_hist.at<float>(i) > b_level)
                b_offs = i;
    }

    float b = b_offs / (float)histSize;
    float g = g_offs / (float)histSize;
    float r = r_offs / (float)histSize;

    //printf("black level: %f %f %f\n", b, g, r);
    //printf("black level: %d %d %d\n", b_offs, g_offs, r_offs);

    split(image, bgr_planes);

    bgr_planes[0] = bgr_planes[0] - b;
    bgr_planes[0] = bgr_planes[0] * (1/(1-b));
    bgr_planes[1] = bgr_planes[1] - g;
    bgr_planes[1] = bgr_planes[1] * (1 / (1 - g));
    bgr_planes[2] = bgr_planes[2] - r;
    bgr_planes[2] = bgr_planes[2] * (1 / (1 - r));

    merge(bgr_planes, image);
}



void black_level_calculate(Mat& image, float b_level, float& b_offs, float& g_offs, float& r_offs) {

    if (debug_flag == 1) {
        cout << "Calculate black level" << endl;
        logfile << "Calculate black level" << endl;
    }

    // Black level calculation, histogram threshold from left - b_level

    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    int histSize = 10000;
    float range[] = { 0.000001, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    //int hist_w = 1000, hist_h = 400;
    //int bin_w = cvRound((double)hist_w / histSize);
    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, 1, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, 1, NORM_MINMAX, -1, Mat());

    r_offs = 0, g_offs = 0, b_offs = 0;

    for (int i = 1; i < histSize; i++) {
        if (r_offs == 0)
            if (r_hist.at<float>(i) > b_level)
                r_offs = i;
        if (g_offs == 0)
            if (g_hist.at<float>(i) > b_level)
                g_offs = i;
        if (b_offs == 0)
            if (b_hist.at<float>(i) > b_level)
                b_offs = i;
    }

    b_offs = b_offs / (float)histSize;
    g_offs = g_offs / (float)histSize;
    r_offs = r_offs / (float)histSize;

    
    //printf("black level: %f %f %f\n", b_offs, g_offs, r_offs);

    
}


void plane_equation(float x1, float y1, float z1,
                    float x2, float y2, float z2,
                    float x3, float y3, float z3,
                    float& a, float& b, float& c, float& d) {

    float a1 = x2 - x1;
    float b1 = y2 - y1;
    float c1 = z2 - z1;
    float a2 = x3 - x1;
    float b2 = y3 - y1;
    float c2 = z3 - z1;
    a = b1 * c2 - b2 * c1;
    b = a2 * c1 - a1 * c2;
    c = a1 * b2 - b1 * a2;
    d = (-a * x1 - b * y1 - c * z1);
    //printf("equation of plane is %.2f x + %.2f"
    //    " y + %.2f z + %.2f = 0.", a, b, c, d);
}


void black_level_gradient(Mat& image, float b_level) {

    if (debug_flag == 1) {
        cout << "Apply black level gradient correction" << endl;
        logfile << "Apply black level gradient correction" << endl;
    }

    // Black level correction with gradient, based on 4 corner pictures, histogram threshold from left - b_level

    float b_offs1, g_offs1, r_offs1; //offsets for left upper corner
    float b_offs2, g_offs2, r_offs2; //offsets for right upper corner
    float b_offs3, g_offs3, r_offs3; //offsets for right down corner
    float b_offs4, g_offs4, r_offs4; //offsets for left down corner

    Mat image2;
    //image.copyTo(image2);
    double resize_factor = 500.0 / image.rows;    //300.0 / image.rows;
    resize(image, image2, Size(0, 0), resize_factor, resize_factor, INTER_AREA);

    int sz = 3; //7;
    blur(image2, image2, Size(sz, sz));

    // apply circular mask for background compensation
    if (circular_mask_background_flag == 1) {
        //Prepare circular mask for background
        Mat circular_mask_b(image2.rows, image2.cols, CV_32FC3, Scalar(2, 2, 2));
        circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);

        image2 = max(image2, circular_mask_b);
    }
    //image2.copyTo(image);  // test

    int half_height = round(image2.rows / 10);  // dimensions of 1/5 tile
    int half_width = round(image2.cols / 10);

    //int row1 = round(image.rows / 10);     int col1 = round(image.cols / 10);   // points for plane equation, centers of corner 1/5 tiles
    //int row2 = round(image.rows / 10);     int col2 = round(image.cols / 10 * 9);
    //int row3 = round(image.rows / 10 * 9); int col3 = round(image.cols / 10 * 9);
    //int row4 = round(image.rows / 10 * 9); int col4 = round(image.cols / 10);
    int row1 = half_height;                     int col1 = half_width;
    int row2 = half_height;                     int col2 = image2.cols - 1 - half_width;
    int row3 = image2.rows - 1 - half_height;   int col3 = image2.cols - 1 - half_width;
    int row4 = image2.rows - 1 - half_height;   int col4 = half_width;
    //cout << row1 << " " << col1 << endl;

    //Mat crop1 = image2(Range(0, round(image2.rows / 5)), Range(0, round(image2.cols / 5)));   // left upper corner, 1/5 tile
    //Mat crop2 = image2(Range(0, round(image2.rows / 5)), Range(round(image2.cols / 5 * 4), image2.cols - 1));   // right upper corner, 1/5 tile
    //Mat crop3 = image2(Range(round(image2.rows / 5 * 4), image2.rows - 1), Range(round(image2.cols / 5 * 4), image2.cols - 1));   // right down corner, 1/5 tile
    //Mat crop4 = image2(Range(round(image2.rows / 5 * 4), image2.rows - 1), Range(0, round(image2.cols / 5)));   // left down corner, 1/5 tile

    if (circular_mask_background_flag == 1) {
        //Mat crop1 = image2(Range(0, round(image2.rows / 2)), Range(0, round(image2.cols / 2)));   // left upper corner, 1/2 tile
        //Mat crop2 = image2(Range(0, round(image2.rows / 2)), Range(round(image2.cols / 2), image2.cols - 1));   // right upper corner, 1/2 tile
        //Mat crop3 = image2(Range(round(image2.rows / 2), image2.rows - 1), Range(round(image2.cols / 2), image2.cols - 1));   // right down corner, 1/2 tile
        //Mat crop4 = image2(Range(round(image2.rows / 2), image2.rows - 1), Range(0, round(image2.cols / 2)));   // left down corner, 1/2 tile

        row1 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));   // points for plane equation, boundary of circular mask
        col1 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        row2 = round(image2.rows / 2 - (circular_mask_background_size * image2.rows * 0.35));
        col2 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row3 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col3 = round(image2.cols / 2 + (circular_mask_background_size * image2.rows * 0.35));

        row4 = round(image2.rows / 2 + (circular_mask_background_size * image2.rows * 0.35));
        col4 = round(image2.cols / 2 - (circular_mask_background_size * image2.rows * 0.35));

        if ((row1 - half_height) < 0) row1 = half_height;
        if ((col1 - half_width) < 0) col1 = half_width;
        if ((row2 - half_height) < 0) row2 = half_height;
        if ((col2 + half_width) > (image2.cols - 1)) col2 = image2.cols - 1 - half_width;
        if ((row3 + half_height) > (image2.rows - 1)) row3 = image2.rows - 1 - half_height;
        if ((col3 + half_width) > (image2.cols - 1)) col3 = image2.cols - 1 - half_width;
        if ((row4 + half_height) > (image2.rows - 1)) row4 = image2.rows - 1 - half_height;
        if ((col4 - half_width) < 0) col4 = half_width;
    }

    Mat crop1 = image2(Range(row1 - half_height, row1 + half_height), Range(col1 - half_width, col1 + half_width));   // left upper corner, center on circle, 1/5 tile
    Mat crop2 = image2(Range(row2 - half_height, row2 + half_height), Range(col2 - half_width, col2 + half_width));   // right upper corner, center on circle, 1/5 tile
    Mat crop3 = image2(Range(row3 - half_height, row3 + half_height), Range(col3 - half_width, col3 + half_width));   // right down corner, center on circle, 1/5 tile
    Mat crop4 = image2(Range(row4 - half_height, row4 + half_height), Range(col4 - half_width, col4 + half_width));   // left down corner, center on circle, 1/5 tile

    //imshow("corner", crop3 * 5);


    black_level_calculate(crop1, b_level, b_offs1, g_offs1, r_offs1);
    black_level_calculate(crop2, b_level, b_offs2, g_offs2, r_offs2);
    black_level_calculate(crop3, b_level, b_offs3, g_offs3, r_offs3);
    black_level_calculate(crop4, b_level, b_offs4, g_offs4, r_offs4);
    
    row1 = round((double)row1 / resize_factor);
    row2 = round((double)row2 / resize_factor);
    row3 = round((double)row3 / resize_factor);
    row4 = round((double)row4 / resize_factor);
    col1 = round((double)col1 / resize_factor);
    col2 = round((double)col2 / resize_factor);
    col3 = round((double)col3 / resize_factor);
    col4 = round((double)col4 / resize_factor);

    // recalculate points for plane equation, if circular mask for background
    /*
    if (circular_mask_background_flag == 1) {
        row1 = round(image.rows / 2 - (circular_mask_background_size * image.rows * 0.35));
        col1 = round(image.cols / 2 - (circular_mask_background_size * image.rows * 0.35));   // points for plane equation, boundary of circular mask

        row2 = round(image.rows / 2 - (circular_mask_background_size * image.rows * 0.35));
        col2 = round(image.cols / 2 + (circular_mask_background_size * image.rows * 0.35));

        row3 = round(image.rows / 2 + (circular_mask_background_size * image.rows * 0.35));
        col3 = round(image.cols / 2 + (circular_mask_background_size * image.rows * 0.35));

        row4 = round(image.rows / 2 + (circular_mask_background_size * image.rows * 0.35));
        col4 = round(image.cols / 2 - (circular_mask_background_size * image.rows * 0.35));
    }
    /**/
    //cout << row1 << " " << col1 << endl;

    float a1_b, b1_b, c1_b, d1_b;
    float a1_g, b1_g, c1_g, d1_g;
    float a1_r, b1_r, c1_r, d1_r;

    float a2_b, b2_b, c2_b, d2_b;
    float a2_g, b2_g, c2_g, d2_g;
    float a2_r, b2_r, c2_r, d2_r;
    

    plane_equation(row1, col1, b_offs1,
                   row3, col3, b_offs3,
                   row4, col4, b_offs4,
                   a1_b, b1_b, c1_b, d1_b);

    plane_equation(row1, col1, g_offs1,
                   row3, col3, g_offs3,
                   row4, col4, g_offs4,
                   a1_g, b1_g, c1_g, d1_g);

    plane_equation(row1, col1, r_offs1,
                   row3, col3, r_offs3,
                   row4, col4, r_offs4,
                   a1_r, b1_r, c1_r, d1_r);




    plane_equation(row2, col2, b_offs2,
                   row3, col3, b_offs3,
                   row4, col4, b_offs4,
                   a2_b, b2_b, c2_b, d2_b);

    plane_equation(row2, col2, g_offs2,
                   row3, col3, g_offs3,
                   row4, col4, g_offs4,
                   a2_g, b2_g, c2_g, d2_g);

    plane_equation(row2, col2, r_offs2,
                   row3, col3, r_offs3,
                   row4, col4, r_offs4,
                   a2_r, b2_r, c2_r, d2_r);



    
    //printf("equation of plane is %.2f x + %.2f"
    //    " y + %.2f z + %.2f = 0.", a1, b1, c1, d1);
    
    //float a_b = a1_b, b_b = b1_b, c_b = c1_b, d_b = d1_b;
    //float a_g = a1_g, b_g = b1_g, c_g = c1_g, d_g = d1_g;
    //float a_r = a1_r, b_r = b1_r, c_r = c1_r, d_r = d1_r;

    float a_b = (a1_b + a2_b) / 2, b_b = (b1_b + b2_b) / 2, c_b = (c1_b + c2_b) / 2, d_b = (d1_b + d2_b) / 2;
    float a_g = (a1_g + a2_g) / 2, b_g = (b1_g + b2_g) / 2, c_g = (c1_g + c2_g) / 2, d_g = (d1_g + d2_g) / 2;
    float a_r = (a1_r + a2_r) / 2, b_r = (b1_r + b2_r) / 2, c_r = (c1_r + c2_r) / 2, d_r = (d1_r + d2_r) / 2;

    //Mat plane_32fc3(image.rows, image.cols, CV_32FC3);
    //vector<Mat> bgr_planes;
    //split(plane_32fc3, bgr_planes);

    
    //Mat b_plane(image.rows, image.cols, CV_32FC1);
    //Mat g_plane(image.rows, image.cols, CV_32FC1);
    //Mat r_plane(image.rows, image.cols, CV_32FC1);
    Mat planes(image.rows, image.cols, CV_32FC3);

    /*
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            //bgr_planes[0].at<float>(i, j) = (-a_b * i - b_b * j - d_b) / c_b;
            //bgr_planes[1].at<float>(i, j) = (-a_g * i - b_g * j - d_g) / c_g;
            //bgr_planes[2].at<float>(i, j) = (-a_r * i - b_r * j - d_r) / c_r;

            b_plane.at<float>(i, j) = (-a_b * i - b_b * j - d_b) / c_b;
            g_plane.at<float>(i, j) = (-a_g * i - b_g * j - d_g) / c_g;
            r_plane.at<float>(i, j) = (-a_r * i - b_r * j - d_r) / c_r;
        }
    /**/

    if ( planes.isContinuous()) // check, if gaps in memory
    //if (false)
    {
        // using point arithmetics
        
        float* p = (float*)planes.data;

        for (int i = 0; i < planes.rows; i++)
            for (int j = 0; j < planes.cols; j++) {
                *p = (-a_b * i - b_b * j - d_b) / c_b;  //B
                p++;
                *p = (-a_g * i - b_g * j - d_g) / c_g;  //G
                p++;
                *p = (-a_r * i - b_r * j - d_r) / c_r;  //R
                p++;
            }
    }
    else {
        // using iterators - safe, if gaps in memory
        MatIterator_<Vec3f> it, end;
        int i = 0, j = 0;
        for (it = planes.begin<Vec3f>(), end = planes.end<Vec3f>(); it != end; ++it)
        {
            (*it)[2] = (-a_r * i - b_r * j - d_r) / c_r;  //R
            (*it)[1] = (-a_g * i - b_g * j - d_g) / c_g;  //G
            (*it)[0] = (-a_b * i - b_b * j - d_b) / c_b;  //B
            j++;
            if (j == planes.cols) {
                j = 0;
                i++;
            }

        }
    }


    //imshow("plane", bgr_planes[2]*5);

    /*
    vector<Mat> image_bgr_planes;
    split(image, image_bgr_planes);

    //image_bgr_planes[0] = image_bgr_planes[0] - bgr_planes[0];
    //image_bgr_planes[1] = image_bgr_planes[1] - bgr_planes[1];
    //image_bgr_planes[2] = image_bgr_planes[2] - bgr_planes[2];

    image_bgr_planes[0] = image_bgr_planes[0] - b_plane;
    image_bgr_planes[1] = image_bgr_planes[1] - g_plane;
    image_bgr_planes[2] = image_bgr_planes[2] - r_plane;

    merge(image_bgr_planes, image);
    /**/



    //vector<Mat> bgr_planes;

    //bgr_planes.push_back(b_plane);
    //bgr_planes.push_back(g_plane);
    //bgr_planes.push_back(r_plane);

    //Mat planes;
    //merge(bgr_planes, planes);

    image = image - planes;
}



void show_histogram(Mat& image) {
    vector<Mat> bgr_planes;
    split(image, bgr_planes);
    int histSize = 1000;
    float range[] = { 0, 1 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
    int hist_w = 1000, hist_h = 400;
    int bin_w = cvRound((double)hist_w / histSize);
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
            Scalar(255, 0, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
            Scalar(0, 255, 0), 2, 8, 0);
        line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
            Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
            Scalar(0, 0, 255), 2, 8, 0);
    }
    imshow("Histogram", histImage);
}



void show_RAW_histogram(Mat& image) {
    
    int histSize = 1000;
    float range[] = { 0, 65536 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat hist;
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, histRange, uniform, accumulate);

    int hist_w = display_height, hist_h = display_height / 2;
    //int hist_w = 1000, hist_h = 300;
    //int bin_w = cvRound((double)hist_w / histSize);
    float bin_w = (float)hist_w / histSize;
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        line(histImage, Point(cvRound(bin_w * (i - 1)), hist_h - cvRound(hist.at<float>(i - 1))),
            Point(cvRound(bin_w * (i)), hist_h - cvRound(hist.at<float>(i))),
            Scalar(0, 255, 0), 2, 8, 0);
    }
    imshow("RAW Histogram", histImage);
}


void plot_RAW_histogram(Mat& display_image, Mat& RAW_image) {

    int histSize = 1000;
    float range[] = { 0, 65536 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    Mat hist;
    calcHist(&RAW_image, 1, 0, Mat(), hist, 1, &histSize, histRange, uniform, accumulate);

    int hist_w = display_image.cols, hist_h = display_image.rows / 2;;
    float bin_w = (float)hist_w / histSize;

    //Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

    normalize(hist, hist, 0, hist_h, NORM_MINMAX, -1, Mat());
    for (int i = 1; i < histSize; i++)
    {
        //line(histImage, Point(cvRound(bin_w * (i - 1)), hist_h - cvRound(hist.at<float>(i - 1))),
        //    Point(cvRound(bin_w * (i)), hist_h - cvRound(hist.at<float>(i))),
        //    Scalar(0, 255, 0), 2, 8, 0);
        line(display_image, Point(cvRound(bin_w * (i - 1)), display_image.rows - cvRound(hist.at<float>(i - 1))),
            Point(cvRound(bin_w * (i)), display_image.rows - cvRound(hist.at<float>(i))),
            Scalar(200, 200, 200), 2, 8, 0);
    }

    rectangle(display_image, Rect(0, display_image.rows - hist_h, hist_w, hist_h), Scalar(200, 200, 200), 2);
}




void rotate_image(Mat& image, int image_rotation, int image_flip) {

    if (debug_flag == 1) {
        cout << "Rotating image" << endl;
        logfile << "Rotating image" << endl;
    }

    if (image_flip == 1)
        flip(image, image, 1);

    if (image_rotation == 1)
        rotate(image, image, ROTATE_90_CLOCKWISE);
    if (image_rotation == 2)
        rotate(image, image, ROTATE_180);
    if (image_rotation == 3)
        rotate(image, image, ROTATE_90_COUNTERCLOCKWISE);
}



void read_fits_file(char* filename, unsigned char* image_buffer) {
    fitsfile* fptr;  // FITS file pointer
    int status = 0;  // CFITSIO status value MUST be initialized to zero!
    int bitpix, naxis;
    long naxes[2] = { 1,1 }, fpixel[2] = { 1,1 };
    char err_text[80];

    // Open the FITS file
    fits_open_file(&fptr, filename, READONLY, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    // Read the image parameters
    fits_get_img_param(fptr, 2, &bitpix, &naxis, naxes, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    // Check if it's a 2D image
    if (naxis != 2) {
        cout << "Error, reading " << filename << ": not a 2D image" << endl;
        logfile << "Error, reading " << filename << ": not a 2D image" << endl;
        abort_app();
    }

    // Check image data type
    if (bitpix != 16) {
        cout << "Error, reading " << filename << ": wrong data type" << endl;
        logfile << "Error, reading " << filename << ": wrong data type" << endl;
        abort_app();
    }

    // Check image dimensions
    if ((naxes[0] != camera_image_width) || (naxes[1] != camera_image_height)) {
        cout << "Error, reading " << filename << ": wrong image dimensions" << endl;
        cout << filename << " dimensions: " << naxes[0] << "x" << naxes[1] << endl;
        cout << "Camera image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
        logfile << "Error, reading " << filename << ": wrong image dimensions" << endl;
        logfile << filename << " dimensions: " << naxes[0] << "x" << naxes[1] << endl;
        logfile << "Camera image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
        abort_app();
    }

    // Read the image into the buffer
    long imageSize = naxes[0] * naxes[1];
    fits_read_pix(fptr, TUSHORT, fpixel, imageSize, NULL, image_buffer, NULL, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }

    fits_close_file(fptr, &status);
    if (status) {
        fits_get_errstatus(status, err_text); // get mnemonic of error message
        cout << "Fits lib error, reading " << filename << ": " << err_text << endl;
        logfile << "Fits lib error, reading " << filename << ": " << err_text << endl;
        abort_app();
    }
}



void read_darks(Mat& dark_v_32sc1, double& dark_v_mean, Mat& dark_f_32sc1, double& dark_f_mean)
{
    double num_pixel;
    //ifstream myfile;

    if ((dark_v_hotpixel_flag == 1) || (dark_v_subtract_flag == 1)) {

        //char filename[] = "dark_v.fits";
        dark_v_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

        //cout << "Reading dark_v.fits..." << endl;
        //logfile << "Reading dark_v.fits..." << endl;
        cout << "Reading " << dark_v_filename << "..." << endl;
        logfile << "Reading " << dark_v_filename << "..." << endl;
        //read_fits_file(filename, dark_v_image); dark_v_filename
        read_fits_file(dark_v_filename, dark_v_image);


        // ----------- simple read fits file
        /*
        myfile.open("dark_v.fits", ios::in | ios::binary);

        if (myfile.is_open()) {
            printf("Reading dark_v.fits...\n");
            myfile.seekg(2880, ios::beg);
            myfile.read((char*)dark_v_image, image_size);
            myfile.close();

            int16_t* p = (int16_t*)dark_v_image;
            uint16_t* p2 = (uint16_t*)dark_v_image;

            for (long i = 0; i < (image_size / 2); i++) {
                unsigned char t = dark_v_image[i * 2];
                dark_v_image[i * 2] = dark_v_image[i * 2 + 1];
                dark_v_image[i * 2 + 1] = t;
                p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
            }
        }
        else {
            printf("Couldn't find file dark_v.fits\n");
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        /**/
        // ----------- 




        // Copy the data into an OpenCV Mat structure
        //Mat dark_v_16uc1(asi_camera_info[cam]->MaxWidth / bin / monobin_k, asi_camera_info[cam]->MaxHeight / bin / monobin_k, CV_16UC1, dark_v_image);
        Mat dark_v_16uc1(camera_image_height, camera_image_width, CV_16UC1, dark_v_image);

        // Convert to int32
        dark_v_16uc1.convertTo(dark_v_32sc1, CV_32SC1);

        //printf("size: %d, %d\n", dark_v_32sc1.rows, dark_v_32sc1.cols);
        //imshow("dark_v", dark_v_32sc1);
        //printf("pixel: %d, %d\n", dark_v_16uc1.at<uint16_t>(500, 500), dark_v_32sc1.at<int32_t>(500, 500));

        //-----------Calculate dark mean and standard deviation value
        Scalar mean, stddev;
        meanStdDev(dark_v_32sc1, mean, stddev);
        dark_v_mean = mean[0];
        hotpixel_threshold = dark_v_mean + stddev[0] * 7;

        if (debug_flag == 1) {
            cout << "Dark v mean value: " << dark_v_mean << endl;
            cout << "Dark v stdev value: " << stddev[0] << endl;
            cout << "Dark v hotpixel threshold value: " << hotpixel_threshold << endl;
            logfile << "Dark v mean value: " << dark_v_mean << endl;
            logfile << "Dark v stdev value: " << stddev[0] << endl;
            logfile << "Dark v hotpixel threshold value: " << hotpixel_threshold << endl;
        }

        //-----------Search and count hot pixels
        num_hotpixel_v = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                if (dark_v_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    //printf("Hot pixel at: %d, %d\n", i, j);
                    //printf("pixel: %d, %d\n", dark_v_16uc1.at<uint16_t>(i, j), dark_v_32sc1.at<int32_t>(i, j));
                    num_hotpixel_v++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels found in video dark frame: " << num_hotpixel_v << endl;
            logfile << "Hot pixels found in video dark frame: " << num_hotpixel_v << endl;
        }

        //-----------Search and list hot pixels
        hotpixel_list_v = (int*)malloc(sizeof(int) * num_hotpixel_v * 10);
        num_hotpixel_v = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                if (dark_v_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    hotpixel_list_v[num_hotpixel_v * 10 + 0] = i; //save coordinates of hotpixel
                    hotpixel_list_v[num_hotpixel_v * 10 + 1] = j;

                    //printf("Hot pixels num, at: %d, %d, %d\n", num_hotpixel_v, hotpixel_list_v[num_hotpixel_v *10 + 0], hotpixel_list_v[num_hotpixel_v *10 + 1]);

                    /*
                    if ((i > 1) && (i < (dark_v_32sc1.rows - 2)) &&
                        (j > 1) && (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>( (i-2), (j-2) ) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 0;
                    //-----
                    /**/
                    //-----
                    if ((i > 1) &&
                        (j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 2] = 0;
                    //-----
                    if ((i > 1) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 3] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 3] = 0;
                    //-----
                    if ((i > 1) &&
                        (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i - 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 4] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 4] = 0;
                    //-----
                    if ((j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 5] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 5] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (j < (dark_v_32sc1.cols - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 6] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 6] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 7] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 7] = 0;
                    //-----
                    if ((i < (dark_v_32sc1.rows - 2)) &&
                        (j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i + 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 8] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 8] = 0;
                    //-----
                    if ((j > 1) &&
                        (dark_v_32sc1.at<int32_t>((i), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_v[num_hotpixel_v * 10 + 9] = 1;
                    }
                    else
                        hotpixel_list_v[num_hotpixel_v * 10 + 9] = 0;
                    //-----
                    /**/
                    num_hotpixel_v++;
                }

            }

        /*
        //int d = 458;
        for (int d = 0; d < 200; d++) //num_hotpixel_v; d++)
            printf("Hot pixels coord: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", hotpixel_list_v[d*10 + 0], hotpixel_list_v[d*10 + 1],
                hotpixel_list_v[d*10 + 2], hotpixel_list_v[d*10 + 3], hotpixel_list_v[d*10 + 4], hotpixel_list_v[d*10 + 5],
                hotpixel_list_v[d*10 + 6], hotpixel_list_v[d*10 + 7], hotpixel_list_v[d*10 + 8], hotpixel_list_v[d*10 + 9]);
        /**/

        if (debug_flag == 1) {
            cout << "Hot pixels in video dark listed" << endl;
            logfile << "Hot pixels in video dark listed" << endl;
        }

        /*
        //-----------Calculate dark mean value
        num_pixel = 0;
        dark_v_mean = 0;

        for (int i = 0; i < dark_v_32sc1.rows; i++)
            for (int j = 0; j < dark_v_32sc1.cols; j++) {
                if (dark_v_32sc1.at<int32_t>(i, j) < hotpixel_threshold) {
                    dark_v_mean += dark_v_32sc1.at<int32_t>(i, j);
                    num_pixel += 1;
                }

            }
        dark_v_mean = dark_v_mean / num_pixel;
        printf("Dark v mean value: %f\n", dark_v_mean);
        /**/
    }






    if ((dark_f_hotpixel_flag == 1) || (dark_f_subtract_flag == 1)) {

        //char filename[] = "dark_f.fits";
        dark_f_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

        //cout << "Reading dark_f.fits..." << endl;
        //logfile << "Reading dark_f.fits..." << endl;
        cout << "Reading " << dark_f_filename << "..." << endl;
        logfile << "Reading " << dark_f_filename << "..." << endl;
        //read_fits_file(filename, dark_f_image);
        read_fits_file(dark_f_filename, dark_f_image);

        
        
        /*
        myfile.open("dark_f.fits", ios::in | ios::binary);

        if (myfile.is_open()) {
            printf("Reading dark_f.fits...\n");
            myfile.seekg(2880, ios::beg);
            myfile.read((char*)dark_f_image, image_size);
            myfile.close();

            int16_t* p = (int16_t*)dark_f_image;
            uint16_t* p2 = (uint16_t*)dark_f_image;

            for (long i = 0; i < (image_size / 2); i++) {
                unsigned char t = dark_f_image[i * 2];
                dark_f_image[i * 2] = dark_f_image[i * 2 + 1];
                dark_f_image[i * 2 + 1] = t;
                p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
            }
        }
        else {
            printf("Couldn't find file dark_f.fits\n");
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        /**/

        
        // Copy the data into an OpenCV Mat structure
        Mat dark_f_16uc1(camera_image_height, camera_image_width, CV_16UC1, dark_f_image);

        // Convert to int32
        dark_f_16uc1.convertTo(dark_f_32sc1, CV_32SC1);


        //-----------Calculate dark mean and standard deviation value
        Scalar mean, stddev;
        meanStdDev(dark_f_32sc1, mean, stddev);
        dark_f_mean = mean[0];
        hotpixel_threshold = dark_f_mean + stddev[0] * 7;

        if (debug_flag == 1) {
            cout << "Dark f mean value: " << dark_f_mean << endl;
            cout << "Dark f stdev value: " << stddev[0] << endl;
            cout << "Dark f hotpixel threshold value: " << hotpixel_threshold << endl;
            logfile << "Dark f mean value: " << dark_f_mean << endl;
            logfile << "Dark f stdev value: " << stddev[0] << endl;
            logfile << "Dark f hotpixel threshold value: " << hotpixel_threshold << endl;
        }


        //-----------Search and count hot pixels
        num_hotpixel_f = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                if (dark_f_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    num_hotpixel_f++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels found in foto dark frame: " << num_hotpixel_f << endl;
            logfile << "Hot pixels found in foto dark frame: " << num_hotpixel_f << endl;
        }


        //-----------Search and list hot pixels
        hotpixel_list_f = (int*)malloc(sizeof(int) * num_hotpixel_f * 10);
        num_hotpixel_f = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                if (dark_f_32sc1.at<int32_t>(i, j) > hotpixel_threshold) {
                    hotpixel_list_f[num_hotpixel_f * 10 + 0] = i; //save coordinates of hotpixel
                    hotpixel_list_f[num_hotpixel_f * 10 + 1] = j;


                    //-----
                    if ((i > 1) &&
                        (j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 2] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 2] = 0;
                    //-----
                    if ((i > 1) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 3] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 3] = 0;
                    //-----
                    if ((i > 1) &&
                        (j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i - 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 4] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 4] = 0;
                    //-----
                    if ((j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 5] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 5] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (j < (dark_f_32sc1.cols - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j + 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 6] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 6] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 7] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 7] = 0;
                    //-----
                    if ((i < (dark_f_32sc1.rows - 2)) &&
                        (j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i + 2), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 8] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 8] = 0;
                    //-----
                    if ((j > 1) &&
                        (dark_f_32sc1.at<int32_t>((i), (j - 2)) < hotpixel_threshold)) {   // valid neighbor pixel for interpolation
                        hotpixel_list_f[num_hotpixel_f * 10 + 9] = 1;
                    }
                    else
                        hotpixel_list_f[num_hotpixel_f * 10 + 9] = 0;
                    //-----
                    /**/
                    num_hotpixel_f++;
                }

            }

        if (debug_flag == 1) {
            cout << "Hot pixels in foto dark listed" << endl;
            logfile << "Hot pixels in foto dark listed" << endl;
        }


        /*
        //-----------Calculate dark mean value
        num_pixel = 0;
        dark_f_mean = 0;

        for (int i = 0; i < dark_f_32sc1.rows; i++)
            for (int j = 0; j < dark_f_32sc1.cols; j++) {
                if (dark_f_32sc1.at<int32_t>(i, j) < hotpixel_threshold) {
                    dark_f_mean += dark_f_32sc1.at<int32_t>(i, j);
                    num_pixel += 1;
                }

            }
        dark_f_mean = dark_f_mean / num_pixel;
        printf("Dark f mean value: %f\n", dark_f_mean);
        /**/
    }
    

        
}




void correct_hotpixel(Mat& image, int* hotpixel_list, int num_hotpixel)
{

    if (debug_flag == 1) {
        cout << "Hotpixels correction" << endl;
        logfile << "Hotpixels correction" << endl;
    }

    //printf("list test: %d, %d, %d\n", hotpixel_list[0], hotpixel_list[1], hotpixel_list[2]);

    for (int i = 0; i < num_hotpixel; i++) {

        int n = 0;
        int32_t sum = 0;

        if (hotpixel_list[i * 10 + 2] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1] - 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 3] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1]);
            n++;
        }
        if (hotpixel_list[i * 10 + 4] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] - 2, hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 5] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 6] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1] + 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 7] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1]);
            n++;
        }
        if (hotpixel_list[i * 10 + 8] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0] + 2, hotpixel_list[i * 10 + 1] - 2);
            n++;
        }
        if (hotpixel_list[i * 10 + 9] == 1) {
            sum += image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1] - 2);
            n++;
        }


        if (n > 0)
            image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1]) = sum / n;  //correct hotpixel with interpolated value
        else
            image.at<int32_t>(hotpixel_list[i * 10 + 0], hotpixel_list[i * 10 + 1]) = 0;   // no neighbor pixel to interpolate from, set to black

    }

}





void apply_dark(Mat& image, Mat dark_image, double dark_mean)
{
    if (debug_flag == 1) {
        cout << "Apply dark" << endl;
        logfile << "Apply dark" << endl;
    }

    //printf("pixel before: %d\n", image.at<int32_t>(500, 500));
    //printf("pixel dark: %d\n", dark_image.at<int32_t>(500, 500));

    image = image - dark_image;
    //printf("pixel after dark: %d\n", image.at<int32_t>(500, 500));

    //cout << dark_mean << endl;
    //cout << (int32_t)dark_mean << endl;




    image = image + (int32_t)dark_mean; 
    



    //printf("pixel after dark mean: %d\n", image.at<int32_t>(500, 500));

    //printf("dark_mean in int32: %d\n", (int32_t)dark_mean);
    //printf("pixel: %d\n", image.at<int32_t>(500, 500));
    

}






void read_flat(Mat& flat_32fc3, Mat& flat_inv_32fc3) 
{
    

    //char filename[] = "flat.fits";
    flat_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);

    //cout << "Reading flat.fits..." << endl;
    //logfile << "Reading flat.fits..." << endl;
    cout << "Reading " << flat_filename << "..." << endl;
    logfile << "Reading " << flat_filename << "..." << endl;
    //read_fits_file(filename, flat_image);
    read_fits_file(flat_filename, flat_image);

    
    /*
    ifstream myfile;

    myfile.open("flat.fits", ios::in | ios::binary);

    if (myfile.is_open()) {
        printf("Reading flat.fits...\n");
        myfile.seekg(2880, ios::beg);
        myfile.read((char*)flat_image, image_size);
        myfile.close();
        //printf("flat.fits closed\n");

        int16_t* p = (int16_t*)flat_image;
        uint16_t* p2 = (uint16_t*)flat_image;

        for (long i = 0; i < (image_size / 2); i++) {
            unsigned char t = flat_image[i * 2];
            flat_image[i * 2] = flat_image[i * 2 + 1];
            flat_image[i * 2 + 1] = t;
            p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
        }

        //printf("Reading flat.fits done\n");
    }
    else {
        printf("Couldn't find file flat.fits\n");
        cout << "Press Enter to close...";
        cin.get();
        exit(1); // return 1;
    }
    /**/


    //--------------------Copy flat frame to Mat variables

    // Copy the data into an OpenCV Mat structure
    Mat flat_16uc1(camera_image_height, camera_image_width, CV_16UC1, flat_image);
    
    // Decode Bayer data to RGB
    //Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
    //cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
    
    // Decode Bayer data to RGB or mix monochrome to RGB
    Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
    if (asi_camera_info[cam]->IsColorCam == ASI_TRUE)
        cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
    else
        cvtColor(flat_16uc1, mat16uc3_rgb, cv::COLOR_GRAY2BGR);
    
    // Convert to float32
    Mat flat_32fc3_temp(camera_image_height, camera_image_width, CV_32FC3);
    mat16uc3_rgb.convertTo(flat_32fc3_temp, CV_32FC3, 1 / 65536.0);
    
    //---------- normalise rgb planes to 1
    vector<Mat> bgr_planes;
    split(flat_32fc3_temp, bgr_planes);
    double min, max;
    minMaxLoc(bgr_planes[0], &min, &max);
    //printf("flat blue max: %f\n", max);
    bgr_planes[0] = bgr_planes[0] / max;
    
    minMaxLoc(bgr_planes[1], &min, &max);
    //printf("flat green max: %f\n", max);
    bgr_planes[1] = bgr_planes[1] / max;

    minMaxLoc(bgr_planes[2], &min, &max);
    //printf("flat red max: %f\n", max);
    bgr_planes[2] = bgr_planes[2] / max;

    merge(bgr_planes, flat_32fc3_temp);

    
    //------------ inverse flat frame

    flat_32fc3_temp.copyTo(flat_inv_32fc3);

    divide(1, flat_inv_32fc3, flat_inv_32fc3);


    //non-inverse flat must be scaled to display image size
    //resize(flat_32fc3_temp, flat_32fc3, Size(0, 0), display_scale, display_scale, INTER_AREA);
    
    //non-inverse flat must not be scaled to display image size
    flat_32fc3_temp.copyTo(flat_32fc3);
}






void focusing_zoom(Mat& image, double zoom) {
    
    if (debug_flag == 1) {
        cout << "Apply focusing zoom" << endl;
        logfile << "Apply focusing zoom" << endl;
    }

    Mat temp;
    image.copyTo(temp);

    int crop_size = round ((image.cols - 10) / zoom / 3);

    Mat crop;
    crop = temp(Range(round(image.rows / 2 - crop_size / 2), round(image.rows / 2 + crop_size / 2)), Range(round(image.cols / 2 - crop_size / 2), round(image.cols / 2 + crop_size / 2)));

    Mat crop_zoom;
    resize(crop, crop_zoom, Size(0, 0), zoom, zoom, INTER_AREA);



    vector<Mat> image_bgr;
    split(image, image_bgr);

    vector<Mat> crop_bgr;
    split(crop_zoom, crop_bgr);

    Mat insetImage_1_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 - crop_bgr[0].cols / 2 * 3), round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));
    Mat insetImage_2_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 - crop_bgr[0].cols / 2),     round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));
    Mat insetImage_3_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 + crop_bgr[0].cols / 2),     round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));

    Mat insetImage_1_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 - crop_bgr[1].cols / 2 * 3), round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));
    Mat insetImage_2_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 - crop_bgr[1].cols / 2),     round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));
    Mat insetImage_3_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 + crop_bgr[1].cols / 2),     round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));
    
    Mat insetImage_1_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 - crop_bgr[2].cols / 2 * 3), round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));
    Mat insetImage_2_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 - crop_bgr[2].cols / 2),     round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));
    Mat insetImage_3_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 + crop_bgr[2].cols / 2),     round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));

    crop_bgr[2].copyTo(insetImage_1_r);
    crop_bgr[1].copyTo(insetImage_2_g);
    crop_bgr[0].copyTo(insetImage_3_b);

    crop_bgr[0] = crop_bgr[0] / 2;
    crop_bgr[1] = crop_bgr[1] / 2;
    crop_bgr[2] = crop_bgr[2] / 2;

    crop_bgr[2].copyTo(insetImage_1_g);
    crop_bgr[1].copyTo(insetImage_2_b);
    crop_bgr[0].copyTo(insetImage_3_r);

    crop_bgr[2].copyTo(insetImage_1_b);
    crop_bgr[1].copyTo(insetImage_2_r);
    crop_bgr[0].copyTo(insetImage_3_g);

    merge(image_bgr, image);

    /*
    Mat insetImage_1(image, Rect(round(image.cols / 2 - crop_zoom.cols / 2 * 3), round(image.rows / 2 - crop_zoom.rows / 2), crop_zoom.cols, crop_zoom.rows));
    Mat insetImage_2(image, Rect(round(image.cols / 2 - crop_zoom.cols / 2), round(image.rows / 2 - crop_zoom.rows / 2), crop_zoom.cols, crop_zoom.rows));
    Mat insetImage_3(image, Rect(round(image.cols / 2 + crop_zoom.cols / 2), round(image.rows / 2 - crop_zoom.rows / 2), crop_zoom.cols, crop_zoom.rows));

    crop_zoom.copyTo(insetImage_1);
    crop_zoom.copyTo(insetImage_2);
    crop_zoom.copyTo(insetImage_3);
    /**/
    

    
}


#if AI_NOISEREDUCTION
Mat NN_noise_reduction_tile(const fdeep::model& model, const Mat& image, const Mat& window2d) {

    // Split RGB parts of a tile
    //vector<Mat> image_bgr;
    //split(image, image_bgr);

    //Apply window
    //multiply(image_bgr[0], window2d, image_bgr[0]);
    //multiply(image_bgr[1], window2d, image_bgr[1]);
    //multiply(image_bgr[2], window2d, image_bgr[2]);

    //imshow("before", image_bgr[1]);

    /*
    for (int i = 0; i < 3; i++) {
        // convert cv::Mat to fdeep::tensor (image tile to tensor)--------------
        // Prepare the data vector
        std::vector<float> img_data;
        img_data.reserve(image.rows * image.cols);

        // Convert cv::Mat to a vector of floats
        img_data.assign(image_bgr[i].begin<float>(), image_bgr[i].end<float>());

        // Create the tensor
        const fdeep::tensor tensor = fdeep::tensor(
            fdeep::tensor_shape(image.rows, image.cols, 1),
            img_data
        );
        // -----------------------------------------

        // calculate NN response
        const auto result = model.predict({ tensor });

        // convert fdeep::tensor to cv::Mat (tensor to image tile)--------------
        // convert fdeep::tensor to float cv::Mat
        const cv::Mat image_out(cv::Size(result.front().shape().width_, result.front().shape().height_), CV_32F);
        const auto values = result.front().to_vector();
        std::memcpy(image_out.data, values.data(), values.size() * sizeof(float));
        // -----------------------------------------
        image_out.copyTo(image_bgr[i]);
    }
    //merge(image_bgr, image);
    /**/

    cvtColor(image, image, cv::COLOR_BGR2RGB);

    // convert cv::Mat to fdeep::tensor (image tile to tensor)--------------
        // Prepare the data vector
    std::vector<float> img_data;
    img_data.reserve(image.rows * image.cols * 3);
    img_data.resize(image.rows * image.cols * 3);

    // Copy cv::Mat to a vector of floats
    //img_data.assign(image.begin<Vec3f>()[0], image.end<Vec3f>()[2]);

    std::memcpy(img_data.data(), (float*)image.data, (image.rows * image.cols * 3) * sizeof(float));

    // Create the tensor
    const fdeep::tensor tensor = fdeep::tensor(
        fdeep::tensor_shape(image.rows, image.cols, 3),
        img_data
    );
    // -----------------------------------------

    // calculate NN response
    const auto result = model.predict({ tensor });

    // convert fdeep::tensor to cv::Mat (tensor to image tile)--------------
    // convert fdeep::tensor to float cv::Mat
    //const Mat image_out(Size(result.front().shape().width_, result.front().shape().height_), CV_32FC3);
    const auto values = result.front().to_vector();
    //std::memcpy(image_out.data, values.data(), values.size() * sizeof(float));
    std::memcpy(image.data, values.data(), values.size() * sizeof(float));
    // -----------------------------------------
    //image_out.copyTo(image);

    cvtColor(image, image, cv::COLOR_RGB2BGR);
    
    multiply(image, window2d, image);
   
    return image;
}

/*
void blendIntoMainImage(cv::Mat& mainImage, const cv::Mat& tile, const cv::Rect& tileRegion, int overlap) {
    for (int y = 0; y < tileRegion.height; ++y) {
        for (int x = 0; x < tileRegion.width; ++x) {
            // Calculate blend weights
            //float weightX = (x < overlap) ? float(x) / overlap : (x >= tileRegion.width - overlap) ? float(tileRegion.width - x) / overlap : 1.0f;
            //float weightY = (y < overlap) ? float(y) / overlap : (y >= tileRegion.height - overlap) ? float(tileRegion.height - y) / overlap : 1.0f;
            //float weight = weightX * weightY;

            //weight = 1;
            //cout << x << " " << y << " " << weight << endl;

            // Coordinates in the main image
            int mainX = tileRegion.x + x;
            int mainY = tileRegion.y + y;

            // Blend pixels
            cv::Vec3f pixelTile = tile.at<cv::Vec3f>(y, x);
            cv::Vec3f& pixelMain = mainImage.at<cv::Vec3f>(mainY, mainX);
            //pixelMain = pixelMain * (1 - weight) + pixelTile * weight;
            //pixelMain = pixelMain + pixelTile * weight;
            pixelMain = pixelMain + pixelTile;
        }
    }
}/**/


void NN_noise_reduction(const fdeep::model& model, Mat& image, double mix) {

    // prepeare window
    /**/
    int N = 64; // Set the desired size for the window
    Mat window1d(N, 1, CV_32F); // Create a 1D window matrix
    // Populate the 1D window using the Hanning window function
    /*
    for (int i = 0; i < N; i++) {
        float val = 0.5 * (1 - cos(2 * CV_PI * i / (N - 1)));
        //cout << val << endl;
        window1d.at<float>(i, 0) = val;
    }
    /**/
    // Populate 1D window, overlap 3
    /*
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N-2)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N-1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;
        
    }
    /**/
    // Populate 1D window, overlap 5
    /*
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0.1;
        else if (i == 2) window1d.at<float>(i, 0) = 0.5;
        else if (i == 3) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0.1;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }
    /**/
    // Populate 1D window, overlap 7
    /**/
    for (int i = 0; i < N; i++) {
        if (i == 0) window1d.at<float>(i, 0) = 0;
        else if (i == 1) window1d.at<float>(i, 0) = 0;
        else if (i == 2) window1d.at<float>(i, 0) = 0.1;
        else if (i == 3) window1d.at<float>(i, 0) = 0.5;
        else if (i == 4) window1d.at<float>(i, 0) = 0.9;
        else if (i == 5) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 6)) window1d.at<float>(i, 0) = 1.0;
        else if (i == (N - 5)) window1d.at<float>(i, 0) = 0.9;
        else if (i == (N - 4)) window1d.at<float>(i, 0) = 0.5;
        else if (i == (N - 3)) window1d.at<float>(i, 0) = 0.1;
        else if (i == (N - 2)) window1d.at<float>(i, 0) = 0;
        else if (i == (N - 1)) window1d.at<float>(i, 0) = 0;
        else window1d.at<float>(i, 0) = 1.0;

    }
    /**/
    // Create a 2D window by taking the outer product of the 1D window with itself
    Mat window2d = window1d * window1d.t();
    /*
    cout << window2d.at<float>(0, 0) << endl;
    cout << window2d.at<float>(1, 0) << endl;
    cout << window2d.at<float>(2, 0) << endl;
    cout << window2d.at<float>(0, 1) << endl;
    cout << window2d.at<float>(1, 1) << endl;
    cout << window2d.at<float>(2, 1) << endl;
    cout << window2d.at<float>(0, 2) << endl;
    cout << window2d.at<float>(1, 2) << endl;
    cout << window2d.at<float>(2, 2) << endl;
    /**/
    cvtColor(window2d, window2d, cv::COLOR_GRAY2BGR);
    /**/

    //Mat mat16uc3;
    //window2d.convertTo(mat16uc3, CV_16UC3, 65535);
    //imwrite("C:/Users/HOME/Desktop/stacks_test/window.tiff", mat16uc3);
    //imshow("window", window2d);

    /*
    Mat image1;
    image.copyTo(image1);
    Mat cropped_image;
    int y = 350;
    int x = 350;
    cropped_image = image1(Range(y, y+32), Range(x, x+32));

    //imshow("before", cropped_image);

    NN_noise_reduction_tile(model, cropped_image, window2d);

    //imshow("after", cropped_image);
    /**/


    /**/
    cout << "start NN filter..." << endl;
    if (debug_flag == 1) {
        //cout << "start NN filter..." << endl;
        logfile << "start NN filter..." << endl;
    }

    std::vector<std::thread> threads;
    int maxThreads = std::thread::hardware_concurrency(); // Number of CPU cores

    cout << "available threads: " << maxThreads << endl;

    int tileSize = 64;
    int overlap = 7;
    int step = tileSize - overlap;


    Mat bigImage;
    image.copyTo(bigImage);
    copyMakeBorder(image, bigImage, 0, tileSize, 0, tileSize, BORDER_CONSTANT, Scalar(0, 0, 0));

    //define black image for output
    Mat bigImage_out(bigImage.rows, bigImage.cols, CV_32FC3, Scalar(0, 0, 0));
    //Mat bigImage_out; bigImage.copyTo(bigImage_out);


    std::vector<std::future<cv::Mat>> futures;


    //for (int y = 350; y < 351; y += step) {  //process only one tile for test
    //    for (int x = 350; x < 351; x += step) {
    //for (int y = 500; y < 1000; y += step) {
    //    for (int x = 500; x < 1000; x += step) {
    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {
    //for (int y = 0; y < (bigImage.rows); y += step) {
    //    for (int x = 0; x < (bigImage.cols); x += step) {

            //cout << y << " " << x << endl;
            
            // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            // Extract tile
            Mat tile = bigImage(tileRegion).clone();
            

            //--//imshow("before", tile2);
            
            // Filter tile
            
            //Mat tile2 = NN_noise_reduction_tile(model, tile, window2d);
            

            //--//imshow("after", tile2);

            //Apply window
            //--//multiply(tile2, window2d, tile2);

            // Blend or place filtered tile back into big image
            //add(tile2, bigImage_out(tileRegion), tile2);
            //tile2.copyTo(bigImage_out(tileRegion));

            futures.push_back(std::async(std::launch::async, NN_noise_reduction_tile, model, tile, window2d));
        }
        //cout << y << endl;
    }


    int index = 0;
    for (int y = 0; y < (bigImage.rows - tileSize); y += step) {
        for (int x = 0; x < (bigImage.cols - tileSize); x += step) {

            // Define tile region with overlap
            int width = tileSize;
            int height = tileSize;
            cv::Rect tileRegion(x, y, width, height);

            Mat tile2 = futures[index++].get(); // Get the result from future and place in the corresponding region
            add(tile2, bigImage_out(tileRegion), tile2);
            tile2.copyTo(bigImage_out(tileRegion));
        }
    }




    int width = image.cols;
    int height = image.rows;
    Rect cropRegion(0, 0, width, height);
    image = bigImage_out(cropRegion);

    //bigImage_out.copyTo(image);

    cout << "end NN filter" << endl;
    if (debug_flag == 1) {
        //cout << "end NN filter..." << endl;
        logfile << "end NN filter..." << endl;
    }


    /**/
}
#endif /* AI_NOISEREDUCTION */

// Function to apply moving average filter on a one-dimensional array
void movingAverage(const Mat& input, Mat& output, int windowSize) {
    Mat kernel = Mat::ones(windowSize, 1, CV_32F) / (float)windowSize;
    filter2D(input, output, -1, kernel, cv::Point(-1, -1), 0, BORDER_DEFAULT);
}

void movingMedian(const Mat& input, Mat& output, int windowSize) {
    int halfWindow = windowSize / 2;
    output = Mat::zeros(input.size(), input.type());

    for (int i = 0; i < input.rows; ++i) {
        std::vector<float> window;
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = std::max(0, std::min(i + j, input.rows - 1));
            window.push_back(input.at<float>(idx, 0));
        }

        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        output.at<float>(i, 0) = window[window.size() / 2];
    }
}

void banding_filter(Mat& image, int flag, int strength, float threshold) {

    if (debug_flag == 1) {
        cout << "Apply banding filter" << endl;
        logfile << "Apply banding filter" << endl;
    }

    //vector<Mat> bgr_planes;
    //split(image, bgr_planes);

    Scalar m = mean(image);
    //cout << m[0] << " " << m[1] << " " << m[2];


    if (flag == 1) {
        // Step 1: Calculate sum of each row
        Mat rowSums = Mat::zeros(image.rows, 1, CV_32F);

        for (int i = 0; i < image.rows; ++i) {
            int N = 0;
            for (int j = 0; j < image.cols; ++j) {
                //float pixelValue = bgr_planes[1].at<float>(i, j);   // only green here!
                float pixelValue = image.at<Vec3f>(i, j)[1];          // only green here!
                if (pixelValue < (m[1]*threshold)) {                        // image mean value as threshold to exclude stars
                    rowSums.at<float>(i, 0) += pixelValue;
                    N++;
                }
            }
            rowSums.at<float>(i, 0) = rowSums.at<float>(i, 0) / N;
            //cout << N << endl;
            //cout << rowSums.at<float>(i, 0) << endl;
        }

        // Step 2: Apply moving average filter
        Mat filteredRowSums;
        int windowSize = strength; // Define your window size here
        movingAverage(rowSums, filteredRowSums, windowSize);
        //movingMedian(rowSums, filteredRowSums, windowSize);

        // Step 3: Scale original image rows
        for (int i = 0; i < image.rows; ++i) {
            float scale = filteredRowSums.at<float>(i, 0) / rowSums.at<float>(i, 0);
            image.row(i) *= scale;
        }
    }
    if (flag == 2) {
        // Step 1: Calculate sum of each column
        Mat colSums = Mat::zeros(image.cols, 1, CV_32F);

        for (int i = 0; i < image.cols; ++i) {
            int N = 0;
            for (int j = 0; j < image.rows; ++j) {
                //float pixelValue = bgr_planes[1].at<float>(j, i);   // only green here!
                float pixelValue = image.at<Vec3f>(j, i)[1];          // only green here!
                if (pixelValue < (m[1] * threshold)) {                          // threshold here!
                    colSums.at<float>(i, 0) += pixelValue;
                    N++;
                }
            }
            colSums.at<float>(i, 0) = colSums.at<float>(i, 0) / N;
            //cout << N << endl;
            //cout << colSums.at<float>(i, 0) << endl;
        }

        // Step 2: Apply moving average filter
        Mat filteredColSums;
        int windowSize = strength; // Define your window size here
        movingAverage(colSums, filteredColSums, windowSize);

        // Step 3: Scale original image rows
        for (int i = 0; i < image.cols; ++i) {
            float scale = filteredColSums.at<float>(i, 0) / colSums.at<float>(i, 0);
            image.col(i) *= scale;
        }
    }

}




void zoom_in(Mat& image, double zoom) {

    if (debug_flag == 1) {
        cout << "Apply zoom in" << endl;
        logfile << "Apply zoom in" << endl;
    }

    // Calculate the central region size
    int centerX = image.cols / 2;
    int centerY = image.rows / 2;
    int width = static_cast<int>(image.cols / zoom);
    int height = static_cast<int>(image.rows / zoom);

    // Create a region of interest (ROI) for the central part
    Rect centralRegion(centerX - width / 2, centerY - height / 2, width, height);
    Mat zoomedInImage = image(centralRegion);

    // Resize the central part with the zoom factor
    resize(zoomedInImage, zoomedInImage, Size(image.cols, image.rows), 0, 0, INTER_CUBIC);

    // Place the zoomed-in central part back into the original image
    //zoomedInImage.copyTo(image);
    image = zoomedInImage;

}



void square_image(Mat& image) {

    if (debug_flag == 1) {
        cout << "Square image" << endl;
        logfile << "Square image" << endl;
    }

    // Find the smaller dimension (width or height)
    int sideLength = std::min(image.cols, image.rows);

    // Calculate the center of the original image
    cv::Point center(image.cols / 2, image.rows / 2);

    // Define the ROI (Region of Interest)
    cv::Rect roi(center.x - sideLength / 2, center.y - sideLength / 2, sideLength, sideLength);

    // Crop the image
    Mat croppedImage = image(roi);

    // Overwrite the original image variable if needed
    image = croppedImage.clone();

}




void enhance_stars(Mat& stack_image, Mat& final_image, float star_blob_radius, float star_blob_strength)
{

    if (debug_flag == 1) {
        cout << "Enhance stars" << endl;
        logfile << "Enhance stars" << endl;
    }

    Mat stars_image;

    // OpenCL variant
    /**/

    UMat u_stars_image;
    stack_image.copyTo(u_stars_image);
    multiply(u_stars_image, u_stars_image, u_stars_image);
    GaussianBlur(u_stars_image, u_stars_image, Size(star_blob_radius * 2 + 1, star_blob_radius * 2 + 1), 0);
    //normalize(u_stars_image, u_stars_image, 0.0, star_blob_strength, NORM_MINMAX);
    u_stars_image.copyTo(stars_image);
    double minVal, maxVal;
    minMaxLoc(stars_image, &minVal, &maxVal);
    float scale = star_blob_strength / static_cast<float>(maxVal);
    stars_image *= scale;
    
    final_image = final_image + stars_image;
    final_image = final_image / (1 + star_blob_strength);

    //imshow("stars", stars_image * 10.0);

    for (int i = 0; i < final_image.rows; i++)
        for (int j = 0; j < final_image.cols; j++) {
            float thr = 0.8;
            if ( (final_image.at<Vec3f>(i, j)[0] > thr) && (final_image.at<Vec3f>(i, j)[1] > thr) && (final_image.at<Vec3f>(i, j)[2] > thr)   ) {
                float max_ch = std::max({ stars_image.at<Vec3f>(i, j)[0], stars_image.at<Vec3f>(i, j)[1], stars_image.at<Vec3f>(i, j)[2] });

                final_image.at<Vec3f>(i, j)[0] = final_image.at<Vec3f>(i, j)[0] * stars_image.at<Vec3f>(i, j)[0] / max_ch;
                final_image.at<Vec3f>(i, j)[1] = final_image.at<Vec3f>(i, j)[1] * stars_image.at<Vec3f>(i, j)[1] / max_ch;
                final_image.at<Vec3f>(i, j)[2] = final_image.at<Vec3f>(i, j)[2] * stars_image.at<Vec3f>(i, j)[2] / max_ch;
                ;
            }

        }

    //imshow("stars", final_image);


    /**/

    // CPU variant
    /*
    stack_image.copyTo(stars_image);
    multiply(stars_image, stars_image, stars_image);

    GaussianBlur(stars_image, stars_image, Size(star_blob_radius * 2 + 1, star_blob_radius * 2 + 1), 0);

    normalize(stars_image, stars_image, 0.0, star_blob_strength, NORM_MINMAX);
    final_image = final_image + stars_image;
    /**/

    // draft difraction spikes variant
    /*
    int N = star_blob_radius * 2 + 1; // Kernel size
    Mat kernel(N, N, CV_32FC3, cv::Scalar(0, 0, 0)); // Initialize kernel as a 3-channel float matrix

    int centerX = N / 2;
    int centerY = N / 2;

    // Set values for the four perpendicular lines
    for (int i = 0; i < N; ++i) {
        // Horizontal line
        kernel.at<cv::Vec3f>(N*2/3 -i/3, i) = cv::Vec3f(1.0, 1.0, 1.0) * (1.0 - static_cast<float>(abs(i - centerX)) / (N / 2));

        // Vertical line
        kernel.at<cv::Vec3f>(i, N/3 + i/3) = cv::Vec3f(1.0, 1.0, 1.0) * (1.0 - static_cast<float>(abs(i - centerY)) / (N / 2));
    }

    imshow("kernel", kernel);

    // Apply the filter using cv::filter2D
    cv::filter2D(stars_image, stars_image, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    /**/


}


/*
void enhance_stars2(Mat& stack_image, Mat& final_image, float star_blob_radius, float star_blob_strength)
{

    if (debug_flag == 1) {
        cout << "Enhance stars2" << endl;
        logfile << "Enhance stars2" << endl;
    }

    Mat stars_image;

    // OpenCL variant
    

    UMat u_stars_image;
    stack_image.copyTo(u_stars_image);
    multiply(u_stars_image, u_stars_image, u_stars_image);
    GaussianBlur(u_stars_image, u_stars_image, Size(star_blob_radius * 2 + 1, star_blob_radius * 2 + 1), 0);
    normalize(u_stars_image, u_stars_image, 0.0, star_blob_strength, NORM_MINMAX);
    u_stars_image.copyTo(stars_image);
    //final_image = final_image + stars_image;

    //imshow("stars", stars_image * 10.0);

    for (int i = 0; i < final_image.rows; i++)
        for (int j = 0; j < final_image.cols; j++) {

            if ((final_image.at<Vec3f>(i, j)[0] > 0.9) && (final_image.at<Vec3f>(i, j)[1] > 0.9) && (final_image.at<Vec3f>(i, j)[2] > 0.9)) {
                float max_ch = std::max({ stars_image.at<Vec3f>(i, j)[0], stars_image.at<Vec3f>(i, j)[1], stars_image.at<Vec3f>(i, j)[2] });

                final_image.at<Vec3f>(i, j)[0] = final_image.at<Vec3f>(i, j)[0] * (1 - (1 - stars_image.at<Vec3f>(i, j)[0] / max_ch) * 0.3);
                final_image.at<Vec3f>(i, j)[1] = final_image.at<Vec3f>(i, j)[1] * (1 - (1 - stars_image.at<Vec3f>(i, j)[1] / max_ch) * 0.3);
                final_image.at<Vec3f>(i, j)[2] = final_image.at<Vec3f>(i, j)[2] * (1 - (1 - stars_image.at<Vec3f>(i, j)[2] / max_ch) * 0.3);
                ;
            }

        }
}
/**/



void highlight_protection(Mat& image)
{
    Mat dark_image;
    image.copyTo(dark_image);
    gamma_dark_correction(dark_image, 1.0);

    Mat mask_for_dark, mask_for_orig;
    cvtColor(image, mask_for_dark, cv::COLOR_BGR2GRAY);

    int blur_size = std::min(image.cols, image.rows);
    blur_size = blur_size / 300;
    if (blur_size < 4) blur_size = 4;

    medianBlur(mask_for_dark, mask_for_dark, blur_size);
    GaussianBlur(mask_for_dark, mask_for_dark, Size(blur_size, blur_size), 0);

    mask_for_dark = mask_for_dark * 0.4;

    cvtColor(mask_for_dark, mask_for_dark, cv::COLOR_GRAY2BGR);

    mask_for_orig = Scalar(1,1,1) - mask_for_dark;

    multiply(dark_image, mask_for_dark, dark_image);

    multiply(image, mask_for_orig, image);

    image = image + dark_image;


}







int main(int argc, char* argv[])
{

#define sw_name      "Digital Astronomy Eyepiece App"
#define version      "Version 0.200 beta, 02.09.2024"
#define copyright    "Copyright Andriy Melnykov 2024"
#define supp_cameras "This version supports only ZWO ASI cameras"
#define libraries    "Libraries used (see also licenses folder):"
#define license_1    "ASICamera2 SDK, copyright ZWO company"
#define license_2    "OpenCV, Apache License 2.0"
#define license_3    "CFITSIO, copyright National Aeronautics and Space Administration"
#define license_4    "frugally-deep, copyright (c) 2016 Tobias Hermann"
#define license_5    "OpenCV reg module, copyright (C) 2013, Alfonso Sanchez-Beato"

    cout << sw_name << endl;
    cout << version << endl;
    cout << copyright << endl;
    cout << supp_cameras << endl << endl;
    cout << libraries << endl;
    cout << license_1 << endl;
    cout << license_2 << endl;
    cout << license_3 << endl;
    cout << license_4 << endl;
    cout << license_5 << endl << endl;



    if (argc == 2)
        get_config(argv[1]); // get config from file, filename from command argument
    else {
        char config_filename[80] = "config.txt";
        get_config(config_filename); // get config from file
    }


    //opening log file
    if (debug_flag == 1) {
        logfile.open("log.txt", std::ios_base::app);
        if (logfile.is_open()) {

            time_t t = time(0);   // get time now
            struct tm* now = localtime(&t);
            char date_time_str[80];
            strftime(date_time_str, 80, "Log date/time: %Y-%m-%d_%H-%M-%S", now);

            logfile << endl << "---------------------------------------------" << endl;
            logfile << date_time_str << endl;
            logfile << "---------------------------------------------" << endl;
            logfile << sw_name << endl;
            logfile << version << endl;
            logfile << copyright << endl;
            logfile << supp_cameras << endl << endl;
            logfile << libraries << endl;
            logfile << license_1 << endl;
            logfile << license_2 << endl;
            logfile << license_3 << endl;
            logfile << license_4 << endl;
            logfile << license_5 << endl << endl;

            cout << "File log.txt opened" << endl;
        }
        else {
            cout << "Couldn't open file log.txt" << endl;
            cout << "Press Enter to close...";
            cin.get();
            exit(1);
        }
    }

    //init GUI buttons
    initButtons(buttons);

    /**/
    cv::ocl::Context ctx = cv::ocl::Context::getDefault();
    if (!ctx.ptr()) {
        cout << "OpenCL is not available" << endl;
        logfile << "OpenCL is not available" << endl;
    }
    else {
        cout << "OpenCL is available" << endl;
        logfile << "OpenCL is available" << endl;
    }
    /**/

#if AI_NOISEREDUCTION
    // load neural network model
    cout << "Reading AI model: " << AI_noise_model_filename << endl;
    logfile << "Reading AI model: " << AI_noise_model_filename << endl;
    const auto model = fdeep::load_model(AI_noise_model_filename);
#endif /* AI_NOISEREDUCTION */

    // check for "saved_pictures" folder
    fs::path dir = "saved_pictures";
    if (!fs::exists(dir)) {
        // The directory does not exist, so try to create it
        if (!fs::create_directory(dir)) {
            throw std::runtime_error("Failed to create directory: " + dir.string());
        }
        else {
            cout << "Directory 'saved_pictures' created" << endl;
            logfile << "Directory 'saved_pictures' created" << endl;
        }
    }

    color_palette = palette_rgb;

    check_cameras();
    
    cam = 0; // Camera index, always first camera
    cout << "Set camera index: " << cam << endl;
    logfile << "Set camera index: " << cam << endl;

    get_camera_properties();
    
    if (camera_from_file == 1)
        asi_camera_info[cam]->IsColorCam = ASI_TRUE;//ASI_FALSE; //ASI_TRUE;

    open_init_camera();

    state = video_state;
    old_state = foto_state;

    gamma = init_gamma;
    compute_LUT(gamma); // first LUT initialisation
    
    focusing_flag = 0;

    //Read darks and flats
    if ( (dark_v_hotpixel_flag == 1) || (dark_v_subtract_flag == 1) || (dark_f_hotpixel_flag == 1) || (dark_f_subtract_flag == 1) )
        read_darks(dark_v_32sc1, dark_v_mean, dark_f_32sc1, dark_f_mean);

    if ( (flat_v_flag == 1) || (flat_f_flag == 1))
        read_flat(flat_32fc3, flat_inv_32fc3);
    

    //Start eyepiece window
    if ((eyepiece_display_flag == 1) || (eyepiece_display_flag == 2) ) {
        //Mat img_test = imread("C:/Users/HOME/Desktop/stacks_test/stack_2024-03-08_21-46-02.tiff", IMREAD_UNCHANGED);

        Mat black_image(100, 100, CV_8UC3, Scalar(0, 0, 0));

        namedWindow("Eyepiece", WINDOW_NORMAL);
        moveWindow("Eyepiece", second_display_X, second_display_Y);
        setWindowProperty("Eyepiece", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
        imshow("Eyepiece", black_image);
        waitKey(1);
        //destroyWindow("Eyepiece");
    }

    key = -1;
    while ((key != key_exit) && (buttons[7].pressed == false)) {

        if (debug_flag == 1) {
            if (key != -1) {
                cout << "Key pressed: " << (char)key << endl;
                logfile << "Key pressed: " << (char)key << endl;
            }
        }

        // check color palette changes
        if ((key == key_palette) || (buttons[2].pressed == true)) {
            if (color_palette == palette_rgb) color_palette = palette_duo;
            else if (color_palette == palette_duo) color_palette = palette_rgb;
            
            new_picture = 1;
            buttons[2].pressed = false;
        }

        // state machine change state
        if ((key == key_mode) || (buttons[3].pressed == true)) {
            if (state == video_state) state = foto_state;
            else state = video_state;

            buttons[3].pressed = false;
        }

        // focusing zoom
        if ((key == key_focusing) || (buttons[4].pressed == true)) {
            if (focusing_flag == 1) focusing_flag = 0;
            else focusing_flag = 1;

            new_picture = 1;
            buttons[4].pressed = false;
        }

        // gain change
        if ((key == key_plus) || (buttons[1].pressed == true)) {
            gamma *= 1.5;
            //gamma /= 1.1;
            compute_LUT(gamma);
            new_picture = 1;
            buttons[1].pressed = false;
        }
        if ((key == key_minus) || (buttons[0].pressed == true)) {
            if (gamma > 0.5) gamma /= 1.5;
            //gamma *= 1.1;
            compute_LUT(gamma);
            new_picture = 1;
            buttons[0].pressed = false;
        }
        // save image
        if ((key == key_save_image) || (buttons[6].pressed == true)) {

            cout << "Saving images..." << endl;
            logfile << "Saving images..." << endl;

            time_t t = time(0);   // get time now
            struct tm* now = localtime(&t);
            char filename[80];
            
            Mat mat16uc3;

            strftime(filename, 80, "saved_pictures/final_%Y-%m-%d_%H-%M-%S.tiff", now);

            final_image.convertTo(mat16uc3, CV_16UC3, 65535);
            imwrite(filename, mat16uc3);

            strftime(filename, 80, "saved_pictures/display_%Y-%m-%d_%H-%M-%S.tiff", now);

            display_image.convertTo(mat16uc3, CV_16UC3, 65535);
            imwrite(filename, mat16uc3);

            strftime(filename, 80, "saved_pictures/stack_%Y-%m-%d_%H-%M-%S.tiff", now);

            stack_image.convertTo(mat16uc3, CV_16UC3, 65535);
            imwrite(filename, mat16uc3);

            // saving info to text file
            strftime(filename, 80, "saved_pictures/info_%Y-%m-%d_%H-%M-%S.txt", now);
            picfile.open(filename, std::ios_base::app);

            picfile << "frames stacked: " << frames_stacked << endl;
            if (state == video_state) {
                picfile << "integration time: " << frames_stacked << "x" << (exposure_time_v / 1000000) << "s = " << (frames_stacked * exposure_time_v / (double)1000000) << "s" << endl;
                picfile << "camera gain: " << gain_v << endl;
            }
            else {
                picfile << "integration time: " << frames_stacked << "x" << (exposure_time_f / 1000000) << "s = " << (frames_stacked * exposure_time_f / 1000000) << "s" << endl;
                picfile << "camera gain: " << gain_f << endl;
            }

            picfile.close();



            cout << "Images saved" << endl;
            logfile << "Images saved" << endl;

            new_picture = 1;
            buttons[6].pressed = false;
        }

        // check histogram show key
        if ((key == key_histogram) || (buttons[5].pressed == true)) {
            if (hist_show_state == 1) hist_show_state = 0;
            else hist_show_state = 1;
            buttons[5].pressed = false;
        }

        double t;

        if ( (old_state == video_state) && (state == video_state) ) {  // video mode, check for new frame
            
            if ( (exposure_time_v < exposure_threshold) || ( (exposure_time_v >= exposure_threshold) && (exposure_status() == 1) ) ) {
            
            //if (true) {
                //get_video_frame();
            //if (exposure_status() == 1) {

                    //cout << "cycle start" << endl;
                    t = (double)getTickCount();
            

                if (exposure_time_v < exposure_threshold) {    //video mode
                    get_video_frame();
                    //cout << "after get_video_frame in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;
                }
                else {
                    get_foto_frame();
                    //cout << "after get_foto_frame in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;
                    start_exposure();
                    //cout << "after start_exposure in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;
                }

                frames_stacked = 1;

                //process video frame
                // Copy the data into an OpenCV Mat structure

                    //double t = (double)getTickCount();
                Mat mat16uc1_bayer2(camera_image_height, camera_image_width, CV_16UC1, asi_image);
                Mat mat16uc1_bayer(camera_image_height, camera_image_width, CV_16UC1);
                mat16uc1_bayer2.copyTo(mat16uc1_bayer);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 3ms

                // check if RAW histogram should be shown
                if (hist_show_state == 1) {
                    //show_RAW_histogram(mat16uc1_bayer);
                    mat16uc1_bayer.copyTo(RAW_image);
                }
                //else
                //    if (getWindowProperty("RAW Histogram", WND_PROP_VISIBLE) != 0) {
                //        destroyWindow("RAW Histogram");
                //    }

                /**/
                // Convert to int32

                    //double t = (double)getTickCount();
                Mat mat32sc1_bayer(camera_image_height, camera_image_width, CV_32SC1);
                mat16uc1_bayer.convertTo(mat32sc1_bayer, CV_32SC1);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 5ms

                //Dark Frame

                    //double t = (double)getTickCount();
                if (dark_v_subtract_flag == 1)
                    apply_dark(mat32sc1_bayer, dark_v_32sc1, dark_v_mean);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 6ms

                //HotPixels

                    //double t = (double)getTickCount();
                if (dark_v_hotpixel_flag == 1)
                    correct_hotpixel(mat32sc1_bayer, hotpixel_list_v, num_hotpixel_v);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 0.1ms

                // Convert to uint16
                    //double t = (double)getTickCount();
                mat32sc1_bayer.convertTo(mat16uc1_bayer, CV_16UC1);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 3ms
                /**/

                // Decode Bayer data to RGB or mix monochrome to RGB
                    //double t = (double)getTickCount();
                Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
                if (asi_camera_info[cam]->IsColorCam == ASI_TRUE)
                    cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
                else
                    cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_GRAY2BGR);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 6ms



                // Convert to float32
                //Mat mat32fc3_rgb(asi_camera_info[cam]->MaxWidth / bin / monobin_k, asi_camera_info[cam]->MaxHeight / bin / monobin_k, CV_32FC3);
                    //double t = (double)getTickCount();
                mat16uc3_rgb.convertTo(stack_image, CV_32FC3, 1 / 65536.0);
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 7ms

                //Apply flat
                    //double t = (double)getTickCount();
                if (flat_v_flag == 1) {
                    if (debug_flag == 1) {
                        cout << "Apply flat" << endl;
                        logfile << "Apply flat" << endl;
                    }

                    if ((dark_v_subtract_flag == 1) || (dark_v_hotpixel_flag == 1)) {
                        stack_image = stack_image - Scalar(dark_v_mean / 65536.0, dark_v_mean / 65536.0, dark_v_mean / 65536.0);
                        //stack_image = stack_image - scl_dark_v_mean;
                        multiply(stack_image, flat_inv_32fc3, stack_image);   // full applying of flat frame
                        stack_image = stack_image + Scalar(dark_v_mean / 65536.0, dark_v_mean / 65536.0, dark_v_mean / 65536.0);
                        //stack_image = stack_image + scl_dark_v_mean;
                    }
                    else
                        multiply(stack_image, flat_inv_32fc3, stack_image);   // full applying of flat frame
                }
                    //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 29ms!!! can be 13ms without 2x dark_v_mean

                //Scalar scl_dark_v_mean = (dark_v_mean / 65536.0, dark_v_mean / 65536.0, dark_v_mean / 65536.0);
                //stack_image = stack_image + scl_dark_v_mean;

                new_picture = 1;
                    //cout << "after flat in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;
                buttons[5].pressed = false;
            }

        }

        else if ( (old_state == foto_state) && (state == foto_state) ) {  // long exposure mode, check for new frame
            if (exposure_status() == 1)
            {
                get_foto_frame();

                start_exposure();

                frames_stacked++;

                // Copy the data into an OpenCV Mat structure
                Mat mat16uc1_bayer(camera_image_height, camera_image_width, CV_16UC1, asi_image);

                // check if RAW histogram should be shown
                if (hist_show_state == 1) {
                    //show_RAW_histogram(mat16uc1_bayer);
                    mat16uc1_bayer.copyTo(RAW_image);
                }
                //else
                //    if (getWindowProperty("RAW Histogram", WND_PROP_VISIBLE) != 0) {
                //        destroyWindow("RAW Histogram");
                //    }

                // Convert to int32
                Mat mat32sc1_bayer(camera_image_height, camera_image_width, CV_32SC1);
                mat16uc1_bayer.convertTo(mat32sc1_bayer, CV_32SC1);


                //Dark Frame
                if (dark_f_subtract_flag == 1)
                    apply_dark(mat32sc1_bayer, dark_f_32sc1, dark_f_mean);

                //HotPixels
                if (dark_f_hotpixel_flag == 1)
                    correct_hotpixel(mat32sc1_bayer, hotpixel_list_f, num_hotpixel_f);

                
                // Convert to uint16
                mat32sc1_bayer.convertTo(mat16uc1_bayer, CV_16UC1);


                // Decode Bayer data to RGB or mix monochrome to RGB
                Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
                if (asi_camera_info[cam]->IsColorCam == ASI_TRUE)
                    cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
                else
                    cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_GRAY2BGR);
                
                
                // Convert to float32
                Mat mat32fc3_rgb(camera_image_height, camera_image_width, CV_32FC3);
                mat16uc3_rgb.convertTo(mat32fc3_rgb, CV_32FC3, 1 / 65536.0);


                //Apply flat
                //if (flat_flag == 1)
                //    multiply(mat32fc3_rgb, flat_inv_32fc3, mat32fc3_rgb);   // full applying of flat frame
                //Apply flat
                if (flat_f_flag == 1) {
                    if ((dark_f_subtract_flag == 1) || (dark_f_hotpixel_flag == 1)) {
                        if (debug_flag == 1) {
                            cout << "Apply flat" << endl;
                            logfile << "Apply flat" << endl;
                        }

                        mat32fc3_rgb = mat32fc3_rgb - Scalar(dark_f_mean / 65536.0, dark_f_mean / 65536.0, dark_f_mean / 65536.0);
                        multiply(mat32fc3_rgb, flat_inv_32fc3, mat32fc3_rgb);   // full applying of flat frame
                        mat32fc3_rgb = mat32fc3_rgb + Scalar(dark_f_mean / 65536.0, dark_f_mean / 65536.0, dark_f_mean / 65536.0);
                    }
                    else
                        multiply(mat32fc3_rgb, flat_inv_32fc3, mat32fc3_rgb);   // full applying of flat frame
                }

                //Scalar scl_dark_f_mean = (dark_f_mean / 65536.0, dark_f_mean / 65536.0, dark_f_mean / 65536.0);
                //mat32fc3_rgb = mat32fc3_rgb + scl_dark_f_mean;

                /**/
                if (frames_stacked == 1) {
                    mat32fc3_rgb.copyTo(first_image);
                    mat32fc3_rgb.copyTo(sum_image);                  
                }
                else {  // stacking done here
                    
                    // Register
                    
                    Ptr<Mapper> mapper = makePtr<MapperGradEuclid>();
                    MapperPyramid mappPyr(mapper);

                    if (frames_stacked == 2) 
                        mapPtr = mappPyr.calculate(first_image, mat32fc3_rgb);
                    else
                        mapPtr = mappPyr.calculate(first_image, mat32fc3_rgb, mapPtr_old);

                    mapPtr_old = mapPtr;

                    Ptr<MapAffine> mapAff = MapTypeCaster::toAffine(mapPtr);

                    if (debug_flag == 1) {
                        // Print result
                        cout << endl << "Image shift: " << endl;
                        //cout << Mat(mapAff->getLinTr()) << endl;
                        cout << Mat(mapAff->getShift()) << endl;
                        logfile << endl << "Image shift: " << endl;
                        logfile << Mat(mapAff->getShift()) << endl;
                    }

                    // Alignment
                    MapAffine mapTest(mapAff->getLinTr(), mapAff->getShift());
                    mapTest.inverseWarp(mat32fc3_rgb, mat32fc3_rgb);

                    sum_image = sum_image + mat32fc3_rgb;
                }

                stack_image = sum_image / (float)frames_stacked;
                /**/

                //test without stacking
                //mat32fc3_rgb.copyTo(stack_image);
                
                new_picture = 1;

                
               
                //wait_idle();

                //start_exposure();
            }
        }

        // Show picture, if needed
        if (new_picture == 1) {
            new_picture = 0;
        
                //cout << "start dispaly processing in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;


                //double t = (double)getTickCount();
            stack_image.copyTo(final_image);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 12ms!!!


            if (stack_from_file == 1) {
                //Mat read_image = imread("C:/Users/HOME/Desktop/stacks_test/stack_2024-08-11_00-38-08.tiff", IMREAD_UNCHANGED);
                //Mat read_image = imread("C:/Users/HOME/Desktop/stacks_test/stack_2024-08-10_23-48-17.tiff", IMREAD_UNCHANGED);
                Mat read_image = imread("saved_pictures/test_image_stack.tiff", IMREAD_UNCHANGED);
                //Mat read_image = imread("C:/Users/WinTab/Desktop/test_mode/saved_pictures/stack_2024-08-10_23-48-17.tiff", IMREAD_UNCHANGED);
                read_image.convertTo(final_image, CV_32FC3, 1 / 65535.0);
                //imshow("read", final_image);
            }


            //Banding filter
                //double t = (double)getTickCount();
            if (((banding_filter_flag == 1) || (banding_filter_flag == 2)) && (state == video_state)) {
                banding_filter(final_image, banding_filter_flag, banding_filter_strength, banding_filter_threshold);
            }
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 48ms!!!    29ms
                  


            //for bright stars "blobs"
                //double t = (double)getTickCount();
            if ((enhance_stars_flag == 1) && (focusing_flag == 0))    //
                enhance_stars(final_image, final_image, star_blob_radius, star_blob_strength);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 240ms!!!  64ms


            // WB before stretch
            if ((WBcorr_R < 0.99) || (WBcorr_G < 0.99) || (WBcorr_B < 0.99) || (WBcorr_R > 1.01) || (WBcorr_G > 1.01) || (WBcorr_B > 1.01))
                if (color_palette == palette_rgb)
                    if (focusing_flag == 0)
                        WB_correction(final_image, WBcorr_R, WBcorr_G, WBcorr_B);

            // color correction matrix before stretch
            if ((color_correction_flag == 1) && (color_palette == palette_rgb))
                color_correction(final_image);

            // Dualband palette
            if ((color_palette == palette_duo) && (focusing_flag == 0))
                dualband_colors(final_image);




            // HDR tonemapping attempt
            /*
            vector<Mat> images;
            images.push_back(final_image);
            images.push_back(final_image*5);
            images.push_back(final_image*25);
            Mat fusion;
            Ptr<MergeMertens> merge_mertens = createMergeMertens();
            merge_mertens->process(images, fusion);
            final_image = fusion * 25;
            /**/




            // Black level, WB, Stretch, Black level-------------
            // Black level
                //double t = (double)getTickCount();
            if (background_comp_flag == 1)
                black_level(final_image, black_level_value);
            else if (background_comp_flag == 2)
                black_level_gradient(final_image, black_level_value);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 232/339ms!!!   64ms/74ms


            // WB after first black level, before stretch
            /*
                //double t = (double)getTickCount();
            if ((WBcorr_R < 0.99) || (WBcorr_G < 0.99) || (WBcorr_B < 0.99))
                if (color_palette == palette_rgb)
                    if (focusing_flag == 0)
                        WB_correction(final_image, WBcorr_R, WBcorr_G, WBcorr_B);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 37ms!!!  11ms/18ms


            // color correction matrix
            if ( (color_correction_flag == 1) && (color_palette == palette_rgb) )
                color_correction(final_image);
            /**/

            // Stretch
                //double t = (double)getTickCount();
            gamma_correction(final_image, gamma);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 215ms!!!   34ms/46ms
            
            // Black level
            /*
            if (background_comp_flag == 1)
                black_level(final_image, black_level_value);
            else if (background_comp_flag == 2)
                black_level_gradient(final_image, black_level_value);
            /**/
            black_level(final_image, black_level_value);
            //-----------------------


            // highlight protection
            if ((state == foto_state) && (highlight_protection_flag == 1))
                highlight_protection(final_image);


                //cout << "after stretch/black in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;

             // show circular mask for background compensation
            if ((circular_mask_background_flag == 1) && (circular_mask_background_show == 1)) {
                //Prepare circular mask for background
                Mat circular_mask_b(final_image.rows, final_image.cols, CV_32FC3, Scalar(0.5, 0.5, 0.5));
                circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);

                final_image = max(final_image, circular_mask_b);
            }


            //Apply inverse flat
            //if ((flat_flag == 1) && (flat_inv_factor > 0.001))
            //    multiply(final_image, flat_32fc3 * flat_inv_factor, final_image);   // full removing of flat frame


            //show_histogram(final_image);


#if AI_NOISEREDUCTION
            // here place for NN noise reduction ??
            
            if ((state == foto_state) && (AI_noise_flag == 1) && (frames_stacked > AI_noise_frames) ) 
                NN_noise_reduction(model, final_image, 1.0);
            //}
#endif /* AI_NOISEREDUCTION */





            //Flip / rotate
                //double t = (double)getTickCount();
            rotate_image(final_image, image_rotation, image_flip);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 16ms!!!

            // Dualband palette
            /*
                //double t = (double)getTickCount();
            if ( (color_palette == palette_duo) && (focusing_flag == 0))
                dualband_colors(final_image);
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 86ms!!!   11/15ms
            /**/



            //Zoom in
            if (display_zoom_value > 1.01) {
                zoom_in(final_image, display_zoom_value);
            }
            //cout << final_image.rows << " " << final_image.cols << endl;
            

            // Saturation correction, only for RGB images
            /*
            if ((color_palette == palette_rgb) && (focusing_flag == 0)) {

                cvtColor(final_image, final_image, COLOR_BGR2HSV);

                Mat HSVChannels[3];
                split(final_image, HSVChannels);

                GaussianBlur(HSVChannels[1], HSVChannels[1], Size(15, 15), 0);
                HSVChannels[1] = max(HSVChannels[1], 0.0);
                HSVChannels[1] = min(HSVChannels[1], 1.0);

                //GaussianBlur(HSVChannels[0], HSVChannels[0], Size(5, 5), 0);
                //HSVChannels[0] = max(HSVChannels[0], 0.0);
                //HSVChannels[0] = min(HSVChannels[0], 1.0);

                //imshow("saturation", HSVChannels[1]);

                HSVChannels[1] *= 2;

                
                merge(HSVChannels, 3, final_image);

                cvtColor(final_image, final_image, COLOR_HSV2BGR);
            } /**/


            Mat final_image2;
            final_image.copyTo(final_image2);

            //Crop to square, if needed
                //double t = (double)getTickCount();
            if (circular_mask_flag == 1) {
                square_image(final_image2); //crop to square
            }
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 19ms!!!

            //Focusing zoom
            if (focusing_flag == 1)
                focusing_zoom(final_image2, focusing_zoom_value);


            // Resize image for display
                //double t = (double)getTickCount();
            //cout << display_height << endl;
            double display_scale = (float)display_height / final_image2.rows;
            if (display_scale < 0.99)
                resize(final_image2, display_image, Size(0, 0), display_scale, display_scale, INTER_AREA);
            else if (display_scale > 1.01)
                resize(final_image2, display_image, Size(0, 0), display_scale, display_scale, INTER_CUBIC);
            else final_image2.copyTo(display_image);
            //cout << display_image.rows << " " << display_image.cols << endl;
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 8ms
            



            //show_histogram(display_image);

            
            // Noise reduction
                //double t = (double)getTickCount();
            if (noise_reduction_flag == 1) {

                if (debug_flag == 1) {
                    cout << "Apply noise reduction" << endl;
                    logfile << "Apply noise reduction" << endl;
                }

                Mat display_image2;
                display_image.copyTo(display_image2);
                int d = 10;
                int s_v = (int)round(d * filter_strength_1);
                int s_f = (int)round(d * filter_strength_2);
                //int max_frames = 5;
                if ((state == video_state) && (s_v > 0)) {
                    bilateralFilter(display_image2, display_image, s_v, 200.0, 200.0);
                    //cout << round(d * filter_strength_2) << endl;
                }
                if ((state == foto_state) && (s_f > 0)) {
                    bilateralFilter(display_image2, display_image, s_f, 200.0, 200.0);;
                    //cout << round(d * filter_strength_1) << endl;
                }

            }
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 15ms

            /*
            // Noise reduction exp
            Mat display_image2;
            display_image.copyTo(display_image2);
            display_image2.convertTo(display_image2, CV_8UC3, 255.0);
            fastNlMeansDenoisingColored(display_image2, display_image2, 5, 10, 7, 21);
            display_image2.convertTo(display_image, CV_32FC3, 1 / 255.0);
            //dctDenoising();
            //imshow("Display filtered", display_image2);
            /**/


            
#if AI_NOISEREDUCTION
            // here place for NN noise reduction ??
            //if (state == foto_state) {
            //    NN_noise_reduction(model, display_image, 1.0);
            //}
#endif /* AI_NOISEREDUCTION */

            
            //Apply circular mask
                //double t = (double)getTickCount();
            if (circular_mask_flag == 1) {
                
                //square_image(display_image); //crop to square
                //Prepare circular mask for display
                Mat circular_mask = Mat::zeros(display_image.rows, display_image.cols, CV_32FC3);
                circle(circular_mask, Point(circular_mask.cols / 2, circular_mask.rows / 2), round(circular_mask.rows * 0.49), Scalar(1, 1, 1), FILLED, LINE_AA);
                blur(circular_mask, circular_mask, Size(round(circular_mask.rows * 0.02), round(circular_mask.rows * 0.02)));
                
                multiply(display_image, circular_mask, display_image);
            }
                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 12ms

                //cout << "before imshow in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;

                //double t = (double)getTickCount();
            
           



            
            // check if RAW histogram should be shown
            if (hist_show_state == 1)
                plot_RAW_histogram(display_image, RAW_image);
            
            namedWindow("Display window");

            if (GUI_flag == 1) {
                setMouseCallback("Display window", onMouse, &buttons);
                Mat display_image_Buttons = addButtonField(display_image, buttons);
                imshow("Display window", display_image_Buttons);
            }
            else
                imshow("Display window", display_image);

            //setWindowProperty("Display window", WND_PROP_TOPMOST, 1);


            //imshow("Display window", display_image);


                //t = ((double)getTickCount() - t) / getTickFrequency() * 1000.0; cout << "Times passed in ms: " << t << endl;  // 3ms

            //hwnd = (HWND)FindWindow(NULL, L"Display window");



                //cout << "after imshow in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;





            //show eyepiece window
            if ((eyepiece_display_flag == 1) || (eyepiece_display_flag == 2)) {
                
                int interpupillary_distance_pixels = interpupillary_distance_mm * eyepiece_display_X_pixels / eyepiece_display_X_mm;
                int eyepiece_image_radius_pixels;
                int eyepiece_image_size_pixels;

                if (eyepiece_display_flag == 1) { //single image
                    eyepiece_image_radius_pixels = min(eyepiece_display_Y_pixels / 2, eyepiece_display_X_pixels / 2);
                    eyepiece_image_size_pixels = eyepiece_image_radius_pixels * 2;
                }
                else {  //stereo image
                    int image_radius_pixels_1 = interpupillary_distance_pixels / 2;
                    int image_radius_pixels_2 = eyepiece_display_Y_pixels / 2;
                    int image_radius_pixels_3 = (eyepiece_display_X_pixels - interpupillary_distance_pixels) / 2;
                    eyepiece_image_radius_pixels = min({ image_radius_pixels_1, image_radius_pixels_2, image_radius_pixels_3 });
                    eyepiece_image_size_pixels = eyepiece_image_radius_pixels * 2;
                }

                //cout << image_radius_pixels_1 << endl; cout << image_radius_pixels_2 << endl; cout << image_radius_pixels_3 << endl;

                Mat final_image_eyepiece;
                final_image.copyTo(final_image_eyepiece);

                //crop to square
                square_image(final_image_eyepiece); 

                // Resize image for display
                resize(final_image_eyepiece, final_image_eyepiece, Size(eyepiece_image_size_pixels, eyepiece_image_size_pixels), INTER_AREA);
                
                /**/
                // Noise reduction
                if (noise_reduction_flag == 1) {

                    if (debug_flag == 1) {
                        cout << "Apply noise reduction eyepiece image" << endl;
                        logfile << "Apply noise reduction eyepiece image" << endl;
                    }

                    Mat temp_image;
                    final_image_eyepiece.copyTo(temp_image);
                    int d = 10;
                    int s_v = (int)round(d * filter_strength_1);
                    int s_f = (int)round(d * filter_strength_2);
                    if ((state == video_state) && (s_v > 0)) {
                        bilateralFilter(temp_image, final_image_eyepiece, s_v, 200.0, 200.0);
                        //cout << round(d * filter_strength_2) << endl;
                    }
                    if ((state == foto_state) && (s_f > 0)) {
                        bilateralFilter(temp_image, final_image_eyepiece, s_f, 200.0, 200.0);;
                        //cout << round(d * filter_strength_1) << endl;
                    }

                }/**/

                //Apply circular mask
                Mat circular_mask = Mat::zeros(final_image_eyepiece.rows, final_image_eyepiece.cols, CV_32FC3);
                circle(circular_mask, Point(circular_mask.cols / 2, circular_mask.rows / 2), round(circular_mask.rows * 0.49), Scalar(1, 1, 1), FILLED, LINE_AA);
                blur(circular_mask, circular_mask, Size(round(circular_mask.rows * 0.02), round(circular_mask.rows * 0.02)));

                multiply(final_image_eyepiece, circular_mask, final_image_eyepiece);

                //Black base image
                Mat eyepiece_image(eyepiece_display_Y_pixels, eyepiece_display_X_pixels, CV_32FC3, Scalar(0, 0, 0));

                //Copy small images into large one
                if (eyepiece_display_flag == 1) {
                    Rect roi1(eyepiece_display_X_pixels / 2 - eyepiece_image_radius_pixels,
                              eyepiece_display_Y_pixels / 2 - eyepiece_image_radius_pixels,
                              final_image_eyepiece.cols,
                              final_image_eyepiece.rows);
                    final_image_eyepiece.copyTo(eyepiece_image(roi1));
                }
                else {
                    Rect roi1(eyepiece_display_X_pixels / 2 - interpupillary_distance_pixels / 2 - eyepiece_image_radius_pixels,
                              eyepiece_display_Y_pixels / 2 - eyepiece_image_radius_pixels,
                              final_image_eyepiece.cols,
                              final_image_eyepiece.rows);
                    final_image_eyepiece.copyTo(eyepiece_image(roi1));
                    Rect roi2(eyepiece_display_X_pixels / 2 + interpupillary_distance_pixels / 2 - eyepiece_image_radius_pixels,
                              eyepiece_display_Y_pixels / 2 - eyepiece_image_radius_pixels,
                              final_image_eyepiece.cols,
                              final_image_eyepiece.rows);
                    final_image_eyepiece.copyTo(eyepiece_image(roi2));
                }

                rotate_image(eyepiece_image, eyepiece_display_rotation, 0);


                //Mat img_test = imread("C:/Users/HOME/Desktop/stacks_test/stack_2024-03-08_21-46-02.tiff", IMREAD_UNCHANGED);
                imshow("Eyepiece", eyepiece_image);
            }






            // cycle time and fps measurement
            t_cycle_old = t_cycle;
            t_cycle = (double)getTickCount();
            t_delta = (t_cycle - t_cycle_old) / getTickFrequency();
            //cout << "Cycle time in ms: " << t_delta * 1000.0 << "  " << "FPS: " << 1 / t_delta << endl;
       
        }

        else if ( (old_state == foto_state) && (state == video_state) ) {  // change to video mode
            if (exposure_time_v < exposure_threshold) {  //video mode
                stop_exposure();
                set_camera_controls();
                start_video();
                get_video_frame(); // dummy frame
            }
            else {                            // single exposure mode
                stop_exposure();
                set_camera_controls();
                start_exposure();
                frames_stacked = 0;
            }
        }
        else if ( (old_state == video_state) && (state == foto_state) ) {  // change to long exposure mode
            if (exposure_time_v < exposure_threshold) {  //video mode
                stop_video();
                set_camera_controls();
                start_exposure();
                frames_stacked = 0;
            }
            else {                             // single exposure mode
                stop_exposure();
                set_camera_controls();
                start_exposure();
                frames_stacked = 0;
            }
        }

        
       
        

        key = waitKey(1); // Wait for a keystroke in the window
        //key = pollKey(); 
        //cout << key << endl;
        /*
        if (key != -1) {
            printf("key: %d\n", key);   
        }  /**/



        // update old state machine state
        old_state = state;

            //cout << "after waitKey in ms: " << ((double)getTickCount() - t) / getTickFrequency() * 1000.0 << endl;

    }
    
    if (state == foto_state)
        stop_exposure();

    if (state == video_state)
        if (exposure_time_v < exposure_threshold)
            stop_video();
        else
            stop_exposure();

    close_camera();

    //cout << "Press any key to close...";
    //key = waitKey(0);


    return 0;
}



//TODO


            // GUI from right
            // 2x zoom button
            // plate solve button
            // NN filter for linear image?
            // ignore windows scale (dpi awarness)
            // ROI variant?
            // black border, why colors there?
            // delete sattelite trails
            // Fourier after NN filter
            // dark mean level as parameter
            // color noise reduction (gaus r=2-3 on color)

            // correct black level search areas
            // change black level from histogram to blur+min? only for video? only for plane? (+ mask center)
            // WB tolerant for star colors?
            // show temperature
            // cooler depending on properties?
            // WB_R/B depending on mono parameter?
            // alternative keys for mode, separate for foto and video mode (other functions?) (for ext controller)
            // video stacking with afterglow - haha, good luck
            // mono support: hot pixel list, hot pixel correction
            // Check all parameters limits
            // check all camera numbers, cam or 0?
            // hot pixel detection without dark frame
            // find camera per name or number
            // picture shift - black borders
            // picture shift to last frame position?
            // limit number of stacked pictures?
            // camera reset, if something wrong?
            // 8-bit video mode?
            // banding filter median filter?
            // faster video? - how?

// V0.100 beta
// initial

// V0.200 beta
// - added documentation of used libraries and licenses
// - performance upgrade
// - added information file to saved pictures
// - added color correction matrix support for rgb mode
// - added highlight protection option (similar to HDR tonemaping for highlights)
// - added optional screen GUI buttons
// - add AI noise reduction
// - optimized banding filter