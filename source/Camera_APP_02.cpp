// Copyright 2026, Andriy Melnykov
// https://github.com/andriymelnykov/Digital_Eyepiece_APP
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)

// This file contains the 'main' function. Program execution begins and ends there.
//

#define _CRT_SECURE_NO_WARNINGS

#include <windows.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

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

#include <opencv2/dnn.hpp>

#include "camera_functions.h"

#include "opencv2/reg/map.hpp"
#include "opencv2/reg/mapper.hpp"
#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mapperpyramid.hpp"

#include "pop_effect.hpp"
#include "backgr_comp.hpp"
#include "spline_gain_corr.hpp"
#include "sigma_stat.hpp"
#include "LineDetection.hpp"

#include "fdeep/fdeep.hpp"


#include <algorithm>
#include <cctype>
#include <cmath>


using namespace std;
using namespace cv;
using namespace reg;

long image_size; // , image_size_v, image_size_f;

bool use_video_mode = false; // defines, if video or single exposure mode used

ofstream logfile, picfile;    //logfile with debug information, picfile with saved picture information

ofstream Rfile, Gfile, Bfile, Lfile;  //for light intensity measurement
uint16_t r_meas, g_meas, b_meas, l_meas;

char camera_name_from_file[64] = "";

int debug_flag;

int auto_save_pictures;
int auto_save_pictures_n;

double t_cycle_old = 0;   // for cycle time measurement and fps
double t_cycle = 0;
double t_delta;

double t_cycle_1;
double t_cycle_2;
double t_delta_12;

long exposure_time, exposure_time_v, exposure_time_f;
long gain, gain_v, gain_f;
long WB_R, WB_R_v, WB_R_f;
long WB_G, WB_G_v, WB_G_f;
long WB_B, WB_B_v, WB_B_f;
long offset, offset_v, offset_f;
int highspeed_v;
long bandwidth; // , bandwidth_v, bandwidth_f;
float hot_pixel_sigma;
int ROI_zoom;
int scale_internalimage_height;
int crop_internalimage_flag;
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
int add_hotpixel_flag_f;
int flat_v_flag, flat_f_flag;
char dark_v_filename[80] = "dark_v.fits";
char dark_f_filename[80] = "dark_f.fits";
char flat_filename[80] = "flat.fits";
int spline_corr_flag = 0;
std::vector<float> spline_radius;
std::vector<float> spline_rValues;
std::vector<float> spline_gValues;
std::vector<float> spline_bValues;
float flat_inv_factor;
float circ_vign_factor;
float circ_vign_radius;
float blkp_x1_monitor;
float blkp_y1_monitor;
float blkp_x1_eyepiece;
float blkp_y1_eyepiece;
int cooler_activation;
int display_height;
int background_comp_flag, noise_reduction_flag;
float filter_strength_1;
float filter_strength_2;
float filter_strength_NV_1 = 0;
float filter_strength_NV_2 = 0;
int midtone_radius;
float midtone_width;
float midtone_strength;
float sharpen_sigma;
float sharpen_amount;
float black_level_value_v;
float black_level_value_f;
float black_point_offset;
int circular_mask_background_flag;
float circular_mask_background_size;
int circular_mask_background_show;
int circular_mask_flag;
int enhance_stars_flag;
int star_blob_radius;
float star_blob_strength;
float highlight_protection_par;
int reject_satellittes_flag;
float sattellites_decay;
float reject_shaky_factor;
float reject_cloudy_factor;
float init_gamma;
float lum_stretch_factor;
float star_protection_factor;
float star_factor;
float WBcorr_R, WBcorr_G, WBcorr_B;

int color_correction_flag;
float CC11, CC12, CC13;  //color correction matrix
float CC21, CC22, CC23;
float CC31, CC32, CC33;

float aR, bR, cR;  //dual band colors for R
float aG, bG, cG;  //dual band colors for G
float aB, bB, cB;  //dual band colors for B
std::vector<ColorPaletteConfig> color_palettes;
double focusing_zoom_value, zoom_value;
int focusing_zoom_type;
double display_zoom_value, display_zoom_value_stored;

int main_display_flag;
int GUI_flag;
int show_clock_flag;
int show_status_flag;
int special_setup_01;  // special setup without main screen

float AI_noise_factor;
float AI_noise_min;
float AI_noise_max;
float AI_noise_factor_min;
float AI_noise_factor_max;
int AI_noise_frames;
char AI_noise_model_filename[80];
int AI_num_threads;

char AI_noise_model_NV_filename[80];
float AI_noise_factor_NV_1 = 0, AI_noise_factor_NV_2 = 0;
float motion_gain_reduction;
int motion_number_frames;

int eyepiece_display_flag;
int eyepiece_display_X_pixels;
int eyepiece_display_Y_pixels;
float eyepiece_display_X_mm;
float eyepiece_display_Y_mm;
float interpupillary_distance_mm;
int eyepiece_display_rotation;
int second_display_X;
int second_display_Y;
int circular_mask_eyepiece_flag;

int NV_mode;
int average_type;
float kalman_alfa;
float kalman_beta;
float threshold_low;
float threshold_high;


int focusing_flag;

bool is_color_cam;
int bayer_pattern;
//BAYER_RG = 0,
//BAYER_BG,
//BAYER_GR,
//BAYER_GB

int asi_connected_cameras;
int asi_num_controls;
ASI_CAMERA_INFO** asi_camera_info;
ASI_CONTROL_CAPS** asi_control_caps;
unsigned char* asi_image;
int camera_image_width, camera_image_height;

int svb_connected_cameras;
int svb_num_controls;
SVB_CAMERA_INFO** svb_camera_info;
SVB_CAMERA_PROPERTY** svb_camera_property;
SVB_CONTROL_CAPS** svb_control_caps;
//extern int svb_cameraID_array[20];

int toup_connected_cameras;
ToupcamDeviceV2 toup_camera_info[TOUPCAM_MAX];
unsigned toup_raw_fourcc;
unsigned toup_bits_per_pixel;
HToupcam toup_handle = NULL;


unsigned char* dark_v_image;
unsigned char* dark_f_image;

Mat dark_v_32sc1;
Mat dark_f_32sc1;
int num_hotpixel_v, num_hotpixel_f, num_hotpixel_add;
int* hotpixel_list_v;  //list of hot pixels with coordinates and neighbors for interpolation
int* hotpixel_list_f;  //list of hot pixels with coordinates and neighbors for interpolation
int* hotpixel_list_add; //additional list of hot pixels found in frame, with coordinates and neighbors for interpolation
int32_t hotpixel_threshold = 5000;
int32_t coldpixel_threshold = 1;
double dark_v_mean = 0, dark_f_mean = 0;
double dark_v_stdev = 200, dark_f_stdev = 200;

unsigned char* flat_image;
Mat flat_32fc3;
Mat flat_inv_32fc3;
Mat flat_inv_32fc1;
Mat flat_revign_32fc3;  // (1-f) + f*flat
Mat flat_revign_32fc1;  // grayscale version for 1-channel pipeline
Mat flat_circvign_32fc3;  // circular vigneting
Mat flat_circvign_32fc1;  // grayscale version of circular vigneting

Mat spline_flat_corr;    // additional gain correction after flat

int cam;
int key;
int monobin_k; // , monobin_k_v, monobin_k_f;
int ROI_zoom_k;

int state;
int old_state;
int frames_stacked; // number of stacked frames in foto mode
int saved_at_frames_stacked;  // flag: images was already automatically saved at this frame counter
int color_palette;
int new_picture = 0;
int hist_show_state = 0;

float gamma;
float gamma_dark;
//#define LUT_size 1000
#define LUT_size 65536
//#define LUT_size 655360
#define LUT_size_noise 200000
float LUT_in[LUT_size];
float LUT_out[LUT_size];
float LUT_max_y;
float LUT_dark_out[LUT_size];  // this LUT for highlight protection algorithm
float LUT_star_out[LUT_size];  // this LUT for star protection algorithm
float LUT_dark_max_y, LUT_star_max_y;
float LUT_noise_in[LUT_size_noise];
float LUT_noise_out[LUT_size_noise];  // this LUT for noise filtering thread
float LUT_noise_inv[LUT_size_noise];  // this LUT for noise filtering thread
float LUT_noise_max_y;

float LUT_blkp_monitor[LUT_size];
float LUT_blkp_eyepiece[LUT_size];


Mat RAW_image;    // direct from camera, copy for histogram plot
Mat final_image;  // final image before resize, with all processing
Mat dark_image;    // as final_image, but less stretch, for highlight protection
Mat star_image;    // as final_image, but less stretch, for bright stars protection
Mat star_linear;    // for mask for bright stars protection
Mat stack_image_acq;   // only stack with darks/flats, before further processing, used in acquisition thread
Mat stack_image_gui;  // image, copied from acquisition thread to gui thread
Mat sum_image_acq;   // sum image for stacking, used in acquisition thread
Mat first_image_acq; // first image in for stacking, used in acquisition thread
Mat sat_image_acq;  // image containing satellittes, used in acquisition thread
Mat display_image;  // resized image for display
Ptr<Map> mapPtr; //current affine map, used for image registration
Ptr<Map> mapPtr_old; //previos affine map, used as initial estimation for next image registration

Mat final_image_eyepiece; // single image for eyepiece, before integration in single/double eyepiece image

Mat sub_base_image;  // dataset generation mode: single image for backgroung compensation calculation

//Control keys
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

// Global shared frames and synchronization primitives
Mat shared_grab_image;
Mat shared_stack_image;
Mat shared_RAW_image;
Mat shared_filtered_image;
mutex grab_image_mutex;
mutex stack_image_mutex;
mutex RAW_image_mutex;
mutex filt_image_mutex;
atomic<bool> new_grab_frame_available(false);
atomic<bool> new_frame_available(false);
atomic<bool> new_filt_frame_available(false);
atomic<bool> grabbing_running(false);
atomic<bool> capture_running(true);
atomic<bool> mode_change(false);
atomic<bool> palette_change(false);
atomic<bool> prohibit_new_frame(true);

atomic<bool> motion_NV(false);

Mat mask;

static thread grbThread;

float fps_grabber, fps_aqc, fps_filt, fps_main;


void cdk_square_resize_after_flat(Mat& image);


//only code text past (exclude from build)
#include "lut.cpp"
#include "color_correction.cpp"
#include "black_level.cpp"
#include "histogram.cpp"
#include "measurements.cpp"
#include "image_geometry.cpp"
#include "calibration.cpp"
#include "NN_noise_reduction.cpp"
#include "banding_filter.cpp"
#include "star_blobs.cpp"
#include "highlight_protection.cpp"
//#include "satellites.cpp"
#include "frame_rejection.cpp"
#include "gui.cpp"






void grabber_thread() {
    cout << "grabber_thread started" << endl;
    logfile << "grabber_thread started" << endl;

    double t_cycle_old = 0, t_cycle = 0, t_delta;

    while (grabbing_running) {

        get_video_frame();

        Mat mat16uc1_bayer2(camera_image_height, camera_image_width, CV_16UC1, asi_image);

        {
            lock_guard<mutex> lock(grab_image_mutex);
            shared_grab_image = mat16uc1_bayer2.clone();
            new_grab_frame_available = true;
        }

        // cycle time measurement
        t_cycle_old = t_cycle;
        t_cycle = (double)getTickCount();
        t_delta = (t_cycle - t_cycle_old) / getTickFrequency();
        fps_grabber = 1 / t_delta;
        //cout << "Grab Cycle time in ms: " << t_delta * 1000.0 << "  " << "FPS: " << 1 / t_delta << endl;

    }

    cout << "grabber_thread stopping..." << endl;
    logfile << "grabber_thread stopping..." << endl;
}


// start helper
inline void start_grabber() {
    grabbing_running = true;
    grbThread = thread(grabber_thread);
}


// stop helper
inline void stop_grabber() {
    grabbing_running = false;
    //grbThread.join();
    if (grbThread.joinable()) {
        grbThread.join();
    }

}


// Post-flat geometry: crop processed frames to square and limit large frames for downstream processing.
void cdk_square_resize_after_flat(Mat& image) {
    if (crop_internalimage_flag == 1) {
        square_image(image);
    }

    if ((scale_internalimage_height > 0) && (image.rows > scale_internalimage_height)) {
        double scale = (double)scale_internalimage_height / image.rows;
        resize(image, image, Size(0, 0), scale, scale, INTER_AREA);
    }
}


// Acquisition thread: captures and pre-processes frames
void acquisition_thread() {
    
    cout << "acquisition_thread started" << endl;
    logfile << "acquisition_thread started" << endl;

    double t_cycle_old = 0, t_cycle = 0, t_delta;

    // Kalman per-pixel
    Mat denoised;
    Mat old_frame;
    bool start_NV_mode = false;
    float gain_reduction = 1;
    int number_frames = 0;
    bool shaky_history_valid = false;
    float best_shaky_size = 0.0f;
    float best_shaky_elongation = 0.0f;


    while (capture_running) {

        if (mode_change == true) {   // check if mode should be changed
            prohibit_new_frame = true;

            if (state == video_state) state = foto_state;
            else state = video_state;

            mode_change = false;
        }

        if ((old_state == foto_state) && (state == video_state)) {  // change to video mode
            if (use_video_mode) {  //video mode
                stop_video();
                set_camera_controls();
                start_video();
                if ( (NV_mode == 1) && (camera_from_file != 1) ) {
                    start_grabber();

                    start_NV_mode = true;
                }
                //get_video_frame(); // dummy frame
                frames_stacked = 0;
                shaky_history_valid = false;
            }
            else {                            // single exposure mode
                stop_exposure();
                set_camera_controls();
                start_exposure();
                frames_stacked = 0;
                shaky_history_valid = false;
            }
        }
        else if ((old_state == video_state) && (state == foto_state)) {  // change to foto/stacking exposure mode
            if (use_video_mode) {  //video mode
                if ((NV_mode == 1) && (camera_from_file != 1)) {
                    stop_grabber();
                }
                stop_video();
                set_camera_controls();
                start_video();
                //get_video_frame(); // dummy frame
                frames_stacked = 0;
                shaky_history_valid = false;
            }
            else {                             // single exposure mode
                stop_exposure();
                set_camera_controls();
                start_exposure();
                frames_stacked = 0;
                shaky_history_valid = false;
            }
        }
        else if ((old_state == video_state) && (state == video_state)) {  // video mode, check for new frame

            if ( use_video_mode || ((!use_video_mode) && (exposure_status() == 1))) {

                if (use_video_mode) {    //video mode
                    if ((NV_mode == 1) && (camera_from_file != 1))
                        while (!new_grab_frame_available) {}  // waiting for new frame from grabbing thread
                    else
                        get_video_frame();
                }
                else {
                    get_foto_frame();
                    start_exposure();
                }

                // cycle time measurement
                t_cycle_old = t_cycle;
                t_cycle = (double)getTickCount();
                t_delta = (t_cycle - t_cycle_old) / getTickFrequency();
                fps_aqc = 1 / t_delta;
                //cout << "Acq Cycle time in ms: " << t_delta * 1000.0 << "  " << "FPS: " << 1 / t_delta << endl;

                Mat mat16uc1_bayer;

                //process video frame

                // Copy the data into an OpenCV Mat structure
                if ((NV_mode == 1) && (camera_from_file != 1)) {
                    // Copy image from grabbing thread
                    {
                        lock_guard<mutex> lock(grab_image_mutex);
                        mat16uc1_bayer = shared_grab_image.clone();
                        new_grab_frame_available = false;
                    }
                }
                else {
                    Mat mat16uc1_bayer2(camera_image_height, camera_image_width, CV_16UC1, asi_image);
                    mat16uc1_bayer = mat16uc1_bayer2.clone();
                }

                // copy raw frame for showing RAW histogram
                {
                    lock_guard<mutex> lock(RAW_image_mutex);
                    shared_RAW_image = mat16uc1_bayer.clone();
                }

                // Convert to int32
                Mat mat32sc1_bayer(camera_image_height, camera_image_width, CV_32SC1);
                mat16uc1_bayer.convertTo(mat32sc1_bayer, CV_32SC1);

                //Dark Frame
                if (dark_v_subtract_flag == 1)
                    apply_dark(mat32sc1_bayer, dark_v_32sc1, dark_v_mean);

                //HotPixels
                if (dark_v_hotpixel_flag == 1)
                    correct_hotpixel(mat32sc1_bayer, hotpixel_list_v, num_hotpixel_v);

                // Convert to uint16
                mat32sc1_bayer.convertTo(mat16uc1_bayer, CV_16UC1);

                // Decode Bayer data to RGB or mix monochrome to RGB
                Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
                if (is_color_cam) {
                    if (bayer_pattern == 0)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
                    else if (bayer_pattern == 1)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerBGGR2BGR);
                    else if (bayer_pattern == 2)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerGRBG2BGR);
                    else if (bayer_pattern == 3)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerGBRG2BGR);
                    else  // not known, default
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR);
                }
                else {
                    if (NV_mode != 1)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_GRAY2BGR);
                }


                if (meas_mode == 1) {
                    r_meas = mean(mat16uc3_rgb)[2];
                    g_meas = mean(mat16uc3_rgb)[1];
                    b_meas = mean(mat16uc3_rgb)[0];
                    l_meas = (r_meas * 1.36 + g_meas + b_meas * 2.21) / 3;
                    cout << r_meas << " " << g_meas << " " << b_meas << l_meas << endl;
                }


                // Convert to float32
                if (NV_mode == 1) 
                    mat16uc1_bayer.convertTo(stack_image_acq, CV_32FC1, 1 / 65536.0);
                else
                    mat16uc3_rgb.convertTo(stack_image_acq, CV_32FC3, 1 / 65536.0);


                if (NV_mode == 1) {
                    square_image(stack_image_acq);
                    resize(stack_image_acq, stack_image_acq, Size(0, 0), 0.5, 0.5, INTER_AREA);
                }
                
              
                if (NV_mode == 1) {
                    if (average_type == 2)
                    {
                        if (start_NV_mode)
                        {
                            // >>> Initialize Kalman filter with black CV_32FC1 frames
                            denoised = Mat(stack_image_acq.rows, stack_image_acq.cols, CV_32FC1, Scalar::all(0));
                            old_frame = Mat(stack_image_acq.rows, stack_image_acq.cols, CV_32FC1, Scalar::all(0));

                            //start_NV_mode = false;
                        }

                        // Motion detection

                        // brightness mask from img_old: 8U mask with values {0,255}
                        Mat mask8u;
                        cv::compare(old_frame, threshold_high, mask8u, cv::CMP_GT);

                        // Number of masked pixels (avoid division by zero later)
                        int64_t cnt = cv::countNonZero(mask8u);

                        // sum over masked pixels in img_old
                        float a1 = static_cast<float>(cv::mean(old_frame, mask8u)[0]) * static_cast<float>(cnt);

                        // sum over masked pixels in img_curr (using the SAME mask)
                        float b1 = static_cast<float>(cv::mean(stack_image_acq, mask8u)[0]) * static_cast<float>(cnt);

                        // proportion b/a (guard tiny a)
                        const float eps = 1e-12f;
                        float r1 = (std::abs(a1) > eps) ? (b1 / a1) : 0.0;

                        //cout << "r1: " << r1 << endl;

                        float K = 0.5;
                        if (r1 < threshold_low) {
                            if (number_frames < motion_number_frames) number_frames++;
                            else
                            {
                                K = kalman_alfa;
                                motion_NV = true;
                            }
                        }
                        else {
                            if (number_frames > 0) number_frames--;
                            else
                            {
                                K = kalman_beta;
                                motion_NV = false;
                            }
                        }
                        //cout << "number_frames: " << number_frames << endl;

                        if ( abs(display_zoom_value - display_zoom_value_stored * focusing_zoom_value) < 0.01)
                            K = 1.0;

                        if (start_NV_mode)
                        {
                            K = kalman_beta;
                            start_NV_mode = false;
                        }
                        

                        Mat temp;
                        subtract(stack_image_acq, denoised, temp);
                        multiply(K, temp, temp);
                        add(denoised, temp, denoised);

                        old_frame = stack_image_acq;

                        stack_image_acq = denoised.clone();

                        
                        
                        if (motion_NV)
                        {
                            gain_reduction = motion_gain_reduction;
                        }
                        else
                        {
                            gain_reduction += kalman_beta* (1.0 - gain_reduction);
                        }
                        if (gain_reduction < 0.95)
                            stack_image_acq = stack_image_acq * gain_reduction;
                    }
                }

                //Apply flat
                if (flat_v_flag == 1) {
                    if (debug_flag == 1) {
                        cout << "Apply flat" << endl;
                        logfile << "Apply flat" << endl;
                    }

                    if ((dark_v_subtract_flag == 1) || (dark_v_hotpixel_flag == 1)) {

                        if (stack_image_acq.channels() == 3) { //  BGR path
                            stack_image_acq = stack_image_acq - Scalar(dark_v_mean / 65536.0, dark_v_mean / 65536.0, dark_v_mean / 65536.0);
                            multiply(stack_image_acq, flat_inv_32fc3, stack_image_acq);   // full applying of flat frame
                            stack_image_acq = stack_image_acq + Scalar(dark_v_mean / 65536.0, dark_v_mean / 65536.0, dark_v_mean / 65536.0);
                        }
                        else {   // mono path
                            stack_image_acq = stack_image_acq - (dark_v_mean / 65536.0);
                            multiply(stack_image_acq, flat_inv_32fc1, stack_image_acq);   // full applying of flat frame
                            stack_image_acq = stack_image_acq + (dark_v_mean / 65536.0);
                        }
                    }
                    else {
                        if (stack_image_acq.channels() == 3) { //  BGR path
                            multiply(stack_image_acq, flat_inv_32fc3, stack_image_acq);   // full applying of flat frame
                        }
                        else {// mono path
                            multiply(stack_image_acq, flat_inv_32fc1, stack_image_acq);   // full applying of flat frame
                        }
                    }
                }

                cdk_square_resize_after_flat(stack_image_acq);
 

                {
                    lock_guard<mutex> lock(stack_image_mutex);
                    shared_stack_image = stack_image_acq.clone();
                    new_frame_available = true;
                }

                frames_stacked = 1;
                prohibit_new_frame = false;

            }
            else
                if (NV_mode == 1)
                    this_thread::sleep_for(chrono::milliseconds(10));
                else
                    this_thread::sleep_for(chrono::milliseconds(50));

        }
       
        else if ((old_state == foto_state) && (state == foto_state)) {  // long exposure mode, check for new frame

            if (  ((use_video_mode) && (get_video_frame() == 1))   ||   ((!use_video_mode) && (exposure_status() == 1))    ) {

                if (use_video_mode) {    //video mode
                    // already done
                    //get_video_frame();
                }
                else {
                    get_foto_frame();
                    start_exposure();
                }

                double sensor_temperature = 0.0;
                if (get_sensor_temperature(sensor_temperature)) {
                    cout << "Sensor temperature: " << sensor_temperature << " C" << endl;
                }

                // Copy the data into an OpenCV Mat structure
                Mat mat16uc1_bayer(camera_image_height, camera_image_width, CV_16UC1, asi_image);

                // copy raw frame for showing histogram
                {
                    lock_guard<mutex> lock(RAW_image_mutex);
                    shared_RAW_image = mat16uc1_bayer.clone();
                }

                // Convert to int32
                Mat mat32sc1_bayer(camera_image_height, camera_image_width, CV_32SC1);
                mat16uc1_bayer.convertTo(mat32sc1_bayer, CV_32SC1);

                //Dark Frame
                if (dark_f_subtract_flag == 1)
                    apply_dark(mat32sc1_bayer, dark_f_32sc1, dark_f_mean);

                //HotPixels
                if (dark_f_hotpixel_flag == 1)
                    correct_hotpixel(mat32sc1_bayer, hotpixel_list_f, num_hotpixel_f);


                //additional HotPixels from first frame
                if (add_hotpixel_flag_f == 1) {
                    if (frames_stacked == 0)
                        add_frame_hotpixels(mat32sc1_bayer, dark_f_stdev);
                    correct_hotpixel(mat32sc1_bayer, hotpixel_list_add, num_hotpixel_add);
                }


                // Convert to uint16
                mat32sc1_bayer.convertTo(mat16uc1_bayer, CV_16UC1);


                // Decode Bayer data to RGB or mix monochrome to RGB
                Mat mat16uc3_rgb(camera_image_height, camera_image_width, CV_16UC3);
                if (is_color_cam) {
                    if (bayer_pattern == 0)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR_EA);
                    else if (bayer_pattern == 1)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerBGGR2BGR_EA);
                    else if (bayer_pattern == 2)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerGRBG2BGR_EA);
                    else if (bayer_pattern == 3)
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerGBRG2BGR_EA);
                    else  // not known, default
                        cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_BayerRGGB2BGR_EA);
                }
                else
                    cvtColor(mat16uc1_bayer, mat16uc3_rgb, cv::COLOR_GRAY2BGR);


                // Convert to float32
                Mat mat32fc3_rgb(camera_image_height, camera_image_width, CV_32FC3);
                mat16uc3_rgb.convertTo(mat32fc3_rgb, CV_32FC3, 1 / 65536.0);


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

                cout << "Frame " << frames_stacked + 1 << endl;

                cdk_square_resize_after_flat(mat32fc3_rgb);


                Mat delta_image;
                int diff_pixels = 0;
                bool reject_frame = false;
                bool reject_frame_satellite = false;

                if (frames_stacked == 0) {
                    mat32fc3_rgb.copyTo(first_image_acq);
                    mat32fc3_rgb.copyTo(sum_image_acq);
                    mat32fc3_rgb.copyTo(sat_image_acq);
                    sat_image_acq = sat_image_acq * 0;
                    shaky_history_valid = false;

                    if (reject_shaky_factor >= 1.01f) {
                        ShakyFrameParams param_set;
                        ShakyFrameMetrics metrics = calculate_shaky_frame_metrics(mat32fc3_rgb, param_set);
                        if (metrics.valid && (metrics.size > 0.0f) && (metrics.elongation > 0.0f)) {
                            best_shaky_size = metrics.size;
                            best_shaky_elongation = metrics.elongation;
                            shaky_history_valid = true;
                        }
                    }
                }
                else {  // stacking done here

                    // Register

                    Ptr<Mapper> mapper = makePtr<MapperGradEuclid>();
                    MapperPyramid mappPyr(mapper);

                    // calculation for square registration region inside circular mask
                    int N = round(0.7 * first_image_acq.rows * circular_mask_background_size);
                    int x_center = (first_image_acq.cols - N) / 2;
                    int y_center = (first_image_acq.rows - N) / 2;
                    Rect roi(x_center, y_center, N, N);
                    

                    if (frames_stacked == 1)
                        if (circular_mask_background_flag == 1)  // registration for square region inside circular mask only
                            mapPtr = mappPyr.calculate(first_image_acq(roi).clone(), mat32fc3_rgb(roi).clone());
                        else                                       // registration for full image
                            mapPtr = mappPyr.calculate(first_image_acq, mat32fc3_rgb);
                    else
                        if (circular_mask_background_flag == 1)
                            mapPtr = mappPyr.calculate(first_image_acq, mat32fc3_rgb, mapPtr_old);
                        else
                            mapPtr = mappPyr.calculate(first_image_acq(roi).clone(), mat32fc3_rgb(roi).clone(), mapPtr_old);


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

                    // Check frame rejection because of satellites
                    if (reject_satellittes_flag) {
                        get_delta_image(first_image_acq, mat32fc3_rgb, delta_image, diff_pixels);
                        if (diff_pixels < 1000) reject_frame = false;
                        else {
                            reject_frame = true;
                            reject_frame_satellite = true;
                            cout << "frame rejected: satellite" << endl;
                        }
                    }

                    // Check frame rejection because of vibrations
                    if (reject_shaky_factor >= 1.01f) {
                        ShakyFrameParams param_set;
                        ShakyFrameMetrics metrics = calculate_shaky_frame_metrics(mat32fc3_rgb, param_set);

                        if (metrics.valid && (metrics.size > 0.0f) && (metrics.elongation > 0.0f) && !reject_frame_satellite) {
                            if (!shaky_history_valid) {
                                best_shaky_size = metrics.size;
                                best_shaky_elongation = metrics.elongation;
                                shaky_history_valid = true;
                            }
                            else if ((metrics.size > best_shaky_size * reject_shaky_factor) ||
                                     (metrics.elongation > best_shaky_elongation * reject_shaky_factor)) {
                                reject_frame = true;
                                cout << "frame rejected: shaky" << endl;
                            }
                            else {
                                best_shaky_size = min(best_shaky_size, metrics.size);
                                best_shaky_elongation = min(best_shaky_elongation, metrics.elongation);
                            }
                        }
                        cout << "Best size: " << best_shaky_size << " elongation: " << best_shaky_elongation << endl;
                    }

                    // Check frame rejection because of clouds
                    // ToDo

                    // Compute frames sum for stacking
                    if (!reject_frame) {
                        sum_image_acq = sum_image_acq + mat32fc3_rgb;
                    }

                    if (save_subs == 1)
                        save_sub_image(mat32fc3_rgb);
                }
                
                // compute stack from frames sum
                if (!reject_frame) {
                    stack_image_acq = sum_image_acq / (float)(frames_stacked + 1);
                }
                else {
                    stack_image_acq = sum_image_acq / (float)(frames_stacked);
                }

                // add satellite image if needed
                if (sattellites_decay > 0.01) {
                    if (reject_frame_satellite)
                        sat_image_acq = sat_image_acq + delta_image * 0.3;
                    stack_image_acq = stack_image_acq + sat_image_acq;
                    sat_image_acq = sat_image_acq * sattellites_decay;
                }

                //test without stacking
                //mat32fc3_rgb.copyTo(stack_image_acq);

                {
                    lock_guard<mutex> lock(stack_image_mutex);
                    shared_stack_image = stack_image_acq.clone();
                    new_frame_available = true;
                }

                if (!reject_frame)
                    frames_stacked++;

                prohibit_new_frame = false;
            }
            else
                if (NV_mode == 1)
                    this_thread::sleep_for(chrono::milliseconds(10));
                else
                    this_thread::sleep_for(chrono::milliseconds(50));
        }

        old_state = state;

    }
    

    cout << "acquisition_thread stopping..." << endl;
    logfile << "acquisition_thread stopping..." << endl;
}


// Filtering thread: NN filtering frames
void filtering_thread() {

    Mat stack_image_filt;

    float gamma_n = 0;
    float gamma_n1 = 0;

    cout << "filtering_thread started" << endl;
    logfile << "filtering_thread started" << endl;


    // load neural network model
    cout << "Reading AI model: " << AI_noise_model_filename << endl;
    logfile << "Reading AI model: " << AI_noise_model_filename << endl;
    const auto model = fdeep::load_model(AI_noise_model_filename);


    while (capture_running) {

        if ((new_frame_available || palette_change) && (prohibit_new_frame == false) ) {
            if (palette_change)
                palette_change = false;


            // Copy image from camera thread
            {
                lock_guard<mutex> lock(stack_image_mutex);
                stack_image_filt = shared_stack_image.clone();
                new_frame_available = false;
            }

            //cout << "test_file" << endl;

            if (stack_from_file == 1) { // test mode
                Mat read_image = imread("saved_pictures/test_image_stack.tiff", IMREAD_UNCHANGED);
                read_image.convertTo(stack_image_filt, CV_32FC3, 1 / 65535.0);
                if (NV_mode == 1)
                    extractChannel(stack_image_filt, stack_image_filt, 1); // 0=B, 1=G, 2=R
            }

            // for test only
            //ShakyFrameParams param_set;
            //calculate_shaky_frame_metrics(stack_image_filt, param_set);

            if (spline_corr_flag == 1)
            {
                multiply(stack_image_filt, spline_flat_corr, stack_image_filt);
            }

            //Banding filter
            if (((banding_filter_flag == 1) || (banding_filter_flag == 2)) && (state == video_state)) {
                banding_filter(stack_image_filt, banding_filter_flag, banding_filter_strength, banding_filter_threshold);
            }

            //for bright stars "blobs"
            if (NV_mode != 1)   
                if ((enhance_stars_flag == 1) && (focusing_flag == 0))    //
                    enhance_stars(stack_image_filt, stack_image_filt, star_blob_radius, star_blob_strength);

            bool rgb_focusing_active = (state == video_state) && (focusing_flag == 1) && (focusing_zoom_type == 2) && (is_color_cam == true);
            int active_palette_index = color_palette;
            if ((active_palette_index < 0) || (active_palette_index >= (int)color_palettes.size()))
                active_palette_index = 0;

            // Palette WB before stretch
            if ((NV_mode != 1) && (rgb_focusing_active == false) && (!color_palettes.empty())) {
                const ColorPaletteConfig& palette = color_palettes[active_palette_index];
                if ((palette.WB_R < 0.99) || (palette.WB_G < 0.99) || (palette.WB_B < 0.99) || (palette.WB_R > 1.01) || (palette.WB_G > 1.01) || (palette.WB_B > 1.01))
                    WB_correction(stack_image_filt, palette.WB_R, palette.WB_G, palette.WB_B);
            }

            min(stack_image_filt, Scalar::all(1.0f), stack_image_filt);
            max(stack_image_filt, Scalar::all(0.0f), stack_image_filt);;

            // Palette color correction matrix before stretch
            if ((NV_mode != 1) && (rgb_focusing_active == false) && (!color_palettes.empty())) {
                const ColorPaletteConfig& palette = color_palettes[active_palette_index];
                bool matrix_is_identity =
                    (fabs(palette.CC11 - 1.0f) < 0.01f) && (fabs(palette.CC12) < 0.01f) && (fabs(palette.CC13) < 0.01f) &&
                    (fabs(palette.CC21) < 0.01f) && (fabs(palette.CC22 - 1.0f) < 0.01f) && (fabs(palette.CC23) < 0.01f) &&
                    (fabs(palette.CC31) < 0.01f) && (fabs(palette.CC32) < 0.01f) && (fabs(palette.CC33 - 1.0f) < 0.01f);
                if (matrix_is_identity == false)
                    palette_color_correction(stack_image_filt, palette);
            }

            min(stack_image_filt, Scalar::all(1.0f), stack_image_filt);
            max(stack_image_filt, Scalar::all(0.0f), stack_image_filt);

            if (state == foto_state && blur_stack == 1) {
                Mat temp;
                stack_image_filt.copyTo(temp);
                //bilateralFilter(temp, stack_image_filt, -1, 1.5, 0.7);
                //medianBlur(temp, stack_image_filt, 3);
                GaussianBlur(temp, stack_image_filt, Size(0, 0), blur_stack_sigma, blur_stack_sigma);
            }


            

            // Black level
            if (background_comp_flag == 1)
                if (state == video_state)
                    if (stack_image_filt.channels() == 3)
                        black_level(stack_image_filt, black_level_value_v);
                    else
                        black_level_mono(stack_image_filt, black_level_value_v);
                else
                    if (stack_image_filt.channels() == 3)
                        black_level(stack_image_filt, black_level_value_f);
                    else
                        black_level_mono(stack_image_filt, black_level_value_f);
            else if (background_comp_flag == 2)
                if (state == video_state)
                    if (stack_image_filt.channels() == 3)
                        black_level_gradient(stack_image_filt, black_level_value_v);
                    else
                        black_level_gradient_mono(stack_image_filt, black_level_value_v);
                else
                    if (stack_image_filt.channels() == 3)
                        black_level_gradient(stack_image_filt, black_level_value_f);
                    else
                        black_level_gradient_mono(stack_image_filt, black_level_value_f);

            
            


            //bool star_correction = abs(star_protection_factor - star_factor) > 0.01;

            //if ((state == foto_state) && (star_correction)) {
            //    stack_image_filt.copyTo(star_linear);  // for using as darker image for stars protection
            //    stack_image_filt.copyTo(star_image);
            //}


            bool star_correction = (state == foto_state) && (abs(star_protection_factor - star_factor) > 0.01);
            if (star_correction)
                star_linear = stack_image_filt.clone();

            if (state == foto_state) {
                bool doNR = (AI_noise_factor > 0.01f) && (frames_stacked > AI_noise_frames);
                bool doPop = (midtone_strength > 0.01f);
                bool needsGamma = doNR || doPop;

                //cout << doNR << doPop << needsGamma << endl;

                Mat before_NN_image;

                if (doNR || doPop) {
                    stack_image_filt.copyTo(before_NN_image);
                }

                if (needsGamma) {
                    if (abs(gamma_n - gamma) > 1e-6f) {
                        gamma_n = gamma;
                        compute_LUT_noise(gamma_n);
                    }
                    //add blackpoint offset, compensated with stretch curve slope
                    float stretch_slope = (LUT_noise_out[10] - LUT_noise_out[0]) / (LUT_noise_in[10] - LUT_noise_in[0]);
                    add(stack_image_filt, Scalar(black_point_offset / stretch_slope, black_point_offset / stretch_slope, black_point_offset / stretch_slope), stack_image_filt);

                    gamma_noise_correction(stack_image_filt, 0.0);
                }

                if (doNR) {
                    if ( (AI_noise_factor > 0.99) && (AI_noise_factor < 1.01))
                        NN_noise_reduction(model, stack_image_filt, 1.0);
                    else if (AI_noise_factor <= 0.99)
                        NN_noise_reduction(model, stack_image_filt, AI_noise_factor);
                    else {
                        Mat noise_image;
                           stack_image_filt.copyTo(noise_image);

                        NN_noise_reduction(model, stack_image_filt, 1.0);

                        noise_image = noise_image - stack_image_filt;
                        //imshow("noise", noise_image); waitKey(1);

                        // square inside circular_mask_background
                        int N = round(0.7 * noise_image.rows * circular_mask_background_size);
                        int x_center = (noise_image.cols - N) / 2;
                        int y_center = (noise_image.rows - N) / 2;
                        Rect roi(x_center, y_center, N, N);
                        //Rect roi(noise_image.cols/4, noise_image.rows / 4, noise_image.cols / 2, noise_image.rows / 2);

                        Scalar mean, stddev;
                        meanStdDev(noise_image(roi).clone(), mean, stddev);
                        float noise = sqrt((stddev[0] * stddev[0] + stddev[1] * stddev[1] + stddev[2] * stddev[2]) / 3);

                        float factor = (noise - AI_noise_min) * ((AI_noise_factor_max - AI_noise_factor_min) / (AI_noise_max - AI_noise_min)) + AI_noise_factor_min;
                        if (factor < AI_noise_factor_min) factor = AI_noise_factor_min;
                        if (factor > AI_noise_factor_max) factor = AI_noise_factor_max;
                        cout << "Noise amplitude: " << noise << " Filter strength: " << factor << endl;

                        stack_image_filt = stack_image_filt + noise_image * (1 - factor);
                    }
                }

                

                //gamma_star_correction(star_image, 1.0);  // for bright star protection
                //star_protection(stack_image_filt, star_image, star_linear, 1.0, 0.1);




                if (doPop) {
                    // contrast enhancement "Pop"

                    popfx::Params p;
                    p.strength = midtone_strength;  // main knob
                    p.radius = midtone_radius;     // 20-40 typical (scale with resolution)
                    p.eps = 1.5e-4f;// maps to bilateral sigmaColor internally
                    p.micro = 0.1f;   // fine "crispness"
                    p.midWidth = midtone_width;  // midtone emphasis
                    p.base_ds = 0.3f;   // downsample base (speed!)
                    p.use_opencl = true;   // try OpenCL if available

                    stack_image_filt = popfx::popEffect(stack_image_filt, p);
                }

                if (needsGamma) {
                    gamma_noise_inv_correction(stack_image_filt, 0.0);

                    //subtract blackpoint offset, compensated with stretch curve slope
                    float stretch_slope = (LUT_noise_out[10] - LUT_noise_out[0]) / (LUT_noise_in[10] - LUT_noise_in[0]);
                    add(stack_image_filt, Scalar(-black_point_offset / stretch_slope, -black_point_offset / stretch_slope, -black_point_offset / stretch_slope), stack_image_filt);
                }

                if (doNR || doPop) {
                    highlight_protection2(stack_image_filt, before_NN_image, 1.0, 0.2);
                }
            }


            //check if star_correction and background compensation needed
            //bool star_correction = (state == foto_state) && (abs(star_protection_factor - star_factor) > 0.01);  // calculated at top
            bool bkg_compensation = (bkg_mode == 1) && (hist_show_state == 1) && !((NV_mode == 1) && (state == video_state));
            bool needsGamma = star_correction || bkg_compensation;

            // additional "spot" background compensation
            if (bkg_compensation) {
                BgCompParams p;
                cv::Mat bg;
                cv::Mat corrected = compensateBackgroundTPS(stack_image_filt, p, BgMode::Additive, &bg);
                corrected.copyTo(stack_image_filt);
            }

            if (star_correction) {
                //star_linear = stack_image_filt.clone();  //copied at top before NN noise
                if (abs(gamma_n1 - gamma) > 1e-6f) {
                    gamma_n1 = gamma;
                    compute_LUT_star(gamma_n1);
                }
            }

            if (needsGamma && (state == video_state)) {
                if (abs(gamma_n - gamma) > 1e-6f) {
                    gamma_n = gamma;
                    compute_LUT_noise(gamma_n);
                }
            }

            if (needsGamma) {
                //add blackpoint offset, compensated with stretch curve slope
                float stretch_slope = (LUT_noise_out[10] - LUT_noise_out[0]) / (LUT_noise_in[10] - LUT_noise_in[0]);
                add(stack_image_filt, Scalar(black_point_offset / stretch_slope, black_point_offset / stretch_slope, black_point_offset / stretch_slope), stack_image_filt);
            }

            if (star_correction)
                star_image = stack_image_filt.clone();

            if (needsGamma) {
                gamma_noise_correction(stack_image_filt, 0.0);
            }

            if (star_correction) {
                gamma_star_correction(star_image, 0.0);  // for bright star protection
                star_protection(stack_image_filt, star_image, star_linear, 1.0, 0.1);
            }

            if (bkg_compensation) {
                if (stack_image_filt.channels() == 3) {
                    if (!flat_circvign_32fc3.empty() && flat_circvign_32fc3.size() == stack_image_filt.size()) {
                        cv::multiply(stack_image_filt, flat_circvign_32fc3, stack_image_filt);
                    }
                    else {
                        printf("Skipping circular vignetting correction: image size %dx%d, correction size %dx%d\n",
                            stack_image_filt.cols, stack_image_filt.rows, flat_circvign_32fc3.cols, flat_circvign_32fc3.rows);
                    }
                }
                else {
                    if (!flat_circvign_32fc1.empty() && flat_circvign_32fc1.size() == stack_image_filt.size()) {
                        cv::multiply(stack_image_filt, flat_circvign_32fc1, stack_image_filt);
                    }
                    else {
                        printf("Skipping circular vignetting correction: image size %dx%d, correction size %dx%d\n",
                            stack_image_filt.cols, stack_image_filt.rows, flat_circvign_32fc1.cols, flat_circvign_32fc1.rows);
                    }
                }
            }

            if (needsGamma) {
                gamma_noise_inv_correction(stack_image_filt, 0.0);

                //subtract blackpoint offset, compensated with stretch curve slope
                float stretch_slope = (LUT_noise_out[10] - LUT_noise_out[0]) / (LUT_noise_in[10] - LUT_noise_in[0]);
                add(stack_image_filt, Scalar(-black_point_offset / stretch_slope, -black_point_offset / stretch_slope, -black_point_offset / stretch_slope), stack_image_filt);
            }



            //imshow("linear", stack_image_filt);
            //waitKey(1);


            {
                lock_guard<mutex> lock(filt_image_mutex);
                shared_filtered_image = stack_image_filt.clone();
                if ( (palette_change == false) && (mode_change == false) )
                    new_filt_frame_available = true;
            }
        }
        else
            if (NV_mode == 1)
                this_thread::sleep_for(chrono::milliseconds(10));
            else
                this_thread::sleep_for(chrono::milliseconds(50));
        
    }

    cout << "filtering_thread stopping..." << endl;
    logfile << "filtering_thread stopping..." << endl;
}




int main(int argc, char* argv[])
{

#define sw_name      "Digital Astronomy Eyepiece App"
#define version      "Version 0.700 beta, 21.09.2026"
#define copyright    "Copyright Andriy Melnykov 2026"
#define supp_cameras "This version supports ZWO ASI, SVBony, ToupTek cameras"
#define libraries    "Libraries used (see also licenses folder):"
#define license_1    "ASICamera2 SDK, copyright ZWO company"
#define license_2    "SVBCamera SDK, copyright SVBony company"
#define license_3    "ToupTek SDK, copyright ToupTek Astro"
#define license_4    "OpenCV, Apache License 2.0"
#define license_5    "CFITSIO, copyright National Aeronautics and Space Administration"
#define license_6    "frugally-deep, copyright (c) 2016 Tobias Hermann"
#define license_7    "OpenCV reg module, copyright (C) 2013, Alfonso Sanchez-Beato"

    cout << sw_name << endl;
    cout << version << endl;
    cout << copyright << endl;
    cout << supp_cameras << endl << endl;
    cout << libraries << endl;
    cout << license_1 << endl;
    cout << license_2 << endl;
    cout << license_3 << endl;
    cout << license_4 << endl;
    cout << license_5 << endl;
    cout << license_6 << endl;
    cout << license_7 << endl << endl;

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
            logfile << license_5 << endl;
            logfile << license_6 << endl;
            logfile << license_7 << endl << endl;

            cout << "File log.txt opened" << endl;
        }
        else {
            cout << "Couldn't open file log.txt" << endl;
            cout << "Press Enter to close...";
            cin.get();
            exit(1);
        }
    }

    if (meas_mode == 1) {
        Rfile.open("R.txt", std::ios_base::out);
        Gfile.open("G.txt", std::ios_base::out);
        Bfile.open("B.txt", std::ios_base::out);
        Lfile.open("L.txt", std::ios_base::out);
    }

    //init GUI buttons
    initButtons(buttons);

    //init hotkeys
    bool hotkeys_init = false;
    if (use_hotkeys) {
        hotkeys_init = true;
        if (!RegisterHotKey(NULL, 1, MOD_CONTROL | MOD_ALT, VK_F13))
            hotkeys_init = false;
        if (!RegisterHotKey(NULL, 2, MOD_CONTROL | MOD_ALT, VK_F14))
            hotkeys_init = false;
        if (!RegisterHotKey(NULL, 3, MOD_CONTROL | MOD_ALT, VK_F15))
            hotkeys_init = false;
        if (!RegisterHotKey(NULL, 4, MOD_CONTROL | MOD_ALT, VK_F16))
            hotkeys_init = false;
        if (!RegisterHotKey(NULL, 5, MOD_CONTROL | MOD_ALT, VK_F17))
            hotkeys_init = false;
        if (!RegisterHotKey(NULL, 6, MOD_CONTROL | MOD_ALT, VK_F18))
            hotkeys_init = false;
        if (!hotkeys_init) {
            cout << "Failed to register hotkeys" << endl;
            logfile << "Failed to register hotkeys" << endl;
        }
        else {
            cout << "Hotkeys registered" << endl;
            logfile << "Hotkeys registered" << endl;
        }
    }


    cv::ocl::Context ctx = cv::ocl::Context::getDefault();
    if (!ctx.ptr()) {
        cout << "OpenCL is not available" << endl;
        logfile << "OpenCL is not available" << endl;
    }
    else {
        cout << "OpenCL is available" << endl;
        logfile << "OpenCL is available" << endl;
    }


    cv::dnn::Net model_NV_onnx;
    if (NV_mode == 1)
    {
        cout << "Reading AI model NV: " << AI_noise_model_NV_filename << endl;
        logfile << "Reading AI model NV: " << AI_noise_model_NV_filename << endl;

        // Load ONNX model
        model_NV_onnx = cv::dnn::readNetFromONNX(AI_noise_model_NV_filename);

        // Prefer CPU
        model_NV_onnx.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        //model_NV_onnx.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        model_NV_onnx.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    }


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

    // check for "subs" folder
    if (save_subs == 1) {
        fs::path dir = "subs";
        if (!fs::exists(dir)) {
            // The directory does not exist, so try to create it
            if (!fs::create_directory(dir)) {
                throw std::runtime_error("Failed to create directory: " + dir.string());
            }
            else {
                cout << "Directory 'subs' created" << endl;
            }
        }
    }

    

    check_cameras();


    if (asi_connected_cameras > 0)
        use_video_mode = true;
    else if (svb_connected_cameras > 0)
        use_video_mode = true;
    else if (toup_connected_cameras > 0)
        use_video_mode = true;
    
    if ( (camera_from_file == 1) || (stack_from_file == 1) )
        use_video_mode = false;

    if ((vign_mode == 1) || (meas_mode == 1))
        use_video_mode = false;


    cam = 0; // Camera index, always first camera
    cout << "Set camera index: " << cam << endl;
    logfile << "Set camera index: " << cam << endl;

    get_camera_properties();

    /**/
    if ((cdk_mode == 1) && (camera_from_file == 0))
    {
        std::string camera_name_to_find = camera_name_from_file;
        std::string camera_name_to_find_lower = camera_name_to_find;
        std::transform(camera_name_to_find_lower.begin(), camera_name_to_find_lower.end(), camera_name_to_find_lower.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (camera_name_to_find_lower != "any")
        {
            bool defined_camera_found = false;

            for (int i = 0; i < asi_connected_cameras; i++)
            {
                cout << "ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                cout << "Camera Name from file: " << camera_name_from_file << endl;
                logfile << "ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                logfile << "Camera Name from file: " << camera_name_from_file << endl;

                std::string asi_camera_name = asi_camera_info[i]->Name;
                if (asi_camera_name.find(camera_name_to_find) != std::string::npos) {
                    cam = i;
                    defined_camera_found = true;
                    cout << "ASI camera found" << endl;
                    logfile << "ASI camera found" << endl;
                    get_camera_properties();  // here for calculation of image size
                    if (asi_camera_info[cam]->IsColorCam == ASI_TRUE)
                        is_color_cam = true;
                    else
                        is_color_cam = false;
                    bayer_pattern = asi_camera_info[cam]->BayerPattern;
                    break;
                }
            }

            if (!defined_camera_found && (svb_connected_cameras > 0))
            {
                for (int i = 0; i < svb_connected_cameras; i++)
                {
                    cout << "SVBony Camera Name: " << svb_camera_info[i]->FriendlyName << endl;
                    cout << "Camera Name from file: " << camera_name_from_file << endl;
                    logfile << "SVBony Camera Name: " << svb_camera_info[i]->FriendlyName << endl;
                    logfile << "Camera Name from file: " << camera_name_from_file << endl;

                    std::string svb_camera_name = svb_camera_info[i]->FriendlyName;
                    if (svb_camera_name.find(camera_name_to_find) != std::string::npos) {
                        cam = i;
                        asi_connected_cameras = 0;
                        defined_camera_found = true;
                        cout << "SVBony camera found" << endl;
                        logfile << "SVBony camera found" << endl;
                        get_camera_properties();  // here for calculation of image size
                        if (svb_camera_property[cam]->IsColorCam == SVB_TRUE)
                            is_color_cam = true;
                        else
                            is_color_cam = false;
                        bayer_pattern = svb_camera_property[cam]->BayerPattern;
                        break;
                    }
                }
            }

            if (!defined_camera_found && (toup_connected_cameras > 0))
            {
                unsigned cnt = Toupcam_EnumV2(toup_camera_info);
                toup_connected_cameras = static_cast<int>(cnt);

                for (int i = 0; i < toup_connected_cameras; i++)
                {
                    if (toup_camera_info[i].model == NULL)
                        continue;

#ifdef _WIN32
                    std::wstring camera_name_to_find_w;
                    for (const char* p = camera_name_from_file; *p != 0; ++p)
                        camera_name_to_find_w += static_cast<wchar_t>(*p);
                    std::wstring toup_display_name = toup_camera_info[i].displayname;
                    std::wstring toup_model_name = toup_camera_info[i].model->name;

                    wcout << L"ToupTek Camera Name: " << toup_display_name << endl;
                    wcout << L"ToupTek Model Name: " << toup_model_name << endl;
                    cout << "Camera Name from file: " << camera_name_from_file << endl;

                    bool toup_name_match = (toup_display_name.find(camera_name_to_find_w) != std::wstring::npos) ||
                        (toup_model_name.find(camera_name_to_find_w) != std::wstring::npos);
#else
                    std::string toup_display_name = toup_camera_info[i].displayname;
                    std::string toup_model_name = toup_camera_info[i].model->name;

                    cout << "ToupTek Camera Name: " << toup_display_name << endl;
                    cout << "ToupTek Model Name: " << toup_model_name << endl;
                    cout << "Camera Name from file: " << camera_name_from_file << endl;

                    bool toup_name_match = (toup_display_name.find(camera_name_to_find) != std::string::npos) ||
                        (toup_model_name.find(camera_name_to_find) != std::string::npos);
#endif
                    logfile << "ToupTek Camera " << i << endl;
                    logfile << "Camera Name from file: " << camera_name_from_file << endl;

                    if (toup_name_match) {
                        cam = i;
                        asi_connected_cameras = 0;
                        svb_connected_cameras = 0;
                        defined_camera_found = true;
                        cout << "ToupTek camera found" << endl;
                        logfile << "ToupTek camera found" << endl;
                        get_camera_properties();  // here for calculation of image size
                        break;
                    }
                }
            }

            if (!defined_camera_found)
            {
                cout << "No defined camera found. Press Enter to close...";
                logfile << "No defined camera found. Press Enter to close...";
                cin.get();
                exit(1);  // return 0;
            }
        }
    }/**/

    cout << "camera index: " << cam << endl;
    

    if (camera_from_file == 1) {
        is_color_cam = true;
        //is_color_cam = false;
        bayer_pattern = 0;
    }

    open_init_camera();

    
    //is_color_cam = false; // for test, force mono
    
    //for test
    /*
    state = foto_state;
    old_state = foto_state;
    set_camera_controls();
    close_camera();

    cout << "Test ended. Press Enter to close...";
    cin.get();
    exit(1);  // return 1;
    /**/
    //for test

    if (main_display_flag == 0) {
        special_setup_01 = 1;
        if (eyepiece_display_flag == 0)
            eyepiece_display_flag = 2;
    }
    else {
        special_setup_01 = 0;
    }

    
    //Read darks and flats
    if ( (dark_v_hotpixel_flag == 1) || (dark_v_subtract_flag == 1) || (dark_f_hotpixel_flag == 1) || (dark_f_subtract_flag == 1) )
        read_darks(dark_v_32sc1, dark_v_mean, dark_f_32sc1, dark_f_mean);

    if ((flat_v_flag == 1) || (flat_f_flag == 1)) {
        read_flat(flat_32fc3, flat_inv_32fc3);
        cvtColor(flat_inv_32fc3, flat_inv_32fc1, cv::COLOR_BGR2GRAY);
    }


    // spline gain correction preparation
    //Mat spline_flat_corr(camera_image_height, camera_image_width, CV_32FC3, Scalar(1.0f, 1.0f, 1.0f));
    spline_flat_corr.create(camera_image_height, camera_image_width, CV_32FC3);
    spline_flat_corr.setTo(Scalar(1.0f, 1.0f, 1.0f));
    if (spline_corr_flag == 1)
        RadialSplineCorrection::BuildRadialGainImage(
            spline_flat_corr,
            spline_radius,
            spline_rValues,
            spline_gValues,
            spline_bValues
        );
    cdk_square_resize_after_flat(spline_flat_corr);

    //Start eyepiece window
    if ((eyepiece_display_flag == 1) || (eyepiece_display_flag == 2) ) {

        Mat black_image(100, 100, CV_8UC3, Scalar(0, 0, 0));

        namedWindow("Eyepiece", WINDOW_NORMAL);
        moveWindow("Eyepiece", second_display_X, second_display_Y);
        setWindowProperty("Eyepiece", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
        imshow("Eyepiece", black_image);
        waitKey(1);
    }


    //Start main window
    if (main_display_flag == 1) {
        Mat black_image(display_height, display_height, CV_32FC3, Scalar(0, 0, 0));
        namedWindow("Display window");
        setMouseCallback("Display window", onMouse, &buttons);
        if (GUI_flag == 1) {
            Mat display_image_Buttons = addButtonField(black_image, buttons);
            imshow("Display window", display_image_Buttons);
        }
        else {
            Mat display_image_Status = addStatusField(black_image);
            imshow("Display window", display_image_Status);
        }
    }


    gamma = init_gamma;
    compute_LUT(gamma); // first LUT initialisation

    focusing_flag = 0;

    color_palette = palette_rgb;

    state = foto_state;
    old_state = foto_state;

    mode_change = true;


    // starting separate thread for camera frame acquisition and pre-processing
    thread acqThread(acquisition_thread);
    this_thread::sleep_for(chrono::milliseconds(1));

    // starting separate thread for camera frame filtering
    thread filtThread(filtering_thread);
    this_thread::sleep_for(chrono::milliseconds(1));
 

    new_picture = 0;

    int N_cycle = 0;

    key = -1;
    int hotkey = 0;
    while ((key != key_exit) && (buttons[7].pressed == false)) {

        //keyboard part
        if (debug_flag == 1) {
            if (key != -1) {
                logfile << "Key pressed: " << (char)key << endl;
            }
        }

        //save measured light amplitude
        if ((meas_mode == 1) && (key == (int)'l')) {
            Rfile << r_meas << " ";
            Gfile << g_meas << " ";
            Bfile << b_meas << " ";
            Lfile << l_meas << " ";
            cout << "saved saved saved saved saved" << endl;
            cout << "saved saved saved saved saved" << endl;
            cout << "saved saved saved saved saved" << endl;
            cout << "saved saved saved saved saved" << endl;
        }

        // check color palette changes
        if ((key == key_palette) || (buttons[2].pressed == true) || (hotkey == 3)) {

            show_clock();

            if (!color_palettes.empty()) {
                color_palette++;
                if (color_palette >= (int)color_palettes.size())
                    color_palette = 0;
            }
            
            //new_picture = 1;
            palette_change = true;
            new_filt_frame_available = false;

            buttons[2].pressed = false;

            logfile << "Key/button pressed: palette" << endl;
        }

        // state machine change state
        if ((key == key_mode) || (buttons[3].pressed == true) || (hotkey == 4)) {

            show_clock();

            prohibit_new_frame = true;

            mode_change = true;

            //discard any pending "old - mode" frames so GUI won't show them
            new_frame_available = false;
            new_filt_frame_available = false;

            // reset all zooms
            focusing_flag = 0;
            display_zoom_value = display_zoom_value_stored;

            // reset background spot correction
            //if (bkg_mode == 1)
            //    hist_show_state = 0;

            buttons[3].pressed = false;

            //cout << "state change" << endl;
            logfile << "Key/button pressed: mode" << endl;
        }

        // focusing zoom / zoom
        if ((key == key_focusing) || (buttons[4].pressed == true) || (hotkey == 5)) {

            show_clock();

            if (state == video_state) {
                int effective_focusing_zoom_type = focusing_zoom_type;
                if ((is_color_cam == false) && (effective_focusing_zoom_type == 2))
                    effective_focusing_zoom_type = 1;

                if (effective_focusing_zoom_type == 1) {
                    if (display_zoom_value > (display_zoom_value_stored * 1.1))
                        display_zoom_value = display_zoom_value_stored;
                    else
                        display_zoom_value = display_zoom_value_stored * focusing_zoom_value;
                    focusing_flag = 0;
                    new_picture = 1;
                }
                else if ((effective_focusing_zoom_type == 2) || (effective_focusing_zoom_type == 3)) {
                    if (focusing_flag == 1) focusing_flag = 0;
                    else focusing_flag = 1;
                    if (effective_focusing_zoom_type == 2) {
                        palette_change = true;
                        new_filt_frame_available = false;
                    }
                }
            }
            else {
                if (display_zoom_value > (display_zoom_value_stored * 1.1))
                    display_zoom_value = display_zoom_value_stored;
                else
                    display_zoom_value = display_zoom_value_stored * zoom_value;
                new_picture = 1;
            }

            buttons[4].pressed = false;

            logfile << "Key/button pressed: zoom" << endl;
        }

        // gain change
        if ((key == key_plus) || (buttons[1].pressed == true) || (hotkey == 2)) {

            show_clock();

            gamma *= 1.5;
            //gamma /= 1.1;
            compute_LUT(gamma);
            new_picture = 1;
            buttons[1].pressed = false;

            logfile << "Key/button pressed: +" << endl;
        }
        if ((key == key_minus) || (buttons[0].pressed == true) || (hotkey == 1)) {

            show_clock();

            if (gamma > 0.5) gamma /= 1.5;
            //gamma *= 1.1;
            compute_LUT(gamma);
            new_picture = 1;
            buttons[0].pressed = false;

            logfile << "Key/button pressed: -" << endl;
        }

        // save image
        if ( (key == key_save_image) || 
             (buttons[6].pressed == true) ||
             ( (auto_save_pictures == 1) && (frames_stacked > 1) && ((frames_stacked % auto_save_pictures_n) == 0) && (saved_at_frames_stacked != frames_stacked) )  ) {

            show_clock();

            if ((key != key_save_image) && (buttons[6].pressed != true))
                saved_at_frames_stacked = frames_stacked;

            cout << "Saving images..." << endl;
            logfile << "Saving images..." << endl;

            time_t t = time(0);   // get time now
            struct tm* now = localtime(&t);
            char filename[80];
            
            Mat mat16uc3;
            Mat mat8uc3;

            strftime(filename, 80, "saved_pictures/final_%Y-%m-%d_%H-%M-%S.tiff", now);
            final_image.convertTo(mat16uc3, CV_16UC3, 65535);
            imwrite(filename, mat16uc3);
                       
            if (special_setup_01 == 1) { // no main window

                Mat temp_image;
                final_image.copyTo(temp_image);

                square_image(temp_image); //crop to square

                // Circluar mask
                Mat circular_mask = Mat::zeros(temp_image.rows, temp_image.cols, CV_32FC3);
                circle(circular_mask, Point(circular_mask.cols / 2, circular_mask.rows / 2), round(circular_mask.rows * 0.49), Scalar(1, 1, 1), FILLED, LINE_AA);
                blur(circular_mask, circular_mask, Size(round(circular_mask.rows * 0.02), round(circular_mask.rows * 0.02)));
                multiply(temp_image, circular_mask, temp_image);
                
                strftime(filename, 80, "saved_pictures/display_%Y-%m-%d_%H-%M-%S.jpg", now);
                temp_image.convertTo(mat8uc3, CV_8UC3, 256);
                imwrite(filename, mat8uc3);
            }
            else {
                strftime(filename, 80, "saved_pictures/display_%Y-%m-%d_%H-%M-%S.jpg", now);
                display_image.convertTo(mat8uc3, CV_8UC3, 256);
                imwrite(filename, mat8uc3);
            }

            Mat stack_for_save;
            // Copy image from camera thread
            {
                lock_guard<mutex> lock(stack_image_mutex);
                stack_for_save = shared_stack_image.clone();
            }

            strftime(filename, 80, "saved_pictures/stack_%Y-%m-%d_%H-%M-%S.tiff", now);
            stack_for_save.convertTo(mat16uc3, CV_16UC3, 65535);
            imwrite(filename, mat16uc3);

            // saving info to text file
            strftime(filename, 80, "saved_pictures/info_%Y-%m-%d_%H-%M-%S.txt", now);
            picfile.open(filename, std::ios_base::app);

            picfile << "frames stacked: " << frames_stacked << endl;
            if (state == video_state) {
                picfile << "integration time: " << frames_stacked << "x" << (exposure_time_v / (double)1000000) << "s = " << (frames_stacked * exposure_time_v / (double)1000000) << "s" << endl;
                picfile << "camera gain: " << gain_v << endl;
            }
            else {
                picfile << "integration time: " << frames_stacked << "x" << (exposure_time_f / (double)1000000) << "s = " << (frames_stacked * exposure_time_f / (double)1000000) << "s" << endl;
                picfile << "camera gain: " << gain_f << endl;
            }

            picfile.close();

            cout << "Images saved" << endl;
            logfile << "Images saved" << endl;

            //new_picture = 1;
            buttons[6].pressed = false;
        }

        // check histogram show key
        if ((key == key_histogram) || (buttons[5].pressed == true) || (hotkey == 6)) {

            show_clock();

            if (hist_show_state == 1) hist_show_state = 0;
            else hist_show_state = 1;

            buttons[5].pressed = false;
            if (bkg_mode == 0)
                new_picture = 1;

            logfile << "Key/button pressed: histogram" << endl;
        }

        // Show picture, if new frame available
        if (new_filt_frame_available && (prohibit_new_frame == false))
            new_picture = 1;

        // Show picture, if needed
        if (new_picture == 1) {
            new_picture = 0;
        
            
            // Copy image from filtering thread
            {
                lock_guard<mutex> lock(filt_image_mutex);
                if (new_filt_frame_available && (prohibit_new_frame == false)) {
                    stack_image_gui = shared_filtered_image.clone();
                    new_filt_frame_available = false;
                }
            }

            
            stack_image_gui.copyTo(final_image);


            if (vign_mode == 1) plot_cut(final_image);  //special mode for showing of vigneting as graph


            /*
            // additional "spot" background compensation
            if ((bkg_mode == 1) && (hist_show_state == 1)) {
                BgCompParams p;
                cv::Mat bg;
                cv::Mat corrected = compensateBackgroundTPS(final_image, p, BgMode::Additive, &bg);
                corrected.copyTo(final_image);
            }/**/



            /**///----------------
            //bool star_correction = abs(star_protection_factor - star_factor) > 0.01;
            //if ((state == foto_state) && (star_correction)) {
            //    final_image.copyTo(star_linear);  // for using as darker image for stars protection
            //}/**///----------------




            //add blackpoint offset, compensated with stretch curve slope
            float stretch_slope = (LUT_out[10] - LUT_out[0]) / (LUT_in[10] - LUT_in[0]);
            //cout << stretch_slope << endl;
            if (final_image.channels() == 3)
                add(final_image, Scalar(black_point_offset/ stretch_slope, black_point_offset/ stretch_slope, black_point_offset/ stretch_slope), final_image);
            else
                add(final_image, black_point_offset / stretch_slope, final_image);

            //save_sub_image(final_image);
            //imshow("linear", final_image);
            //waitKey(1);

            if ((state == foto_state) && (highlight_protection_par > 0.01))
                final_image.copyTo(dark_image);  // for using as darker image for highlight protection

            


            /**///----------------
            //if ((state == foto_state) && (star_correction)) {
            //    final_image.copyTo(star_image);  // for using as darker image for stars protection
            //}/**///----------------



            float active_lum_stretch_factor = lum_stretch_factor;
            if (!color_palettes.empty()) {
                int active_palette_index = color_palette;
                if ((active_palette_index < 0) || (active_palette_index >= (int)color_palettes.size()))
                    active_palette_index = 0;
                active_lum_stretch_factor = color_palettes[active_palette_index].lum_stretch_factor;
            }

            // Stretch
            gamma_correction(final_image, active_lum_stretch_factor);
            
            if ((state == foto_state) && (highlight_protection_par > 0.01))
                gamma_dark_correction(dark_image, active_lum_stretch_factor);   // for using as darker image for highlight protection

            //if (1)
            //    gamma_star_correction(star_image, 1.0);  // for bright star protection

            // Black level
            /**/
            if ( (background_comp_flag == 1) || (background_comp_flag == 2) )
            if (stack_from_file != 2) { // not for dataset generation mode
                if (state == video_state)
                    if (final_image.channels() == 3)
                        black_level(final_image, black_level_value_v);
                    else
                        black_level_mono(final_image, black_level_value_v);
                else
                    if (final_image.channels() == 3)
                        black_level(final_image, black_level_value_f);
                    else
                        black_level_mono(final_image, black_level_value_f);

                if ((state == foto_state) && (highlight_protection_par > 0.01))
                    if (dark_image.channels() == 3)
                        black_level(dark_image, black_level_value_f);   // for using as darker image for highlight protection
                    else
                        black_level_mono(dark_image, black_level_value_f);

                //if ((state == foto_state) && (star_correction))
                //    if (star_image.channels() == 3)
                //        black_level(star_image, black_level_value_f);
                //    else
                //        black_level_mono(star_image, black_level_value_f);
            }
            /**/
 

            //add blackpoint offset, after second black level
            if (final_image.channels() == 3)
                add(final_image, Scalar(black_point_offset, black_point_offset, black_point_offset), final_image);
            else
                add(final_image, black_point_offset, final_image);

            if ((state == foto_state) && (highlight_protection_par > 0.01))
                if (dark_image.channels() == 3)
                    add(dark_image, Scalar(black_point_offset, black_point_offset, black_point_offset), dark_image);
                else
                    add(dark_image, black_point_offset, dark_image);

            //if ((state == foto_state) && (star_correction))
            //    if (star_image.channels() == 3)
            //        add(star_image, Scalar(black_point_offset, black_point_offset, black_point_offset), star_image);
            //    else
            //        add(star_image, black_point_offset, star_image);


            

            // highlight protection
            if ((state == foto_state) && (highlight_protection_par > 0.01))
                highlight_protection(final_image, dark_image, highlight_protection_par, 0.0);

            
            
            /**///----------------
            // bright stars protection
            //if ((state == foto_state) && (star_correction)) {
            //    star_protection(final_image, star_image, star_linear, 1.0, 0.1);  //0.2?
            //}
            /**///----------------






            //cout << motion_NV << endl;
            if ((NV_mode == 1) && (state == video_state) && (final_image.channels() == 1))
            {  
                float AI_noise_factor_NV;
                float filter_strength_NV;

                if (motion_NV) {
                    AI_noise_factor_NV = AI_noise_factor_NV_1;
                    filter_strength_NV = filter_strength_NV_1;
                }
                else {
                    AI_noise_factor_NV += 2 * kalman_beta * (AI_noise_factor_NV_2 - AI_noise_factor_NV);
                    filter_strength_NV += 2 * kalman_beta * (filter_strength_NV_2 - filter_strength_NV);
                }

                // AI NV Noise reduction
                // time measurement
                t_cycle_1 = (double)getTickCount();

                //NN_noise_reduction_mono(*model_NV, final_image, AI_noise_factor_NV);
                if (AI_noise_factor_NV > 0.01)
                    NN_noise_reduction_mono_onnx(model_NV_onnx, final_image, AI_noise_factor_NV);

                t_cycle_2 = (double)getTickCount();
                t_delta_12 = (t_cycle_2 - t_cycle_1) / getTickFrequency() * 1000; //in ms

                // NV Noise reduction
                if (filter_strength_NV > 0.01) {
                    Mat final_image2;
                    final_image.copyTo(final_image2);
                    int d = 10;
                    int s = (int)round(d * filter_strength_NV);
                    if (s > 0)
                        bilateralFilter(final_image2, final_image, s, 200.0, 200.0);
                }

            }
       
            // Apply circular vignetting (fractional) after stretch/background compensation
            // only for "spot" background compensation mode
            /*
            if ((bkg_mode == 1 ) && (circ_vign_factor > 0.01) && (hist_show_state == 1)) {
                if (final_image.channels() == 3) {
                    cv::multiply(final_image, flat_circvign_32fc3, final_image);
                }
                else {
                    // if final_image is 1-channel float
                    cv::multiply(final_image, flat_circvign_32fc1, final_image);
                }
            }/**/

             // show circular mask for background compensation
            if ((circular_mask_background_flag == 1) && (circular_mask_background_show == 1)) {
                //Prepare circular mask for background
                if (final_image.channels() == 3) {
                    Mat circular_mask_b(final_image.rows, final_image.cols, CV_32FC3, Scalar(0.5, 0.5, 0.5));
                    circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), Scalar(0, 0, 0), FILLED, LINE_AA);
                    final_image = max(final_image, circular_mask_b);
                }
                else {
                    Mat circular_mask_b(final_image.rows, final_image.cols, CV_32FC1, 0.5);
                    circle(circular_mask_b, Point(circular_mask_b.cols / 2, circular_mask_b.rows / 2), round(0.5 * circular_mask_b.rows * circular_mask_background_size), 0, FILLED, LINE_AA);
                    final_image = max(final_image, circular_mask_b);
                }
            }

            // sharpen image using "unsharp mask" algorithm
            // without is mostly better
            if ((state == foto_state) && (sharpen_amount > 0.01)) {
                logfile << "sharpen" << endl;

                Mat blurred; 
                GaussianBlur(final_image, blurred, Size(), sharpen_sigma, sharpen_sigma);

                Mat sharpened = final_image * (1 + sharpen_amount) - blurred * (sharpen_amount);

                sharpened.copyTo(final_image);
            }
            

            //Flip / rotate
            rotate_image(final_image, image_rotation, image_flip);


            Mat final_image2;
            if (final_image.channels() == 3) {
                final_image.copyTo(final_image2);
                final_image.copyTo(final_image_eyepiece);
            }
            else {
                cvtColor(final_image, final_image2, cv::COLOR_GRAY2BGR);
                cvtColor(final_image, final_image_eyepiece, cv::COLOR_GRAY2BGR);
            }


            //Zoom in
            if (display_zoom_value > 1.01) {
                zoom_in(final_image2, display_zoom_value);
                zoom_in(final_image_eyepiece, display_zoom_value);
            }
           




            //show main window

            if (main_display_flag == 1) {

                //Crop to square, if needed
                if (circular_mask_flag == 1) {
                    square_image(final_image2); //crop to square
                }

                //Focusing zoom
                if (focusing_flag == 1) {
                    if ((focusing_zoom_type == 2) && (is_color_cam == true))
                        focusing_zoom(final_image2, focusing_zoom_value);
                    else if (focusing_zoom_type == 3)
                        focusing_zoom_edges(final_image2, focusing_zoom_value);
                }


                // Resize image for display
                double display_scale = (float)display_height / final_image2.rows;

                if (display_scale < 0.99)
                    resize(final_image2, display_image, Size(0, 0), display_scale, display_scale, INTER_AREA);
                else if (display_scale > 1.01)
                    resize(final_image2, display_image, Size(0, 0), display_scale, display_scale, INTER_CUBIC);
                else final_image2.copyTo(display_image);

                // Noise reduction
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
                    }
                    if ((state == foto_state) && (s_f > 0)) {
                        bilateralFilter(display_image2, display_image, s_f, 200.0, 200.0);;
                    }

                }


                //Apply circular mask
                if (circular_mask_flag == 1) {

                    //square_image(display_image); //crop to square
                    //Prepare circular mask for display
                    Mat circular_mask = Mat::zeros(display_image.rows, display_image.cols, CV_32FC3);
                    circle(circular_mask, Point(circular_mask.cols / 2, circular_mask.rows / 2), round(circular_mask.rows * 0.49), Scalar(1, 1, 1), FILLED, LINE_AA);
                    blur(circular_mask, circular_mask, Size(round(circular_mask.rows * 0.02), round(circular_mask.rows * 0.02)));

                    multiply(display_image, circular_mask, display_image);
                }


                // check if RAW histogram should be shown
                if ((bkg_mode == 0) && (hist_show_state == 1)) {
                    {
                        lock_guard<mutex> lock(RAW_image_mutex);
                        RAW_image = shared_RAW_image.clone();
                    }
                    plot_RAW_histogram(display_image, RAW_image);
                }

                if ((blkp_mode == 1) && (abs(blkp_x1_monitor - blkp_y1_monitor) > 0.0001))
                    blkp_monitor_correction(display_image, 0.0);


                if (GUI_flag == 1) {
                    setMouseCallback("Display window", onMouse, &buttons);
                    Mat display_image_Buttons = addButtonField(display_image, buttons);
                    imshow("Display window", display_image_Buttons);
                }
                else {
                    Mat display_image_Status = addStatusField(display_image);
                    imshow("Display window", display_image_Status);
                }

            }





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


                //crop to square
                square_image(final_image_eyepiece); 

                //Focusing zoom
                if (focusing_flag == 1) {
                    if ((focusing_zoom_type == 2) && (is_color_cam == true))
                        focusing_zoom(final_image_eyepiece, focusing_zoom_value);
                    else if (focusing_zoom_type == 3)
                        focusing_zoom_edges(final_image_eyepiece, focusing_zoom_value);
                }

                // Resize image for display
                resize(final_image_eyepiece, final_image_eyepiece, Size(eyepiece_image_size_pixels, eyepiece_image_size_pixels), INTER_AREA);
                
                
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

                }

                //Apply circular mask
                if (circular_mask_eyepiece_flag == 1) {
                    Mat circular_mask = Mat::zeros(final_image_eyepiece.rows, final_image_eyepiece.cols, CV_32FC3);
                    circle(circular_mask, Point(circular_mask.cols / 2, circular_mask.rows / 2), round(circular_mask.rows * 0.49), Scalar(1, 1, 1), FILLED, LINE_AA);
                    blur(circular_mask, circular_mask, Size(round(circular_mask.rows * 0.02), round(circular_mask.rows * 0.02)));

                    multiply(final_image_eyepiece, circular_mask, final_image_eyepiece);
                }


                // check if RAW histogram should be shown
                //if ( (hist_show_state == 1) && (special_setup_01 == 1)) {
                if ((bkg_mode == 0) && (hist_show_state == 1)) {
                    {
                        lock_guard<mutex> lock(RAW_image_mutex);
                        RAW_image = shared_RAW_image.clone();
                    }
                    plot_RAW_histogram(final_image_eyepiece, RAW_image);     
                }


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

                if ((blkp_mode == 1) && (abs(blkp_x1_eyepiece - blkp_y1_eyepiece) > 0.0001))
                    blkp_eyepiece_correction(eyepiece_image, 0.0);

                imshow("Eyepiece", eyepiece_image);

                // protection from locking main screen without connected eyepiece screen
                setMouseCallback("Eyepiece", onMouse_Eyepiece);
            }



            // cycle time and fps measurement
            t_cycle_old = t_cycle;
            t_cycle = (double)getTickCount();
            t_delta = (t_cycle - t_cycle_old) / getTickFrequency();
            fps_main = 1 / t_delta;

            if (NV_mode == 1)
            {
                N_cycle++;
                if (N_cycle > (fps_main * 3))
                {
                    N_cycle = 0;
                    cout << "FPS: " << fps_grabber << "  " << fps_aqc << "  " << fps_main << endl;
                    cout << "NN mono filter, ms: " << t_delta_12 << endl;
                }
            }
            //cout << "Main Cycle time in ms: " << t_delta * 1000.0 << "  " << "FPS: " << 1 / t_delta << endl;
            //cout << "FPS: " << fps_grabber << "  " << fps_aqc << "  " << fps_main << endl;
       
        }
        else
            if (NV_mode == 1)
                this_thread::sleep_for(chrono::milliseconds(10));
            else
                this_thread::sleep_for(chrono::milliseconds(50));


        //read hotkey message
        if (use_hotkeys && hotkeys_init) {
            MSG msg;
            hotkey = 0;
            while (PeekMessage(&msg, NULL, WM_HOTKEY, WM_HOTKEY, PM_REMOVE)) {
                int id = (int)msg.wParam;

                UINT modifiers = LOWORD(msg.lParam);
                UINT vk = HIWORD(msg.lParam);

                cout << "WM_HOTKEY id=" << id
                    << " modifiers=" << modifiers
                    << " vk=" << vk
                    << endl;

                if ((msg.wParam >= 1) && (msg.wParam <= 6)) {
                    hotkey = (int)msg.wParam;
                    cout << "message hotkey: " << msg.wParam << endl;
                    logfile << "message hotkey: " << msg.wParam << endl;
                }
            }
        }


        key = waitKey(1); // Wait for a keystroke in the window
    }
    

    // Stopping programm

    show_clock();

    // stopping acquisition and pre-processing threads
    capture_running = false;
    acqThread.join();
    filtThread.join();


    if (use_video_mode) {
        if (NV_mode == 1) stop_grabber();
        stop_video();
    }
    else
        stop_exposure();

    close_camera();

    //cout << "Press any key to close...";
    //key = waitKey(0);


    return 0;
}



//TODO

            // yaml format for config + parameter names check
            // switch config on-the-fly per key                 
            // plate solve button
            // black border, why colors there?
            // dark mean level as parameter
            // color noise reduction (gaus r=2-3 on color)
            // thread priority?

            // different gains for modes?
            // changing gamma dark->bright during stacking?
            // change black level from histogram to blur+min? only for video? only for plane? (+ mask center)
            // cooler depending on properties?
            // WB_R/B depending on mono parameter?
            // alternative keys for mode, separate for foto and video mode (other functions?) (for ext controller)
            // mono support: hot pixel list, hot pixel correction
            // full mono support
            // Check all parameters limits
            // picture shift - black borders
            // picture shift to last frame position?
            // limit number of stacked pictures?
            // camera reset, if something wrong?
            // banding filter median filter?

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

// V0.3
// - added alternative zoom function to focus zoom button
// - added automatic saving of pictures
// - added black screen mode, acitivated with mouse click on image
// - added control of highlights part of stretching curve - star protection factor
// - added control of mixing original and AI noise filtered image
// - added correct DPI awareness mode 

// V0.4
// - added separate threads for camera frame acquisition and pre-processing
// - added hot pixel detection without dark frame
// - corrected bug, working with some image sizes
// - updated ASI SDK

// V0.5
// - added support of SVBony cameras
// - added support of different bayer patterns
// - changed NN filter algorithm to prevent grid artefacts
// - changed interaction of NN filter and GUI - more responsive
// - trained new NN filtes
// - added automatic mode for NN filter strength
// - updated frugally deep library
// - updated white balance and color correction
// - updated stretch tone curve
// - added parameter to highlight protection
// - released eyepiece-only mode
// - added optional clock waiting symbol
// - updated registration/alignment area for special cases
// - added midtone contrast enhancement algorithm
// - added sigma parameter for hotpixel correction
// - added black point offset parameter

// V0.6
// - added simple check for config file
// - added delete and show sattelite trails

// V0.7
// - added support of ToupTek cameras
// - added experimental mono high-fps night vision mode
// - added finding camera per name (for multiple connected cameras)
// - corrected black level search areas
// - added output of sensor temperature
// - added hotkeys for external controller
// - added spline flat correction (for special cases)
// - added 2x ROI zoom mode
// - added optional scaling of internal image (like additional fractional bin)
// - added optional blackpoint correction for monitor and eyepiece
// - added recovery of saturated stars
// - added general configuration of color processing (white balance and color matrix)
// - added chroma preserving stretch
// - added rejecting of frames with vibrations (experimental)
// - added new focusing zoom mode (center/edges)
// - added status field for main GUI window