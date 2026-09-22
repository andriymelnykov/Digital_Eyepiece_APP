// Copyright 2026, Andriy Melnykov
// https://github.com/andriymelnykov/Digital_Eyepiece_APP
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)

#ifndef CAMERA_FUNCTIONS_H
#define CAMERA_FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ASICamera2.h>
#include <SVBCameraSDK.h>
#include <toupcam.h>

//#define special_setup_01 0  // special setup without main screen - changed in variable

#define camera_from_file 0  //0 - ASI camera, 1 - image from fits file
#define stack_from_file  0  //0 - ASI camera or fits file, 1 - stack image from saved tiff file, 2 - special mode for dataset generation

#define save_subs  0        //0 - nothing, 1 - save calibrated and registered subs to tiff file

//#define NV_mode 0           //special mode for high fps NV mono - changed in variable

#define vign_mode 0           //special mode for showing of vigneting as graph

#define meas_mode 0        //special mode for light intensity measurement with camera sensor

#define cdk_mode 1        //special mode for cdk: clock in both displays, no rgb focusing, search for camera name

#define bkg_mode 1        //mode for exchange histogram button with background correction function

#define blkp_mode 1      //mode for black point correction for both monitor and eyepiece

#define spline_gain_corr 1   //additional spline based gain correction after flat

#define roi_zoom 1   // roi 2x zoom active depending on config

#define blur_stack 1   // additional blur of stack to prevent artifacts
#define blur_stack_sigma 0.4  //0.6

#define use_hotkeys 1


extern long image_size; // , image_size_v, image_size_f;

extern std::ofstream logfile;

extern char camera_name_from_file[64];

extern int debug_flag;

extern int auto_save_pictures;
extern int auto_save_pictures_n;

extern long exposure_time, exposure_time_v, exposure_time_f;
extern long gain, gain_v, gain_f;
extern long WB_R, WB_R_v, WB_R_f;
extern long WB_G, WB_G_v, WB_G_f;
extern long WB_B, WB_B_v, WB_B_f;
extern long offset, offset_v, offset_f;
extern int highspeed_v;
extern long bandwidth; // , bandwidth_v, bandwidth_f;
extern float hot_pixel_sigma;
extern int ROI_zoom;
extern int scale_internalimage_height;
extern int crop_internalimage_flag;
extern long monobin; // , monobin_v, monobin_f;
extern int banding_filter_flag;
extern int banding_filter_strength;
extern float banding_filter_threshold;
extern long target_temperature;
extern int bin; // , bin_v, bin_f;
extern int image_bytes; // , image_bytes_v, image_bytes_f;
extern int image_flip;
extern int image_rotation;
extern int dark_v_hotpixel_flag, dark_v_subtract_flag;
extern int dark_f_hotpixel_flag, dark_f_subtract_flag;
extern int add_hotpixel_flag_f;
extern int flat_v_flag, flat_f_flag;
extern char dark_v_filename[80], dark_f_filename[80], flat_filename[80];
extern int spline_corr_flag;
extern std::vector<float> spline_radius;
extern std::vector<float> spline_rValues;
extern std::vector<float> spline_gValues;
extern std::vector<float> spline_bValues;
extern float flat_inv_factor;
extern float circ_vign_factor;
extern float circ_vign_radius;
extern float blkp_x1_monitor;
extern float blkp_y1_monitor;
extern float blkp_x1_eyepiece;
extern float blkp_y1_eyepiece;
extern int cooler_activation;
extern int display_height;
extern int background_comp_flag, noise_reduction_flag;
extern float filter_strength_1;
extern float filter_strength_2;
extern float filter_strength_NV_1;
extern float filter_strength_NV_2;
extern int midtone_radius;
extern float midtone_width;
extern float midtone_strength;
extern float sharpen_sigma;
extern float sharpen_amount;
extern float black_level_value_v;
extern float black_level_value_f;
extern float black_point_offset;
extern int circular_mask_background_flag;
extern float circular_mask_background_size;
extern int circular_mask_background_show;
extern int circular_mask_flag;
extern int enhance_stars_flag;
extern int star_blob_radius;
extern float star_blob_strength;
extern float highlight_protection_par;
extern int reject_satellittes_flag;
extern float sattellites_decay;
extern float reject_shaky_factor;
extern float reject_cloudy_factor;
extern float init_gamma;
extern float lum_stretch_factor;
extern float star_protection_factor;
extern float star_factor;
extern float WBcorr_R, WBcorr_G, WBcorr_B;

extern int color_correction_flag;
extern float CC11, CC12, CC13;  //color correction matrix
extern float CC21, CC22, CC23;
extern float CC31, CC32, CC33;

extern float aR, bR, cR;  //dual band colors for R
extern float aG, bG, cG;  //dual band colors for G
extern float aB, bB, cB;  //dual band colors for B

struct ColorPaletteConfig {
    std::string name;
    float lum_stretch_factor;
    float WB_R, WB_G, WB_B;
    float CC11, CC12, CC13;
    float CC21, CC22, CC23;
    float CC31, CC32, CC33;
};

extern std::vector<ColorPaletteConfig> color_palettes;

extern double focusing_zoom_value, zoom_value;
extern int focusing_zoom_type;
extern double display_zoom_value, display_zoom_value_stored;
extern int key_exit;       //(int)'x'   // exit
extern int key_mode;       //(int)'m'   //mode change foto, video
extern int key_plus;       //(int)'+'   //gain +
extern int key_minus;      //(int)'-'   //gain -
extern int key_palette;    //(int)'p'   //palette change foto, video
extern int key_save_image; //(int)'s'   //save images
extern int key_focusing;   //(int)'f'   //focusing zoom
extern int key_histogram;   //(int)'h'   //show histogram

extern int main_display_flag;
extern int GUI_flag;
extern int show_clock_flag;
extern int show_status_flag;

extern float AI_noise_factor;
extern float AI_noise_min;
extern float AI_noise_max;
extern float AI_noise_factor_min;
extern float AI_noise_factor_max;
extern int AI_noise_frames;
extern char AI_noise_model_filename[80];
extern int AI_num_threads;

extern char AI_noise_model_NV_filename[80];
extern float AI_noise_factor_NV_1, AI_noise_factor_NV_2;
extern float motion_gain_reduction;
extern int motion_number_frames;

extern int main_display_flag;

extern int eyepiece_display_flag;
extern int eyepiece_display_X_pixels;
extern int eyepiece_display_Y_pixels;
extern float eyepiece_display_X_mm;
extern float eyepiece_display_Y_mm;
extern float interpupillary_distance_mm;
extern int eyepiece_display_rotation;
extern int second_display_X;
extern int second_display_Y;
extern int circular_mask_eyepiece_flag;

extern int NV_mode;
extern int average_type;
extern float kalman_alfa;
extern float kalman_beta;
extern float threshold_low;
extern float threshold_high;

extern bool is_color_cam;
extern int bayer_pattern;

extern int asi_connected_cameras;
extern int asi_num_controls;
extern ASI_CAMERA_INFO** asi_camera_info;
extern ASI_CONTROL_CAPS** asi_control_caps;
extern unsigned char* asi_image;
//extern ASI_EXPOSURE_STATUS asi_exp_status;
extern int camera_image_width, camera_image_height;

extern int svb_connected_cameras;
extern int svb_num_controls;
extern SVB_CAMERA_INFO** svb_camera_info;
extern SVB_CAMERA_PROPERTY** svb_camera_property;
extern SVB_CONTROL_CAPS** svb_control_caps;
//extern int svb_cameraID_array[20];

extern int toup_connected_cameras;
extern ToupcamDeviceV2 toup_camera_info[TOUPCAM_MAX];
extern unsigned toup_raw_fourcc;
extern unsigned toup_bits_per_pixel;
extern HToupcam toup_handle;



extern int cam;
extern int key;
extern int monobin_k; // , monobin_k_v, monobin_k_f;
extern int ROI_zoom_k;

extern int state;
extern int old_state;
extern int frames_stacked; // number of stacked frames in foto mode
extern int color_palette;


#define video_state 0 //video state for state machine
#define foto_state  1  //long exposure state for state machine


void abort_app();

void check_cameras();

void get_camera_properties();

void open_init_camera();

void close_camera();

void set_camera_controls();

void start_video();

int get_video_frame();

void stop_video();

void start_exposure();

void stop_exposure();

int exposure_status();

bool get_sensor_temperature(double& temperature_c);

void get_foto_frame();

void wait_idle();

void get_config(char* filename);

#endif
