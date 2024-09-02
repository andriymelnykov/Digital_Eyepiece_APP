// Copyright 2024, Andriy Melnykov
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

#include <ASICamera2.h>


#define camera_from_file 0  //0 - ASI camera, 1 - image from fits file
#define stack_from_file  0  //0 - ASI camera or fits file, 1 - stack image from saved tiff file


extern long image_size; // , image_size_v, image_size_f;

extern std::ofstream logfile;

extern int debug_flag;

extern long exposure_time, exposure_time_v, exposure_time_f;
extern long gain, gain_v, gain_f;
extern long WB_R, WB_R_v, WB_R_f;
extern long WB_B, WB_B_v, WB_B_f;
extern long offset, offset_v, offset_f;
extern int highspeed_v;
extern long bandwidth; // , bandwidth_v, bandwidth_f;
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
extern int flat_v_flag, flat_f_flag;
extern char dark_v_filename[80], dark_f_filename[80], flat_filename[80];
extern float flat_inv_factor;
extern int cooler_activation;
extern int display_height;
extern int background_comp_flag, noise_reduction_flag;
extern float filter_strength_1;
extern float filter_strength_2;
extern float black_level_value;
extern int circular_mask_background_flag;
extern float circular_mask_background_size;
extern int circular_mask_background_show;
extern int circular_mask_flag;
extern int enhance_stars_flag;
extern int star_blob_radius;
extern float star_blob_strength;
extern int highlight_protection_flag;
extern float init_gamma;
extern float WBcorr_R, WBcorr_G, WBcorr_B;

extern int color_correction_flag;
extern float CC11, CC12, CC13;  //color correction matrix
extern float CC21, CC22, CC23;
extern float CC31, CC32, CC33;

extern float aR, bR, cR;  //dual band colors for R
extern float aG, bG, cG;  //dual band colors for G
extern float aB, bB, cB;  //dual band colors for B
extern double focusing_zoom_value;
extern double display_zoom_value;
extern int key_exit;       //(int)'x'   // exit
extern int key_mode;       //(int)'m'   //mode change foto, video
extern int key_plus;       //(int)'+'   //gain +
extern int key_minus;      //(int)'-'   //gain -
extern int key_palette;    //(int)'p'   //palette change foto, video
extern int key_save_image; //(int)'s'   //save images
extern int key_focusing;   //(int)'f'   //focusing zoom
extern int key_histogram;   //(int)'h'   //show histogram

extern int GUI_flag;

extern int AI_noise_flag;
extern int AI_noise_frames;
extern char AI_noise_model_filename[80];

extern int eyepiece_display_flag;
extern int eyepiece_display_X_pixels;
extern int eyepiece_display_Y_pixels;
extern float eyepiece_display_X_mm;
extern float eyepiece_display_Y_mm;
extern float interpupillary_distance_mm;
extern int eyepiece_display_rotation;
extern int second_display_X;
extern int second_display_Y;


extern int asi_connected_cameras;
extern int asi_num_controls;
extern ASI_CAMERA_INFO** asi_camera_info;
extern ASI_CONTROL_CAPS** asi_control_caps;
extern unsigned char* asi_image;
//extern ASI_EXPOSURE_STATUS asi_exp_status;
extern int camera_image_width, camera_image_height;


extern int cam;
extern int key;
extern int monobin_k; // , monobin_k_v, monobin_k_f;

extern int state;
extern int old_state;
extern int frames_stacked; // number of stacked frames in foto mode


#define video_state 0 //video state for state machine
#define foto_state  1  //long exposure state for state machine
/*
#define palette_rgb 0 //for color palettes in foto mode
#define palette_duo 1
#define palette_halpha 2
/**/

void abort_app();

void check_cameras();

void get_camera_properties();

void open_init_camera();

void close_camera();

void set_camera_controls();

void start_video();

void get_video_frame();

void stop_video();

void start_exposure();

void stop_exposure();

int exposure_status();

void get_foto_frame();

void wait_idle();

void get_config(char* filename);

#endif