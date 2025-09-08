// Copyright 2025, Andriy Melnykov
// https://github.com/andriymelnykov/Digital_Eyepiece_APP
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)

#include "camera_functions.h"

using namespace std;

void abort_app()
{
    close_camera();
    cout << "Press Enter to close...";
    cin.get();
    exit(1);
}

void check_cameras()
{
    if (camera_from_file == 1) {
        asi_connected_cameras = 1;
        cout << "Using image file instead of camera. Number of cameras set to: " << asi_connected_cameras << endl;
    }
    else {
        // Read the number of connected cameras
        asi_connected_cameras = ASIGetNumOfConnectedCameras();
        cout << "Number of connected ZWO cameras: " << asi_connected_cameras << endl;
        logfile << "Number of connected ZWO cameras: " << asi_connected_cameras << endl;

        svb_connected_cameras = SVBGetNumOfConnectedCameras();
        cout << "Number of connected SVBony cameras: " << svb_connected_cameras << endl;
        logfile << "Number of connected SVBony cameras: " << svb_connected_cameras << endl;

        if ( (asi_connected_cameras < 1) && (svb_connected_cameras < 1) ) {
            cout << "No cameras found. Press Enter to close...";
            logfile << "No cameras found. Press Enter to close...";
            cin.get();
            exit(1);  // return 0;
        }
    }
    
}



void get_camera_properties()
{
    bayer_pattern = 0;  //default RGGB

    if (camera_from_file == 1) {
        asi_camera_info = (ASI_CAMERA_INFO**)malloc(sizeof(ASI_CAMERA_INFO*) * asi_connected_cameras);
        asi_camera_info[0] = (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));
        asi_camera_info[0]->MaxWidth = 3008;
        asi_camera_info[0]->MaxHeight = 3008;
        cout << "Using image file instead of camera. Properties set: " << asi_camera_info[0]->MaxWidth << asi_camera_info[0]->MaxHeight << endl;
    }
    else {
        // Get each connected camera's properties into an array
        int get_property_success = 0;


        if (asi_connected_cameras > 0) {
            asi_camera_info = (ASI_CAMERA_INFO**)malloc(sizeof(ASI_CAMERA_INFO*) * asi_connected_cameras);
            for (int i = 0; i < asi_connected_cameras; i++) {
                asi_camera_info[i] = (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));
                ASI_ERROR_CODE ret = ASIGetCameraProperty(asi_camera_info[i], i);
                if (ret == ASI_SUCCESS) {
                    get_property_success = 1;

                    if (asi_camera_info[i]->IsColorCam == ASI_TRUE)
                        is_color_cam = true;
                    else
                        is_color_cam = false;

                    bayer_pattern = asi_camera_info[i]->BayerPattern;
                    //cout << "bayer int: " << bayer_pattern << endl;

                    if (debug_flag == 1) {
                        // Print camera's properties
                        cout << "ZWO Camera " << i << endl;
                        cout << "  ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                        cout << "  Camera ID: " << asi_camera_info[i]->CameraID << endl;
                        cout << "  Width and Height: " << asi_camera_info[i]->MaxWidth << "x" << asi_camera_info[i]->MaxHeight << endl;
                        cout << "  Color: " << (asi_camera_info[i]->IsColorCam == ASI_TRUE ? "Yes" : "No") << endl;
                        cout << "  Bayer pattern: " << asi_camera_info[i]->BayerPattern << endl;
                        cout << "  Pixel size: " << asi_camera_info[i]->PixelSize << " um" << endl;
                        //printf("  e-/ADU: %1.2f\n", asi_camera_info[i]->ElecPerADU);
                        cout << "  Bit depth: " << asi_camera_info[i]->BitDepth << endl;
                        cout << "  Trigger cam: " << (asi_camera_info[i]->IsTriggerCam == 0 ? "No" : "Yes") << endl;

                        logfile << "Camera " << i << endl;
                        logfile << "  ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                        logfile << "  Camera ID: " << asi_camera_info[i]->CameraID << endl;
                        logfile << "  Width and Height: " << asi_camera_info[i]->MaxWidth << "x" << asi_camera_info[i]->MaxHeight << endl;
                        logfile << "  Color: " << (asi_camera_info[i]->IsColorCam == ASI_TRUE ? "Yes" : "No") << endl;
                        logfile << "  Bayer pattern: " << asi_camera_info[i]->BayerPattern << endl;
                        logfile << "  Pixel size: " << asi_camera_info[i]->PixelSize << "µm" << endl;
                        logfile << "  Bit depth: " << asi_camera_info[i]->BitDepth << endl;
                        logfile << "  Trigger cam: " << (asi_camera_info[i]->IsTriggerCam == 0 ? "No" : "Yes") << endl;
                    }
                }
                else {
                    cout << "Camera ZWO: " << i << endl;
                    cout << "Error code: " << ret << endl;
                    logfile << "Camera ZWO: " << i << endl;
                    logfile << "Error code: " << ret << endl;
                }
            }
        }



        if (svb_connected_cameras > 0) {

            svb_camera_info = (SVB_CAMERA_INFO**)malloc(sizeof(SVB_CAMERA_INFO*) * svb_connected_cameras);
            svb_camera_property = (SVB_CAMERA_PROPERTY**)malloc(sizeof(SVB_CAMERA_PROPERTY*) * svb_connected_cameras);

            for (int i = 0; i < svb_connected_cameras; i++) {
                
                svb_camera_info[i] = (SVB_CAMERA_INFO*)malloc(sizeof(SVB_CAMERA_INFO));

                SVB_ERROR_CODE ret_info;
                ret_info = SVBGetCameraInfo(svb_camera_info[i], i);
                if (ret_info == SVB_SUCCESS)
                {
                    if (debug_flag == 1) {
                        cout << "SVBony Friendly name: " << svb_camera_info[i]->FriendlyName << endl;
                        cout << "Port type: " << svb_camera_info[i]->PortType << endl;
                        cout << "SN: " << svb_camera_info[i]->CameraSN << endl;
                        cout << "Device ID: " << svb_camera_info[i]->DeviceID << endl;
                        cout << "Camera ID: " << svb_camera_info[i]->CameraID << endl;
                        logfile << "SVBony Friendly name: " << svb_camera_info[i]->FriendlyName << endl;
                        logfile << "Port type: " << svb_camera_info[i]->PortType << endl;
                        logfile << "SN: " << svb_camera_info[i]->CameraSN << endl;
                        logfile << "Device ID: " << svb_camera_info[i]->DeviceID << endl;
                        logfile << "Camera ID: " << svb_camera_info[i]->CameraID << endl;
                    }
                }
                else {
                    cout << "Can not get info from SVBony camera: " << i << endl;
                    cout << "Error code: " << ret_info << endl;
                    logfile << "Can not get info from SVBony camera: " << i << endl;
                    logfile << "Error code: " << ret_info << endl;
                    cout << "Press Enter to close...";
                    cin.get();
                    exit(1); // return 1;
                }
                

                /**/
                SVB_ERROR_CODE ret_open;
                ret_open = SVBOpenCamera(svb_camera_info[i]->CameraID);
                if (ret_open != SVB_SUCCESS)
                {
                    cout << "Can not open SVBony camera: " << i << endl;
                    cout << "Error code: " << ret_open << endl;
                    logfile << "Can not open SVBony camera: " << i << endl;
                    logfile << "Error code: " << ret_open << endl;
                    cout << "Press Enter to close...";
                    cin.get();
                    exit(1); // return 1;
                }/**/

                svb_camera_property[i] = (SVB_CAMERA_PROPERTY*)malloc(sizeof(SVB_CAMERA_PROPERTY));
                SVB_ERROR_CODE ret_property = SVBGetCameraProperty(svb_camera_info[cam]->CameraID, svb_camera_property[i]);
                if (ret_property == SVB_SUCCESS) {
                    get_property_success = 1;

                    if (svb_camera_property[i]->IsColorCam == SVB_TRUE)
                        is_color_cam = true;
                    else
                        is_color_cam = false;

                    bayer_pattern = svb_camera_property[i]->BayerPattern;

                    if (debug_flag == 1) {
                        // Print camera's properties
                        cout << "SVB Camera " << i << endl;
                        cout << "  Width and Height: " << svb_camera_property[i]->MaxWidth << "x" << svb_camera_property[i]->MaxHeight << endl;
                        cout << "  Color: " << (svb_camera_property[i]->IsColorCam == SVB_TRUE ? "Yes" : "No") << endl;
                        cout << "  Bayer pattern: " << svb_camera_property[i]->BayerPattern << endl;
                        cout << "  Bit depth: " << svb_camera_property[i]->MaxBitDepth << endl;
                        cout << "  Trigger cam: " << (svb_camera_property[i]->IsTriggerCam == 0 ? "No" : "Yes") << endl;

                        logfile << "Camera " << i << endl;
                        logfile << "  Width and Height: " << svb_camera_property[i]->MaxWidth << "x" << svb_camera_property[i]->MaxHeight << endl;
                        logfile << "  Color: " << (svb_camera_property[i]->IsColorCam == SVB_TRUE ? "Yes" : "No") << endl;
                        logfile << "  Bayer pattern: " << svb_camera_property[i]->BayerPattern << endl;
                        logfile << "  Bit depth: " << svb_camera_property[i]->MaxBitDepth << endl;
                        logfile << "  Trigger cam: " << (svb_camera_property[i]->IsTriggerCam == 0 ? "No" : "Yes") << endl;

                    }
                }
                else {
                    cout << "Camera SVBony: " << i << endl;
                    cout << "Error code: " << ret_property << endl;
                    logfile << "Camera SVBony: " << i << endl;
                    logfile << "Error code: " << ret_property << endl;
                }

                SVBCloseCamera(svb_camera_info[i]->CameraID);

            }
        }




        if (get_property_success == 0) {
            cout << "Can not get camera properties" << endl;
            logfile << "Can not get camera properties" << endl;
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        




    }
    
    //----------- image size calcultion for v and f mode
    if (monobin == 0) monobin_k = 1;
    else if (monobin == 1) monobin_k = 2;
    if (ROI_zoom == 0) ROI_zoom_k = 1;
    else if (ROI_zoom == 1) ROI_zoom_k = 2;

    // Calculate image size (see bin!!!)
    //image_size = asi_camera_info[cam]->MaxWidth * asi_camera_info[cam]->MaxHeight / bin / bin / monobin_k / monobin_k / ROI_zoom_k / ROI_zoom_k;
    //image_size *= image_bytes;
 
    if (asi_connected_cameras > 0) {
        camera_image_width = asi_camera_info[cam]->MaxWidth / bin / monobin_k / ROI_zoom_k;
        camera_image_height = asi_camera_info[cam]->MaxHeight / bin / monobin_k / ROI_zoom_k;
    }
    else if (svb_connected_cameras > 0) {
        camera_image_width = svb_camera_property[cam]->MaxWidth / bin / monobin_k / ROI_zoom_k;
        camera_image_height = svb_camera_property[cam]->MaxHeight / bin / monobin_k / ROI_zoom_k;
    }
    
        
    if (ROI_zoom != 0) {
        camera_image_width = camera_image_width / 8 * 8;
        camera_image_height = camera_image_height / 2 * 2;
    }


    image_size = camera_image_width * camera_image_height;
    image_size *= image_bytes;

    if (debug_flag == 1) {
        cout << "Calculated image size: " << image_size << " bytes" << endl;
        cout << "Using image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
        logfile << "Calculated image size: " << image_size << " bytes" << endl;
        logfile << "Using image dimensions: " << camera_image_width << "x" << camera_image_height << endl;
    }

    asi_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);  //used for all cameras
}



void open_init_camera()
{
    if (camera_from_file == 1) {
        //do nothing
        printf("Not opening camera\n");
    }
    else {

        if (asi_connected_cameras > 0) {
            cout << "Opening ZWO camera..." << endl;
            logfile << "Opening ZWO camera..." << endl;

            if (ASIOpenCamera(asi_camera_info[cam]->CameraID) != ASI_SUCCESS) {
                cout << "Error opening ZWO camera" << endl;
                logfile << "Error opening ZWO camera" << endl;
                cout << "Press Enter to close...";
                cin.get();
                exit(1);  // return 1;
            }

            // Get camera's controls
            asi_num_controls = 0;
            ASIGetNumOfControls(asi_camera_info[cam]->CameraID, &asi_num_controls);
            if (asi_num_controls == 0) {
                cout << "Error num of controls 0" << endl;
                logfile << "Error num of controls 0" << endl;
                abort_app();
            }

            //cout << asi_num_controls << endl;
            asi_control_caps = (ASI_CONTROL_CAPS**)malloc(sizeof(ASI_CONTROL_CAPS*) * asi_num_controls);
            //ASI_CONTROL_CAPS** asi_control_caps = (ASI_CONTROL_CAPS**)malloc(sizeof(ASI_CONTROL_CAPS*) * asi_num_controls);
            for (int i = 0; i < asi_num_controls; i++) {
                asi_control_caps[i] = (ASI_CONTROL_CAPS*)malloc(sizeof(ASI_CONTROL_CAPS));
                if (ASIGetControlCaps(asi_camera_info[cam]->CameraID, i, asi_control_caps[i]) == ASI_SUCCESS) {
                    // Print camera's properties
                    if (debug_flag == 1) {
                        cout << "  Property " << asi_control_caps[i]->Name << ": [" << asi_control_caps[i]->MinValue << " " << asi_control_caps[i]->MaxValue
                            << "], default = " << asi_control_caps[i]->DefaultValue << endl
                            << "   is writable: " << asi_control_caps[i]->IsWritable << endl
                            << "   description: " << asi_control_caps[i]->Description << endl
                            << "   control type: " << asi_control_caps[i]->ControlType << endl;
                        logfile << "  Property " << asi_control_caps[i]->Name << ": [" << asi_control_caps[i]->MinValue << " " << asi_control_caps[i]->MaxValue
                            << "], default = " << asi_control_caps[i]->DefaultValue << endl
                            << "   is writable: " << asi_control_caps[i]->IsWritable << endl
                            << "   description: " << asi_control_caps[i]->Description << endl
                            << "   control type: " << asi_control_caps[i]->ControlType << endl;
                    }
                }
                else {
                    cout << "Error getting ZWO control caps" << endl;
                    logfile << "Error getting ZWO control caps" << endl;
                    abort_app();
                }
            }

            // Initialize camera
            cout << "Initializing camera..." << endl;
            logfile << "Initializing camera..." << endl;
            if (ASIInitCamera(asi_camera_info[cam]->CameraID) != ASI_SUCCESS) {
                cout << "Error initializing camera" << endl;
                logfile << "Error initializing camera" << endl;
                abort_app();
            }
        }

        else if (svb_connected_cameras > 0) {
            cout << "Opening SVB camera..." << endl;
            logfile << "Opening SVB camera..." << endl;

            if (SVBOpenCamera(svb_camera_info[cam]->CameraID) != SVB_SUCCESS) {
                cout << "Error opening SVB camera" << endl;
                logfile << "Error opening SVB camera" << endl;
                cout << "Press Enter to close...";
                cin.get();
                exit(1);  // return 1;
            }

            // Get camera's controls
            svb_num_controls = 0;
            SVBGetNumOfControls(svb_camera_info[cam]->CameraID, &svb_num_controls);
            if (svb_num_controls == 0) {
                cout << "Error num of controls 0" << endl;
                logfile << "Error num of controls 0" << endl;
                abort_app();
            }

            svb_control_caps = (SVB_CONTROL_CAPS**)malloc(sizeof(SVB_CONTROL_CAPS*) * svb_num_controls);
            for (int i = 0; i < svb_num_controls; i++) {
                svb_control_caps[i] = (SVB_CONTROL_CAPS*)malloc(sizeof(SVB_CONTROL_CAPS));
                if (SVBGetControlCaps(svb_camera_info[cam]->CameraID, i, svb_control_caps[i]) == SVB_SUCCESS) {
                    // Print camera's properties
                    if (debug_flag == 1) {
                        cout << "  Property " << svb_control_caps[i]->Name << ": [" << svb_control_caps[i]->MinValue << " " << svb_control_caps[i]->MaxValue
                            << "], default = " << svb_control_caps[i]->DefaultValue << endl
                            << "   is writable: " << svb_control_caps[i]->IsWritable << endl
                            << "   description: " << svb_control_caps[i]->Description << endl
                            << "   control type: " << svb_control_caps[i]->ControlType << endl;
                        logfile << "  Property " << svb_control_caps[i]->Name << ": [" << svb_control_caps[i]->MinValue << " " << svb_control_caps[i]->MaxValue
                            << "], default = " << svb_control_caps[i]->DefaultValue << endl
                            << "   is writable: " << svb_control_caps[i]->IsWritable << endl
                            << "   description: " << svb_control_caps[i]->Description << endl
                            << "   control type: " << svb_control_caps[i]->ControlType << endl;
                    }
                }
                else {
                    cout << "Error getting SVB control caps" << endl;
                    logfile << "Error getting SVB control caps" << endl;
                    abort_app();
                }
            }

            // Initialize camera
            /*
            cout << "Initializing camera..." << endl;
            logfile << "Initializing camera..." << endl;
            if (SVBInitCamera(svb_camera_info[cam]->CameraID) != SVB_SUCCESS) {
                cout << "Error initializing camera" << endl;
                logfile << "Error initializing camera" << endl;
                abort_app();
            }/**/
        }




       
    }
}



void close_camera()
{
    if (camera_from_file == 1) {
        //do nothing
        printf("Not closing camera\n");
    }
    else {
        // Close camera
        cout << "Closing camera" << endl;
        logfile << "Closing camera" << endl;

        if (asi_connected_cameras > 0)
            ASICloseCamera(asi_camera_info[cam]->CameraID);
        if (svb_connected_cameras > 0)
            SVBCloseCamera(svb_camera_info[cam]->CameraID);
    }
}



void set_camera_controls()
{
    if (state == video_state) {
        exposure_time = exposure_time_v;
        gain = gain_v;
        WB_R = WB_R_v;
        WB_G = WB_G_v;
        WB_B = WB_B_v;
        offset = offset_v;
        //bandwidth = bandwidth_v;
        //monobin = monobin_v;
        //bin = bin_v;
        //image_bytes = image_bytes_v;
    }
    else {
        exposure_time = exposure_time_f;
        gain = gain_f;
        WB_R = WB_R_f;
        WB_G = WB_G_f;
        WB_B = WB_B_f;
        offset = offset_f;
        //bandwidth = bandwidth_f;
        //monobin = monobin_f;
        //bin = bin_f;
        //image_bytes = image_bytes_f;
    }

    //if (monobin == 0) monobin_k = 1;
    //else if (monobin == 1) monobin_k = 2;
    //if (ROI_zoom == 0) ROI_zoom_k = 1;
    //else if (ROI_zoom == 1) ROI_zoom_k = 2;

    // Calculate image size (see bin!!!)
    //image_size = asi_camera_info[cam]->MaxWidth * asi_camera_info[cam]->MaxHeight / bin / bin / monobin_k / monobin_k;   
    //image_size *= image_bytes;

    //camera_image_width = asi_camera_info[cam]->MaxWidth / bin / monobin_k / ROI_zoom_k;
    //camera_image_height = asi_camera_info[cam]->MaxHeight / bin / monobin_k / ROI_zoom_k;
    //if (ROI_zoom != 0) {
    //    camera_image_width = camera_image_width / 8 * 8;
    //    camera_image_height = camera_image_height / 2 * 2;
    //}

    //image_size = camera_image_width * camera_image_height;
    //image_size *= image_bytes;

    //printf("Image size: %d bytes\n", image_size);

    //asi_image = (unsigned char*)malloc(sizeof(unsigned char) * image_size);  //used for all cameras




    if (camera_from_file == 1) {
        //do nothing
    }

    else {
        if (asi_connected_cameras > 0) {
            
            ASI_ERROR_CODE ret;

            // Set image type
            cout << "Set image type" << endl;
            logfile << "Set image type" << endl;
            if (image_bytes == 1) {
                //ASISetROIFormat(asi_camera_info[cam]->CameraID, asi_camera_info[cam]->MaxWidth / bin / monobin_k, asi_camera_info[cam]->MaxHeight / bin / monobin_k, bin, ASI_IMG_RAW8);
                ret = ASISetROIFormat(asi_camera_info[cam]->CameraID, camera_image_width, camera_image_height, bin, ASI_IMG_RAW8);
                logfile << "return code: " << ret << endl;
                //here ASISetStartPos should be used for ROI zoom
            }
            else if (image_bytes == 2) {
                //ASISetROIFormat(asi_camera_info[cam]->CameraID, asi_camera_info[cam]->MaxWidth / bin / monobin_k, asi_camera_info[cam]->MaxHeight / bin / monobin_k, bin, ASI_IMG_RAW16);
                ret = ASISetROIFormat(asi_camera_info[cam]->CameraID, camera_image_width, camera_image_height, bin, ASI_IMG_RAW16);
                logfile << "return code: " << ret << endl;
                //here ASISetStartPos should be used for ROI zoom
            }
            else
            {
                cout << "byte per pixel value wrong" << endl;
                logfile << "byte per pixel value wrong" << endl;
                cout << "Press Enter to close...";
                abort_app();
            }

            // Set exposure time
            cout << "Set exposure time, ms: " << (exposure_time / 1000) << endl;
            logfile << "Set exposure time, ms: " << (exposure_time / 1000) << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_EXPOSURE, exposure_time, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set gain
            cout << "Set gain: " << gain << endl;
            logfile << "Set gain: " << gain << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_GAIN, gain, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set WB
            cout << "Set WB_R, WB_B: " << WB_R << " " << WB_B << endl;
            logfile << "Set WB_R, WB_B: " << WB_R << " " << WB_B << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_WB_R, WB_R, ASI_FALSE);
            logfile << "return code: " << ret << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_WB_B, WB_B, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set offset
            cout << "Set offset: " << offset << endl;
            logfile << "Set offset: " << offset << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_OFFSET, offset, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set bandwidth
            cout << "Set bandwidth: " << bandwidth << endl;
            logfile << "Set bandwidth: " << bandwidth << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_BANDWIDTHOVERLOAD, bandwidth, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set speed mode
            if ((state == foto_state) || ((state == video_state) && (highspeed_v == 0))) {
                cout << "Set high speed mode: " << 0 << endl;
                logfile << "Set high speed mode: " << 0 << endl;
                ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_HIGH_SPEED_MODE, 0, ASI_FALSE);
                logfile << "return code: " << ret << endl;
            }
            else {
                cout << "Set high speed mode: " << highspeed_v << endl;
                logfile << "Set high speed mode: " << highspeed_v << endl;
                ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_HIGH_SPEED_MODE, highspeed_v, ASI_FALSE);
                logfile << "return code: " << ret << endl;
            }

            // Set monobin
            cout << "Set monobin: " << monobin << endl;
            logfile << "Set monobin: " << monobin << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_MONO_BIN, monobin, ASI_FALSE);
            logfile << "return code: " << ret << endl;

            // Set flip
            cout << "Set no flip" << endl;
            logfile << "Set no flip" << endl;
            ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_FLIP, ASI_FLIP_NONE, ASI_FALSE);
            logfile << "return code: " << ret << endl;
            // -------------------------


            if (cooler_activation == 1) {
                // Set target temperature
                cout << "Set target temperature: " << target_temperature << endl;
                logfile << "Set target temperature: " << target_temperature << endl;
                ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_TARGET_TEMP, target_temperature, ASI_FALSE);
                logfile << "return code: " << ret << endl;
                // Set cooler active
                cout << "Set cooler active" << endl;
                logfile << "Set cooler active" << endl;
                ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_COOLER_ON, 1, ASI_FALSE);
                logfile << "return code: " << ret << endl;
                // Set fan active
                cout << "Set fan active" << endl;
                logfile << "Set fan active" << endl;
                ret = ASISetControlValue(asi_camera_info[cam]->CameraID, ASI_FAN_ON, 1, ASI_FALSE);
                logfile << "return code: " << ret << endl;
            }

            // -------------------------
        }

        else if (svb_connected_cameras > 0) {

            SVB_ERROR_CODE ret;

            // Set image type
            cout << "Set image type" << endl;
            logfile << "Set image type" << endl;
            if (image_bytes == 1) {
                ret = SVBSetROIFormat(svb_camera_info[cam]->CameraID, 0, 0, camera_image_width, camera_image_height, bin);
                logfile << "return code: " << ret << endl;
                //here x,y StartPos should be used for ROI zoom
                ret = SVBSetOutputImageType(svb_camera_info[cam]->CameraID, SVB_IMG_RAW8);
                logfile << "return code: " << ret << endl;
            }
            else if (image_bytes == 2) {
                ret = SVBSetROIFormat(svb_camera_info[cam]->CameraID, 0, 0, camera_image_width, camera_image_height, bin);
                logfile << "return code: " << ret << endl;
                //here x,y StartPos should be used for ROI zoom
                ret = SVBSetOutputImageType(svb_camera_info[cam]->CameraID, SVB_IMG_RAW16);
                logfile << "return code: " << ret << endl;
            }
            else
            {
                cout << "byte per pixel value wrong" << endl;
                logfile << "byte per pixel value wrong" << endl;
                cout << "Press Enter to close...";
                abort_app();
            }
            // Set exposure time
            cout << "Set exposure time, ms: " << (exposure_time / 1000) << endl;
            logfile << "Set exposure time, ms: " << (exposure_time / 1000) << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_EXPOSURE, exposure_time, SVB_FALSE);
            logfile << "return code: " << ret << endl;

            // Set gain
            cout << "Set gain: " << gain << endl;
            logfile << "Set gain: " << gain << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_GAIN, gain, SVB_FALSE);
            logfile << "return code: " << ret << endl;

            // Set WB
            cout << "Set WB_R, WB_G, WB_B: " << WB_R << " " << WB_G << " " << WB_B << endl;
            logfile << "Set WB_R, WB_G, WB_B: " << WB_R << " " << WB_G << " " << WB_B << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_WB_R, WB_R, SVB_FALSE);
            logfile << "return code: " << ret << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_WB_G, WB_G, SVB_FALSE);
            logfile << "return code: " << ret << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_WB_B, WB_B, SVB_FALSE);
            logfile << "return code: " << ret << endl;

            // Set offset
            cout << "Set offset: " << offset << endl;
            logfile << "Set offset: " << offset << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_BLACK_LEVEL, offset, SVB_FALSE);
            logfile << "return code: " << ret << endl;
                        
            // Set flip
            cout << "Set no flip" << endl;
            logfile << "Set no flip" << endl;
            ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_FLIP, SVB_FLIP_NONE, SVB_FALSE);
            logfile << "return code: " << ret << endl;
            // -------------------------


            if (cooler_activation == 1) {
                // Set target temperature
                cout << "Set target temperature: " << target_temperature << endl;
                logfile << "Set target temperature: " << target_temperature << endl;
                ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_TARGET_TEMPERATURE, target_temperature*10, SVB_FALSE);
                logfile << "return code: " << ret << endl;
                // Set cooler active
                cout << "Set cooler active" << endl;
                logfile << "Set cooler active" << endl;
                ret = SVBSetControlValue(svb_camera_info[cam]->CameraID, SVB_COOLER_ENABLE, 1, SVB_FALSE);
                logfile << "return code: " << ret << endl;
            }

            cout << "Set camera mode" << endl;
            logfile << "Set camera mode" << endl;
            ret = SVBSetCameraMode(svb_camera_info[cam]->CameraID, SVB_MODE_NORMAL);
            logfile << "return code: " << ret << endl;

            // -------------------------
        }
    }
}



void  start_video()
{
    if ((camera_from_file == 1)) {
        //reading image file here   !!!!!!!!!!!!!!!!!!!!
        /**/
        ifstream myfile;
        
        myfile.open("frame_v.fits", ios::in | ios::binary);

        if (myfile.is_open()) {
            printf("Reading frame_v.fits...\n");
            myfile.seekg(2880, ios::beg);
            myfile.read((char*)asi_image, image_size);
            myfile.close();

            int16_t* p = (int16_t*)asi_image;
            uint16_t* p2 = (uint16_t*)asi_image;

            for (long i = 0; i < (image_size / 2); i++) {
                unsigned char t = asi_image[i * 2];
                asi_image[i * 2] = asi_image[i * 2 + 1];
                asi_image[i * 2 + 1] = t;
                p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
            }            
        }
        else {
            printf("Couldn't find file frame_v.fits\n");
            cout << "Press Enter to close...";
            cin.get();
            exit(1); // return 1;
        }
        /**/
    }
    else {
        cout << "Start video..." << endl;
        logfile << "Start video..." << endl;

        if (asi_connected_cameras > 0) {
            if (ASIStartVideoCapture(asi_camera_info[cam]->CameraID) != ASI_SUCCESS) {
                cout << "Cannot start ZWO video." << endl;
                logfile << "Cannot start ZWO video." << endl;
                abort_app();
            }
        }

        else if (svb_connected_cameras > 0) {
            if (SVBStartVideoCapture(svb_camera_info[cam]->CameraID) != SVB_SUCCESS) {
                cout << "Cannot start SVB video." << endl;
                logfile << "Cannot start SVB video." << endl;
                abort_app();
            }
        }

    }
}



int get_video_frame()
{
    int get_frame_success = 0;

    if (debug_flag == 1) {
        cout << "Get video frame..." << endl;
        logfile << "Get video frame..." << endl;
    }

    if (camera_from_file == 1) {
        //do nothing
        std::this_thread::sleep_for(std::chrono::microseconds(exposure_time));
    }
    else {

        if (asi_connected_cameras > 0) {
            if (state == video_state) {
                if (ASIGetVideoData(asi_camera_info[cam]->CameraID, asi_image, image_size, (exposure_time / 1000 * 2 + 500)) != ASI_SUCCESS) {
                    cout << "Cannot get ZWO video frame." << endl;
                    logfile << "Cannot get ZWO video frame." << endl;
                    abort_app();
                }
                get_frame_success = 1;
            }
            else {
                //int ret = ASIGetVideoData(asi_camera_info[cam]->CameraID, asi_image, image_size, 500);
                if (ASIGetVideoData(asi_camera_info[cam]->CameraID, asi_image, image_size, 500) != ASI_SUCCESS)
                //if (ret != 0)
                    get_frame_success = 0;
                else
                    get_frame_success = 1;

                //cout << "return code: " << ret << endl;
            }
        }

        else if (svb_connected_cameras > 0) {
            if (state == video_state) {
                if (SVBGetVideoData(svb_camera_info[cam]->CameraID, asi_image, image_size, (exposure_time / 1000 * 2 + 500)) != SVB_SUCCESS) {
                    cout << "Cannot get SVB video frame." << endl;
                    logfile << "Cannot get SVB video frame." << endl;
                    abort_app();
                }
                get_frame_success = 1;
            }
            else {
                if (SVBGetVideoData(svb_camera_info[cam]->CameraID, asi_image, image_size, 500) != SVB_SUCCESS)
                    get_frame_success = 0;
                else
                    get_frame_success = 1;
            }
        }

    }

    return get_frame_success;
}



void stop_video()
{
    if (camera_from_file == 1) {
        //do nothing
    }
    else {
        cout << "Stop video..." << endl;
        logfile << "Stop video..." << endl;

        if (asi_connected_cameras > 0) {
            if (ASIStopVideoCapture(asi_camera_info[cam]->CameraID) != ASI_SUCCESS) {
                cout << "Cannot stop ZWO video." << endl;
                logfile << "Cannot stop ZWO video." << endl;
                abort_app();
            }
        }

        else if (svb_connected_cameras > 0) {
            if (SVBStopVideoCapture(svb_camera_info[cam]->CameraID) != SVB_SUCCESS) {
                cout << "Cannot stop SVB video." << endl;
                logfile << "Cannot stop SVB video." << endl;
                abort_app();
            }
        }

    }
}



void start_exposure()
{
    if ((camera_from_file == 1)) {
        //reading image file here   !!!!!!!!!!!!!!!!!!!!
        ifstream myfile;

        if (state == foto_state) {

            //string filename = "frame_f.fits"
            string filename = "frame_f" + to_string(frames_stacked % 5) + ".fits";
            myfile.open(filename, ios::in | ios::binary);

            if (myfile.is_open()) {
                printf("Reading file %s...\n", filename);
                myfile.seekg(2880, ios::beg);
                myfile.read((char*)asi_image, image_size);
                myfile.close();

                int16_t* p = (int16_t*)asi_image;
                uint16_t* p2 = (uint16_t*)asi_image;

                for (long i = 0; i < (image_size / 2); i++) {
                    unsigned char t = asi_image[i * 2];
                    asi_image[i * 2] = asi_image[i * 2 + 1];
                    asi_image[i * 2 + 1] = t;
                    p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
                }

                //std::this_thread::sleep_for(std::chrono::microseconds(exposure_time_f));
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            else {
                printf("Couldn't find file %s\n", filename);
                cout << "Press Enter to close...";
                cin.get();
                exit(1); // return 1;
            }
        }
        else {
            myfile.open("frame_v.fits", ios::in | ios::binary);

            if (myfile.is_open()) {
                printf("Reading frame_v.fits...\n");
                myfile.seekg(2880, ios::beg);
                myfile.read((char*)asi_image, image_size);
                myfile.close();

                int16_t* p = (int16_t*)asi_image;
                uint16_t* p2 = (uint16_t*)asi_image;

                for (long i = 0; i < (image_size / 2); i++) {
                    unsigned char t = asi_image[i * 2];
                    asi_image[i * 2] = asi_image[i * 2 + 1];
                    asi_image[i * 2 + 1] = t;
                    p2[i] = (uint16_t)((int32_t)p[i] + 32768);   // see how unsigned 16 bit is stored as signed + offset in FITS file format
                }

                //std::this_thread::sleep_for(std::chrono::microseconds(exposure_time_v));
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
            }
            else {
                printf("Couldn't find file frame_v.fits\n");
                cout << "Press Enter to close...";
                cin.get();
                exit(1); // return 1;
            }
        }
    }
    else {
        // Start exposure
        cout << "Start exposure..." << endl;
        logfile << "Start exposure..." << endl;

        if (ASIStartExposure(asi_camera_info[cam]->CameraID, ASI_FALSE) != ASI_SUCCESS) {
            cout << "Cannot start exposure." << endl;
            logfile << "Cannot start exposure." << endl;
            abort_app();
        }
    }
}



void stop_exposure()
{
    if (camera_from_file == 1) {
        //do nothing
    }
    else {
        // Start exposure
        cout << "Stop exposure..." << endl;
        logfile << "Stop exposure..." << endl;
        if (ASIStopExposure(asi_camera_info[cam]->CameraID) != ASI_SUCCESS) {
            cout << "Cannot stop exposure." << endl;
            logfile << "Cannot stop exposure." << endl;
            abort_app();
        }
    }
}



int exposure_status()
{
    if (camera_from_file == 1) {
        return 1;  // allways ready
    }
    else {
        ASI_EXPOSURE_STATUS asi_exp_status;
        ASIGetExpStatus(asi_camera_info[cam]->CameraID, &asi_exp_status);
        if (asi_exp_status == ASI_EXP_SUCCESS) {
            cout << "Successful exposure" << endl;
            logfile << "Successful exposure" << endl;

            return 1;//break;
        }
        else if (asi_exp_status == ASI_EXP_FAILED) {
            cout << "Failed exposure" << endl;
            logfile << "Failed exposure" << endl;
            abort_app();
        }
        else if (asi_exp_status == ASI_EXP_WORKING) {
            return 0;
        }
    }
}



void get_foto_frame()
{
    if (camera_from_file == 1) {
        //do nothing
    }
    else {
        if (debug_flag == 1) {
            cout << "Get foto frame" << endl;
            logfile << "Get foto frame" << endl;
        }

        if (ASIGetDataAfterExp(asi_camera_info[cam]->CameraID, asi_image, image_size) != ASI_SUCCESS) {
            cout << "Couldn't read exposure data" << endl;
            logfile << "Couldn't read exposure data" << endl;
            abort_app();
        }
    }
}



void wait_idle()
{
    if (camera_from_file == 1) {
        //do nothing
    }
    else {
        cout << "Waiting for camera idle state..." << endl;
        logfile << "Waiting for camera idle state..." << endl;
        ASI_EXPOSURE_STATUS asi_exp_status;
        ASIGetExpStatus(asi_camera_info[cam]->CameraID, &asi_exp_status);
        while (asi_exp_status != ASI_EXP_IDLE) {
            ASIGetExpStatus(asi_camera_info[cam]->CameraID, &asi_exp_status);
        }
    }
}



void get_config(char* filename)
{
    //printf("get config start\n");

    debug_flag = 0;

    auto_save_pictures = 0;
    auto_save_pictures_n = 1;

    //-------------------Default Video Parameters
    exposure_time_v = 400000; // us
    gain_v = 600;
    WB_R_v = 50;
    WB_G_v = 50;
    WB_B_v = 50;
    offset_v = 100; 
    highspeed_v = 1;
    dark_v_hotpixel_flag = 0;
    dark_v_subtract_flag = 0;
    flat_v_flag = 0;

    banding_filter_flag = 0;
    banding_filter_strength = 100;
    banding_filter_threshold = 1.2;
    
    //-------------------Default Foto Parameters
    exposure_time_f = 4000000; // us
    gain_f = 600;
    WB_R_f = 50;
    WB_G_f = 50;
    WB_B_f = 50;
    offset_f = 100;
    dark_f_hotpixel_flag = 0;
    add_hotpixel_flag_f = 0;
    dark_f_subtract_flag = 0;
    flat_f_flag = 0;

    //--------------- Default file names
    // see definitions of dark_v_filename, dark_f_filename, flat_filename
    //dark_v_filename = "dark_v.fits";
    //dark_f_filename = "dark_f.fits";
    //flat_filename = "flat.fits";

    //-------------------Default Video and Foto Parameters
    monobin = 0;
    bin = 2;
    image_bytes = 2;  // 1 for RAW8, 2 for RAW16
    bandwidth = 100;

    hot_pixel_sigma = 7.0;

    flat_inv_factor = 0.0;
    ROI_zoom = 0;

    //-------------------Other Parameters
    target_temperature = 10;
    cooler_activation = 0;

    display_height = 750;

    image_flip = 0;
    image_rotation = 0;

    background_comp_flag = 2;
    black_level_value_v = 0.1;
    black_level_value_f = 0.1;
    black_point_offset = 0.01;


    circular_mask_background_flag = 1;
    circular_mask_background_size = 1.0;
    circular_mask_background_show = 0;

    noise_reduction_flag = 0;
    filter_strength_1 = 0.3;
    filter_strength_2 = 0.2;

    midtone_radius = 20;
    midtone_width = 0.2;
    midtone_strength = 0.0;

    sharpen_sigma = 2.0;
    sharpen_amount = 0.5;

    circular_mask_flag = 1;
    init_gamma = 15.0;
    star_protection_factor = 1.0;

    WBcorr_R = 1.0;   // WB correction for RGB palette
    WBcorr_G = 1.0;
    WBcorr_B = 1.0;
    
    aR = 1.0; bR = 0.0; cR = 0.0;  //dual band colors for R
    aG = 0.8; bG = 0.0; cG = 0.1;  //dual band colors for G
    aB = 0.0; bB = 0.4; cB = 0.4;  //dual band colors for B
    

    enhance_stars_flag = 0;
    star_blob_radius = 20;
    star_blob_strength = 0.2;

    highlight_protection_par = 0.4;

    focusing_zoom_value = 4.0;
    zoom_value = 1.5;

    display_zoom_value = 1.0;
    display_zoom_value_stored = 1.0;

    key_exit = (int)'x';   // exit
    key_mode = (int)'m';   //mode change foto, video
    key_plus = (int)'+';   //gain +
    key_minus = (int)'-';   //gain -
    key_palette = (int)'p';   //palette change foto, video
    key_save_image = (int)'s';   //save images
    key_focusing = (int)'f';   //focusing zoom
    key_histogram = (int)'h';   //show histogram

    main_display_flag = 1;

    GUI_flag = 1;

    show_clock_flag = 0;

    //--------------- AI noise reduction

    AI_noise_factor = 0;

    AI_noise_min = 0.01;
    AI_noise_max = 0.04;
    AI_noise_factor_min = 0.1;
    AI_noise_factor_max = 0.95;
    
    AI_noise_frames = 1;
    
    //AI_noise_model_filename = "train.jason";

    AI_num_threads = 0;

    //-------------------Eyepiece display

    eyepiece_display_flag = 0;

    eyepiece_display_X_pixels = 2560;
    eyepiece_display_Y_pixels = 1440;

    eyepiece_display_X_mm = 121;
    eyepiece_display_Y_mm = 68;

    interpupillary_distance_mm = 60;

    eyepiece_display_rotation = 3;

    second_display_X = 2500;
    second_display_Y = 500;

    circular_mask_eyepiece_flag = 1;



    //-----------Try get parameters from config file
    string line;
    ifstream myfile;
    stringstream iss;

    //myfile.open("config.txt");
    myfile.open(filename);
    

    if (myfile.is_open()) {
        //cout << "Reading config.txt..." << endl;
        //logfile << "Reading config.txt..." << endl;
        cout << "Reading config from " << filename << "..." << endl;
        logfile << "Reading config from " << filename << "..." << endl;


        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> debug_flag;
        iss.str("");


        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> auto_save_pictures >> auto_save_pictures_n;
        iss.str("");


        getline(myfile, line); //dummy line //Camera and preprocessing Parameters for video mode
        iss.str("");


        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> exposure_time_v;
        exposure_time_v *= 1000; //ms -> µs
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> gain_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_R_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_G_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_B_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> offset_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> highspeed_v;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_v_hotpixel_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_v_subtract_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> flat_v_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> banding_filter_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> banding_filter_strength;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> banding_filter_threshold;
        iss.str("");



        getline(myfile, line); //dummy line //Camera and preprocessing Parameters for foto mode
        iss.str("");



        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> exposure_time_f;
        exposure_time_f *= 1000; //ms -> µs
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> gain_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_R_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_G_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WB_B_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> offset_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_f_hotpixel_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> add_hotpixel_flag_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_f_subtract_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> flat_f_flag;
        iss.str("");



        getline(myfile, line); //dummy line //Calibration files
        iss.str("");



        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_v_filename;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> dark_f_filename;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> flat_filename;
        iss.str("");



        getline(myfile, line); //dummy line //Parameters for both video and foto mode
        iss.str("");

                

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> monobin;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> bin;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> image_bytes;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> bandwidth;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> hot_pixel_sigma;
        iss.str("");

        /*
        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> ROI_zoom;
        iss.str("");
        /**/

        /*
        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> flat_inv_factor;
        iss.str("");
        /**/

        getline(myfile, line); //dummy line //Other parameters
        iss.str("");



        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> target_temperature;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> cooler_activation;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> display_height;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> image_flip;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> image_rotation;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> background_comp_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> black_level_value_v >> black_level_value_f;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> black_point_offset;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_background_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_background_size;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_background_show;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> noise_reduction_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> filter_strength_1 >> filter_strength_2;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> midtone_radius >> midtone_width >> midtone_strength;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> sharpen_sigma >> sharpen_amount;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> init_gamma;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> star_protection_factor;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> WBcorr_R >> WBcorr_G >> WBcorr_B;  //white balance correction
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> color_correction_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> CC11 >> CC12 >> CC13;  //dual band colors for R
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> CC21 >> CC22 >> CC23;  //dual band colors for G
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> CC31 >> CC32 >> CC33;  //dual band colors for B
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> aR >> bR >> cR;  //dual band colors for R
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> aG >> bG >> cG;  //dual band colors for G
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> aB >> bB >> cB;  //dual band colors for B
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> enhance_stars_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> star_blob_radius >> star_blob_strength;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> highlight_protection_par;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> focusing_zoom_value >> zoom_value;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> display_zoom_value;
        iss.str("");
        display_zoom_value_stored = display_zoom_value;

        char c1, c2, c3, c4, c5, c6, c7, c8;
        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> c1 >> c2 >> c3 >> c4 >> c5 >> c6 >> c7 >> c8;
        key_exit = (int)c1;         //exit
        key_mode = (int)c2;         //mode change foto, video
        key_plus = (int)c3;         //gain +
        key_minus = (int)c4;        //gain -
        key_palette = (int)c5;      //palette change foto, video
        key_save_image = (int)c6;   //save images
        key_focusing = (int)c7;     //focusing zoom
        key_histogram = (int)c8;     //show histogram
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> main_display_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> GUI_flag;
        iss.str("");


        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> show_clock_flag;
        iss.str("");

        


        getline(myfile, line); //dummy line //AI noise reduction
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> AI_noise_factor;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> AI_noise_min >> AI_noise_max >> AI_noise_factor_min >> AI_noise_factor_max;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> AI_noise_frames;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> AI_noise_model_filename;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> AI_num_threads;
        iss.str("");



        getline(myfile, line); //dummy line //Eyepiece display
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> eyepiece_display_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> eyepiece_display_X_pixels >> eyepiece_display_Y_pixels;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> eyepiece_display_X_mm >> eyepiece_display_Y_mm;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> interpupillary_distance_mm;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> eyepiece_display_rotation;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> second_display_X >> second_display_Y;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_eyepiece_flag;
        iss.str("");







        myfile.close();
        //cout << "Reading config.txt done" << endl;
        //logfile << "Reading config.txt done" << endl;
        cout << "Reading config from " << filename << " done" << endl;
        logfile << "Reading config from " << filename << " done" << endl;

    }
    else {
        //cout<< "Couldn't find config.txt" << endl;
        //logfile << "Couldn't find config.txt" << endl;
        cout << "Couldn't find " << filename << endl;
        logfile << "Couldn't find " << filename << endl;
        cout << "Press Enter to close...";
        cin.get();
        exit(1);
    }

    //printf("get config done\n");
    //printf("exp time v: %ld\n", exposure_time_v);
    //printf("gain v: %ld\n", gain_v);
    //printf("rotation: %d\n", image_rotation);
}

