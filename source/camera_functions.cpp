// Copyright 2026, Andriy Melnykov
// https://github.com/andriymelnykov/Digital_Eyepiece_APP
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)

#include "camera_functions.h"

using namespace std;

int frame_load_number = 0;

static string trim_config_line(const string& value)
{
    size_t first = value.find_first_not_of(" \t\r\n");
    if (first == string::npos)
        return "";
    size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

static string strip_config_comment(const string& value)
{
    size_t comment_pos = value.find("//");
    if (comment_pos == string::npos)
        return value;
    return value.substr(0, comment_pos);
}

static string config_first_token(const string& value)
{
    istringstream token_stream(trim_config_line(strip_config_comment(value)));
    string token;
    token_stream >> token;
    return token;
}

static bool parse_palette_float_line(ifstream& file, float& v1)
{
    string line;
    if (!getline(file, line))
        return false;
    istringstream value_stream(strip_config_comment(line));
    return (value_stream >> v1) ? true : false;
}

static bool parse_palette_float_line(ifstream& file, float& v1, float& v2, float& v3)
{
    string line;
    if (!getline(file, line))
        return false;
    istringstream value_stream(strip_config_comment(line));
    return (value_stream >> v1 >> v2 >> v3) ? true : false;
}

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
        svb_connected_cameras = 0;
        toup_connected_cameras = 0;
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

        toup_connected_cameras = static_cast<int>(Toupcam_EnumV2(NULL));
        cout << "Number of connected ToupTek cameras: " << toup_connected_cameras << endl;
        logfile << "Number of connected ToupTek cameras: " << toup_connected_cameras << endl;

        if ( (asi_connected_cameras < 1) && (svb_connected_cameras < 1) && (toup_connected_cameras < 1) ) {
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
    toup_raw_fourcc = 0;
    toup_bits_per_pixel = 0;

    if (camera_from_file == 1) {
        asi_camera_info = (ASI_CAMERA_INFO**)malloc(sizeof(ASI_CAMERA_INFO*) * asi_connected_cameras);
        asi_camera_info[0] = (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));
        asi_camera_info[0]->MaxWidth = 3008;
        asi_camera_info[0]->MaxHeight = 3008;
        cout << "Using image file instead of camera. Properties set: " << asi_camera_info[0]->MaxWidth << asi_camera_info[0]->MaxHeight << endl;
    }
    else {
        // Get each connected camera's properties into an array. Vendor priority is ASI, SVBony, then ToupTek.
        int get_property_success = 0;

        if (asi_connected_cameras > 0) {
            asi_camera_info = (ASI_CAMERA_INFO**)malloc(sizeof(ASI_CAMERA_INFO*) * asi_connected_cameras);
            for (int i = 0; i < asi_connected_cameras; i++) {
                asi_camera_info[i] = (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));
                ASI_ERROR_CODE ret = ASIGetCameraProperty(asi_camera_info[i], i);
                if (ret == ASI_SUCCESS) {
                    get_property_success = 1;

                    if (i == cam) {
                        if (asi_camera_info[i]->IsColorCam == ASI_TRUE)
                            is_color_cam = true;
                        else
                            is_color_cam = false;

                        bayer_pattern = asi_camera_info[i]->BayerPattern;
                    }

                    if (debug_flag == 1) {
                        // Print camera's properties
                        cout << "ZWO Camera " << i << endl;
                        cout << "  ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                        cout << "  Camera ID: " << asi_camera_info[i]->CameraID << endl;
                        cout << "  Width and Height: " << asi_camera_info[i]->MaxWidth << "x" << asi_camera_info[i]->MaxHeight << endl;
                        cout << "  Color: " << (asi_camera_info[i]->IsColorCam == ASI_TRUE ? "Yes" : "No") << endl;
                        cout << "  Bayer pattern: " << asi_camera_info[i]->BayerPattern << endl;
                        cout << "  Pixel size: " << asi_camera_info[i]->PixelSize << " um" << endl;
                        cout << "  Bit depth: " << asi_camera_info[i]->BitDepth << endl;
                        cout << "  Trigger cam: " << (asi_camera_info[i]->IsTriggerCam == 0 ? "No" : "Yes") << endl;

                        logfile << "Camera " << i << endl;
                        logfile << "  ASI Camera Name: " << asi_camera_info[i]->Name << endl;
                        logfile << "  Camera ID: " << asi_camera_info[i]->CameraID << endl;
                        logfile << "  Width and Height: " << asi_camera_info[i]->MaxWidth << "x" << asi_camera_info[i]->MaxHeight << endl;
                        logfile << "  Color: " << (asi_camera_info[i]->IsColorCam == ASI_TRUE ? "Yes" : "No") << endl;
                        logfile << "  Bayer pattern: " << asi_camera_info[i]->BayerPattern << endl;
                        logfile << "  Pixel size: " << asi_camera_info[i]->PixelSize << " um" << endl;
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
        else if (svb_connected_cameras > 0) {

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
                SVB_ERROR_CODE ret_property = SVBGetCameraProperty(svb_camera_info[i]->CameraID, svb_camera_property[i]);
                if (ret_property == SVB_SUCCESS) {
                    get_property_success = 1;

                    if (i == cam) {
                        if (svb_camera_property[i]->IsColorCam == SVB_TRUE)
                            is_color_cam = true;
                        else
                            is_color_cam = false;

                        bayer_pattern = svb_camera_property[i]->BayerPattern;
                    }

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
        else if (toup_connected_cameras > 0) {
            unsigned cnt = Toupcam_EnumV2(toup_camera_info);
            toup_connected_cameras = static_cast<int>(cnt);

            for (int i = 0; i < toup_connected_cameras; i++) {
                if (toup_camera_info[i].model == NULL) {
                    cout << "Can not get info from ToupTek camera: " << i << endl;
                    logfile << "Can not get info from ToupTek camera: " << i << endl;
                    continue;
                }

                bool toup_is_color = ((toup_camera_info[i].model->flag & TOUPCAM_FLAG_MONO) == 0);
                unsigned fourcc = 0;
                unsigned bits_per_pixel = 0;
                int camera_bayer_pattern = 0;

                HToupcam hcam = Toupcam_Open(toup_camera_info[i].id);
                if (hcam == NULL) {
                    cout << "Can not open ToupTek camera: " << i << endl;
                    logfile << "Can not open ToupTek camera: " << i << endl;
                    cout << "Press Enter to close...";
                    cin.get();
                    exit(1); // return 1;
                }

                HRESULT ret_raw = Toupcam_get_RawFormat(hcam, &fourcc, &bits_per_pixel);
                int max_bit_depth = Toupcam_get_MaxBitDepth(hcam);
                Toupcam_Close(hcam);

                if (FAILED(ret_raw)) {
                    cout << "Can not get raw format from ToupTek camera: " << i << endl;
                    logfile << "Can not get raw format from ToupTek camera: " << i << endl;
                }
                else {
                    if (fourcc == MAKEFOURCC('Y', 'Y', 'Y', 'Y')) {
                        toup_is_color = false;
                    }
                    else if (fourcc == MAKEFOURCC('R', 'G', 'G', 'B')) {
                        camera_bayer_pattern = 0;
                    }
                    else if (fourcc == MAKEFOURCC('B', 'G', 'G', 'R')) {
                        camera_bayer_pattern = 1;
                    }
                    else if (fourcc == MAKEFOURCC('G', 'R', 'B', 'G')) {
                        camera_bayer_pattern = 2;
                    }
                    else if (fourcc == MAKEFOURCC('G', 'B', 'R', 'G')) {
                        camera_bayer_pattern = 3;
                    }
                    else {
                        camera_bayer_pattern = 0;
                    }
                }

                if (i == cam) {
                    get_property_success = 1;
                    is_color_cam = toup_is_color;
                    bayer_pattern = camera_bayer_pattern;
                    toup_raw_fourcc = fourcc;
                    toup_bits_per_pixel = bits_per_pixel;
                }

                if (debug_flag == 1) {
                    char fourcc_str[5] = {
                        static_cast<char>(fourcc & 0xff),
                        static_cast<char>((fourcc >> 8) & 0xff),
                        static_cast<char>((fourcc >> 16) & 0xff),
                        static_cast<char>((fourcc >> 24) & 0xff),
                        0
                    };

                    cout << "ToupTek Camera " << i << endl;
#ifdef _WIN32
                    wcout << L"  ToupTek Camera Name: " << toup_camera_info[i].displayname << endl;
                    wcout << L"  ToupTek Model Name: " << toup_camera_info[i].model->name << endl;
#else
                    cout << "  ToupTek Camera Name: " << toup_camera_info[i].displayname << endl;
                    cout << "  ToupTek Model Name: " << toup_camera_info[i].model->name << endl;
#endif
                    cout << "  Width and Height: " << toup_camera_info[i].model->res[0].width << "x" << toup_camera_info[i].model->res[0].height << endl;
                    cout << "  Color: " << (toup_is_color ? "Yes" : "No") << endl;
                    cout << "  Bayer pattern: " << camera_bayer_pattern << endl;
                    cout << "  Raw format: " << fourcc_str << endl;
                    cout << "  Raw bit depth: " << bits_per_pixel << endl;
                    cout << "  Max bit depth: " << max_bit_depth << endl;
                    cout << "  Pixel size: " << toup_camera_info[i].model->xpixsz << "x" << toup_camera_info[i].model->ypixsz << " um" << endl;
                    cout << "  Trigger cam: " << ((toup_camera_info[i].model->flag & (TOUPCAM_FLAG_TRIGGER_SOFTWARE | TOUPCAM_FLAG_TRIGGER_EXTERNAL)) ? "Yes" : "No") << endl;

                    logfile << "ToupTek Camera " << i << endl;
                    logfile << "  Width and Height: " << toup_camera_info[i].model->res[0].width << "x" << toup_camera_info[i].model->res[0].height << endl;
                    logfile << "  Color: " << (toup_is_color ? "Yes" : "No") << endl;
                    logfile << "  Bayer pattern: " << camera_bayer_pattern << endl;
                    logfile << "  Raw format: " << fourcc_str << endl;
                    logfile << "  Raw bit depth: " << bits_per_pixel << endl;
                    logfile << "  Max bit depth: " << max_bit_depth << endl;
                    logfile << "  Pixel size: " << toup_camera_info[i].model->xpixsz << "x" << toup_camera_info[i].model->ypixsz << " um" << endl;
                    logfile << "  Trigger cam: " << ((toup_camera_info[i].model->flag & (TOUPCAM_FLAG_TRIGGER_SOFTWARE | TOUPCAM_FLAG_TRIGGER_EXTERNAL)) ? "Yes" : "No") << endl;
                }
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

    if (asi_connected_cameras > 0) {
        camera_image_width = asi_camera_info[cam]->MaxWidth / bin / monobin_k / ROI_zoom_k;
        camera_image_height = asi_camera_info[cam]->MaxHeight / bin / monobin_k / ROI_zoom_k;
    }
    else if (svb_connected_cameras > 0) {
        camera_image_width = svb_camera_property[cam]->MaxWidth / bin / monobin_k / ROI_zoom_k;
        camera_image_height = svb_camera_property[cam]->MaxHeight / bin / monobin_k / ROI_zoom_k;
    }
    else if (toup_connected_cameras > 0) {
        camera_image_width = static_cast<int>(toup_camera_info[cam].model->res[0].width) / bin / monobin_k / ROI_zoom_k;
        camera_image_height = static_cast<int>(toup_camera_info[cam].model->res[0].height) / bin / monobin_k / ROI_zoom_k;
    }
    
        
    if (ROI_zoom != 0) {
        if (toup_connected_cameras > 0) {
            camera_image_width = camera_image_width / 2 * 2;
            camera_image_height = camera_image_height / 2 * 2;
        }
        else {
            camera_image_width = camera_image_width / 8 * 8;
            camera_image_height = camera_image_height / 2 * 2;
        }
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
        frame_load_number = 0;
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

        }

        else if (toup_connected_cameras > 0) {
            cout << "Opening ToupTek camera..." << endl;
            logfile << "Opening ToupTek camera..." << endl;

            toup_handle = Toupcam_Open(toup_camera_info[cam].id);
            if (toup_handle == NULL) {
                cout << "Error opening ToupTek camera" << endl;
                logfile << "Error opening ToupTek camera" << endl;
                cout << "Press Enter to close...";
                cin.get();
                exit(1);  // return 1;
            }

            HRESULT ret;

            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_RAW, 1);
            if (FAILED(ret)) {
                cout << "Error setting ToupTek RAW mode" << endl;
                logfile << "Error setting ToupTek RAW mode" << endl;
                abort_app();
            }

            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BITDEPTH, (image_bytes == 2) ? 1 : 0);
            if (FAILED(ret)) {
                cout << "Error setting ToupTek bit depth" << endl;
                logfile << "Error setting ToupTek bit depth" << endl;
                abort_app();
            }

            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_TRIGGER, 0);
            if (FAILED(ret)) {
                cout << "Error setting ToupTek video trigger mode" << endl;
                logfile << "Error setting ToupTek video trigger mode" << endl;
                abort_app();
            }

            if (debug_flag == 1) {
                unsigned exp_min = 0, exp_max = 0, exp_def = 0;
                unsigned short gain_min = 0, gain_max = 0, gain_def = 0;

                if (SUCCEEDED(Toupcam_get_ExpTimeRange(toup_handle, &exp_min, &exp_max, &exp_def))) {
                    cout << "  Exposure range: [" << exp_min << " " << exp_max << "], default = " << exp_def << endl;
                    logfile << "  Exposure range: [" << exp_min << " " << exp_max << "], default = " << exp_def << endl;
                }

                if (SUCCEEDED(Toupcam_get_ExpoAGainRange(toup_handle, &gain_min, &gain_max, &gain_def))) {
                    cout << "  Gain range: [" << gain_min << " " << gain_max << "], default = " << gain_def << endl;
                    logfile << "  Gain range: [" << gain_min << " " << gain_max << "], default = " << gain_def << endl;
                }

                HRESULT fan_max = Toupcam_get_FanMaxSpeed(toup_handle);
                if (SUCCEEDED(fan_max)) {
                    cout << "  Fan speed range: [0 " << fan_max << "]" << endl;
                    logfile << "  Fan speed range: [0 " << fan_max << "]" << endl;
                }
                else {
                    cout << "  Fan speed range: not supported, return code = " << fan_max << endl;
                    logfile << "  Fan speed range: not supported, return code = " << fan_max << endl;
                }

                if (toup_camera_info[cam].model != NULL) {
                    cout << "  Model max fan speed: " << toup_camera_info[cam].model->maxfanspeed << endl;
                    logfile << "  Model max fan speed: " << toup_camera_info[cam].model->maxfanspeed << endl;
                }
            }
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
        else if (svb_connected_cameras > 0)
            SVBCloseCamera(svb_camera_info[cam]->CameraID);
        else if ((toup_connected_cameras > 0) && (toup_handle != NULL)) {
            Toupcam_Close(toup_handle);
            toup_handle = NULL;
        }
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
        frame_load_number = 0;
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
                //if (ROI_zoom == 1) {
                //    ret = ASISetStartPos(asi_camera_info[cam]->CameraID, camera_image_width, camera_image_height);
                //}
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

        else if (toup_connected_cameras > 0) {

            HRESULT ret;

            if (toup_handle == NULL) {
                cout << "ToupTek camera handle is NULL" << endl;
                logfile << "ToupTek camera handle is NULL" << endl;
                abort_app();
            }

            // Set image type
            cout << "Set ToupTek image type" << endl;
            logfile << "Set ToupTek image type" << endl;
            if ((image_bytes != 1) && (image_bytes != 2)) {
                cout << "byte per pixel value wrong" << endl;
                logfile << "byte per pixel value wrong" << endl;
                cout << "Press Enter to close...";
                abort_app();
            }

            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_RAW, 1); // pure raw sensor data
            logfile << "return code RAW: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek RAW mode" << endl;
                logfile << "Error setting ToupTek RAW mode" << endl;
                abort_app();
            }

            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BITDEPTH, (image_bytes == 2) ? 1 : 0);
            logfile << "return code BITDEPTH: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek bit depth" << endl;
                logfile << "Error setting ToupTek bit depth" << endl;
                abort_app();
            }

            if (image_bytes == 2) {
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_ZERO_PADDING, 1); // low-order padding shifts sensor data toward the full 16-bit range
                logfile << "return code ZERO_PADDING: " << ret << endl;
            }

            unsigned active_fourcc = 0;
            unsigned active_bits_per_pixel = 0;
            ret = Toupcam_get_RawFormat(toup_handle, &active_fourcc, &active_bits_per_pixel);
            if (SUCCEEDED(ret)) {
                char active_fourcc_str[5] = {
                    static_cast<char>(active_fourcc & 0xff),
                    static_cast<char>((active_fourcc >> 8) & 0xff),
                    static_cast<char>((active_fourcc >> 16) & 0xff),
                    static_cast<char>((active_fourcc >> 24) & 0xff),
                    0
                };
                toup_raw_fourcc = active_fourcc;
                toup_bits_per_pixel = active_bits_per_pixel;
                cout << "ToupTek active raw format: " << active_fourcc_str << ", bit depth: " << active_bits_per_pixel << endl;
                logfile << "ToupTek active raw format: " << active_fourcc_str << ", bit depth: " << active_bits_per_pixel << endl;
            }
            else {
                logfile << "return code RAW_FORMAT_AFTER_BITDEPTH: " << ret << endl;
            }

            int toup_bin = (bin > 1) ? (0x80 | bin) : bin; // average binning for ToupTek
            cout << "Set ToupTek bin: " << bin << ", option: " << toup_bin << endl;
            logfile << "Set ToupTek bin: " << bin << ", option: " << toup_bin << endl;
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BINNING, toup_bin);
            logfile << "return code BINNING: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek binning" << endl;
                logfile << "Error setting ToupTek binning" << endl;
                abort_app();
            }

            cout << "Set ToupTek ROI" << endl;
            logfile << "Set ToupTek ROI" << endl;
            if (ROI_zoom == 0) {
                ret = Toupcam_put_Roi(toup_handle, 0, 0, 0, 0); // clear ROI, full sensor
            }
            else {
                unsigned sensor_width = toup_camera_info[cam].model->res[0].width;
                unsigned sensor_height = toup_camera_info[cam].model->res[0].height;
                unsigned roi_width = sensor_width / ROI_zoom_k;
                unsigned roi_height = sensor_height / ROI_zoom_k;

                roi_width = (roi_width / 2) * 2;
                roi_height = (roi_height / 2) * 2;
                if (roi_width < 8) roi_width = 8;
                if (roi_height < 8) roi_height = 8;

                unsigned x_offset = ((sensor_width - roi_width) / 2) / 2 * 2;
                unsigned y_offset = ((sensor_height - roi_height) / 2) / 2 * 2;
                ret = Toupcam_put_Roi(toup_handle, x_offset, y_offset, roi_width, roi_height);
            }
            logfile << "return code ROI: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek ROI" << endl;
                logfile << "Error setting ToupTek ROI" << endl;
                abort_app();
            }

            /*
            // DIAGNOSTIC: log active ToupTek ROI and SDK output size after binning/ROI setup.
            unsigned diag_roi_x = 0;
            unsigned diag_roi_y = 0;
            unsigned diag_roi_width = 0;
            unsigned diag_roi_height = 0;
            HRESULT diag_ret_roi = Toupcam_get_Roi(toup_handle, &diag_roi_x, &diag_roi_y, &diag_roi_width, &diag_roi_height);
            int diag_sdk_width = 0;
            int diag_sdk_height = 0;
            HRESULT diag_ret_size = Toupcam_get_Size(toup_handle, &diag_sdk_width, &diag_sdk_height);
            long diag_sdk_image_size = static_cast<long>(diag_sdk_width) * static_cast<long>(diag_sdk_height) * image_bytes;
            cout << "DIAGNOSTIC ToupTek ROI: ret=" << diag_ret_roi << ", offset=" << diag_roi_x << "x" << diag_roi_y
                << ", size=" << diag_roi_width << "x" << diag_roi_height << endl;
            cout << "DIAGNOSTIC ToupTek output size: ret=" << diag_ret_size << ", SDK=" << diag_sdk_width << "x" << diag_sdk_height
                << ", app=" << camera_image_width << "x" << camera_image_height << ", bin=" << bin << ", ROI_zoom=" << ROI_zoom << endl;
            cout << "DIAGNOSTIC ToupTek image bytes: SDK=" << diag_sdk_image_size << ", app allocated=" << image_size << endl;
            logfile << "DIAGNOSTIC ToupTek ROI: ret=" << diag_ret_roi << ", offset=" << diag_roi_x << "x" << diag_roi_y
                << ", size=" << diag_roi_width << "x" << diag_roi_height << endl;
            logfile << "DIAGNOSTIC ToupTek output size: ret=" << diag_ret_size << ", SDK=" << diag_sdk_width << "x" << diag_sdk_height
                << ", app=" << camera_image_width << "x" << camera_image_height << ", bin=" << bin << ", ROI_zoom=" << ROI_zoom << endl;
            logfile << "DIAGNOSTIC ToupTek image bytes: SDK=" << diag_sdk_image_size << ", app allocated=" << image_size << endl;
            /**/

            // Disable auto exposure and set exposure time
            cout << "Set ToupTek exposure time, ms: " << (exposure_time / 1000) << endl;
            logfile << "Set ToupTek exposure time, ms: " << (exposure_time / 1000) << endl;
            ret = Toupcam_put_AutoExpoEnable(toup_handle, 0);
            logfile << "return code AUTO_EXPO: " << ret << endl;
            ret = Toupcam_put_ExpoTime(toup_handle, static_cast<unsigned>(exposure_time));
            logfile << "return code EXPO_TIME: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek exposure time" << endl;
                logfile << "Error setting ToupTek exposure time" << endl;
                abort_app();
            }

            // Hardcoded HCG when conversion gain is supported. Set before gain because it can affect gain behavior.
            if ((toup_camera_info[cam].model->flag & (TOUPCAM_FLAG_CG | TOUPCAM_FLAG_CGHDR)) != 0) {
                cout << "Set ToupTek HCG mode" << endl;
                logfile << "Set ToupTek HCG mode" << endl;
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_CG, 1);
                logfile << "return code CG: " << ret << endl;
            }
            else {
                cout << "ToupTek HCG mode not supported" << endl;
                logfile << "ToupTek HCG mode not supported" << endl;
            }

            // Set gain
            cout << "Set ToupTek gain: " << gain << endl;
            logfile << "Set ToupTek gain: " << gain << endl;
            ret = Toupcam_put_ExpoAGain(toup_handle, static_cast<unsigned short>(gain));
            logfile << "return code GAIN: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Error setting ToupTek gain" << endl;
                logfile << "Error setting ToupTek gain" << endl;
                abort_app();
            }

            // Set WB as best-effort. In pure RAW mode this can be unsupported or have no effect.
            int toup_wb[3] = { static_cast<int>(WB_R), static_cast<int>(WB_G), static_cast<int>(WB_B) };
            cout << "Set ToupTek WB_R, WB_G, WB_B: " << WB_R << " " << WB_G << " " << WB_B << endl;
            logfile << "Set ToupTek WB_R, WB_G, WB_B: " << WB_R << " " << WB_G << " " << WB_B << endl;
            ret = Toupcam_put_WhiteBalanceGain(toup_handle, toup_wb);
            cout << "return code WB: " << ret << endl;
            logfile << "return code WB: " << ret << endl;

            // Set offset / black level as best-effort.
            cout << "Set ToupTek offset: " << offset << endl;
            logfile << "Set ToupTek offset: " << offset << endl;
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BLACKLEVEL_AUTOADJUST, 0);
            logfile << "return code BLACKLEVEL_AUTOADJUST: " << ret << endl;
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BLACKLEVEL, static_cast<int>(offset));
            logfile << "return code BLACKLEVEL: " << ret << endl;

            // Set bandwidth
            cout << "Set ToupTek bandwidth: " << bandwidth << endl;
            logfile << "Set ToupTek bandwidth: " << bandwidth << endl;
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_BANDWIDTH, static_cast<int>(bandwidth));
            logfile << "return code BANDWIDTH: " << ret << endl;

            // Hardcoded low-noise mode. Some models can return E_NOTIMPL; keep it non-fatal.
            cout << "Set ToupTek low noise mode" << endl;
            logfile << "Set ToupTek low noise mode" << endl;
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_LOW_NOISE, 1);
            cout << "return code LOW_NOISE: " << ret << endl;
            logfile << "return code LOW_NOISE: " << ret << endl;

            if (cooler_activation == 1) {
                // Set target temperature
                cout << "Set ToupTek target temperature: " << target_temperature << endl;
                logfile << "Set ToupTek target temperature: " << target_temperature << endl;
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_TECTARGET, static_cast<int>(target_temperature * 10));
                logfile << "return code TECTARGET: " << ret << endl;
                // Set cooler active
                cout << "Set ToupTek cooler active" << endl;
                logfile << "Set ToupTek cooler active" << endl;
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_TEC, 1);
                logfile << "return code TEC: " << ret << endl;
                // Set fan active
                cout << "Set ToupTek fan active" << endl;
                logfile << "Set ToupTek fan active" << endl;
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_FAN, -1);
                logfile << "return code FAN: " << ret << endl;
            }

            // Hardcoded heater off. Some models can return E_NOTIMPL; keep it non-fatal.
            if ((toup_camera_info[cam].model != NULL) && ((toup_camera_info[cam].model->flag & TOUPCAM_FLAG_HEAT) != 0)) {
                int heat_max = 0;
                ret = Toupcam_get_Option(toup_handle, TOUPCAM_OPTION_HEAT_MAX, &heat_max);
                cout << "ToupTek heater max level: " << heat_max << endl;
                cout << "return code HEAT_MAX: " << ret << endl;
                logfile << "ToupTek heater max level: " << heat_max << endl;
                logfile << "return code HEAT_MAX: " << ret << endl;

                cout << "Set ToupTek heater off" << endl;
                logfile << "Set ToupTek heater off" << endl;
                ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_HEAT, 0);
                cout << "return code HEAT: " << ret << endl;
                logfile << "return code HEAT: " << ret << endl;
            }
            else {
                cout << "ToupTek heater not supported" << endl;
                logfile << "ToupTek heater not supported" << endl;
            }

            // Keep normal video mode for now. Foto/trigger mode will be handled separately.
            ret = Toupcam_put_Option(toup_handle, TOUPCAM_OPTION_TRIGGER, 0);
            logfile << "return code TRIGGER: " << ret << endl;
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
            //myfile.seekg(2880, ios::beg);
            myfile.seekg(5760, ios::beg);
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

        else if (toup_connected_cameras > 0) {
            if (toup_handle == NULL) {
                cout << "ToupTek camera handle is NULL" << endl;
                logfile << "ToupTek camera handle is NULL" << endl;
                abort_app();
            }

            HRESULT ret = Toupcam_StartPullModeWithCallback(toup_handle, NULL, NULL);
            logfile << "return code START_PULL: " << ret << endl;
            if (FAILED(ret)) {
                cout << "Cannot start ToupTek video." << endl;
                logfile << "Cannot start ToupTek video." << endl;
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

        else if (toup_connected_cameras > 0) {
            if (toup_handle == NULL) {
                cout << "ToupTek camera handle is NULL" << endl;
                logfile << "ToupTek camera handle is NULL" << endl;
                abort_app();
            }

            /*
            // DIAGNOSTIC: Toupcam_get_Size reports the base preview size, not necessarily the final ROI/bin frame size.
            int diag_sdk_width = 0;
            int diag_sdk_height = 0;
            HRESULT diag_ret_size = Toupcam_get_Size(toup_handle, &diag_sdk_width, &diag_sdk_height);
            if (SUCCEEDED(diag_ret_size)) {
                cout << "DIAGNOSTIC ToupTek pre-pull get_Size: SDK=" << diag_sdk_width << "x" << diag_sdk_height
                    << ", app buffer=" << camera_image_width << "x" << camera_image_height << " bytes=" << image_size << endl;
                logfile << "DIAGNOSTIC ToupTek pre-pull get_Size: SDK=" << diag_sdk_width << "x" << diag_sdk_height
                    << ", app buffer=" << camera_image_width << "x" << camera_image_height << " bytes=" << image_size << endl;
            }
            else {
                logfile << "DIAGNOSTIC ToupTek pre-pull Toupcam_get_Size return code: " << diag_ret_size << endl;
            }
            /**/

            ToupcamFrameInfoV4 info = { 0 };
            unsigned timeout_ms;
            if (state == video_state)
                timeout_ms = static_cast<unsigned>(exposure_time / 1000 * 2 + 500);
            else
                timeout_ms = 500;

            HRESULT ret = Toupcam_WaitImageV4(toup_handle, timeout_ms, asi_image, 0, 0, -1, &info);
            if (SUCCEEDED(ret)) {
                if ((static_cast<int>(info.v3.width) != camera_image_width) || (static_cast<int>(info.v3.height) != camera_image_height)) {
                    cout << "ToupTek frame size mismatch: " << info.v3.width << "x" << info.v3.height
                         << ", expected " << camera_image_width << "x" << camera_image_height << endl;
                    logfile << "ToupTek frame size mismatch: " << info.v3.width << "x" << info.v3.height
                            << ", expected " << camera_image_width << "x" << camera_image_height << endl;
                    abort_app();
                }
                get_frame_success = 1;
            }
            else if (state == video_state) {
                cout << "Cannot get ToupTek video frame. Return code: " << ret << endl;
                logfile << "Cannot get ToupTek video frame. Return code: " << ret << endl;
                abort_app();
            }
            else {
                get_frame_success = 0;
            }
        }

    }

    return get_frame_success;
}



void stop_video()
{
    if (camera_from_file == 1) {
        frame_load_number = 0;
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

        else if (toup_connected_cameras > 0) {
            if (toup_handle != NULL) {
                HRESULT ret = Toupcam_Stop(toup_handle);
                logfile << "return code STOP: " << ret << endl;
                if (FAILED(ret)) {
                    cout << "Cannot stop ToupTek video." << endl;
                    logfile << "Cannot stop ToupTek video." << endl;
                    abort_app();
                }
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
            //string filename = "frame_f" + to_string(frames_stacked % 10) + ".fits";
            string filename = "frame_f" + to_string(frame_load_number % 10) + ".fits";
            frame_load_number++;
            myfile.open(filename, ios::in | ios::binary);

            if (myfile.is_open()) {
                printf("Reading file %s...\n", filename);
                //myfile.seekg(2880, ios::beg);
                myfile.seekg(5760, ios::beg);
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

                std::this_thread::sleep_for(std::chrono::microseconds(exposure_time_f));
                //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
                //myfile.seekg(2880, ios::beg);
                myfile.seekg(5760, ios::beg);
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



bool get_sensor_temperature(double& temperature_c)
{
    if (camera_from_file == 1)
        return false;

    if (asi_connected_cameras > 0) {
        long value = 0;
        ASI_BOOL auto_state = ASI_FALSE;
        if (ASIGetControlValue(asi_camera_info[cam]->CameraID, ASI_TEMPERATURE, &value, &auto_state) == ASI_SUCCESS) {
            temperature_c = value / 10.0;
            return true;
        }
    }
    else if (svb_connected_cameras > 0) {
        long value = 0;
        SVB_BOOL auto_state = SVB_FALSE;
        if (SVBGetControlValue(svb_camera_info[cam]->CameraID, SVB_CURRENT_TEMPERATURE, &value, &auto_state) == SVB_SUCCESS) {
            temperature_c = value / 10.0;
            return true;
        }
    }
    else if (toup_connected_cameras > 0) {
        if (toup_handle == NULL)
            return false;

        short value = 0;
        if (SUCCEEDED(Toupcam_get_Temperature(toup_handle, &value))) {
            temperature_c = value / 10.0;
            return true;
        }
    }

    return false;
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

    ROI_zoom = 0;

    scale_internalimage_height = 1500;
    crop_internalimage_flag = 0;

    hot_pixel_sigma = 7.0;

    //not used
    flat_inv_factor = 0.0;

    // used only in background "spot" correction mode
    //circ_vign_factor = 1.0;
    circ_vign_factor = 0.3;
    //circ_vign_radius = 0.7;
    circ_vign_radius = 0.5;

    blkp_x1_monitor = 0.0;
    blkp_y1_monitor = 0.0;
    blkp_x1_eyepiece = 0.0;
    blkp_y1_eyepiece = 0.0;

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
    lum_stretch_factor = 0.5;
    star_protection_factor = 1.0;
    star_factor = 0.3;

    WBcorr_R = 1.0;   // WB correction for RGB palette
    WBcorr_G = 1.0;
    WBcorr_B = 1.0;
    color_correction_flag = 0;
    CC11 = 1.0; CC12 = 0.0; CC13 = 0.0;
    CC21 = 0.0; CC22 = 1.0; CC23 = 0.0;
    CC31 = 0.0; CC32 = 0.0; CC33 = 1.0;
    
    aR = 1.0; bR = 0.0; cR = 0.0;  //dual band colors for R
    aG = 0.8; bG = 0.0; cG = 0.1;  //dual band colors for G
    aB = 0.0; bB = 0.4; cB = 0.4;  //dual band colors for B

    color_palettes.clear();
    ColorPaletteConfig default_palette;
    default_palette.name = "Default";
    default_palette.lum_stretch_factor = lum_stretch_factor;
    default_palette.WB_R = WBcorr_R;
    default_palette.WB_G = WBcorr_G;
    default_palette.WB_B = WBcorr_B;
    default_palette.CC11 = CC11; default_palette.CC12 = CC12; default_palette.CC13 = CC13;
    default_palette.CC21 = CC21; default_palette.CC22 = CC22; default_palette.CC23 = CC23;
    default_palette.CC31 = CC31; default_palette.CC32 = CC32; default_palette.CC33 = CC33;
    color_palettes.push_back(default_palette);
    

    enhance_stars_flag = 0;
    star_blob_radius = 20;
    star_blob_strength = 0.2;

    highlight_protection_par = 0.4;

    reject_satellittes_flag = 0;
    sattellites_decay = 0;

    reject_shaky_factor = 0;
    reject_cloudy_factor = 0;

    focusing_zoom_value = 4.0;
    zoom_value = 1.5;

    focusing_zoom_type = 1;

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

    show_status_flag = 0;

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

    //-------------------NV Mode

    NV_mode = 0;

    average_type = 2;
    kalman_alfa = 0.5;
    kalman_beta = 0.05;
    threshold_low = 0.2;
    threshold_high = 0.5;
    //AI_noise_model_NV_filename;
    AI_noise_factor_NV_1 = 1.0;
    AI_noise_factor_NV_2 = 1.0;



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

        if (cdk_mode == 1)
        {
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> camera_name_from_file;
            iss.str("");
        }

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
        exposure_time_v *= 1000; //ms -> �s
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
        exposure_time_f *= 1000; //ms -> �s
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

        /**/
        if (spline_gain_corr == 1)
        {
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> spline_corr_flag;
            iss.str("");

            float v;

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            while (iss >> v)
            {
                spline_radius.push_back(v);
            }
            iss.clear();
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            while (iss >> v)
            {
                spline_rValues.push_back(v);
            }
            iss.clear();
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            while (iss >> v)
            {
                spline_gValues.push_back(v);
            }
            iss.clear();
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            while (iss >> v)
            {
                spline_bValues.push_back(v);
            }
            iss.clear();
            iss.str("");
        }/**/



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

        if (roi_zoom == 1) {
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> ROI_zoom;
            iss.str("");
        }
        
        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> scale_internalimage_height >> crop_internalimage_flag;
        iss.str("");

        /*
        if (cdk_mode == 1)
        {
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> flat_inv_factor;
            iss.str("");


            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> circ_vign_factor >> circ_vign_radius;
            iss.str("");
        }/**/

        if (blkp_mode == 1)
        {
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> blkp_x1_monitor >> blkp_y1_monitor;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> blkp_x1_eyepiece >> blkp_y1_eyepiece;
            iss.str("");
        }


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
        //iss >> star_protection_factor;
        iss >> star_protection_factor >> star_factor;
        iss.str("");

        color_palettes.clear();
        while (getline(myfile, line)) {
            string parsed_line = trim_config_line(strip_config_comment(line));
            if (parsed_line.empty())
                continue;

            string first_token = config_first_token(parsed_line);
            if (first_token == "End")
                break;

            if (parsed_line.rfind("Palette:", 0) == 0) {
                ColorPaletteConfig palette;
                palette.name = trim_config_line(parsed_line.substr(8));
                if (palette.name.empty())
                    palette.name = "Palette " + to_string(color_palettes.size() + 1);

                bool palette_ok =
                    parse_palette_float_line(myfile, palette.lum_stretch_factor) &&
                    parse_palette_float_line(myfile, palette.WB_R, palette.WB_G, palette.WB_B) &&
                    parse_palette_float_line(myfile, palette.CC11, palette.CC12, palette.CC13) &&
                    parse_palette_float_line(myfile, palette.CC21, palette.CC22, palette.CC23) &&
                    parse_palette_float_line(myfile, palette.CC31, palette.CC32, palette.CC33);

                if (palette_ok)
                    color_palettes.push_back(palette);
                else
                    break;
            }
        }

        if (color_palettes.empty()) {
            ColorPaletteConfig default_palette;
            default_palette.name = "Default";
            default_palette.lum_stretch_factor = lum_stretch_factor;
            default_palette.WB_R = WBcorr_R;
            default_palette.WB_G = WBcorr_G;
            default_palette.WB_B = WBcorr_B;
            default_palette.CC11 = 1.0; default_palette.CC12 = 0.0; default_palette.CC13 = 0.0;
            default_palette.CC21 = 0.0; default_palette.CC22 = 1.0; default_palette.CC23 = 0.0;
            default_palette.CC31 = 0.0; default_palette.CC32 = 0.0; default_palette.CC33 = 1.0;
            color_palettes.push_back(default_palette);
        }

        color_palette = 0;

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
        iss >> reject_satellittes_flag;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> sattellites_decay;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> reject_shaky_factor;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> reject_cloudy_factor;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> focusing_zoom_value >> zoom_value;
        iss.str("");

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> focusing_zoom_type;
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

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> show_status_flag;
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
        //cout << second_display_Y << endl;

        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> circular_mask_eyepiece_flag;
        iss.str("");
        //cout << circular_mask_eyepiece_flag << endl;


        getline(myfile, line); //dummy line //Night Vision Mode
        iss.str("");




        getline(myfile, line);
        //cout << "line: " << line;
        iss << line;
        iss >> NV_mode;
        iss.str("");

        if (NV_mode == 1) {

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> average_type;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> kalman_alfa >> kalman_beta;
            iss.str("");
            
            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> threshold_low >> threshold_high;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> motion_number_frames;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> AI_noise_model_NV_filename;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> AI_noise_factor_NV_1 >> AI_noise_factor_NV_2;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> filter_strength_NV_1 >> filter_strength_NV_2;
            iss.str("");

            getline(myfile, line);
            //cout << "line: " << line;
            iss << line;
            iss >> motion_gain_reduction;
            iss.str("");
            
        }


        // get the CHECK line
        line = "fault";
        getline(myfile, line);
        cout << "Config CHECK line: " << line << endl;
        if (line.find("CHECK") == std::string::npos)
        {
            cout << "Config file corrupted!" << endl;
            abort_app();
        }



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

