//---------------- GUI
struct Button {
    cv::Rect rect;
    std::string text1;
    std::string text2;
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
    if (state == video_state) {
        cv::Size textSize = cv::getTextSize(button.text1, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
        cv::Point textOrg(button.rect.x + (button.rect.width - textSize.width) / 2, button.rect.y + (button.rect.height + textSize.height) / 2);
        cv::putText(img, button.text1, textOrg, cv::FONT_HERSHEY_SIMPLEX, text_scale, color, 1);
    }
    else {
        cv::Size textSize = cv::getTextSize(button.text2, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
        cv::Point textOrg(button.rect.x + (button.rect.width - textSize.width) / 2, button.rect.y + (button.rect.height + textSize.height) / 2);
        cv::putText(img, button.text2, textOrg, cv::FONT_HERSHEY_SIMPLEX, text_scale, color, 1);
    }
}

std::string getStatusText() {
    std::string mode_text = (state == video_state) ? "Mode: real-time" : "Mode: stacking";
    std::string palette_name = "unknown";
    if (!color_palettes.empty()) {
        int active_palette_index = color_palette;
        if ((active_palette_index < 0) || (active_palette_index >= (int)color_palettes.size()))
            active_palette_index = 0;
        palette_name = color_palettes[active_palette_index].name;
    }
    std::string palette_text = "Palette: " + palette_name;
    std::string bkg_text = ((bkg_mode == 1) && (hist_show_state == 1)) ? "Background correction: ON" : "Background correction: OFF";
    return mode_text + "   " + palette_text + "   " + bkg_text;
}

void drawStatusField(cv::Mat& img, const cv::Rect& rect, double text_scale) {
    cv::Scalar rect_color(0.4, 0.4, 0.4);
    cv::Scalar text_color(0.8, 0.8, 0.8);
    cv::rectangle(img, rect, rect_color, 2);

    std::string text = getStatusText();
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
    while ((textSize.width > rect.width - 10) && (text_scale > 0.1)) {
        text_scale *= 0.9;
        textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
    }

    cv::Point textOrg(rect.x + 10, rect.y + (rect.height + textSize.height) / 2);
    cv::putText(img, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, text_scale, text_color, 1);
}

int getStatusFieldHeight(int originalHeight) {
    int buttonFieldHeight = originalHeight / (double)3.5;
    int buttonHeight = buttonFieldHeight / 4;
    return std::max(1, (buttonHeight * 3) / 4);
}

cv::Mat addStatusField(cv::Mat& img) {
    if (show_status_flag != 1)
        return img;

    int originalHeight = img.rows;
    int originalWidth = img.cols;
    int statusFieldHeight = getStatusFieldHeight(originalHeight);
    double text_scale = ((double)originalWidth / 2.0) / 490.0 * 0.75;

    cv::Mat newImg(originalHeight + statusFieldHeight, originalWidth, img.type());
    newImg.setTo(cv::Scalar(0, 0, 0));
    img.copyTo(newImg(cv::Rect(0, 0, originalWidth, originalHeight)));

    cv::Rect statusRect(5, originalHeight + 5, originalWidth - 5, std::max(1, statusFieldHeight - 5));
    drawStatusField(newImg, statusRect, text_scale);

    return newImg;
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
            if (k == 0) btn.text1 = "-";
            if (k == 0) btn.text2 = "-";

            if (k == 1) btn.text1 = "+";
            if (k == 1) btn.text2 = "+";

            if (k == 2) btn.text1 = "Palette change";
            if (k == 2) btn.text2 = "Palette change";

            if (k == 3) btn.text1 = "Mode change";
            if (k == 3) btn.text2 = "Mode change";

            if (k == 4) btn.text1 = "Focus zoom";
            if (k == 4) btn.text2 = "Zoom";

            if (k == 5) btn.text1 = "RAW histogram";
            if (k == 5) btn.text2 = "RAW histogram";
            if (bkg_mode == 1) {
                if (k == 5) btn.text1 = "Background correction";
                if (k == 5) btn.text2 = "Background correction";
            }

            if (k == 6) btn.text1 = "Save picture";
            if (k == 6) btn.text2 = "Save picture";

            if (k == 7) btn.text1 = "Exit";
            if (k == 7) btn.text2 = "Exit";

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
    int statusFieldHeight = (show_status_flag == 1) ? getStatusFieldHeight(originalHeight) : 0;

    // Create new image with additional button field space
    cv::Mat newImg(originalHeight + buttonFieldHeight + statusFieldHeight, originalWidth, img.type());
    newImg.setTo(cv::Scalar(0, 0, 0));
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

    if (show_status_flag == 1) {
        double text_scale = ((double)buttonWidth) / 490.0 * 0.75;
        cv::Rect statusRect(5, originalHeight + buttonFieldHeight + 5, originalWidth - 5, std::max(1, statusFieldHeight - 5));
        drawStatusField(newImg, statusRect, text_scale);
    }

    return newImg;
}


void onMouse_Black_Window(int event, int x, int y, int, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN)
        destroyWindow("Black_Window");
}


void onMouse(int event, int x, int y, int, void* userdata) {
    //if (event != cv::EVENT_LBUTTONDOWN) {
    //    return;
    //}

    // process mouse click on buttons
    if (event == cv::EVENT_LBUTTONDOWN) {
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

    // show black screen on mouse click on image
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Rect rect = cv::Rect(0, 0, display_image.cols, display_image.rows * 0.9);
        if (rect.contains(cv::Point(x, y))) {
            Mat black_image(800, 800, CV_8UC3, Scalar(0, 0, 0));
            cv::Point textOrg(50, 50);
            cv::Scalar color = cv::Scalar(20, 20, 255);
            cv::putText(black_image, "Black screen mode, mouse click to close", textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);

            namedWindow("Black_Window", WINDOW_NORMAL);
            moveWindow("Black_Window", 0, 0);
            setWindowProperty("Black_Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
            imshow("Black_Window", black_image);

            setMouseCallback("Black_Window", onMouse_Black_Window);

            waitKey(1);
            //destroyWindow("Black_Window");
        }
    }

}
//---------------- GUI


// protection from locking main screen without connected eyepiece screen
void onMouse_Eyepiece(int event, int x, int y, int, void* userdata) {
    //if (event != cv::EVENT_LBUTTONDBLCLK) {
    //    return;
    //}

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::vector<Button>* buttons = reinterpret_cast<std::vector<Button>*>(userdata);
        buttons->at(7).pressed = true;  // Exit button activated
    }
}


void draw_clock(Mat& image) {

    const int H = image.rows;
    const int W = image.cols;
    //const int thickness = 3;
    const int thickness = static_cast<int>(std::lround(image.rows / 250.0));

    // Position & size
    const cv::Point center(W / 2, static_cast<int>(std::lround(0.9 * H)));
    const int radius = static_cast<int>(std::lround(0.04 * H));          //
    const int handLen1 = static_cast<int>(std::lround(0.8 * radius));   // slightly shorter than radius
    const int handLen2 = static_cast<int>(std::lround(0.7 * radius));   // slightly shorter than radius

    // Color: BGR = (0.2, 0.2, 1.0)
    const cv::Scalar color(0.2, 0.2, 1.0);  // B, G, R

    // Draw circle
    cv::circle(image, center, radius, color, thickness, cv::LINE_AA);

    // Draw hands pointing to 12 and 3
    const cv::Point up(center.x, center.y - handLen1); // 12 o'clock
    const cv::Point right(center.x + handLen2, center.y);         // 3 o'clock
    cv::line(image, center, up, color, thickness, cv::LINE_AA);
    cv::line(image, center, right, color, thickness, cv::LINE_AA);

}


void show_clock() {
    if (show_clock_flag == 1) {

        if (main_display_flag == 1) {
            Mat display_image2 = display_image.clone();

            draw_clock(display_image2);

            if (GUI_flag == 1) {
                setMouseCallback("Display window", onMouse, &buttons);
                Mat display_image_Buttons = addButtonField(display_image2, buttons);
                imshow("Display window", display_image_Buttons);
            }
            else {
                Mat display_image_Status = addStatusField(display_image2);
                imshow("Display window", display_image_Status);
            }
            key = waitKey(1);
        }

        //if ((main_display_flag == 0) || ((cdk_mode == 1) && (eyepiece_display_flag == 1))) {
        else if ((eyepiece_display_flag != 0)) {
            draw_clock(final_image_eyepiece);

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

            //Mat img_test = imread("C:/Users/HOME/Desktop/stacks_test/stack_2024-03-08_21-46-02.tiff", IMREAD_UNCHANGED);
            imshow("Eyepiece", eyepiece_image);

            key = waitKey(1);
        };
    }

}
