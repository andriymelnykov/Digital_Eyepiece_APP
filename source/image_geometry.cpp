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


void focusing_zoom(Mat& image, double zoom) {

    if (debug_flag == 1) {
        cout << "Apply focusing zoom" << endl;
        logfile << "Apply focusing zoom" << endl;
    }

    Mat temp;
    image.copyTo(temp);

    int crop_size = round((image.cols - 10) / zoom / 3);

    Mat crop;
    crop = temp(Range(round(image.rows / 2 - crop_size / 2), round(image.rows / 2 + crop_size / 2)), Range(round(image.cols / 2 - crop_size / 2), round(image.cols / 2 + crop_size / 2)));

    Mat crop_zoom;
    resize(crop, crop_zoom, Size(0, 0), zoom, zoom, INTER_AREA);



    vector<Mat> image_bgr;
    split(image, image_bgr);

    vector<Mat> crop_bgr;
    split(crop_zoom, crop_bgr);

    Mat insetImage_1_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 - crop_bgr[0].cols / 2 * 3), round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));
    Mat insetImage_2_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 - crop_bgr[0].cols / 2), round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));
    Mat insetImage_3_b(image_bgr[0], Rect(round(image_bgr[0].cols / 2 + crop_bgr[0].cols / 2), round(image_bgr[0].rows / 2 - crop_bgr[0].rows / 2), crop_bgr[0].cols, crop_bgr[0].rows));

    Mat insetImage_1_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 - crop_bgr[1].cols / 2 * 3), round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));
    Mat insetImage_2_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 - crop_bgr[1].cols / 2), round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));
    Mat insetImage_3_g(image_bgr[1], Rect(round(image_bgr[1].cols / 2 + crop_bgr[1].cols / 2), round(image_bgr[1].rows / 2 - crop_bgr[1].rows / 2), crop_bgr[1].cols, crop_bgr[1].rows));

    Mat insetImage_1_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 - crop_bgr[2].cols / 2 * 3), round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));
    Mat insetImage_2_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 - crop_bgr[2].cols / 2), round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));
    Mat insetImage_3_r(image_bgr[2], Rect(round(image_bgr[2].cols / 2 + crop_bgr[2].cols / 2), round(image_bgr[2].rows / 2 - crop_bgr[2].rows / 2), crop_bgr[2].cols, crop_bgr[2].rows));

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


// Five-zone focusing zoom: center plus four edge crops for inspecting the frame center and borders.
void focusing_zoom_edges(Mat& image, double zoom) {

    if (debug_flag == 1) {
        cout << "Apply focusing edge zoom" << endl;
        logfile << "Apply focusing edge zoom" << endl;
    }

    if ((image.cols < 3) || (image.rows < 3) || (zoom < 1.01))
        return;

    Mat source;
    image.copyTo(source);

    Mat result(image.size(), image.type(), Scalar::all(0));

    int tile_width = image.cols / 3;
    int tile_height = image.rows / 3;
    int crop_width = max(1, min(image.cols, (int)round(tile_width / zoom)));
    int crop_height = max(1, min(image.rows, (int)round(tile_height / zoom)));
    int border_width = max(1, tile_height / 50);

    auto clamp_source_rect = [&](int center_x, int center_y) {
        int x = center_x - crop_width / 2;
        int y = center_y - crop_height / 2;
        x = max(0, min(x, image.cols - crop_width));
        y = max(0, min(y, image.rows - crop_height));
        return Rect(x, y, crop_width, crop_height);
    };

    auto copy_zoomed_crop = [&](Rect source_rect, Rect target_rect) {
        Mat crop = source(source_rect);
        Mat crop_zoom;
        resize(crop, crop_zoom, target_rect.size(), 0, 0, INTER_AREA);
        crop_zoom.copyTo(result(target_rect));
        rectangle(result, target_rect, Scalar::all(0), border_width);
    };

    int tile_x_center = (image.cols - tile_width) / 2;
    int tile_y_center = (image.rows - tile_height) / 2;

    Rect target_center(tile_x_center, tile_y_center, tile_width, tile_height);
    Rect target_left(0, tile_y_center, tile_width, tile_height);
    Rect target_right(image.cols - tile_width, tile_y_center, tile_width, tile_height);
    Rect target_top(tile_x_center, 0, tile_width, tile_height);
    Rect target_bottom(tile_x_center, image.rows - tile_height, tile_width, tile_height);

    copy_zoomed_crop(clamp_source_rect(image.cols / 2, image.rows / 2), target_center);
    copy_zoomed_crop(clamp_source_rect(crop_width / 2, image.rows / 2), target_left);
    copy_zoomed_crop(clamp_source_rect(image.cols - crop_width / 2, image.rows / 2), target_right);
    copy_zoomed_crop(clamp_source_rect(image.cols / 2, crop_height / 2), target_top);
    copy_zoomed_crop(clamp_source_rect(image.cols / 2, image.rows - crop_height / 2), target_bottom);

    image = result;
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
