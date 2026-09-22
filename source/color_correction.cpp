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


void palette_color_correction(Mat& image, const ColorPaletteConfig& palette) {

    if (debug_flag == 1) {
        cout << "Apply palette color correction matrix" << endl;
        logfile << "Apply palette color correction matrix" << endl;
    }

    if (image.isContinuous())
    {
        int nrows = image.rows;
        int ncols = image.cols;

        float* p = (float*)image.data;
        float* p1 = (float*)image.data;
        for (unsigned int i = 0; i < ncols * nrows; ++i) {
            float B = *p1;
            p1++;
            float G = *p1;
            p1++;
            float R = *p1;
            p1++;

            *p = R * palette.CC31 + G * palette.CC32 + B * palette.CC33;  //B
            p++;
            *p = R * palette.CC21 + G * palette.CC22 + B * palette.CC23;  //G
            p++;
            *p = R * palette.CC11 + G * palette.CC12 + B * palette.CC13;  //R
            p++;
        }
    }
    else {
        MatIterator_<Vec3f> it, end;
        for (it = image.begin<Vec3f>(), end = image.end<Vec3f>(); it != end; ++it)
        {
            float B = (*it)[0];
            float G = (*it)[1];
            float R = (*it)[2];

            (*it)[2] = R * palette.CC11 + G * palette.CC12 + B * palette.CC13;  //R
            (*it)[1] = R * palette.CC21 + G * palette.CC22 + B * palette.CC23;  //G
            (*it)[0] = R * palette.CC31 + G * palette.CC32 + B * palette.CC33;  //B
        }
    }
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
