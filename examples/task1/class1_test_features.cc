/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "util/file_system.h"
#include "util/timer.h"
#include "core/image.h"
#include "core/image_tools.h"
#include "core/image_io.h"

#include "features/surf.h"
#include "features/sift.h"
#include "visualizer.h"

bool
sift_compare (features::Sift::Descriptor const& d1, features::Sift::Descriptor const& d2)
{
    return d1.scale > d2.scale;
}

int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Syntax: " << argv[0] << " <image>" << std::endl;
        return 1;
    }

    /* 加载图像*/
    core::ByteImage::Ptr image;
    std::string image_filename = argv[1];
    try
    {
        std::cout << "Loading " << image_filename << "..." << std::endl;
        image = core::image::load_file(image_filename);
        //image = core::image::rescale_half_size<uint8_t>(image);
        //image = core::image::rescale_half_size<uint8_t>(image);
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }


    /* SIFT 特征检测. */
    features::Sift::Descriptors sift_descr;
    features::Sift::Keypoints sift_keypoints;
    {
        features::Sift::Options sift_options;
        /**
         默认参数：num_samples_per_octave=3;min_octave=0;max_octave=4;contrast_threshold=-1.0f;
         edge_ratio_threshold=10.0f;base_blur_sigma=1.6f;inherent_blur_sigma=0.5f
        **/
        sift_options.verbose_output = true;
        sift_options.debug_output = true;
        features::Sift sift(sift_options);
        sift.set_image(image);//把图片放进去准备计算

        util::WallTimer timer;
        sift.process(); // 主程序,这条指令在计算图像上的sift特征点
        std::cout << "Computed SIFT features in "
                  << timer.get_elapsed() << "ms." << std::endl;

        sift_descr = sift.get_descriptors();
        sift_keypoints = sift.get_keypoints();
    }

    // 对特征点按照尺度从大到小进行排序(降序)
    std::sort(sift_descr.begin(), sift_descr.end(), sift_compare);

    std::vector<features::Visualizer::Keypoint> sift_drawing;
    for (std::size_t i = 0; i < sift_descr.size(); ++i)
    {
        features::Visualizer::Keypoint kp;
        kp.orientation = sift_descr[i].orientation;
        kp.radius = sift_descr[i].scale;
        kp.x = sift_descr[i].x;
        kp.y = sift_descr[i].y;
        if(i<100) std::cout << kp.radius<<" " ; //查看特征点的尺度,这个是会变化的
        sift_drawing.push_back(kp);
    }
    std::cout << std::endl;
    std::cout << "检测到的关键点个数为："<<sift_drawing.size();
    core::ByteImage::Ptr sift_image = features::Visualizer::draw_keypoints(image,
        sift_drawing, features::Visualizer::RADIUS_BOX_ORIENTATION);

    /* 保存图像文件名 */
    std::string sift_out_fname = "./tmp/" + util::fs::replace_extension
        (util::fs::basename(image_filename), "sift.png");
    std::cout << "保存图像: " << sift_out_fname << std::endl;
    core::image::save_file(sift_image, sift_out_fname);

    return 0;
}
