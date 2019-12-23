/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */
#include <iostream>
#include <fstream>
#include "examples/task4/defines.h"
#include "examples/task4/functions.h"
#include "sfm/bundler_common.h"
#include "sfm/bundler_features.h"
#include "sfm/bundler_matching.h"
#include "sfm/bundler_intrinsics.h"
#include "sfm/bundler_init_pair.h"
#include "sfm/bundler_tracks.h"
#include "sfm/bundler_incremental.h"
#include "core/scene.h"
#include "util/timer.h"

#include <util/file_system.h>
#include <core/bundle_io.h>
#include <core/camera.h>
#include <fstream>
#include <iostream>
#include <cassert>
#include "util/aligned_memory.h"
#include "util/timer.h"
#include "core/image.h"
#include "core/image_tools.h"
#include "core/image_io.h"

#include "features/surf.h"
#include "features/sift.h"
#include "features/matching.h"
#include "sfm/feature_set.h"
#include "visualizer.h"

/*features::Matching::Result -> vector<int> matches_1_2, matches_1_2*/
core::ByteImage::Ptr
visualize_matching (features::Matching::Result const& matching,
    core::ByteImage::Ptr image1, core::ByteImage::Ptr image2,
    std::vector<math::Vec2f> const& pos1, std::vector<math::Vec2f> const& pos2)
{
    /* Visualize keypoints. */
    sfm::Correspondences2D2D vis_matches;
    /* vector<int> matches_1_2 : Matches from set 1 in set 2. */
    for (std::size_t i = 0; i < matching.matches_1_2.size(); ++i)
    {
        if (matching.matches_1_2[i] < 0)
            continue;
        int const j = matching.matches_1_2[i];

        sfm::Correspondence2D2D match;
        std::copy(pos1[i].begin(), pos1[i].end(), match.p1); /*把该点的坐标值赋给match.p1*/
        std::copy(pos2[j].begin(), pos2[j].end(), match.p2);/*把该点的对应点坐标值赋给match.p2*/
        vis_matches.push_back(match); /*一个match中只有两个点的坐标值，也就是说一个match包含了一对点*/
    }

    std::cout << "Drawing " << vis_matches.size() << " matches..." << std::endl;/*输出有效的实际匹配对数目*/
    core::ByteImage::Ptr match_image = sfm::Visualizer::draw_matches
        (image1, image2, vis_matches);
    return match_image;
}

#define DISCRETIZE_DESCRIPTORS 0
/*SIFT::Descriptor: float x,y,scale,orientation,Vector<float, 128> data ;Descriptors为Descriptor的向量集合*/
template <typename T>
void
convert_sift_descriptors(features::Sift::Descriptors const& sift_descr,
    util::AlignedMemory<math::Vector<T, 128> >* aligned_descr)
{
    aligned_descr->resize(sift_descr.size());/*size为128*/
    T* data_ptr = aligned_descr->data()->begin();
    for (std::size_t i = 0; i < sift_descr.size(); ++i, data_ptr += 128)
    {
        sfm::Sift::Descriptor const& d = sift_descr[i];

#if DISCRETIZE_DESCRIPTORS
        for (int j = 0; j < 128; ++j)
        {
            float value = d.data[j];
            value = math::clamp(value, 0.0f, 1.0f);
            value = math::round(value * 255.0f);
            data_ptr[j] = static_cast<unsigned char>(value);
        }
#else
        std::copy(d.data.begin(), d.data.end(), data_ptr); /*把sift_descriptors描述符集合里面的每一个描述符的data赋值给aligned_descr->data(),等价于aligned_descr->data = Descriptor.data */
#endif
    }
}

template <typename T>
void
convert_surf_descriptors(sfm::Surf::Descriptors const& surf_descr,
    util::AlignedMemory<math::Vector<T, 64> >* aligned_descr)
{
    aligned_descr->resize(surf_descr.size());
    T* data_ptr = aligned_descr->data()->begin();
    for (std::size_t i = 0; i < surf_descr.size(); ++i, data_ptr += 64)
    {
        sfm::Surf::Descriptor const& d = surf_descr[i];
#if DISCRETIZE_DESCRIPTORS
        for (int j = 0; j < 64; ++j)
        {
            float value = d.data[j];
            value = math::clamp(value, -1.0f, 1.0f);
            value = math::round(value * 127.0f);
            data_ptr[j] = static_cast<signed char>(value);
        }
#else
        std::copy(d.data.begin(), d.data.end(), data_ptr);
#endif
    }
}

void
feature_set_matching (core::ByteImage::Ptr image1, core::ByteImage::Ptr image2,std::string outdir_name)
{
    /*FeatureSet 计算并存储一个视角的特征点，包含SIFT和SURF特征点 */
    sfm::FeatureSet::Options feature_set_opts;
    //feature_types设置为FEATURE_ALL表示检测SIFT和SURF两种特征点进行匹配
    feature_set_opts.feature_types = sfm::FeatureSet::FEATURE_ALL;
    feature_set_opts.sift_opts.verbose_output = true; /*详细输出有关sift的处理信息*/
    //feature_set_opts.surf_opts.verbose_output = true;
    //feature_set_opts.surf_opts.contrast_threshold = 500.0f;

    // 计算两幅图像的SIFT和SURF特征点，并分别存储在feat1、feat2中
    sfm::FeatureSet feat1(feature_set_opts);
    feat1.compute_features(image1);

    sfm::FeatureSet feat2(feature_set_opts);
    feat2.compute_features(image2);

    /* 对sift特征描述子进行匹配 */
    // sift 特征描述子匹配参数
    sfm::Matching::Options sift_matching_opts;
    sift_matching_opts.lowe_ratio_threshold = 0.8f;
    sift_matching_opts.descriptor_length = 128;
    sift_matching_opts.distance_threshold = std::numeric_limits<float>::max();/*3.40282e+38*/

#if DISCRETIZE_DESCRIPTORS
    util::AlignedMemory<math::Vec128us, 16> sift_descr1, sift_descr2;
#else
    util::AlignedMemory<math::Vec128f, 16> sift_descr1, sift_descr2;//what is the meaning of 16 ????????
#endif
    // 将描述子转成特定的内存格式
    convert_sift_descriptors(feat1.sift_descriptors, &sift_descr1);
    convert_sift_descriptors(feat2.sift_descriptors, &sift_descr2);

    // 进行双向匹配
    sfm::Matching::Result sift_matching;
    sfm::Matching::twoway_match(sift_matching_opts,
        sift_descr1.data()->begin(), sift_descr1.size(),
        sift_descr2.data()->begin(), sift_descr2.size(),
        &sift_matching);

    // 去除不一致的匹配对，匹配对feature1和feature2是一致的需要满足，feature1的最近邻居
    // 是feature2，feature2的最近邻是feature1
    sfm::Matching::remove_inconsistent_matches(&sift_matching);
    std::cout << "Consistent Sift Matches: "
        << sfm::Matching::count_consistent_matches(sift_matching)
        << std::endl;


    /*  对surf特征描述子进行匹配  */
    // surf特征匹配参数
    sfm::Matching::Options surf_matching_opts;
    // 最近邻和次近邻的比
    surf_matching_opts.lowe_ratio_threshold = 0.7f;
    // 特征描述子的维度
    surf_matching_opts.descriptor_length = 64;
    surf_matching_opts.distance_threshold = std::numeric_limits<float>::max();

#if DISCRETIZE_DESCRIPTORS
    util::AlignedMemory<math::Vec64s, 16> surf_descr1, surf_descr2;
#else
    util::AlignedMemory<math::Vec64f, 16> surf_descr1, surf_descr2;
#endif
    // 将描述子转化成特殊的格式
    convert_surf_descriptors(feat1.surf_descriptors, &surf_descr1);
    convert_surf_descriptors(feat2.surf_descriptors, &surf_descr2);

    // 进行surf描述子的双向匹配
    sfm::Matching::Result surf_matching;
    sfm::Matching::twoway_match(surf_matching_opts,
        surf_descr1.data()->begin(), surf_descr1.size(),
        surf_descr2.data()->begin(), surf_descr2.size(),
        &surf_matching);
    // 去除不一致的匹配对，匹配对feature 1 和 feature 2 互为最近邻
    sfm::Matching::remove_inconsistent_matches(&surf_matching);
    std::cout << "Consistent Surf Matches: "
        << sfm::Matching::count_consistent_matches(surf_matching)
        << std::endl;

    // 对sift匹配的结果和surf匹配的结果进行融合
    sfm::Matching::Result matching;
    sfm::Matching::combine_results(sift_matching, surf_matching, &matching);

    std::cout << "Consistent Matches: "
        << sfm::Matching::count_consistent_matches(matching)
        << std::endl;

    /* 特征匹配可视化 */
    /* Draw features. */
    std::vector<sfm::Visualizer::Keypoint> features1;
    for (std::size_t i = 0; i < feat1.sift_descriptors.size(); ++i)
    {
        if (matching.matches_1_2[i] == -1)
            continue;

        sfm::Sift::Descriptor const& descr = feat1.sift_descriptors[i];
        sfm::Visualizer::Keypoint kp;
        kp.orientation = descr.orientation;
        kp.radius = descr.scale * 3.0f; /*为什么要乘3*/
        kp.x = descr.x;
        kp.y = descr.y;
        features1.push_back(kp);
    }

    std::vector<sfm::Visualizer::Keypoint> features2;
    for (std::size_t i = 0; i < feat2.sift_descriptors.size(); ++i)
    {
        if (matching.matches_2_1[i] == -1)
            continue;

        sfm::Sift::Descriptor const& descr = feat2.sift_descriptors[i];
        sfm::Visualizer::Keypoint kp;
        kp.orientation = descr.orientation;
        kp.radius = descr.scale * 3.0f;
        kp.x = descr.x;
        kp.y = descr.y;
        features2.push_back(kp);
    }

    image1 = sfm::Visualizer::draw_keypoints(image1,
        features1, sfm::Visualizer::RADIUS_BOX_ORIENTATION);
    image2 = sfm::Visualizer::draw_keypoints(image2,
        features2, sfm::Visualizer::RADIUS_BOX_ORIENTATION);

    core::ByteImage::Ptr match_image = visualize_matching(
        matching, image1, image2, feat1.positions, feat2.positions);
    /* 保存图像文件名 */
    std::string output_filename = "./tmp/" + outdir_name + ".png";
    std::cout << "Saving visualization to " << output_filename << std::endl;
    core::image::save_file(match_image, output_filename);
}

int main (int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "Syntax: " << argv[0] << " image1 image2" << std::endl;
        return 1;
    }

 // 用于加速
#ifdef __SSE2__
    std::cout << "SSE2 is enabled!" << std::endl;
#endif
#ifdef __SSE3__
    std::cout << "SSE3 is enabled!" << std::endl;
#endif

    /* Regular two-view matching. */
    core::ByteImage::Ptr image1, image2;
    try
    {
        std::cout << "Loading " << argv[1] << "..." << std::endl;
        image1 = core::image::load_file(std::string(argv[1]));
        // 图像尺寸减半
        image1 = core::image::rescale_half_size<uint8_t>(image1);
        //image1 = core::image::rescale_half_size<uint8_t>(image1);
        //image1 = core::image::rotate<uint8_t>(image1, core::image::ROTATE_CCW);

        std::cout << "Loading " << argv[2] << "..." << std::endl;
        image2 = core::image::load_file(argv[2]);
        // 图像尺寸减半
        image2 = core::image::rescale_half_size<uint8_t>(image2);
        //image2 = core::image::rescale_half_size<uint8_t>(image2);
        //image2 = core::image::rotate<uint8_t>(image2, core::image::ROTATE_CCW);
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    // 进行特征提取和特征匹配
    feature_set_matching(image1, image2,argv[3]);

    return 0;
}
