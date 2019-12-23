#include <iostream>
#include <fstream>
#include "util/file_system.h"
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

bool
sift_compare (features::Sift::Descriptor const& d1, features::Sift::Descriptor const& d2)
{
    return d1.scale > d2.scale;
}
bool
surf_compare (features::Surf::Descriptor const& d1, features::Surf::Descriptor const& d2)
{
    return d1.scale > d2.scale;
}

int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Syntax: " << argv[0] << " image1 " << std::endl;
        return 1;
    }

    // 用于加速
#ifdef __SSE2__
    std::cout << "SSE2 is enabled!" << std::endl;
#endif
#ifdef __SSE3__
    std::cout << "SSE3 is enabled!" << std::endl;
#endif
    core::ByteImage::Ptr image;
    try
    {
        std::cout << "Loading " << argv[1] << "..." << std::endl;
        image = core::image::load_file(std::string(argv[1]));
        /**
        // 图像尺寸减半
        image = core::image::rescale_half_size<uint8_t>(image);
        //image = core::image::rescale_half_size<uint8_t>(image);
        //image = core::image::rotate<uint8_t>(image, core::image::ROTATE_CCW);
        **/
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    /** 保存图像文件名 **/
    std::string feature_out_fname1 = "./tmp/" + util::fs::replace_extension
            (util::fs::basename(argv[1]), "siftsurf001.png");
    std::cout << "保存图像: " << feature_out_fname1 << std::endl;
    core::image::save_file(image, feature_out_fname1);
    /** 进行特征提取 **/
    util::WallTimer timer;
    /*FeatureSet 计算并存储一个视角的特征点，包含SIFT和SURF特征点 */
    sfm::FeatureSet::Options feature_set_opts;
    //feature_types设置为FEATURE_ALL表示检测SIFT和SURF两种特征点
    feature_set_opts.feature_types = sfm::FeatureSet::FEATURE_ALL;
    feature_set_opts.sift_opts.verbose_output = true;
    feature_set_opts.surf_opts.verbose_output = true;
    //feature_set_opts.surf_opts.contrast_threshold = 500.0f;
    // 计算该幅图像的SIFT和SURF特征点，并存储在feat中
    sfm::FeatureSet feat(feature_set_opts);
    feat.compute_features(image);
    //载入计算的sift和surf特征点
    features::Sift::Descriptors sift_descr = feat.sift_descriptors;
    features::Surf::Descriptors surf_descr = feat.surf_descriptors;
    std::cout << "Computed SIFT and SURF features in "<< timer.get_elapsed() << "ms." << std::endl;
    // 对特征点按照尺度从大到小进行排序
    std::sort(sift_descr.begin(), sift_descr.end(), sift_compare);
    std::sort(surf_descr.begin(), surf_descr.end(), surf_compare);

    /**绘制sift特征点和surf特征点**/
    std::vector<features::Visualizer::Keypoint> sift_drawing;
    for (std::size_t i = 0; i < sift_descr.size(); ++i)
    {
        features::Visualizer::Keypoint kp;
        kp.orientation = sift_descr[i].orientation;
        kp.radius = sift_descr[i].scale;
        kp.x = sift_descr[i].x;
        kp.y = sift_descr[i].y;
     //   if(i<100) std::cout << kp.radius<<" " ; //查看特征点的尺度,这个是会变化的
        sift_drawing.push_back(kp);
    }

    std::vector<features::Visualizer::Keypoint> surf_drawing;
    for (std::size_t i = 0; i < surf_descr.size(); ++i)
    {
        features::Visualizer::Keypoint kp;
        kp.orientation = surf_descr[i].orientation;
        kp.radius = surf_descr[i].scale;
        kp.x = surf_descr[i].x;
        kp.y = surf_descr[i].y;
        //if(i<100) std::cout << kp.radius<<" " ; //查看特征点的尺度,这个是会变化的
        surf_drawing.push_back(kp);
    }
    std::cout << std::endl;
    std::cout << "检测到的sift关键点个数为："<<sift_drawing.size()<<";surf关键点个数为："<<surf_drawing.size()<<std::endl;

    core::ByteImage::Ptr feature_image = features::Visualizer::draw_keypoints(image,
            sift_drawing, features::Visualizer::RADIUS_BOX_ORIENTATION );
    feature_image = features::Visualizer::draw_keypoints(feature_image,
            surf_drawing, features::Visualizer::RADIUS_CIRCLE_ORIENTATION);

    /** 保存图像文件名 **/
    std::string feature_out_fname = "./tmp/" + util::fs::replace_extension
                (util::fs::basename(argv[1]), "siftsurf.png");
    std::cout << "保存图像: " << feature_out_fname << std::endl;
    core::image::save_file(feature_image, feature_out_fname);
    return 0;
}
