#include <features/matching.h>
#include <sfm/ransac_fundamental.h>
#include <core/image_exif.h>
#include <fstream>
#include <cassert>
#include "math/matrix.h"
#include "math/vector.h"

#include "core/image_io.h"
#include "core/image.h"
#include "core/image_tools.h"

#include "sfm/camera_pose.h"
#include "sfm/fundamental.h"

#include "sfm/feature_set.h"
#include "sfm/correspondence.h"
#include "sfm/bundle_adjustment.h"
#include "sfm/correspondence.h"

#include "sfm/camera_database.h"
#include "sfm/extract_focal_length.h"

#include "sfm/triangulate.h"

#define MAX_PIXELS 1000000

//从图像名中获取焦距信息，打印fl.first和fl.second，并返回fl.first
float extract_focal_len(const std::string& img_name)
{
    std::string exif_str;
    core::image::load_jpg_file(img_name.c_str(), &exif_str); //Loads a JPEG file. The EXIF data blob may be loaded into 'exif'.
    std::cout << exif_str.c_str()<<exif_str.size()<< std::endl; //EXIF 41867
    core::image::ExifInfo exif = core::image::exif_extract(exif_str.c_str(), exif_str.size(), true);
    sfm::FocalLengthEstimate fl = sfm::extract_focal_length(exif);
    std::cout <<"Focal length: " <<fl.first << " " << fl.second << std::endl; //Focal length: 1 2
    return fl.first;
}

int
main (int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Syntax: " << argv[0] << " <img> " << std::endl;
        return 1;
    }
    /* 1.0 提取相机焦距*/
    float f1  = extract_focal_len(argv[1]);
    std::cout<<"focal length: f1 "<<f1<<std::endl;
    return 0;
}
