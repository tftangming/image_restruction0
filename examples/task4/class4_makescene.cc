#include "defines.h"
#include "functions.h"
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
/**
 *\description 创建一个场景
 * @param image_folder_path
 * @param scene_path
 * @return
 */
core::Scene::Ptr
make_scene(const std::string & image_folder_path, const std::string & scene_path){

    util::WallTimer timer;

    /*** 创建文件夹 ***/
    const std::string views_path = util::fs::join_path(scene_path, "views/");
    util::fs::mkdir(scene_path.c_str());
    util::fs::mkdir(views_path.c_str());

    /***扫描文件夹，获取所有的图像文件路径***/
    util::fs::Directory dir;
    try {dir.scan(image_folder_path);
    }
    catch (std::exception&e){
        std::cerr << "Error scanning input dir: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::cout << "Found " << dir.size() << " directory entries." << std::endl; //输出该Directory中的图像文件数目

    core::Scene::Ptr scene= core::Scene::create("");

    /**** 开始加载图像 ****/
    std::sort(dir.begin(), dir.end());
    int num_imported = 0;
    for(std::size_t i=0; i< dir.size(); i++){
        // 检测到一个文件夹则忽略跳过
        if(dir[i].is_dir){
            std::cout<<"Skipping directory "<<dir[i].name<<std::endl;
            continue;
        }
        //获取文件名字以及绝对名字(路径+名字)
        std::string fname = dir[i].name;
        std::string afname = dir[i].get_absolute_name();

        // 从可交换信息文件中读取图像焦距
        std::string exif;
        core::ImageBase::Ptr image = load_any_image(afname, & exif);
     //   std::cout << "exif :"<< exif<< std::endl; //这个exif中存储了blob
        if(image == nullptr){
            continue;
        }
        core::View::Ptr view = core::View::create();
        view->set_id(num_imported);// Sets the view ID,设置为meta_data.data该字典的键值 (key："view.id" ;value:ID )
        view->set_name(remove_file_extension(fname));// Sets the view name(该幅图像的文件名),设置为meta_data.data该字典的键值 (key："view.name" ;value:DSC～ )

        // 限制图像尺寸,名为original的图像尺寸满足阈值要求
        int orig_width = image->width();
        image = limit_image_size(image, MAX_PIXELS);
        if (orig_width == image->width() && has_jpeg_extension(fname))
            view->set_image_ref(afname, "original");// Sets an image reference into the current view
        else
            view->set_image(image, "original");//Sets an image into the current view and marks it dirty

        add_exif_to_view(view, exif);//add the blob into the view

        scene->get_views().push_back(view);

        /***保存视角信息到本地****/ //以view_00xx_mve保存一个VIEW的内容
        std::string mve_fname = make_image_name(num_imported);
        std::cout << "Importing image: " << fname
                  << ", writing MVE view: " << mve_fname << "..." << std::endl;
        view->save_view_as(util::fs::join_path(views_path, mve_fname));//Save meta data, images and BLOBS

        num_imported+=1;
    }

    std::cout << "Imported " << num_imported << " input images, "
              << "took " << timer.get_elapsed() << " ms." << std::endl;

    return scene;
}
int main(int argc, char *argv[])
{

    if(argc < 3){
        std::cout<<"Usage: [input]image_dir [output]scene_dir"<<std::endl;
        return -1;
    }

    core::Scene::Ptr scene = make_scene(argv[1], argv[2]);
    std::cout<<"Scene has "<<scene->get_views().size()<<" views. "<<std::endl;
    return 0;
}

