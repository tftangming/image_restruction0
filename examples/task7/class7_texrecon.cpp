/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <fstream>
#include <vector>

#include <util/timer.h>
#include <util/system.h>
#include <util/file_system.h>
#include <core/mesh_io_ply.h>
#include <core/image_tools.h>
#include <core/image_drawing.h>
#include "core/image_io.h"

#include "texturing/util.h"
#include "texturing/timer.h"
#include "texturing/debug.h"
#include "texturing/texturing.h"
#include "texturing/progress_counter.h"

#include "arguments.h"
int main(int argc, char **argv) {
#ifdef RESEARCH
    std::cout << "******************************************************************************" << std::endl
              << " Due to use of the -DRESEARCH=ON compile option, this program is licensed "     << std::endl
              << " for research purposes only. Please pay special attention to the gco license."  << std::endl
              << "******************************************************************************" << std::endl;
#endif

    util::system::register_segfault_handler();
    Timer timer;
    timer.measure("Start");
    util::WallTimer wtimer;
    Arguments conf;
    try {
        //利用输入参数配置conf
        conf = parse_args(argc, argv);
/**
    conf.settings.data_term = GMI;//1
    conf.settings.smoothness_term = POTTS;//0
    conf.settings.outlier_removal = NONE; //0
    conf.settings.geometric_visibility_test = true;
    conf.settings.global_seam_leveling = true;
    conf.settings.local_seam_leveling = true;

    conf.write_timings = false;
    conf.write_intermediate_results = true;
    conf.write_view_selection_model = false;

    conf.in_scene
    conf.in_mesh
    conf.out_prefix
    conf.data_cost_file
    conf.labeling_file
**/
    } catch (std::invalid_argument & ia) {
        std::cerr << ia.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    //==================================检测输出纹理网格文件名是否存在===================================//
    if (!util::fs::dir_exists(util::fs::dirname(conf.out_prefix).c_str())) {
        std::cerr << "Destination directory does not exist!" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    //==================================加载后缀名为.ply形式的输入网格===================================//
    std::cout << "Load and prepare mesh: " << std::endl;
    core::TriangleMesh::Ptr mesh;
    try {
        mesh = core::geom::load_ply_mesh(conf.in_mesh);
    } catch (std::exception& e) {
        std::cerr << "\tCould not load mesh: "<< e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    //===================================准备网格================================================//
    //根据网格计算顶点信息（邻近面片、邻近顶点等）
    //先移除重复的面片[需要保证顶点和面片向量和法线的一致性（不一致则更新计算顶点和面片的法线ensure_normals)];
    core::VertexInfoList::Ptr vertex_infos = core::VertexInfoList::create(mesh);
    tex::prepare_mesh(vertex_infos, mesh);
    //=================================创建纹理视角======================================//
    std::size_t const num_faces = mesh->get_faces().size() / 3;
    std::cout << "Generating texture views: " << std::endl;
    tex::TextureViews texture_views;//一个纹理视角包含：视图序号、相机中心位置、内参矩阵、外参矩阵、相机朝向、图像宽度和高度、图像名称、图像内容和对应的梯度幅度图、有效掩模向量.  图像的尺寸和畸变矫正的图像一致
    tex::generate_texture_views(conf.in_scene, &texture_views);//由输入参数的场景文件夹和指定的输入图像（undistort.png）产生对应的纹理视角（缺少图像内容和对应的梯度幅度图、有效掩模向量） <from_core_scene>
    write_string_to_file(conf.out_prefix + ".conf", conf.to_string());//输出配置信息,生成.conf配置文件
    timer.measure("Loading");

    //===============================Building adjacency graph=======================//
    std::cout << "Building adjacency graph: " << std::endl; // each facet is corresponding a facet
    tex::Graph graph(num_faces);//由面片数目创建 邻接图(adj_lists向量和labels向量）
    tex::build_adjacency_graph(mesh, vertex_infos, &graph);//根据网格与顶点信息，创建 邻接图（一个向量表示所有面片序号附带的所有邻接面片，一个向量表示 所有面片序号对应的标签 ，一个数据edge表示邻接图上边的数目）
  //  std::cout << graph.get_label(0) << graph.get_label(1) << graph.get_label(2) << std::endl; //测试结果表明 邻接图 初始的标签向量所有元素都为0
    wtimer.reset();

    //===============================View Selection ================================//
    // if labeling file does not exist, compute a view label for each facet via MRF
    if (conf.labeling_file.empty()) {
        std::cout << "View selection:" << std::endl;
        std::size_t const num_faces = mesh->get_faces().size() / 3;
        tex::DataCosts data_costs(num_faces, texture_views.size());//创建稀疏表(列数为面片的数目,行数为纹理视角的数目)
        // if data cost file does not exist, compute the data costs
        if (conf.data_cost_file.empty()) {

            /*******************     计算代价值     ********************/
            //(1)建立 碰撞检测模型,设置三角形数目为面片数,添加每个面片上三个顶点的信息作为模型中的三角形.然后开始处理模型;
            //(2)创建projected_face_infos <->面片投影信息(向量，每个元素表示单个面片在不同纹理视角下的投影信息,所以向量长度为面片数目);
            //(3)遍历每个纹理视角,首先加载纹理视角的图像内容、生成有效掩模向量（反映视角图像上具有有效像素值的像素index）、生成梯度幅度图(sobel算子)、将有效掩模向量中边界附近的位置进行腐蚀置false、计算相机中心位置及相机朝向
            //                  遍历每个面片,进行可视性筛选：(a)计算面片中心位置和视线方向,计算面片法向量与负视线方向夹角α(0<α<75)、视线方向和相机朝向的夹角β（>0),面片投影点应落在纹理视角的有效位置上，若不满足则跳过该面片；
            //                                           (b)对面片三个顶点作碰撞模型检测（顶点发射一条指向相机中心的射线，判断其与模型中的三角形是否存在交点）来判断三个顶点是否全部可见,若不满足则跳过该面片;
            //                             对通过可视性筛选的面片,计算该面片在该纹理视角上的投影信息ProjectedFaceInfo info={纹理视角序号,投影质量,面片投影均值颜色}：投影质量由面片投影面积和投影三角形区域内所有像素梯度和决定;
            //(4)将第三步计算的投影信息填入第二步的projected_face_infos向量中,每一个元素表示对应位置的面片在各个纹理视图（可能不是所有，因为需要可视性筛选）下的投影信息组成的向量;
            //(5)删除碰撞检测模型后,通过光度外子检测方法对投影信息进行调整：可能会出现以下情况->将一个面片投影在各个视图下,在某个视图上的投影均值颜色很奇怪，因此需要剔除该投影信息;该程序中setting配置信息并没有执行外子检测移除步骤;
            //(6)找到projected_face_infos中的最大投影质量并做归一化处理,通过取反加一再乘上MRF能量项因子得到每个面片在每个纹理视角下的代价值,最终填入先前创建的稀疏表中
            tex::calculate_data_costs(mesh, &texture_views, conf.settings, &data_costs);

            //将代价值的稀疏表写成_data_costs.spt的文件,存储在指定的输出路径下
            if (conf.write_intermediate_results) {
                std::cout << "\tWriting data cost file... " << std::flush;
                    ST::save_to_file(data_costs, conf.out_prefix + "_data_costs.spt");
                std::cout << "done." << std::endl;
            }
        }
        else {  // if the data cost file exists, just load it from the file
            std::cout << "\tLoading data cost file... " << std::flush;
            try {
                ST::load_from_file(conf.data_cost_file, &data_costs);
            } catch (util::FileException e) {
                std::cout << "failed!" << std::endl;
                std::cerr << e.what() << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::cout << "done." << std::endl;
        }
        timer.measure("Calculating data costs");
        /*******************     MRF优化（for view selection）     ********************/
        //(1)可能会出现某些面片在所有纹理视角上都不存在代价值(可能是未通过可视性或者光度外子检测剔除了),首先需要在邻接图上将该面片表示的节点周边的边删除掉;
        //(2)初始标签为0.根据标签0对所有面片做聚类(在邻接图上能用边连接起来的那些面片),得到多个连通集合components;
        //(3)创建face infos向量,存储了每个面片的连通index ->（components[i][j]是面片序号作为向量下标, 下标对应的元素{i,j}是该面片序号在components中的地址;
        //(4)构建多个MRF的图,每个图用来对一个连通集合做MRF优化,创建MRF图时需要给定连通集合中的面片(图中的顶点)数目、待计算的标签数目、求解器类型mrf::LBP(优化时的算法思路...看不太懂);
        //(5)创建LBP图时需要设置图上的邻居(边)的信息,当添加完图上的邻居后设置图上顶点的信息(标签+对应的代价值等);
        //(6)在每个连通集合对应的mrf图上,设置平滑函数smooth_cost_func = potts,通过多次迭代优化知道能量不再下降停止优化,最终将mrf图中的顶点标签信息转换到邻接图上的labels向量中,至此完成了所有面片的标签选择流程;

        tex::view_selection(data_costs, &graph, conf.settings);
        timer.measure("Running MRF optimization");

        //将优化得到的各个面片的标签构建一个向量,写成_labeling.vec的文件,存储在指定的输出路径下
        if (conf.write_intermediate_results) {
            std::vector<std::size_t> labeling(graph.num_nodes());
            for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
                labeling[i] = graph.get_label(i);
            }
            vector_to_file(conf.out_prefix + "_labeling.vec", labeling);
        }
    }
    else {  // if labeling file has existed, just read it from the file
        std::cout << "Loading labeling from file... " << std::flush;

        /* Load labeling from file. */
        std::vector<std::size_t> labeling = vector_from_file<std::size_t>(conf.labeling_file);
        if (labeling.size() != graph.num_nodes()) {
            std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        /* Transfer labeling to graph. */
        for (std::size_t i = 0; i < labeling.size(); ++i) {
            const std::size_t label = labeling[i];
            if (label > texture_views.size()){
                std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            graph.set_label(i, label);
        }

        std::cout << "done." << std::endl;
    }
    std::cout << "\tTook: " << wtimer.get_elapsed_sec() << "s" << std::endl;

    /*******************     用不同颜色值渲染网格     ********************/
    {
        std::cout << "Start to render the mesh according to the selected labels" << std::endl;
        std::vector<std::size_t> labeling(graph.num_nodes());
        for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
            labeling[i] = graph.get_label(i);
        }
        std::vector<std::size_t>::iterator max_iter = std::max_element(labeling.begin(), labeling.end());
        int n_labels = *max_iter + 1;
        std::vector<math::Vec3f> colors(n_labels);
        colors[0][0] = 0;
        colors[0][1] = 0;
        colors[0][2] = 0;

        for(int i=1; i< colors.size(); i++){
            colors[i][0] = rand()&255;
            colors[i][1] = rand()&255;
            colors[i][2] = rand()&255;
        }

        std::ofstream out("./intermediate/view_selection_result.ply");
        assert(out.is_open());
        out<<"ply"<<std::endl;
        out<<"format ascii 1.0"<<std::endl;
        out<<"element vertex "<<mesh->get_vertices().size()<<std::endl;
        out<<"property float x"<<std::endl;
        out<<"property float y"<<std::endl;
        out<<"property float z"<<std::endl;
        out<<"element face "<<mesh->get_faces().size()/3<<std::endl;
        out<<"property list uchar int vertex_indices"<<std::endl;
        out<<"property uchar red"<<std::endl;
        out<<"property uchar green"<<std::endl;
        out<<"property uchar blue"<<std::endl;
        out<<"end_header"<<std::endl;

        // Face List
        core::TriangleMesh::FaceList const & mesh_faces = mesh->get_faces();
        // Vertices
        core::TriangleMesh::VertexList const & vertices = mesh->get_vertices();
        for(int i=0; i< vertices.size(); i++){
            out<<vertices[i][0]<<" "<<vertices[i][1]<<" "<<vertices[i][2]<<std::endl;
        }

        std::cout<<"labeling size: "<<labeling.size()<<" faces size: "<<num_faces<<std::endl;
        for(int i=0; i< labeling.size(); i++){
            int label = labeling[i];
            assert(label>=0 && label <labeling.size());
            int v0 = mesh_faces[3*i + 0];
            int v1 = mesh_faces[3*i + 1];
            int v2 = mesh_faces[3*i + 2];
            int r = colors[label][0];
            int g = colors[label][1];
            int b = colors[label][2];
            out<<"3 "<<v0<<" "<<v1<<" "<<v2<<" "<<r<<" "<<g<<" "<<b<<std::endl;
        }
        out.close();
        std::cout << "Success to generate the view_selection_result.ply" << std::endl;
    }

    //=============================================texture_atlases====================================================//
    tex::TextureAtlases texture_atlases;
    {
        /* 纹理视图的创建与调整 */
        tex::TexturePatches texture_patches;//texture_patches包括各个纹理视角下的各个候选纹理面片的texture_patch.
        //顶点投影信息 向量中的每个元素反映了 该处顶点的投影信息,向量长度为网格顶点个数.
        tex::VertexProjectionInfos vertex_projection_infos;// vector<vector<VertexProjectionInfo> >  ;  VertexProjectionInfo 包括 纹理视图id/二维投影坐标/所在面片序号向量
        std::cout << "Generating texture patches:" << std::endl;
        /*******************     生成纹理视图     ********************/
        //(1)遍历每个纹理视角,加载纹理视角中的undistort.png图像,并确定标签(纹理视角的序号+1),在邻接图上获取属于该标签下的所有连通图
        //           遍历每个连通图,结合网格得到连通图上的面片序号和面片顶点序号,计算所有面片上的三个顶点投影在纹理视角下的投影坐标存储在texcoords中,,并记录包裹这些投影点的矩形框bounding_box
        //                        规范化所有二维投影点的坐标,(横纵坐标减去矩形框的左上角横纵坐标)得到相对投影坐标;
        //                        根据矩形框对纹理视角加载的图像进行裁剪,得到image;
        //                        根据标签,连通图,texcoords以及裁剪得到的image生成texture_patch {label,faces,texcoords,image,validity_mask(255),blending_mask(0)}
        //                        {bounding_box, texture_patch}组成一个 TexturePatchCandidate;
        //           将每个连通图的TexturePatchCandidate候选纹理面片组成一个列表,如果一个候选纹理面片的矩形框在另一个候选纹理面片的矩形框中,那么将小矩形框对应的候选纹理面片合并到大矩形框的候选纹理面片中,所以列表的大小可能会改变
        //           遍历候选纹理面片列表,遍历所有候选纹理面片中的texture_patch,首先将他们依次存储在texture_patches中,并记录texture_patch_id
        //                             同时提取所有texture_patch.faces内所有的三角面片的顶点,记录每个顶点的顶点投影信息VertexProjectionInfo={texture_patch_id, projection, {face_id}}
        //(2)将各个纹理视角下 所有顶点的顶点投影信息vertex_projection_infos 放入到vertex_projection_infos向量中,每个元素都是关于每个顶点的投影信息,这些投影信息有可能面片序号不一样,所以需要进行合并即出现了{face_id};
        //(3)处理空洞,只能修补小的空洞;此处没看懂！！！！！！
        tex::generate_texture_patches(graph,
                                      mesh,
                                      vertex_infos,
                                      &texture_views,
                                      &vertex_projection_infos,
                                      &texture_patches);

        //存储纹理视图以及面片投影
        {
            for(int i=0; i< texture_patches.size(); i++)
            {
                if(texture_patches[i]->get_faces().size()<800)continue;

                char image_name[255];
                char validity_mask_name[255];
                char blending_mask_name[255];

                sprintf(image_name,"intermediate/texture_patches_init/texture_patch%d.jpg", i);
                sprintf(blending_mask_name,"intermediate/texture_patches_init/blending_mask%d.jpg", i);
                sprintf(validity_mask_name,"intermediate/texture_patches_init/validity_mask%d.jpg", i);

                core::FloatImage::Ptr image = texture_patches[i]->get_image()->duplicate();
                core::ByteImage::Ptr validity_mask = texture_patches[i]->get_validity_mask()->duplicate();
                core::ByteImage::Ptr blending_mask = texture_patches[i]->get_blending_mask()->duplicate();
                //对纹理面片内的图像进行绘图
                /*
    float color[3]={255, 0,0};
    std::vector<math::Vec2f> texcoords = texture_patches[i]->get_texcoords();
    for(int i=0; i< texcoords.size(); i+=3){
        math::Vec2f v0 = texcoords[i+0];
        math::Vec2f v1 = texcoords[i+1];
        math::Vec2f v2 = texcoords[i+2];

        core::image::draw_line<float>(*image, int(v0[0]), int(v0[1]), int(v1[0]), int(v1[1]), color);
        core::image::draw_line<float>(*image, int(v1[0]), int(v1[1]), int(v2[0]), int(v2[1]), color);
        core::image::draw_line<float>(*image, int(v0[0]), int(v0[1]), int(v2[0]), int(v2[1]), color);

    }
*/
    core::image::save_file(core::image::float_to_byte_image(image), image_name);
    core::image::save_file(validity_mask, validity_mask_name);
    core::image::save_file(blending_mask, blending_mask_name);

}
}

/*******************     全局缝隙优化    ********************/
        //(1)获得所有顶点的所有标签类型;
        //(2)生成稀疏矩阵A,L以及差值向量b;
        //(3)利用共轭梯度法求解上述的线性系统;此处关于如何使用eigen库中的共轭梯度法求解问题需要学习！！！！！！
        //(4)因为系统属于欠约束,需要对求解得到的调整量去均值处理;
        //(5)对texture_patch进行颜色调整,同时更新validity_mask和blend_mask.这里需要注意,程序对texture_patch上三角面片投影下来的区域做了膨胀处理,
        //   即处于投影区域外的附近区域(一个像素)也有对应的颜色调整量,对应的validity_mask值为255,blend_mask值为128.
        //   另外,texture_patch的image上只有有效像素才有颜色值(在validity_mask对应位置的值为255)
        if (conf.settings.global_seam_leveling) {
            std::cout << "Running global seam leveling:" << std::endl;
            tex::global_seam_leveling(graph,
                                      mesh,
                                      vertex_infos,
                                      vertex_projection_infos,
                                      &texture_patches);
            timer.measure("Running global seam leveling");

            {
                for(int i=0; i< texture_patches.size(); i++)
                {
                    if(texture_patches[i]->get_faces().size()<800)continue;

                    char image_name[255];
                    char validity_mask_name[255];
                    char blending_mask_name[255];

                    sprintf(image_name,"intermediate/texture_pathes_color_adjustment/texture_patch%d.jpg", i);
                    sprintf(blending_mask_name,"intermediate/texture_pathes_color_adjustment/blending_mask%d.jpg", i);
                    sprintf(validity_mask_name,"intermediate/texture_pathes_color_adjustment/validity_mask%d.jpg", i);

                    core::FloatImage::Ptr image = texture_patches[i]->get_image()->duplicate();
                    core::ByteImage::Ptr validity_mask = texture_patches[i]->get_validity_mask()->duplicate();
                    core::ByteImage::Ptr blending_mask = texture_patches[i]->get_blending_mask()->duplicate();
            /*
                    float color[3]={255, 0,0};
                    std::vector<math::Vec2f> texcoords = texture_patches[i]->get_texcoords();
                    for(int i=0; i< texcoords.size(); i+=3){
                        math::Vec2f v0 = texcoords[i+0];
                        math::Vec2f v1 = texcoords[i+1];
                        math::Vec2f v2 = texcoords[i+2];

                        core::image::draw_line<float>(*image, int(v0[0]), int(v0[1]), int(v1[0]), int(v1[1]), color);
                        core::image::draw_line<float>(*image, int(v1[0]), int(v1[1]), int(v2[0]), int(v2[1]), color);
                        core::image::draw_line<float>(*image, int(v0[0]), int(v0[1]), int(v2[0]), int(v2[1]), color);

                    }
            */
                    core::image::save_file(core::image::float_to_byte_image(image), image_name);
                    core::image::save_file(validity_mask, validity_mask_name);
                    core::image::save_file(blending_mask, blending_mask_name);

                }
            }

        }
        else {
            ProgressCounter texture_patch_counter("Calculating validity masks for texture patches", texture_patches.size());
            #pragma omp parallel for schedule(dynamic)
            for (std::size_t i = 0; i < texture_patches.size(); ++i) {
                texture_patch_counter.progress<SIMPLE>();
                TexturePatch::Ptr texture_patch = texture_patches[i];
                std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
                texture_patch->adjust_colors(patch_adjust_values);
                texture_patch_counter.inc();
            }
            timer.measure("Calculating texture patch validity masks");
        }

        /*******************     局部缝隙优化（泊松图像编辑）    ********************/
        //(1)备份各个纹理视图的image作为前景图像,真正用于前景图像的那部分是后续计算的边界内部区域的图像
        //(2)找到缝隙上所有的边,对缝隙上的每一条边L(存在于两个纹理视图上):在每一副纹理视图上,都会有L投影下来的一条边,对这条边进行多线段插值拟合,计算各纹理视图上各个节点的像素值且对应的blend_mask值为126
        //(3)对于缝隙处的顶点(含有多个标签属性),需要计算该顶点在各个纹理视图上的投影点的颜色均值α,在各个纹理视图上更新该顶点投影点的颜色值为α且设置对应的blend_mask值为126;(这一步得到了背景图像)
        //(4)寻找边界然后进行多次腐蚀处理,且在每个纹理视图上未混合区域的blend_mask值设为0,最新的边界区域的blend_mask值设为126;
        //(5)泊松混合:根据数学模型构建线性系统,利用前景图像和背景图像混合
        if (conf.settings.local_seam_leveling) {
            std::cout << "Running local seam leveling:" << std::endl;
            tex::local_seam_leveling(graph, mesh, vertex_projection_infos, &texture_patches);
        }
        timer.measure("Running local seam leveling");
        
        /*******************     生成纹理贴图    ********************/
        //(1)找到这些纹理面片上的最大像素值和最小像素值,用来做后期映射;
        //(2)按照纹理面片尺寸升序排列texture_patches;<该程序似乎并没有实现排序>
        //(3)计算纹理贴图的尺寸,并将这些纹理面片排布上去,得到最紧密的排布;装满了一个纹理贴图后,将已完成的纹理面片清楚根据剩余的纹理面片计算纹理贴图的尺寸,继续执行直至将所有纹理面片全部排布上去;
        /* Generate texture atlases. */
        std::cout << "Generating texture atlases:" << std::endl;
        tex::generate_texture_atlases(&texture_patches, &texture_atlases);
    }


    /*******************     生成.obj文件    ********************/

    {
        std::cout << "Building objmodel:" << std::endl;
        tex::Model model;
        tex::build_model(mesh, texture_atlases, &model);
        timer.measure("Building OBJ model");

        std::cout << "\tSaving model... " << std::flush;
        //(1)保存材料库,生成.mtl文件
        //(2)按行写入每一个顶点的三维坐标
        //(3)按行写入每一个纹理坐标的横纵坐标（为何要对对纵坐标取反加一）
        //(4)按行写入每一个顶点的法向量信息
        //(5)配置纹理信息：
        //   usemtl material0000     //这一行表示使用material0000纹理贴图
        //   f 0/1/2 2/3/4 80/20/40  //这一行表示group上一个面片上三个顶点的信息(顶点序号、纹理坐标序号、法向量序号),所以有许多行
        tex::Model::save(model, conf.out_prefix);
        std::cout << "done." << std::endl;
        timer.measure("Saving");
    }

    std::cout << "Whole texturing procedure took: " << wtimer.get_elapsed_sec() << "s" << std::endl;
    timer.measure("Total");
    if (conf.write_timings) {
        timer.write_to_file(conf.out_prefix + "_timings.csv");
    }

    if (conf.write_view_selection_model) {
        texture_atlases.clear();
        std::cout << "Generating debug texture patches:" << std::endl;
        {
            tex::TexturePatches texture_patches;
            generate_debug_embeddings(&texture_views);
            tex::VertexProjectionInfos vertex_projection_infos; // Will only be written
            tex::generate_texture_patches(graph, mesh, vertex_infos, &texture_views, &vertex_projection_infos, &texture_patches);
            tex::generate_texture_atlases(&texture_patches, &texture_atlases);
        }

        std::cout << "Building debug objmodel:" << std::endl;
        {
            tex::Model model;
            tex::build_model(mesh, texture_atlases, &model);
            std::cout << "\tSaving model... " << std::flush;
            tex::Model::save(model, conf.out_prefix + "_view_selection");
            std::cout << "done." << std::endl;
        }
    }
}
