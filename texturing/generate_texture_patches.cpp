/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <set>
#include <list>

#include <util/timer.h>
#include <core/image_tools.h>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "texturing.h"

TEX_NAMESPACE_BEGIN

void merge_vertex_projection_infos(std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos) {
    /* Merge vertex infos within the same texture patch. */
    #pragma omp parallel for
    for (std::size_t i = 0; i < vertex_projection_infos->size(); ++i) {

        // vertex projection infomation of each vertex
        std::vector<VertexProjectionInfo> & infos = vertex_projection_infos->at(i);

        std::map<std::size_t, VertexProjectionInfo> info_map;
        std::map<std::size_t, VertexProjectionInfo>::iterator it;

        for (VertexProjectionInfo const & info : infos) {
            std::size_t texture_patch_id = info.texture_patch_id;
            if((it = info_map.find(texture_patch_id)) == info_map.end()) {
                info_map[texture_patch_id] = info;
            } else {
                it->second.faces.insert(it->second.faces.end(),
                    info.faces.begin(), info.faces.end());
            }
        }

        infos.clear();
        infos.reserve(info_map.size());
        for (it = info_map.begin(); it != info_map.end(); ++it) {
            infos.push_back(it->second);
        }
    }
}

/** Struct representing a TexturePatch candidate - final texture patches are obtained by merging candiates. */
struct TexturePatchCandidate {
    Rect<int> bounding_box;
    TexturePatch::Ptr texture_patch;
};

/** Create a TexturePatchCandidate by calculating
  * the faces' bounding box projected into the view,
  *  relative texture coordinates
  *  and extracting the texture views relevant part
  */
TexturePatchCandidate
generate_candidate(int label, TextureView const & texture_view,
    std::vector<std::size_t> const & faces, core::TriangleMesh::ConstPtr mesh) {

    // view image
    core::ByteImage::Ptr view_image = texture_view.get_image();
    int min_x = view_image->width(), min_y = view_image->height();
    int max_x = 0, max_y = 0;

    // all the faces in the mesh
    core::TriangleMesh::FaceList const & mesh_faces = mesh->get_faces();
    // all the vertices in the mesh
    core::TriangleMesh::VertexList const & vertices = mesh->get_vertices();

    // compute texture coordinates
    std::vector<math::Vec2f> texcoords;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            math::Vec3f vertex = vertices[mesh_faces[faces[i] * 3 + j]];
            math::Vec2f pixel = texture_view.get_pixel_coords(vertex);

            texcoords.push_back(pixel);

            min_x = std::min(static_cast<int>(std::floor(pixel[0])) - texture_patch_border, min_x);// texture_patch_border = 1
            min_y = std::min(static_cast<int>(std::floor(pixel[1])) - texture_patch_border, min_y);
            max_x = std::max(static_cast<int>(std::ceil(pixel[0])) + texture_patch_border, max_x);
            max_y = std::max(static_cast<int>(std::ceil(pixel[1])) + texture_patch_border, max_y);
        }
    }

    /* Calculate the relative texcoords. */
    math::Vec2f min(min_x, min_y);
    for (std::size_t i = 0; i < texcoords.size(); ++i) {
        texcoords[i] = texcoords[i] - min;
    }

    int const width = max_x - min_x;
    int const height = max_y - min_y;

    /* Check for valid projections/erroneous labeling files. */
    assert(min_x >= 0 - texture_patch_border);
    assert(min_y >= 0 - texture_patch_border);
    assert(max_x < view_image->width() + texture_patch_border);
    assert(max_y < view_image->height() + texture_patch_border);

    core::ByteImage::Ptr image;
    image = core::image::crop(view_image, width, height, min_x, min_y, *math::Vec3uc(255, 0, 255));

    // gamma corrections of an image
    core::image::gamma_correct(image, 2.2f);

    TexturePatchCandidate texture_patch_candidate =
        {Rect<int>(min_x, min_y, max_x, max_y),
            TexturePatch::create(label, faces, texcoords, image)};
    return texture_patch_candidate;
}
// generate texture patches
void
generate_texture_patches(UniGraph const & graph, core::TriangleMesh::ConstPtr mesh,
    core::VertexInfoList::ConstPtr vertex_infos,
    std::vector<TextureView> * texture_views,
    std::vector<std::vector<VertexProjectionInfo> > * vertex_projection_infos,
    std::vector<TexturePatch::Ptr> * texture_patches) {

    util::WallTimer timer;

    // 网格面片列表
    core::TriangleMesh::FaceList const & mesh_faces = mesh->get_faces();
    // 网格顶点列表
    core::TriangleMesh::VertexList const & vertices = mesh->get_vertices();
    // 设置 顶点投影信息 向量大小（顶点个数） -> 顶点投影信息向量中的每个元素反映了该处顶点的投影信息
    vertex_projection_infos->resize(vertices.size());

    //==============================生成视图 ================================================//
    std::size_t num_patches = 0;
    std::cout << "\tRunning... " << std::flush;
    #pragma omp parallel for schedule(dynamic)
    //对于每一个纹理视角
    for (std::size_t i = 0; i < texture_views->size(); ++i) {

        // 对每一个纹理视角,在邻接图上获取属于该标签下的 所有连通图
        std::vector<std::vector<std::size_t> > subgraphs;
        int const label = i + 1;
        graph.get_subgraphs(label, &subgraphs);

        // 加载纹理视角中的undistort.png图像
        TextureView * texture_view = &texture_views->at(i);
        texture_view->load_image();

        // 根据标签、纹理视角、网格以及每一个连通图 产生 每一个候选的纹理视图,具体步骤如下：
        //（1）根据连通图上记录的面片序号,结合网格提取这些面片上的顶点位置,并计算所有面片上的三个顶点投影在给定的纹理视角下,得到二位投影坐标,存储在 texcoords中,并记录包裹这些投影点的矩形框(四个方向向外扩张一个像素)
        //（2）规范化所有二维投影点的坐标,横纵坐标减去矩形框的左上角横纵坐标;
        //（3）根据矩形框对纹理视角加载的图像进行裁剪,得到image;
        // (4) 根据提供的标签,连通图,纹理坐标集合以及裁剪得到的image创建一个 texture_patch ,其中自动创建了 validity_mask(值为255)、blending_mask(值为0)； texturep_atch加上矩形框 就是我们要得到的候选纹理视图
        std::list<TexturePatchCandidate> candidates;
        for (std::size_t j = 0; j < subgraphs.size(); ++j) {
            candidates.push_back(generate_candidate(label, *texture_view, subgraphs[j], mesh));
        }
        //释放纹理视角中存储图像的内存
        texture_view->release_image();

        //如果一个候选纹理视图的矩形框在另一个候选纹理视图的矩形框中,那么将小矩形框对应的候选纹理视图合并到大矩形框的候选纹理视图中.
        std::list<TexturePatchCandidate>::iterator it, sit;
        for (it = candidates.begin(); it != candidates.end(); ++it) {
            for (sit = candidates.begin(); sit != candidates.end();) {
                Rect<int> bounding_box = sit->bounding_box;
                if (it != sit && bounding_box.is_inside(&it->bounding_box)) {
                    TexturePatch::Faces & faces = it->texture_patch->get_faces();
                    TexturePatch::Faces & ofaces = sit->texture_patch->get_faces();
                    faces.insert(faces.end(), ofaces.begin(), ofaces.end());

                    TexturePatch::Texcoords & texcoords = it->texture_patch->get_texcoords();
                    TexturePatch::Texcoords & otexcoords = sit->texture_patch->get_texcoords();
                    math::Vec2f offset;
                    offset[0] = sit->bounding_box.min_x - it->bounding_box.min_x;
                    offset[1] = sit->bounding_box.min_y - it->bounding_box.min_y;
                    for (std::size_t i = 0; i < otexcoords.size(); ++i) {
                        texcoords.push_back(otexcoords[i] + offset);
                    }

                    sit = candidates.erase(sit);
                } else {
                    ++sit;
                }
            }
        }

        it = candidates.begin();
        //对于每一个候选的纹理视图
        for (; it != candidates.end(); ++it) {
            std::size_t texture_patch_id;

            #pragma omp critical
            {
                texture_patches->push_back(it->texture_patch);//texture_patches存储这各个纹理视角下的texture_patch
                texture_patch_id = num_patches++;
            }

            std::vector<std::size_t> const & faces = it->texture_patch->get_faces();
            std::vector<math::Vec2f> const & texcoords = it->texture_patch->get_texcoords();
            //对于纹理视图上的每一个面片
            for (std::size_t i = 0; i < faces.size(); ++i) {
                std::size_t const face_id = faces[i];
                std::size_t const face_pos = face_id * 3;
                //对于面片上的每一个顶点
                for (std::size_t j = 0; j < 3; ++j) {
                    std::size_t const vertex_id = mesh_faces[face_pos  + j];
                    math::Vec2f const projection = texcoords[i * 3 + j];
                    //建立 顶点投影信息info,包括 纹理视图id、二维投影坐标、面片序号
                    VertexProjectionInfo info = {texture_patch_id, projection, {face_id}};

                    #pragma omp critical
                    vertex_projection_infos->at(vertex_id).push_back(info);
                }
            }
        }
    }

    //  merge vertex projection information
    merge_vertex_projection_infos(vertex_projection_infos);


    //================================= process (fill) holes ============================================//
    std::size_t num_holes = 0;
    std::size_t num_hole_faces = 0;

    //if (!settings.skip_hole_filling) {
    {
        std::vector<std::vector<std::size_t> > subgraphs;
        graph.get_subgraphs(0, &subgraphs);

        #pragma omp parallel for schedule(dynamic)
        for (std::size_t i = 0; i < subgraphs.size(); ++i) {

            // for each subgraph  // each subgraph is a hole????
            std::vector<std::size_t> const & subgraph = subgraphs[i];

            // neighboring facet of each vertex
            std::map<std::size_t, std::set<std::size_t> > tmp;
            for (std::size_t const face_id : subgraph) {
                std::size_t const v0 = mesh_faces[face_id * 3];
                std::size_t const v1 = mesh_faces[face_id * 3 + 1];
                std::size_t const v2 = mesh_faces[face_id * 3 + 2];
                tmp[v0].insert(face_id);
                tmp[v1].insert(face_id);
                tmp[v2].insert(face_id);
            }


            std::size_t const num_vertices = tmp.size();
            /* Only fill small holes. */
            if (num_vertices > 100) {
                //std::cerr << "Hole to large" << std::endl;
                continue;
            }


            /* Calculate 2D parameterization using the technique from libremesh/patch2d,
             * which was published as sourcecode accompanying the following paper:
             *
             * Isotropic Surface Remeshing
             * Simon Fuhrmann, Jens Ackermann, Thomas Kalbe, Michael Goesele
             */

            std::size_t seed = -1;
            std::vector<bool> is_border(num_vertices, false);
            std::vector<std::vector<std::size_t> > adj_verts_via_border(num_vertices);
            /* Index structures to map from local <-> global vertex id. */
            std::map<std::size_t, std::size_t> g2l;
            std::vector<std::size_t> l2g(num_vertices);
            /* Index structure to determine column in matrix/vector. */
            std::vector<std::size_t> idx(num_vertices);
            std::size_t num_border_vertices = 0;

            bool disk_topology = true;
            std::map<std::size_t, std::set<std::size_t> >::iterator it = tmp.begin();
            for (std::size_t j = 0; j < num_vertices; ++j, ++it) {
                // index of the vertex
                std::size_t vertex_id = it->first;
                g2l[vertex_id] = j;
                l2g[j] = vertex_id;

                /* Check topology in original mesh. */
                if (vertex_infos->at(vertex_id).vclass != core::VERTEX_CLASS_SIMPLE) {
                    //std::cerr << "Complex/Border vertex in original mesh" << std::endl;
                    disk_topology = false;
                    break;
                }

                /* Check new topology and determine if vertex is now at the border. */
                std::vector<std::size_t> const & adj_faces = vertex_infos->at(vertex_id).faces;
                std::set<std::size_t> const & adj_hole_faces = it->second;
                std::vector<std::pair<std::size_t, std::size_t> > fan;

                for (std::size_t k = 0; k < adj_faces.size(); ++k) {
                    std::size_t adj_face = adj_faces[k];
                    if (graph.get_label(adj_faces[k]) == 0 && adj_hole_faces.find(adj_face) != adj_hole_faces.end()) {
                        std::size_t curr = adj_faces[k];
                        std::size_t next = adj_faces[(k + 1) % adj_faces.size()];
                        std::pair<std::size_t, std::size_t> pair(curr, next);
                        fan.push_back(pair);
                    }
                }

                std::size_t gaps = 0;
                for (std::size_t k = 0; k < fan.size(); k++) {
                    std::size_t curr = fan[k].first;
                    std::size_t next = fan[(k + 1) % fan.size()].first;
                    if (fan[k].second != next) {
                        ++gaps;

                        for (std::size_t l = 0; l < 3; ++l) {
                            if(mesh_faces[curr * 3 + l] == vertex_id) {
                                std::size_t second = mesh_faces[curr * 3 + (l + 2) % 3];
                                adj_verts_via_border[j].push_back(second);
                            }
                            if(mesh_faces[next * 3 + l] == vertex_id) {
                                std::size_t first = mesh_faces[next * 3 + (l + 1) % 3];
                                adj_verts_via_border[j].push_back(first);
                            }
                        }
                    }
                }

                is_border[j] = gaps == 1;

                /* Check if vertex is now complex. */
                if (gaps > 1) {
                    //std::cerr << "Complex vertex in hole" << std::endl;
                    disk_topology = false;
                    break;
                }

                if (is_border[j]) {
                    idx[j] = num_border_vertices++;
                    seed = vertex_id;
                } else {
                    idx[j] = j - num_border_vertices;
                }
            }
            tmp.clear();

            if (!disk_topology) continue;
            if (num_border_vertices == 0) {
                //std::cerr << "Genus zero topology" << std::endl;
                continue;
            }

            //===================== get the border of this subgraph =====================================//
            std::vector<std::size_t> border;
            border.reserve(num_border_vertices);
            std::size_t prev = seed;
            std::size_t curr = seed;
            while (prev == seed || curr != seed) {
                std::size_t next = std::numeric_limits<std::size_t>::max();
                std::vector<std::size_t> const & adj_verts = adj_verts_via_border[g2l[curr]];
                for (std::size_t adj_vert : adj_verts) {
                    assert(is_border[g2l[adj_vert]]);
                    if (adj_vert != prev && adj_vert != curr) {
                        next = adj_vert;
                        break;
                    }
                }
                if (next != std::numeric_limits<std::size_t>::max()) {
                    prev = curr;
                    curr = next;
                    border.push_back(next);
                } else {
                    //std::cerr << "No new border vertex" << std::endl;
                    border.clear();
                    break;
                }

                if (border.size() > num_border_vertices) {
                    //std::cerr << "Loop within border" << std::endl;
                    break;
                }
            }

            if (border.size() != num_border_vertices) {
                continue;
            }

            //============================compute the coordinates of the vertices of holes=========================//
            // compute total length and total projection length
            float total_length = 0.0f;
            float total_projection_length = 0.0f;
            for (std::size_t j = 0; j < border.size(); ++j) {
                std::size_t vi0 = border[j];
                std::size_t vi1 = border[(j + 1) % border.size()];
                std::vector<VertexProjectionInfo> const & vpi0 = vertex_projection_infos->at(vi0);
                std::vector<VertexProjectionInfo> const & vpi1 = vertex_projection_infos->at(vi0);
                /* According to the previous checks (vertex class within the origial
                 * mesh and boundary) there already has to be at least one projection
                 * of each border vertex. */
                assert(!vpi0.empty() && !vpi1.empty());
                math::Vec2f vp0(0.0f), vp1(0.0f);
                for (VertexProjectionInfo const & info0 : vpi0) {
                    for (VertexProjectionInfo const & info1 : vpi1) {
                        if (info0.texture_patch_id == info1.texture_patch_id) {
                            vp0 = info0.projection;
                            vp1 = info1.projection;
                            break;
                        }
                    }
                }
                total_projection_length += (vp0 - vp1).norm();
                math::Vec3f const & v0 = vertices[vi0];
                math::Vec3f const & v1 = vertices[vi1];
                total_length += (v0 - v1).norm();
            }
            float radius = total_projection_length / (2.0f * MATH_PI);

            float length = 0.0f;
            std::vector<math::Vec2f> projections(num_vertices);
            for (std::size_t j = 0; j < border.size(); ++j) {
                float angle = 2.0f * MATH_PI * (length / total_length);
                projections[g2l[border[j]]] = math::Vec2f(std::cos(angle), std::sin(angle));
                math::Vec3f const & v0 = vertices[border[j]];
                math::Vec3f const & v1 = vertices[border[(j + 1) % border.size()]];
                length += (v0 - v1).norm();
            }


            typedef Eigen::Triplet<float, int> Triplet;
            std::vector<Triplet> coeff;
            std::size_t matrix_size = num_vertices - border.size();

            Eigen::VectorXf xx(matrix_size), xy(matrix_size);

            if (matrix_size != 0) {
                Eigen::VectorXf bx(matrix_size);
                Eigen::VectorXf by(matrix_size);
                for (std::size_t j = 0; j < num_vertices; ++j) {
                    if (is_border[j]) continue;

                    std::size_t const vertex_id = l2g[j];

                    /* Calculate "Mean Value Coordinates" as proposed by Michael S. Floater */
                    std::map<std::size_t, float> weights;

                    // adjacent facets of each vertex
                    std::vector<std::size_t> const & adj_faces = vertex_infos->at(vertex_id).faces;
                    for (std::size_t adj_face : adj_faces) {
                        std::size_t v0 = mesh_faces[adj_face * 3];
                        std::size_t v1 = mesh_faces[adj_face * 3 + 1];
                        std::size_t v2 = mesh_faces[adj_face * 3 + 2];
                        if (v1 == vertex_id) std::swap(v1, v0);
                        if (v2 == vertex_id) std::swap(v2, v0);

                        math::Vec3f v01 = vertices[v1] - vertices[v0];
                        float v01n = v01.norm();
                        math::Vec3f v02 = vertices[v2] - vertices[v0];
                        float v02n = v02.norm();
                        float alpha = std::acos(v01.dot(v02) / (v01n * v02n));
                        weights[g2l[v1]] += std::tan(alpha / 2.0f) / v01n;
                        weights[g2l[v2]] += std::tan(alpha / 2.0f) / v02n;
                    }

                    // normalization of the weights
                    std::map<std::size_t, float>::iterator it;
                    float sum = 0.0f;
                    for (it = weights.begin(); it != weights.end(); ++it)
                        sum += it->second;
                    for (it = weights.begin(); it != weights.end(); ++it)
                        it->second /= sum;

                    bx[idx[j]] = 0.0f;
                    by[idx[j]] = 0.0f;
                    for (it = weights.begin(); it != weights.end(); ++it) {
                        if (is_border[it->first]) {
                            std::size_t border_vertex_id = border[idx[it->first]];
                            bx[idx[j]] += projections[g2l[border_vertex_id]][0] * it->second;
                            by[idx[j]] += projections[g2l[border_vertex_id]][1] * it->second;
                        } else {
                            coeff.push_back(Triplet(idx[j], idx[it->first], -it->second));
                        }
                    }
                }
                for (std::size_t j = 0; j < matrix_size; ++j) {
                    coeff.push_back(Triplet(j, j, 1.0f));
                }

                typedef Eigen::SparseMatrix<float> SpMat;
                SpMat A(matrix_size, matrix_size);
                A.setFromTriplets(coeff.begin(), coeff.end());

                Eigen::SparseLU<SpMat> solver;
                solver.analyzePattern(A);
                solver.factorize(A);
                xx = solver.solve(bx);
                xy = solver.solve(by);
            }

            int image_size = std::floor(radius * 1.1f) * 2 + 4;
            int scale = image_size / 2 - texture_patch_border;
            for (std::size_t j = 0, k = 0; j < num_vertices; ++j) {
                if (!is_border[j]) {
                    projections[j] = math::Vec2f(xx[k], xy[k]) * scale + image_size / 2;
                    ++k;
                } else {
                    projections[j] = projections[j] * scale + image_size / 2;
                }
            }

            core::ByteImage::Ptr image = core::ByteImage::create(image_size, image_size, 3);
            //DEBUG image->fill_color(*math::Vec4uc(0, 255, 0, 255));
            std::vector<math::Vec2f> texcoords; texcoords.reserve(subgraph.size());
            for (std::size_t const face_id : subgraph) {
                for (std::size_t j = 0; j < 3; ++j) {
                    std::size_t const vertex_id = mesh_faces[face_id * 3 + j];
                    math::Vec2f const & projection = projections[g2l[vertex_id]];
                    texcoords.push_back(projection);
                }
            }
            TexturePatch::Ptr texture_patch = TexturePatch::create(0, subgraph, texcoords, image);
            std::size_t texture_patch_id;
            #pragma omp critical
            {
                texture_patches->push_back(texture_patch);
                texture_patch_id = num_patches++;

                num_hole_faces += subgraph.size();
                ++num_holes;
            }

            for (std::size_t j = 0; j < num_vertices; ++j) {
                std::size_t const vertex_id = l2g[j];
                std::vector<std::size_t> const & adj_faces = vertex_infos->at(vertex_id).faces;
                std::vector<std::size_t> faces;
                faces.reserve(adj_faces.size());
                for (std::size_t adj_face : adj_faces) {
                    if (graph.get_label(adj_face) == 0) {
                        faces.push_back(adj_face);
                    }
                }
                VertexProjectionInfo info = {texture_patch_id, projections[j], faces};
                #pragma omp critical
                vertex_projection_infos->at(vertex_id).push_back(info);
            }
        }
    }

    merge_vertex_projection_infos(vertex_projection_infos);

    std::cout << "done. (Took " << timer.get_elapsed_sec() << "s)" << std::endl;
    std::cout << "\t" << num_patches << " texture patches." << std::endl;
    std::cout << "\t" << num_holes << " holes (" << num_hole_faces << " faces)." << std::endl;
}

TEX_NAMESPACE_END
