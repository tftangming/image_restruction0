/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "defines.h"
#include "texture_atlas.h"
#include "obj_model.h"

TEX_NAMESPACE_BEGIN

void
build_model(core::TriangleMesh::ConstPtr mesh,
    std::vector<TextureAtlas::Ptr> const & texture_atlases, ObjModel * obj_model)  {
    //提取网格顶点列表,网格顶点法线列表,网格面片列表
    core::TriangleMesh::VertexList const & mesh_vertices = mesh->get_vertices();
    core::TriangleMesh::NormalList const & mesh_normals = mesh->get_vertex_normals();
    core::TriangleMesh::FaceList const & mesh_faces = mesh->get_faces();
    //将网格顶点依次放入到obj_model.vertices
    ObjModel::Vertices & vertices = obj_model->get_vertices();
    vertices.insert(vertices.begin(), mesh_vertices.begin(), mesh_vertices.end());
    //将网格顶点法线依次放入到obj_model.normals
    ObjModel::Normals & normals = obj_model->get_normals();
    normals.insert(normals.begin(), mesh_normals.begin(), mesh_normals.end());

    ObjModel::TexCoords & texcoords = obj_model->get_texcoords();

    ObjModel::Groups & groups = obj_model->get_groups();//一个group只包含一个纹理贴图的信息
    MaterialLib & material_lib = obj_model->get_material_lib();

    for (TextureAtlas::Ptr texture_atlas : texture_atlases) {
        groups.push_back(ObjModel::Group());
        ObjModel::Group & group = groups.back();

        const std::size_t n = material_lib.size();
        group.material_name = std::string("material") + util::string::get_filled(n, 4);//material0000,material0001,material0002,material0003

        Material material;
        material.diffuse_map = texture_atlas->get_filename();//随机生成的那个filename：tmp/~
        material_lib.add_material(group.material_name, material);// material_lib包含了多组texture_atlas的group.material_name和Material
        //提取纹理贴图上的面片信息,纹理坐标以及纹理坐标id
        TextureAtlas::Faces const & atlas_faces = texture_atlas->get_faces();
        TextureAtlas::Texcoords const & atlas_texcoords = texture_atlas->get_texcoords();
        TextureAtlas::TexcoordIds const & atlas_texcoord_ids = texture_atlas->get_texcoord_ids();

        std::size_t texcoord_id_offset = texcoords.size();
        //把所有纹理贴图的纹理坐标依次放入obj_model.texcoords
        texcoords.insert(texcoords.end(), atlas_texcoords.begin(),
            atlas_texcoords.end());

        for (std::size_t i = 0; i < atlas_faces.size(); ++i) {
            std::size_t mesh_face_pos = atlas_faces[i] * 3;

            std::size_t vertex_ids[] = {
                mesh_faces[mesh_face_pos],
                mesh_faces[mesh_face_pos + 1],
                mesh_faces[mesh_face_pos + 2]
            };
            std::size_t * normal_ids = vertex_ids;

            std::size_t texcoord_ids[] = {
                texcoord_id_offset + atlas_texcoord_ids[i * 3],
                texcoord_id_offset + atlas_texcoord_ids[i * 3 + 1],
                texcoord_id_offset + atlas_texcoord_ids[i * 3 + 2]
            };
            //完善group中的信息
            group.faces.push_back(ObjModel::Face());
            ObjModel::Face & face = group.faces.back();
            std::copy(vertex_ids, vertex_ids + 3, face.vertex_ids);
            std::copy(texcoord_ids, texcoord_ids + 3, face.texcoord_ids);
            std::copy(normal_ids, normal_ids + 3, face.normal_ids);
        }
    }
    //TODO remove unreferenced vertices/normals.
}

TEX_NAMESPACE_END
