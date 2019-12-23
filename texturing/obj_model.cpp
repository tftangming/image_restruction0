/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iomanip>
#include <fstream>
#include <cstring>
#include <cerrno>

#include <core/mesh.h>
#include <util/exception.h>
#include <util/file_system.h>

#include "obj_model.h"

#define OBJ_INDEX_OFFSET 1

ObjModel::ObjModel() {}

void
ObjModel::save(ObjModel const & model, std::string const & prefix) {
    model.save_to_files(prefix);
}

void
ObjModel::save_to_files(std::string const & prefix) const {
    //保存材料库,生成.mtl文件
    material_lib.save_to_files(prefix);

    std::string name = util::fs::basename(prefix);
    std::ofstream out((prefix + ".obj").c_str());
    if (!out.good())
        throw util::FileException(prefix + ".obj", std::strerror(errno));

    out << "mtllib " << name << ".mtl" << std::endl;
    //首先按行写入每一个顶点的三维坐标
    out << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        out << "v " << vertices[i][0] << " "
            << vertices[i][1] << " "
            << vertices[i][2] << std::endl;
    }
    //然后按行写入每一个纹理坐标的横纵坐标（为何要对对纵坐标取反加一）
    for (std::size_t i = 0; i < texcoords.size(); ++i) {
        out << "vt " << texcoords[i][0] << " "
            << 1.0f - texcoords[i][1] << std::endl;
    }
    //然后按行写入每一个顶点的法向量信息
    for (std::size_t i = 0; i < normals.size(); ++i) {
        out << "vn " << normals[i][0] << " "
            << normals[i][1] << " "
            << normals[i][2] << std::endl;
    }
    // usemtl material0000
    // f 0/1/2 2/3/4 80/20/40 这一行表示group上一个面片上三个顶点的信息,,所以有许多行
    for (std::size_t i = 0; i < groups.size(); ++i) {
        out << "usemtl " << groups[i].material_name << std::endl;
        for (std::size_t j = 0; j < groups[i].faces.size(); ++j) {
            Face const & face =  groups[i].faces[j];
            out << "f";
            for (std::size_t k = 0; k < 3; ++k) {
                out << " " << face.vertex_ids[k]  + OBJ_INDEX_OFFSET
                    << "/" << face.texcoord_ids[k]  + OBJ_INDEX_OFFSET
                    << "/" << face.normal_ids[k]  + OBJ_INDEX_OFFSET;
            }
            out << std::endl;
        }
    }
    out.close();
}
