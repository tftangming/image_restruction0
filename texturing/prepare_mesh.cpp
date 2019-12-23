/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include "texturing.h"

TEX_NAMESPACE_BEGIN
//对面片列表向量上的每一个元素（顶点序号）,遍历它所在面片face1的三个顶点的邻近面片，对于其中的每一个面片，查看是否存在有一个面片和face1上的点一模一样，如存在则记录下来
std::size_t remove_redundant_faces(core::VertexInfoList::ConstPtr vertex_infos, core::TriangleMesh::Ptr mesh) {
    core::TriangleMesh::FaceList & faces = mesh->get_faces();
    core::TriangleMesh::FaceList new_faces;
    new_faces.reserve(faces.size());

    std::size_t num_redundant = 0;
    for (std::size_t i = 0; i < faces.size(); i += 3) {
        std::size_t face_id = i / 3;
        bool redundant = false;

        for (std::size_t j = 0; !redundant && j < 3; ++j) {
            core::MeshVertexInfo::FaceRefList const & adj_faces = vertex_infos->at(faces[i + j]).faces;
            for (std::size_t k = 0; !redundant && k < adj_faces.size(); ++k) {
                std::size_t adj_face_id = adj_faces[k];

                /* Remove only the redundant face with smaller id. */
                if (face_id < adj_face_id) {
                    bool identical = true;
                    /* Faces are considered identical if they consist of the same vertices. */
                    for(std::size_t l = 0; l < 3; ++l) {
                        std::size_t vertex = faces[adj_face_id * 3 + l];
                        //如果未找到vertex
                        if (std::find(&faces[i], &faces[i + 3], vertex) == &faces[i + 3]) {
                            identical = false;
                            break;
                        }
                    }

                    redundant = identical;
                }
            }
        }

        if (redundant) {
            ++num_redundant;
        } else {
            new_faces.insert(new_faces.end(), &faces[i], &faces[i + 3]);
        }
    }

    faces.swap(new_faces);

    return num_redundant;
}

void
prepare_mesh(core::VertexInfoList::Ptr vertex_infos, core::TriangleMesh::Ptr mesh) {

    // remove redudant mesh
    std::size_t num_redundant = remove_redundant_faces(vertex_infos, mesh);
    if (num_redundant > 0) {
        std::cout << "\tRemoved " << num_redundant << " redundant faces." << std::endl;
    }

    /* Ensure face and vertex normals. */
    mesh->ensure_normals(true, true);

    /* Update vertex infos. */
    vertex_infos->calculate(mesh);
}

TEX_NAMESPACE_END
