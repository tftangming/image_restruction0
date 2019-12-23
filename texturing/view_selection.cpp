/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <util/timer.h>

#include "util.h"
#include "texturing.h"

TEX_NAMESPACE_BEGIN

bool IGNORE_LUMINANCE = false;

/** Potts model */
float
potts(int, int, int l1, int l2) {
    return l1 == l2 && l1 != 0 && l2 != 0 ? 0 : 1 * MRF_MAX_ENERGYTERM;
}

struct FaceInfo {
    std::size_t component;
    std::size_t id;
};

/** Setup the neighborhood of the MRF. */
void
set_neighbors(UniGraph const & graph, std::vector<FaceInfo> const & face_infos,
    std::vector<mrf::Graph::Ptr> const & mrfs) {
    for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
        std::vector<std::size_t> adj_faces = graph.get_adj_nodes(i);
        for (std::size_t j = 0; j < adj_faces.size(); ++j) {
            //取graph上的第i个面片的第j个邻接面片
            std::size_t adj_face = adj_faces[j];
            /* The solver expects only one call of setNeighbours for two neighbours a and b. */
            if (i < adj_face) {
                assert(face_infos[i].component == face_infos[adj_face].component);
                const std::size_t component = face_infos[i].component;//获取这两个面片在第几个连通集合内
                const std::size_t cid1 = face_infos[i].id;//获取两个面片在连通集合的具体位置
                const std::size_t cid2 = face_infos[adj_face].id;
                mrfs[component]->set_neighbors(cid1, cid2);//将各个连通集合内的相邻面片设置为MRF图中的邻居顶点(在当前连通集合对应的mrf.edges中添加两条有向边并在当前连通集合对应的mrf.Vertex中的对应位置设置 输入边信息)
            }
        }
    }
}

/** Set the data costs of the MRF. */
void
set_data_costs(std::vector<FaceInfo> const & face_infos, ST const & data_costs,
    std::vector<mrf::Graph::Ptr> const & mrfs) {

    /* Set data costs for all labels except label 0 (undefined) */
    for (std::size_t i = 0; i < data_costs.rows(); i++) {
        ST::Row const & data_costs_for_label = data_costs.row(i);// != n_facets

        std::vector<std::vector<mrf::SparseDataCost> > costs(mrfs.size());
        for(std::size_t j = 0; j < data_costs_for_label.size(); j++) {
            const std::size_t id = data_costs_for_label[j].first;    // id of facet
            const float data_cost = data_costs_for_label[j].second;  // data cost of facet in the view
            const std::size_t component = face_infos[id].component;  // component id
            const std::size_t cid = face_infos[id].id;               // index in the mrf
            //TODO change index type of mrf::Graph
            costs[component].push_back({static_cast<int>(cid), data_cost});//在每一纹理视角下,对每个连通集合设置代价值向量,每一个元素表示每个面片在连通集合中的序号和对应的代价值
        }

        int label = i + 1;

        for (std::size_t j = 0; j < mrfs.size(); ++j) {
            mrfs[j]->set_data_costs(label, costs[j]);//对每个连通集合设置代价值
        }
    }
    // add the extra label
    for (std::size_t i = 0; i < mrfs.size(); ++i) {
        /* Set costs for undefined label */
        std::vector<mrf::SparseDataCost> costs(mrfs[i]->num_sites());
        for (std::size_t j = 0; j < costs.size(); j++) {
            costs[j].site = j;
            costs[j].cost = MRF_MAX_ENERGYTERM;
        }
        mrfs[i]->set_data_costs(0, costs);//在每个连通集合上将标签0也放置在对应的mrf图的顶点信息中
    }
}

/** Remove all edges of nodes which corresponding face has not been seen in any texture view. */
void
isolate_unseen_faces(UniGraph * graph, ST const & data_costs) {
    int num_unseen_faces = 0;
    for (std::uint32_t i = 0; i < data_costs.cols(); i++) {
        ST::Column const & data_costs_for_face = data_costs.col(i);

        if (data_costs_for_face.size() == 0) {
            num_unseen_faces++;

            // get all the adjacent facets of the i-the facet // each facet is corresponding to
            // the a node in the UnifindGraph
            std::vector<std::size_t> const & adj_nodes = graph->get_adj_nodes(i);
            for (std::size_t j = 0; j < adj_nodes.size(); j++)
                graph->remove_edge(i, adj_nodes[j]);
        }

    }
    std::cout << "\t" << num_unseen_faces << " faces have not been seen by a view." << std::endl;
}

void
 view_selection(ST const & data_costs, UniGraph * graph, Settings const & settings) {

    UniGraph mgraph(*graph);
    //可能会出现某些面片在所有纹理视角上都不存在代价值(可能是未通过可视性或者光度外子检测剔除了),然后需要在邻接图上将该面片表示的节点周边的边删除掉;
    isolate_unseen_faces(&mgraph, data_costs);

    unsigned int num_components = 0;
    // face_infos存储了每个面片的连通index ->（components[i][j]是面片序号作为向量下标, 下标对应的元素{i,j}是该面片序号在components中的地址
    std::vector<FaceInfo> face_infos(mgraph.num_nodes());
    std::vector<std::vector<std::size_t> > components;

    // 初始标签为0.根据标签0对所有面片做聚类(在邻接图上能用边连接起来的那些面片),得到多个连通集合
    mgraph.get_subgraphs(0, &components);
    for (std::size_t i = 0; i < components.size(); ++i) {
        if (components.size() > 1000) num_components += 1;
        for (std::size_t j = 0; j < components[i].size(); ++j) {
            face_infos[components[i][j]] = {i, j};
        }
    }
    std::cout << "total " << components.size() << " components" << std::endl;//总共由~块连通区域
    //确定MRF的求解方式为 mrf::LBP
    #ifdef REASERCH
        std::cout << "using the solver_type of MRF" << mrf::GCO<< std::endl;
        mrf::SOLVER_TYPE solver_type = mrf::GCO;
    #else
        std::cout << "using the solver_type of MRF" << mrf::LBP<< std::endl;
        mrf::SOLVER_TYPE solver_type = mrf::LBP;
    #endif

    /* Label 0 is undefined. */
    const std::size_t num_labels = data_costs.rows() + 1;
    //构建多个MRF的图,每个图用来对一块连通集合做MRF优化,创建MRF图时需要给定连通集合中的面片(图中的顶点)数目、待计算的标签数目、求解器类型,由此确定了MRF图上的顶点数目
    std::vector<mrf::Graph::Ptr> mrfs(components.size());
    for (std::size_t i = 0; i < components.size(); ++i) {
        mrfs[i] = mrf::Graph::create(components[i].size(), num_labels, solver_type);//num_sites = components[i].size(), num_labels = data_costs.rows() + 1 = 纹理视角数目+1
    }

    /* Set neighbors must be called prior to set_data_costs (LBP). */
    //创建LBP图时需要设置图上的邻居.
    set_neighbors(mgraph, face_infos, mrfs);
    //创建LBP图时当添加完图上的邻居后,设置图上顶点的信息(标签+对应的代价值等)
    set_data_costs(face_infos, data_costs, mrfs);

    bool multiple_components_simultaneously = false;
    #ifdef RESEARCH
    multiple_components_simultaneously = true;
    #endif
    #ifndef _OPENMP
    multiple_components_simultaneously = false;
    #endif

    if (multiple_components_simultaneously) {
        if (num_components > 0) {
            std::cout << "\tOptimizing " << num_components
                << " components simultaneously." << std::endl;
        }
        std::cout << "\tComp\tIter\tEnergy\t\tRuntime" << std::endl;
    }
    #ifdef RESEARCH
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (std::size_t i = 0; i < components.size(); ++i) {
        switch (settings.smoothness_term) {
            case POTTS:
                mrfs[i]->set_smooth_cost(*potts);//对mrf图设置平滑函数mrfs[i].smooth_cost_func = potts
            break;
        }

        bool verbose = mrfs[i]->num_sites() > 10000;//连通集合中的面片数目(mrf图中的顶点数)超过10000,即只输出那些顶点数目较多的mrfs图的相关信息

        util::WallTimer timer;

        mrf::ENERGY_TYPE const zero = mrf::ENERGY_TYPE(0);
        mrf::ENERGY_TYPE last_energy = zero;
        mrf::ENERGY_TYPE energy = mrfs[i]->compute_energy();//计算mrf图上各个顶点中的代价值总和以及mrf图上各条边的平滑值
        mrf::ENERGY_TYPE diff = last_energy - energy;//本次优化后的能量差值,diff>0表示优化有效果
        unsigned int iter = 0;

        std::string const comp = util::string::get_filled(i, 4);

        if (verbose && !multiple_components_simultaneously) {
            std::cout << "\tComp\tIter\tEnergy\t\tRuntime" << std::endl;
        }
        while (diff != zero) {
            #pragma omp critical
            if (verbose) {
                std::cout << "\t" << comp << "\t" << iter << "\t" << energy
                    << "\t" << timer.get_elapsed_sec() << std::endl;
                /**
                 * 	Comp	Iter	Energy		Runtime
	                0000	0	1.9333e+12	0.004
	                0000	1	1.33735e+12	0.503
	                0000	2	1.1113e+12	0.996
                 */
            }
            last_energy = energy;
            ++iter;
            energy = mrfs[i]->optimize(1);//对mrf图作一次优化,具体的优化思路是怎样子的还需要继续研究！！！！！没有看懂
            diff = last_energy - energy;
            if (diff <= zero) break;
        }

        #pragma omp critical
        if (verbose) {
            std::cout << "\t" << comp << "\t" << iter << "\t" << energy << std::endl;
            if (diff == zero) {
                std::cout << "\t" << comp << "\t" << "Converged" << std::endl;
            }
            if (diff < zero) {
                std::cout << "\t" << comp << "\t"
                    << "Increase of energy - stopping optimization" << std::endl;
            }
        }

        /* Extract resulting labeling from MRF. */
        for (std::size_t j = 0; j < components[i].size(); ++j) {
            int label = mrfs[i]->what_label(static_cast<int>(j));//获取mrf图中第j个顶点的标签类型
            assert(0 <= label && static_cast<std::size_t>(label) < num_labels);
            graph->set_label(components[i][j], static_cast<std::size_t>(label));//设置graph.labels中对应的面片类型
        }
    }
}

TEX_NAMESPACE_END
