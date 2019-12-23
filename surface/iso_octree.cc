/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <vector>
#include <list>
#include <set>
#include <stdexcept>
#include <limits>

#include "util/timer.h"
#include "util/strings.h"
#include "surface/basis_function.h"
#include "surface/sample.h"
#include "surface/iso_octree.h"

FSSR_NAMESPACE_BEGIN

void
IsoOctree::compute_voxels (void)
{
    util::WallTimer timer;
    this->voxels.clear();
    this->compute_all_voxels();
    std::cout << "Generated " << this->voxels.size()
        << " voxels, took " << timer.get_elapsed() << "ms." << std::endl;
}

void
IsoOctree::compute_all_voxels (void)
{
    /* Locate all leafs and store voxels in a vector. */
    std::cout << "Computing sampling of the implicit function..." << std::endl;
    {
        /* Make voxels unique by storing them in a set first. */
        typedef std::set<VoxelIndex> VoxelIndexSet;
        VoxelIndexSet voxel_set;//包含多个立方体的顶点，一个立方体上有8个顶点序号

        /* Add voxels for all leaf nodes. */
        Octree::Iterator iter = this->get_iterator_for_root();
        for (iter.first_leaf();
             iter.current != nullptr; iter.next_leaf()) {

            // 8 vertices of the leaf node
            for (int i = 0; i < 8; ++i) {
                VoxelIndex index;
                index.from_path_and_corner(iter.level, iter.path, i);
                voxel_set.insert(index);
            }
        }

        /* Copy voxels over to a vector. */
        this->voxels.clear();
        this->voxels.reserve(voxel_set.size());
        for (VoxelIndexSet::const_iterator i = voxel_set.begin();
            i != voxel_set.end(); ++i)
            this->voxels.push_back(std::make_pair(*i, VoxelData()));
    }

    std::cout << "Sampling the implicit function at " << this->voxels.size()
        << " positions, fetch a beer..." << std::endl;

    /* Sample the implicit function for every voxel. */
    std::size_t num_processed = 0;
#pragma omp parallel for schedule(dynamic)
    for (std::size_t i = 0; i < voxels.size(); ++i) {

        // index of the voxel
        VoxelIndex index = this->voxels[i].first;

        // 计算体素顶点的位置（根据根节点中心坐标和宽度,体素顶点序号求得）
        math::Vec3d voxel_pos = index.compute_position(
            this->get_root_node_center(), this->get_root_node_size());
        //计算体素顶点的符号距离值、置信度、尺度、颜色分量
        // calculate the voxeldata
        this->voxels[i].second = this->sample_ifn(voxel_pos);

#pragma omp critical
        {
            num_processed += 1;
            this->print_progress(num_processed, this->voxels.size());
        }
    }

    /* Print progress one last time to get the 100% progress output. */
    this->print_progress(this->voxels.size(), this->voxels.size());
    std::cout << std::endl;
}

VoxelData
IsoOctree::sample_ifn (math::Vec3d const& voxel_pos)
{
    // Query samples that influence the voxel.
    std::vector<Sample const*> samples;
    samples.reserve(2048);

    // 体素顶点和参与计算符号距离值的采样点之间的距离不能太远，由此来筛选参与计算的部分采样点，提高运算效率（为什么通过体素顶点到采样点的距离来筛选参与计算的采样点）
    this->influence_query(voxel_pos, 3.0, &samples);

    if (samples.empty())
        return VoxelData();

    /*
     * Handling of scale: Sort the samples according to scale, high-res
     * samples first. If the confidence of the voxel is high enough, no
     * more samples are necessary.
     */
    std::size_t num_samples = samples.size() / 10;
    //按照采样点尺度从小到大进行排列
    std::nth_element(samples.begin(), samples.begin() + num_samples,
        samples.end(), sample_scale_compare);
    float const sample_max_scale = samples[num_samples]->scale * 2.0f;
    //那些尺度比较大的采样点不参与计算（为什么不计算这些尺度比较大的采样点）
#if FSSR_USE_DERIVATIVES

    /*
     *         sum_i f_i(x) w_i(x) c_i     g(x)
     * F(x) = ------------------------- = ------
     *            sum_i w_i(x) c_i         h(x)
     *
     *  d           d/dx_i g(x) * h(x) - g(x) * d/dx_i h(x)
     * ---- F(x) = -----------------------------------------
     * dx_i                          h(x)^2
     */

    double total_value = 0.0;
    double total_weight = 0.0;
    double total_scale = 0.0;
    double total_color_weight = 0.0;
    math::Vector<double, 3> total_value_deriv(0.0);
    math::Vector<double, 3> total_weight_deriv(0.0);
    math::Vector<double, 3> total_color(0.0);

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        Sample const& sample = *samples[i];
        if (sample.scale > sample_max_scale)
            continue;

        /* Evaluate basis and weight function. */
        double value, weight;
        math::Vector<double, 3> value_deriv, weight_deriv;
        evaluate(voxel_pos, sample, &value, &weight,
            &value_deriv, &weight_deriv);

        /* Incrementally update basis and weight. */
        total_value += value * weight * sample.confidence;
        total_weight += weight * sample.confidence;
        total_value_deriv += (value_deriv * weight + weight_deriv * value)
            * sample.confidence;
        total_weight_deriv += weight_deriv * sample.confidence;

        /* Incrementally update color. */
        double const color_weight = gaussian_normalized<double>
            (sample.scale / 5.0f, voxel_pos - sample.pos) * sample.confidence;
        total_scale += sample.scale * color_weight;
        total_color += sample.color * color_weight;
        total_color_weight += color_weight;
    }

    /* Compute final voxel data. */
    VoxelData voxel;
    voxel.value = total_value / total_weight;
    voxel.conf = total_weight;
    voxel.deriv = (total_value_deriv * total_weight
        - total_weight_deriv * total_value) / MATH_POW2(total_weight);
    voxel.scale = total_scale / total_color_weight;
    voxel.color = total_color / total_color_weight;

#else // FSSR_USE_DERIVATIVES

    /* Evaluate implicit function as the sum of basis functions. */
    double total_ifn = 0.0;
    double total_weight = 0.0;
    double total_scale = 0.0;
    math::Vec3d total_color(0.0);
    double total_color_weight = 0.0;

    for (std::size_t i = 0; i < samples.size(); ++i) {

        Sample const& sample = *samples[i];
        if (sample.scale > sample_max_scale)
            continue;

        /* Evaluate basis and weight function. */
        // fixme?? transformed position?
        //体素坐标在采样点的局部坐标系下的坐标表示
        math::Vec3f const tpos = transform_position(voxel_pos, sample);
        //根据采样点尺度和转换后的体素坐标计算符号距离值
        double const value = fssr_basis<double>(sample.scale, tpos);
        //根据采样点尺度和转换后的体素坐标计算权值，再结合采样点的置信度得到新的权值
        double const weight = fssr_weight<double>(sample.scale, tpos)* sample.confidence;

        /* Incrementally update. */
        total_ifn += value * weight;
        total_weight += weight;
        //根据采样点尺度和转换后的体素坐标计算颜色权值，并结合采样点的置信度得到新的颜色权值
        double const color_weight = gaussian_normalized<double>
            (sample.scale / 5.0f, tpos) * sample.confidence;
        total_scale += sample.scale * color_weight;
        total_color += sample.color * color_weight;
        total_color_weight += color_weight;
    }

    /* Compute final voxel data. */
    VoxelData voxel;
    voxel.value = total_ifn / total_weight; // sdf value
    voxel.conf = total_weight; // total weight
    voxel.scale = total_scale / total_color_weight;
    voxel.color = total_color / total_color_weight;

#endif // FSSR_USE_DERIVATIVES

    return voxel;
}

void
IsoOctree::print_progress (std::size_t voxels_done, std::size_t voxels_total)
{
    static std::size_t last_voxels_done = 0;
    static util::WallTimer timer;
    static std::size_t last_elapsed = 0;

    /* Make sure we don't call timer.get_elapsed() too often. */
    if (voxels_done != voxels_total && voxels_done - last_voxels_done < 1000)
        return;
    last_voxels_done = voxels_done;

    /* Make sure we don't print the progress too often, every 100ms. */
    std::size_t elapsed = timer.get_elapsed();
    if (voxels_done != voxels_total && elapsed - last_elapsed < 100)
        return;
    last_elapsed = elapsed;

    /* Compute percentage and nice elapsed and ETA strings. */
    std::size_t elapsed_mins = elapsed / (1000 * 60);
    std::size_t elapsed_secs = (elapsed / 1000) % 60;
    float percentage = static_cast<float>(voxels_done)
        / static_cast<float>(voxels_total) ;
    std::size_t total = static_cast<std::size_t>(elapsed / percentage);
    std::size_t remaining = total - elapsed;
    std::size_t remaining_mins = remaining / (1000 * 60);
    std::size_t remaining_secs = (remaining / 1000) % 60;

    std::cout << "\rProcessing voxel " << voxels_done
        << " (" << util::string::get_fixed(percentage * 100.0f, 2) << "%, "
        << elapsed_mins << ":"
        << util::string::get_filled(elapsed_secs, 2, '0') << ", ETA "
        << remaining_mins << ":"
        << util::string::get_filled(remaining_secs, 2, '0') << ")..."
        << std::flush;
}

FSSR_NAMESPACE_END
