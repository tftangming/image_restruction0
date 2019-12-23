/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <cstdint>
#include <iostream>

#include <math/vector.h>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <core/image_io.h>
#include <core/image_tools.h>

#include "poisson_blending.h"

typedef Eigen::SparseMatrix<float> SpMat;

math::Vec3f simple_laplacian(int i, core::FloatImage::ConstPtr img){
    const int width = img->width();
    assert(i > width + 1 && i < img->get_pixel_amount() - width -1);

    return -4.0f * math::Vec3f(&img->at(i, 0))
        + math::Vec3f(&img->at(i - width, 0))
        + math::Vec3f(&img->at(i - 1, 0))
        + math::Vec3f(&img->at(i + 1, 0))
        + math::Vec3f(&img->at(i + width, 0));
}

bool valid_mask(core::ByteImage::ConstPtr mask){
    const int width = mask->width();
    const int height = mask->height();

    for (int x = 0; x < width; ++x)
        if (mask->at(x, 0, 0) == 255 || mask->at(x, height - 1, 0) == 255)
            return false;

    for (int y = 0; y < height; ++y)
        if (mask->at(0, y, 0) == 255 || mask->at(width - 1, y, 0) == 255)
            return false;

    //TODO check for sane boundary conditions...

    return true;
}

void
poisson_blend(core::FloatImage::ConstPtr src, core::ByteImage::ConstPtr mask,
    core::FloatImage::Ptr dest, float alpha) {
    // alpha = 1,mask = blend_mask(像素值有0,126,128,255),dest = image是我们最终需要的图像内容
    //src是执行local_seam_leveling前的纹理图像,dest是执行了颜色混合后新的纹理图像
    assert(src->width() == mask->width() && mask->width() == dest->width());
    assert(src->height() == mask->height() && mask->height() == dest->height());
    assert(src->channels() == 3 && dest->channels() == 3);
    assert(mask->channels() == 1);
    assert(valid_mask(mask));

    // number of pixels
    const int n = dest->get_pixel_amount();
    // number of image width
    const int width = dest->width();
    // number of image height
    const int height = dest->height();
    // number of channels
    const int channels = dest->channels();
    //创建一副图像indices,尺寸和image一致,index表示indices上出现blend_mask有效像素值的像素序号
    core::Image<int>::Ptr indices = core::Image<int>::create(width, height, 1);
    indices->fill(-1);//indices图像上默认的像素值为-1
    int index = 0;
    for (int i = 0; i < n; ++i) {
        if (mask->at(i) != 0) {
            indices->at(i) = index;
            index++;
        }
    }
    const int nnz = index;//nzz表示blend_mask上具有有效像素值的数目,也就是边界像素总数目，包括红白蓝三区域的像素
    //构建线性系统,变量个数与（边界区域126<一个像素宽>/128<一个像素宽>及其内部255<腐蚀带宽>内）像素个数相同
    std::vector<math::Vec3f> coefficients_b;
    coefficients_b.resize(nnz);

    std::vector<Eigen::Triplet<float, int> > coefficients_A;
    coefficients_A.reserve(nnz); //TODO better estimate...

    for (int i = 0; i < n; ++i) {
        const int row = indices->at(i);
        if (mask->at(i) == 126 || mask->at(i) == 128) {
            Eigen::Triplet<float, int> t(row, row, 1.0f);
            coefficients_A.push_back(t);

            coefficients_b[row] = math::Vec3f(&dest->at(i, 0));
        }

        if (mask->at(i) == 255) {
            const int i01 = indices->at(i - width);
            const int i10 = indices->at(i - 1);
            const int i11 = indices->at(i);
            const int i12 = indices->at(i + 1);
            const int i21 = indices->at(i + width);

            /* All neighbours should be eight border conditions or part of the optimization. */
            assert(i01 != -1 && i10 != -1 && i11 != -1 && i12 != -1 && i21 != -1);

            Eigen::Triplet<float, int> t01(row, i01, 1.0f);

            Eigen::Triplet<float, int> t10(row, i10, 1.0f);
            Eigen::Triplet<float, int> t11(row, i11, -4.0f);
            Eigen::Triplet<float, int> t12(row, i12, 1.0f);

            Eigen::Triplet<float, int> t21(row, i21, 1.0f);

            Eigen::Triplet<float, int> triplets[] = {t01, t10, t11, t12, t21};

            coefficients_A.insert(coefficients_A.end(), triplets, triplets + 5);

            math::Vec3f l_d = simple_laplacian(i, dest);
            math::Vec3f l_s = simple_laplacian(i, src);

            // mixture of gradients
            coefficients_b[row] = (alpha * l_s + (1.0f - alpha) * l_d);//alpha = 1,所以这里的梯度散度为src的简化拉氏算子
        }
    }

    SpMat A(nnz, nnz);
    A.setFromTriplets(coefficients_A.begin(), coefficients_A.end());

    Eigen::SparseLU<SpMat, Eigen::COLAMDOrdering<int> > solver;
    solver.compute(A);

    for (int channel = 0; channel < channels; ++channel) {
        Eigen::VectorXf b(nnz);
        for (std::size_t i = 0; i < coefficients_b.size(); ++i)
            b[i] = coefficients_b[i][channel];

        Eigen::VectorXf x(n);
        x = solver.solve(b);//这里求解得到的x只是一个颜色通道上的解,x的向量维度为(nnz,1)

        for (int i = 0; i < n; ++i) {
            int index = indices->at(i);//indices图像上i处对应的方程解向量x中变量序号
            if (index != -1) dest->at(i, channel) = x[index];
        }
    }
}
