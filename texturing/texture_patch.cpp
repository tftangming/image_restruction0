/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <set>

#include <math/functions.h>
#include <core/image_color.h>
#include <core/image_tools.h>
#include <core/mesh_io_ply.h>

#include "texture_patch.h"

TexturePatch::TexturePatch(int _label, std::vector<std::size_t> const & _faces,
    std::vector<math::Vec2f>  const & _texcoords, core::ByteImage::Ptr _image)
    : label(_label), faces(_faces.begin(), _faces.end()),
    texcoords(_texcoords.begin(), _texcoords.end()) {

    image = core::image::byte_to_float_image(_image);

    validity_mask = core::ByteImage::create(get_width(), get_height(), 1);
    validity_mask->fill(255);
    blending_mask = core::ByteImage::create(get_width(), get_height(), 1);
}

TexturePatch::TexturePatch(TexturePatch const & texture_patch) {
    label = texture_patch.label;
    faces = std::vector<std::size_t>(texture_patch.faces);
    texcoords = std::vector<math::Vec2f>(texture_patch.texcoords);
    image = texture_patch.image->duplicate();
    validity_mask = texture_patch.validity_mask->duplicate();
    if (texture_patch.blending_mask != NULL) {
        blending_mask = texture_patch.blending_mask->duplicate();
    }
}

const float sqrt_2 = sqrt(2);

void
TexturePatch::adjust_colors(std::vector<math::Vec3f> const & adjust_values) {
    assert(blending_mask != NULL);

    validity_mask->fill(0);

    core::FloatImage::Ptr iadjust_values = core::FloatImage::create(get_width(), get_height(), 3);
    for (std::size_t i = 0; i < texcoords.size(); i += 3) {
        math::Vec2f v1 = texcoords[i];
        math::Vec2f v2 = texcoords[i + 1];
        math::Vec2f v3 = texcoords[i + 2];

        Tri tri(v1, v2, v3);

        float area = tri.get_area();
        if (area < std::numeric_limits<float>::epsilon()) continue;

        Rect<float> aabb = tri.get_aabb();
        int const min_x = static_cast<int>(std::floor(aabb.min_x)) - texture_patch_border;
        int const min_y = static_cast<int>(std::floor(aabb.min_y)) - texture_patch_border;
        int const max_x = static_cast<int>(std::ceil(aabb.max_x)) + texture_patch_border;
        int const max_y = static_cast<int>(std::ceil(aabb.max_y)) + texture_patch_border;
        assert(0 <= min_x && max_x <= get_width());
        assert(0 <= min_y && max_y <= get_height());

        for (int y = min_y; y < max_y; ++y) {
            for (int x = min_x; x < max_x; ++x) {

                math::Vec3f bcoords = tri.get_barycentric_coords(x, y);//获取三角形区域内部任意一点的面积坐标（都大于0,如果处于外部最小值必定小于0）
                bool inside = bcoords.minimum() >= 0.0f;
                if (inside) {
                    assert(x != 0 && y != 0);
                    for (int c = 0; c < 3; ++c) {
                        iadjust_values->at(x, y, c) = math::interpolate(
                            adjust_values[i][c], adjust_values[i + 1][c], adjust_values[i + 2][c],
                            bcoords[0], bcoords[1], bcoords[2]);
                    }
                    //对那些位于三角形区域内的像素点,validity_mask和blending_mask值设为255
                    validity_mask->at(x, y, 0) = 255;
                    blending_mask->at(x, y, 0) = 255;
                }
                else {

                    if (validity_mask->at(x, y, 0) == 255)
                        continue;
                    //那些无效的地方往往是在边界区域
                    // 计算该像素点到三角形的距离是否超过一个像素
                    float ha = 2.0f * -bcoords[0] * area / (v2 - v3).norm();
                    float hb = 2.0f * -bcoords[1] * area / (v1 - v3).norm();
                    float hc = 2.0f * -bcoords[2] * area / (v1 - v2).norm();

                    if (ha > sqrt_2 || hb > sqrt_2 || hc > sqrt_2)
                        continue;

                    for (int c = 0; c < 3; ++c) {
                        iadjust_values->at(x, y, c) = math::interpolate(
                            adjust_values[i][c], adjust_values[i + 1][c], adjust_values[i + 2][c],
                            bcoords[0], bcoords[1], bcoords[2]);
                    }
                    validity_mask->at(x, y, 0) = 255;
                    blending_mask->at(x, y, 0) = 128;
                }
            }
        }
    }

    for (int i = 0; i < image->get_pixel_amount(); ++i) {
        if (validity_mask->at(i, 0) != 0){
            for (int c = 0; c < 3; ++c) {
         //       if(i<20) std::cout << image->at(i, c) << " ";
                image->at(i, c) += iadjust_values->at(i, c);
         //       if(i<20) std::cout << image->at(i, c) << " ";
            }

        } else {
            math::Vec3f color(0.0f, 0.0f, 0.0f);
            //DEBUG math::Vec3f color(1.0f, 0.0f, 1.0f);
            std::copy(color.begin(), color.end(), &image->at(i, 0));
        }
    }
}

bool TexturePatch::valid_pixel(math::Vec2f pixel) const {
    float x = pixel[0];
    float y = pixel[1];

    float const height = static_cast<float>(get_height());
    float const width = static_cast<float>(get_width());

    bool valid = (0.0f <= x && x < width && 0.0f <= y && y < height);
    if (valid && validity_mask != NULL){
        /* Only pixel which can be correctly interpolated are valid. */
        float cx = std::max(0.0f, std::min(width - 1.0f, x));
        float cy = std::max(0.0f, std::min(height - 1.0f, y));
        int const floor_x = static_cast<int>(cx);
        int const floor_y = static_cast<int>(cy);
        int const floor_xp1 = std::min(floor_x + 1, get_width() - 1);
        int const floor_yp1 = std::min(floor_y + 1, get_height() - 1);

        float const w1 = cx - static_cast<float>(floor_x);
        float const w0 = 1.0f - w1;
        float const w3 = cy - static_cast<float>(floor_y);
        float const w2 = 1.0f - w3;

        valid = (w0 * w2 == 0.0f || validity_mask->at(floor_x, floor_y, 0) == 255) &&
                (w1 * w2 == 0.0f || validity_mask->at(floor_xp1, floor_y, 0) == 255) &&
                (w0 * w3 == 0.0f || validity_mask->at(floor_x, floor_yp1, 0) == 255) &&
                (w1 * w3 == 0.0f || validity_mask->at(floor_xp1, floor_yp1, 0) == 255);
    }

    return valid;
}

bool
TexturePatch::valid_pixel(math::Vec2i pixel) const {
    int const x = pixel[0];
    int const y = pixel[1];

    bool valid = (0 <= x && x < get_width() && 0 <= y && y < get_height());
    if (valid && validity_mask != NULL) {
        valid = validity_mask->at(x, y, 0) == 255;
    }

    return valid;
}

math::Vec3f
TexturePatch::get_pixel_value(math::Vec2f pixel) const {
    assert(valid_pixel(pixel));

    math::Vec3f color;
    image->linear_at(pixel[0], pixel[1], *color);
    return color;
}

void
TexturePatch::set_pixel_value(math::Vec2i pixel, math::Vec3f color) {
    assert(blending_mask != NULL);
    assert(valid_pixel(pixel));

    std::copy(color.begin(), color.end(), &image->at(pixel[0], pixel[1], 0));
    blending_mask->at(pixel[0], pixel[1], 0) = 126;
}

void
TexturePatch::blend(core::FloatImage::ConstPtr orig) {
    poisson_blend(orig, blending_mask, image, 1.0f);

    /* Invalidate all pixels outside of the boundary. */
    for (int y = 0; y < blending_mask->height(); ++y) {
        for (int x = 0; x < blending_mask->width(); ++x) {
            if (blending_mask->at(x, y, 0) == 128) {
                validity_mask->at(x, y, 0) = 0;
            }
        }
    }
}

typedef std::vector<std::pair<int, int> > PixelVector;
typedef std::set<std::pair<int, int> > PixelSet;

void
TexturePatch::prepare_blending_mask(std::size_t strip_width){
    int const width = blending_mask->width();
    int const height = blending_mask->height();

    //记录纹理视图上边界位置的所有有效像素的集合
    PixelSet valid_border_pixels;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (validity_mask->at(x, y, 0) == 255) {
                /* Valid border pixels need no invalid neighbours. */
                if (x == 0 || x == width - 1 || y == 0 || y == height - 1) {
                    valid_border_pixels.insert(std::pair<int, int>(x, y));
                    continue;
                }

                bool at_border = false;
                /* Check the direct neighbourhood of all invalid pixels. */
                for (int j = -1; j <= 1 && !at_border; ++j) {
                    for (int i = -1; i <= 1 && !at_border; ++i) {
                        int nx = x + i;
                        int ny = y + j;
                        /* If the valid pixel has a invalid neighbour: */
                        if (0 <= nx && nx < width &&
                            0 <= ny && ny < height &&
                            validity_mask->at(nx, ny, 0) == 0) {

                            /* Add the pixel to the set of valid border pixels. */
                            valid_border_pixels.insert(std::pair<int, int>(x, y));
                            at_border = true;
                        }
                    }
                }
            }
        }
    }

    core::ByteImage::Ptr inner_pixel = validity_mask->duplicate();

    //腐蚀所有边界像素,且重复运行多次* Iteratively erode all border pixels. */
    for (std::size_t i = 0; i < strip_width; ++i){
        PixelVector new_invalid_pixels(valid_border_pixels.begin(), valid_border_pixels.end());
        PixelVector::iterator it;
        valid_border_pixels.clear();

        /* Mark the new invalid pixels invalid in the validity mask. */
        for (it = new_invalid_pixels.begin(); it != new_invalid_pixels.end(); ++it) {
             int x = it->first;
             int y = it->second;

             inner_pixel->at(x, y, 0) = 0;
        }

        /* Calculate the set of valid pixels at the border of the valid area. */
        for (it = new_invalid_pixels.begin(); it != new_invalid_pixels.end(); ++it) {
             int x = it->first;
             int y = it->second;

             for (int j = -1; j <= 1; j++){
                 for (int i = -1; i <= 1; i++){
                     int nx = x + i;
                     int ny = y + j;
                     if (0 <= nx && nx < width &&
                         0 <= ny && ny < height &&
                         inner_pixel->at(nx, ny, 0) == 255){

                         valid_border_pixels.insert(std::pair<int, int>(nx, ny));
                     }
                 }
             }
        }
    }

    /* Mark all remaining pixels invalid in the blending_mask. */
    for (int i = 0; i < inner_pixel->get_pixel_amount(); ++i) {
        if (inner_pixel->at(i) == 255) blending_mask->at(i) = 0;//将blending_mask上没有参与边界腐蚀的区域置为0
    }

    /* Mark all border pixels. */
    PixelSet::iterator it;
    for (it = valid_border_pixels.begin(); it != valid_border_pixels.end(); ++it) {
         int x = it->first;
         int y = it->second;

         blending_mask->at(x, y, 0) = 126;//blending_mask上值为126的像素点说明在边界
    }
}
