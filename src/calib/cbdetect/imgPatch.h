#ifndef _CBDETECT_IMG_PATCH_H_
#define _CBDETECT_IMG_PATCH_H_

#include <unordered_map>
#include <vector>
#include "config.h"

namespace ReallinkCB {
	std::unordered_map<int, Mat> weight_mask(const std::vector<int>& radius);
	std::vector<std::pair<int, double>> find_modes_meanshift(const std::vector<double>& hist, double sigma);
	void get_image_patch(const Mat& img, double u, double v, int r, Mat& img_sub);
	void get_image_patch_with_mask(const Mat& img, const Mat& mask, double u, double v, int r, Mat& img_sub);
	void polynomial_fit(const Mat& img, Corner& corners, const Params& params);
	void hessian_response(const Mat& img_in, Mat& img_out);
	void create_correlation_patch(std::vector<Mat>& template_kernel, double angle_1, double angle_2, int radius);
	void image_normalization_and_gradients(Mat& img, Mat& img_du, Mat& img_dv,
		Mat& img_angle, Mat& img_weight, const Params& params);

}

#endif //LIBCBDETECT_GROW_BOARD_H
