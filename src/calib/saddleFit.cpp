#include "saddleFit.h"

static void get_image_patch_with_mask(const Mat& img, const Mat& mask, double u, double v, int r, Mat& img_sub) {
	int iu = u;
	int iv = v;
	double du = u - iu;
	double dv = v - iv;
	double a00 = 1 - du - dv + du * dv;
	double a01 = du - du * dv;
	double a10 = dv - du * dv;
	double a11 = du * dv;

	img_sub.create((2 * r + 1) * (2 * r + 1), 1, CV_64F);
	int num = 0;
	for (int j = -r; j <= r; ++j) {
		for (int i = -r; i <= r; ++i) {
			if (mask.at<double>(j + r, i + r) >= 1e-6) {
				img_sub.at<double>(num, 0) =
					a00 * img.at<double>(iv + j, iu + i) + a01 * img.at<double>(iv + j, iu + i + 1) +
					a10 * img.at<double>(iv + j + 1, iu + i) + a11 * img.at<double>(iv + j + 1, iu + i + 1);
				++num;
			}
		}
	}
	img_sub.resize(num);
}

static int create_cone_filter_kernel(Mat& kernel, int r) {
	kernel.create(2 * r + 1, 2 * r + 1, CV_64F);
	double sum = 0.0;
	int nzs = 0;
	for (int i = -r; i <= r; ++i) {
		for (int j = -r; j <= r; ++j) {
			kernel.at<double>(i + r, j + r) = std::max(0.0, r + 1 - std::sqrt(i * i + j * j));
			sum += kernel.at<double>(i + r, j + r);
			if (kernel.at<double>(i + r, j + r) < 1e-6) {
				++nzs;
			}
		}
	}
	kernel /= sum;
	return nzs;
}

vector<int> ReallinkSaddlePointFit(const Mat& gray, vector<Point2f>& corners, int r)
{
	int max_iteration = 5;
	double eps = 0.01;
	int width = gray.cols;
	int height = gray.rows;
	vector<int> choose(corners.size(), 0);
	Mat imgNorm;
	gray.convertTo(imgNorm, CV_64F, 1 / 255.0, 0);
	// cone filter
	Mat blur_kernel, blur_img, mask;
	create_cone_filter_kernel(blur_kernel, r);
	int nzs = create_cone_filter_kernel(mask, r);
	filter2D(imgNorm, blur_img, -1, blur_kernel, Point(-1, -1), 0, BORDER_REPLICATE);

	Mat A((2 * r + 1) * (2 * r + 1) - nzs, 6, CV_64F);
	int A_row = 0;
	for (int j = -r; j <= r; ++j) {
		for (int i = -r; i <= r; ++i) {
			if (mask.at<double>(j + r, i + r) >= 1e-6) {
				A.at<double>(A_row, 0) = i * i;
				A.at<double>(A_row, 1) = j * j;
				A.at<double>(A_row, 2) = i * j;
				A.at<double>(A_row, 3) = i;
				A.at<double>(A_row, 4) = j;
				A.at<double>(A_row, 5) = 1;
				++A_row;
			}
		}
	}
	Mat invAtAAt = (A.t() * A).inv(DECOMP_SVD) * A.t();

	// for all corners do
	parallel_for_(Range(0, corners.size()), [&](const Range& range) -> void {
		for (int i = range.start; i < range.end; ++i) {
			double u_init = corners[i].x;
			double v_init = corners[i].y;
			double u_cur = u_init, v_cur = v_init;
			bool is_saddle_point = true;

			// fit f(x, y) = k0 * x^2 + k1 * y^2 + k2 * x * y + k3 * x + k4 * y + k5
			// coef: [k0; k1; k2; k3; k4; k5]
			for (int num_it = 0; num_it < max_iteration; ++num_it) {
				Mat k, b;
				if (u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) {
					is_saddle_point = false;
					break;
				}
				get_image_patch_with_mask(blur_img, mask, u_cur, v_cur, r, b);
				k = invAtAAt * b;

				// check if it is still a saddle point
				double det = 4 * k.at<double>(0, 0) * k.at<double>(1, 0) - k.at<double>(2, 0) * k.at<double>(2, 0);
				if (det > 0) {
					is_saddle_point = false;
					break;
				}

				// saddle point is the corner
				double dx = (k.at<double>(2, 0) * k.at<double>(4, 0) - 2 * k.at<double>(1, 0) * k.at<double>(3, 0)) / det;
				double dy = (k.at<double>(2, 0) * k.at<double>(3, 0) - 2 * k.at<double>(0, 0) * k.at<double>(4, 0)) / det;

				u_cur += dx;
				v_cur += dy;

				double dist = std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init));
				if (dist > r) {
					is_saddle_point = false;
					break;
				}
				if (std::sqrt(dx * dx + dy * dy) <= eps) {
					break;
				}
			}

			// add to corners
			if (is_saddle_point) {
				choose[i] = 1;
				corners[i] = Point2f(u_cur, v_cur);
			}
		}
	});

	return choose;
}