#include <opencv2/core/hal/hal.hpp>
#include "imgPatch.h"

namespace ReallinkCB {

std::unordered_map<int, Mat> weight_mask(const std::vector<int>& radius) {
  std::unordered_map<int, Mat> mask;
  for(const auto& r : radius) {
    mask[r]      = Mat::zeros(r * 2 + 1, r * 2 + 1, CV_64F);
    Mat& mat = mask[r];
    for(int v = 0; v < r * 2 + 1; ++v) {
      for(int u = 0; u < r * 2 + 1; ++u) {
        double dist          = std::sqrt((u - r) * (u - r) + (v - r) * (v - r)) / r;
        dist                 = std::min(std::max(dist, 0.7), 1.3);
        mat.at<double>(v, u) = (1.3 - dist) / 0.6;
      }
    }
  }
  return mask;
};

// efficient mean-shift approximation by histogram smoothing
std::vector<std::pair<int, double>> find_modes_meanshift(const std::vector<double>& hist, double sigma) {
  std::unordered_map<int, double> hash_table;
  std::vector<std::pair<int, double>> modes;

  int r = static_cast<int>(std::round(2 * sigma));
  std::vector<double> weight(2 * r + 1, 0);
  for(int i = 0; i < 2 * r + 1; ++i) {
    weight[i] = std::exp(-0.5 * (i - r) * (i - r) / sigma / sigma) / std::sqrt(2 * M_PI) / sigma;
  }

  // compute smoothed histogram
  int n = hist.size();
  std::vector<double> hist_smoothed(n, 0);
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < 2 * r + 1; ++j) {
      hist_smoothed[(i + r) % n] += hist[(i + j) % n] * weight[j];
    }
  }

  // check if at least one entry is non-zero
  // (otherwise mode finding may run infinitly)
  auto max_hist_val = std::max_element(hist_smoothed.begin(), hist_smoothed.end());
  if(*max_hist_val < 1e-6) {
    return modes;
  }

  // mode finding
  std::vector<int> visited(n, 0);
  for(int i = 0; i < n; ++i) {
    int j = i;
    if(!visited[j]) {
      while(1) {
        visited[j] = 1;
        int j1 = (j + 1) % n, j2 = (j + n - 1) % n;
        double h0 = hist_smoothed[j];
        double h1 = hist_smoothed[j1];
        double h2 = hist_smoothed[j2];
        if(h1 >= h0 && h1 >= h2) {
          j = j1;
        } else if(h2 > h0 && h2 > h1) {
          j = j2;
        } else {
          break;
        }
      }
      hash_table[j] = hist_smoothed[j];
    }
  }

  for(const auto& i : hash_table) {
    modes.emplace_back(i);
  }
  std::sort(modes.begin(), modes.end(), [](const auto& i1, const auto& i2) -> bool {
    return i1.second > i2.second;
  });

  return modes;
};

void get_image_patch(const Mat& img, double u, double v, int r, Mat& img_sub) {
		int iu = u;
		int iv = v;
		double du = u - iu;
		double dv = v - iv;
		double a00 = 1 - du - dv + du * dv;
		double a01 = du - du * dv;
		double a10 = dv - du * dv;
		double a11 = du * dv;

		img_sub.create(2 * r + 1, 2 * r + 1, CV_64F);
		for (int j = -r; j <= r; ++j) {
			for (int i = -r; i <= r; ++i) {
				img_sub.at<double>(j + r, i + r) =
					a00 * img.at<double>(iv + j, iu + i) + a01 * img.at<double>(iv + j, iu + i + 1) +
					a10 * img.at<double>(iv + j + 1, iu + i) + a11 * img.at<double>(iv + j + 1, iu + i + 1);
			}
		}
	}

void get_image_patch_with_mask(const Mat& img, const Mat& mask, double u, double v, int r, Mat& img_sub) {
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
  int nzs    = 0;
  for(int i = -r; i <= r; ++i) {
    for(int j = -r; j <= r; ++j) {
      kernel.at<double>(i + r, j + r) = std::max(0.0, r + 1 - std::sqrt(i * i + j * j));
      sum += kernel.at<double>(i + r, j + r);
      if(kernel.at<double>(i + r, j + r) < 1e-6) {
        ++nzs;
      }
    }
  }
  kernel /= sum;
  return nzs;
}

static void polynomial_fit_saddle(const Mat& img, int r, Corner& corners) {
  // maximum iterations and precision
  int max_iteration = 5;
  double eps        = 0.01;
  int width         = img.cols;
  int height        = img.rows;

  std::vector<Point2d> corners_out_p, corners_out_v1, corners_out_v2;
  std::vector<int> corners_out_r;
  std::vector<int> choose(corners.p.size(), 0);

  // cone filter
  Mat blur_kernel, blur_img, mask;
  create_cone_filter_kernel(blur_kernel, r);
  int nzs = create_cone_filter_kernel(mask, r);
  filter2D(img, blur_img, -1, blur_kernel, Point(-1, -1), 0, BORDER_REPLICATE);

  Mat A((2 * r + 1) * (2 * r + 1) - nzs, 6, CV_64F);
  int A_row = 0;
  for(int j = -r; j <= r; ++j) {
    for(int i = -r; i <= r; ++i) {
      if(mask.at<double>(j + r, i + r) >= 1e-6) {
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
  parallel_for_(Range(0, corners.p.size()), [&](const Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      double u_init = corners.p[i].x;
      double v_init = corners.p[i].y;
      double u_cur = u_init, v_cur = v_init;
      bool is_saddle_point = true;

      // fit f(x, y) = k0 * x^2 + k1 * y^2 + k2 * x * y + k3 * x + k4 * y + k5
      // coef: [k0; k1; k2; k3; k4; k5]
      for(int num_it = 0; num_it < max_iteration; ++num_it) {
        Mat k, b;
        if(u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) {
          is_saddle_point = false;
          break;
        }
        get_image_patch_with_mask(blur_img, mask, u_cur, v_cur, r, b);
        k = invAtAAt * b;

        // check if it is still a saddle point
        double det = 4 * k.at<double>(0, 0) * k.at<double>(1, 0) - k.at<double>(2, 0) * k.at<double>(2, 0);
        if(det > 0) {
          is_saddle_point = false;
          break;
        }

        // saddle point is the corner
        double dx = (k.at<double>(2, 0) * k.at<double>(4, 0) - 2 * k.at<double>(1, 0) * k.at<double>(3, 0)) / det;
        double dy = (k.at<double>(2, 0) * k.at<double>(3, 0) - 2 * k.at<double>(0, 0) * k.at<double>(4, 0)) / det;

        u_cur += dx;
        v_cur += dy;

        double dist = std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init));
        if(dist > r) {
          is_saddle_point = false;
          break;
        }
        if(std::sqrt(dx * dx + dy * dy) <= eps) {
          break;
        }
      }

      // add to corners
      if(is_saddle_point) {
        choose[i]    = 1;
        corners.p[i] = Point2d(u_cur, v_cur);
      }
    }
  });

  for(int i = 0; i < corners.p.size(); ++i) {
    if(choose[i] == 1) {
      corners_out_p.emplace_back(corners.p[i]);
      corners_out_r.emplace_back(corners.r[i]);
      corners_out_v1.emplace_back(corners.v1[i]);
      corners_out_v2.emplace_back(corners.v2[i]);
    }
  }
  corners.p  = std::move(corners_out_p);
  corners.r  = std::move(corners_out_r);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
}

void polynomial_fit(const Mat& img, Corner& corners, const Params& params) {
    polynomial_fit_saddle(img, params.polynomial_fit_half_kernel_size, corners);
}

void hessian_response(const Mat& img_in, Mat& img_out) {
  const int rows   = img_in.rows;
  const int cols   = img_in.cols;
  const int stride = cols;
  // allocate output
  img_out = Mat::zeros(rows, cols, CV_64F);
  parallel_for_(Range(1, rows - 1), [&img_in, &img_out, &stride, &cols](const Range& range) -> void {
    // setup input and output pointer to be centered at 1,0 and 1,1 resp.
    auto* in  = img_in.ptr<double>(range.start);
    auto* out = img_out.ptr<double>(range.start) + 1;

    for(int i = range.start; i < range.end; ++i) {
      double v11, v12, v21, v22, v31, v32;
      /* fill in shift registers at the beginning of the row */
      v11 = in[-stride];
      v12 = in[1 - stride];
      v21 = in[0];
      v22 = in[1];
      v31 = in[+stride];
      v32 = in[1 + stride];
      /* move input pointer to (1,2) of the 3x3 square */
      in += 2;
      for(int c = 1; c < cols - 1; ++c) {
        /* fetch remaining values (last column) */
        const double v13 = in[-stride];
        const double v23 = *in;
        const double v33 = in[+stride];

        // compute 3x3 Hessian values from symmetric differences.
        double Lxx = (v21 - 2 * v22 + v23);
        double Lyy = (v12 - 2 * v22 + v32);
        double Lxy = (v13 - v11 + v31 - v33) / 4.;

        /* normalize and write out */
        *out = Lxx * Lyy - Lxy * Lxy;
        /* move window */
        v11 = v12;
        v12 = v13;
        v21 = v22;
        v22 = v23;
        v31 = v32;
        v32 = v33;
        /* move input/output pointers */
        in++;
        out++;
      }
      out += 2;
    }
  });
}

void create_correlation_patch(std::vector<Mat>& template_kernel, double angle_1, double angle_2, int radius) {
	// width and height
	int width = radius * 2 + 1;
	int height = radius * 2 + 1;

	// initialize template
	template_kernel[0] = Mat::zeros(height, width, CV_64F);
	template_kernel[1] = Mat::zeros(height, width, CV_64F);
	template_kernel[2] = Mat::zeros(height, width, CV_64F);
	template_kernel[3] = Mat::zeros(height, width, CV_64F);

	// midpoint
	int mu = radius + 1;
	int mv = radius + 1;

	// compute normals from angles
	double n1[2]{ -std::sin(angle_1), std::cos(angle_1) };
	double n2[2]{ -std::sin(angle_2), std::cos(angle_2) };

	// for all points in template do
	for (int u = 0; u < width; ++u) {
		for (int v = 0; v < height; ++v) {
			// vector
			int vec[2]{ u + 1 - mu, v + 1 - mv };
			double dist = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

			// check on which side of the normals we are
			double s1 = vec[0] * n1[0] + vec[1] * n1[1];
			double s2 = vec[0] * n2[0] + vec[1] * n2[1];

			if (dist <= radius) {
				if (s1 <= -0.1 && s2 <= -0.1) {
					template_kernel[0].at<double>(v, u) = 1;
				}
				else if (s1 >= 0.1 && s2 >= 0.1) {
					template_kernel[1].at<double>(v, u) = 1;
				}
				else if (s1 <= -0.1 && s2 >= 0.1) {
					template_kernel[2].at<double>(v, u) = 1;
				}
				else if (s1 >= 0.1 && s2 <= -0.1) {
					template_kernel[3].at<double>(v, u) = 1;
				}
			}
		}
	}

	// normalize
	double sum = cv::sum(template_kernel[0])[0];
	if (sum > 1e-5) {
		template_kernel[0] /= sum;
	}
	sum = cv::sum(template_kernel[1])[0];
	if (sum > 1e-5) {
		template_kernel[1] /= sum;
	}
	sum = cv::sum(template_kernel[2])[0];
	if (sum > 1e-5) {
		template_kernel[2] /= sum;
	}
	sum = cv::sum(template_kernel[3])[0];
	if (sum > 1e-5) {
		template_kernel[3] /= sum;
	}
}

static void box_filter(const Mat& img, Mat& blur_img, int kernel_size_x, int kernel_size_y) {
	if (kernel_size_y < 0) {
		kernel_size_y = kernel_size_x;
	}
	blur_img.create(img.size(), CV_64F);
	std::vector<double> buf(img.cols, 0);
	std::vector<int> count_buf(img.cols, 0);
	int count = 0;
	for (int j = 0; j < std::min(kernel_size_y, img.rows - 1); ++j) {
		for (int i = 0; i < img.cols; ++i) {
			buf[i] += img.at<double>(j, i);
			++count_buf[i];
		}
	}
	for (int j = 0; j < img.rows; ++j) {
		if (j > kernel_size_y) {
			for (int i = 0; i < img.cols; ++i) {
				buf[i] -= img.at<double>(j - kernel_size_y - 1, i);
				--count_buf[i];
			}
		}
		if (j + kernel_size_y < img.rows) {
			for (int i = 0; i < img.cols; ++i) {
				buf[i] += img.at<double>(j + kernel_size_y, i);
				++count_buf[i];
			}
		}
		blur_img.at<double>(j, 0) = 0;
		count = 0;
		for (int i = 0; i <= std::min(kernel_size_x, img.cols - 1); ++i) {
			blur_img.at<double>(j, 0) += buf[i];
			count += count_buf[i];
		}
		for (int i = 1; i < img.cols; ++i) {
			blur_img.at<double>(j, i) = blur_img.at<double>(j, i - 1);
			blur_img.at<double>(j, i - 1) /= count;
			if (i > kernel_size_x) {
				blur_img.at<double>(j, i) -= buf[i - kernel_size_x - 1];
				count -= count_buf[i - kernel_size_x - 1];
			}
			if (i + kernel_size_x < img.cols) {
				blur_img.at<double>(j, i) += buf[i + kernel_size_x];
				count += count_buf[i + kernel_size_x];
			}
		}
		blur_img.at<double>(j, img.cols - 1) /= count;
	}
}

void image_normalization_and_gradients(Mat& img, Mat& img_du, Mat& img_dv,
	Mat& img_angle, Mat& img_weight, const Params& params) {
	// normalize image
	if (params.norm) {
		Mat blur_img;
		box_filter(img, blur_img, params.norm_half_kernel_size, -1);
		img = img - blur_img;
		img = 2.5 * (max(min(img + 0.2, 0.4), 0));
	}

	// sobel masks
#if CV_VERSION_MAJOR == 4
	Mat_<double> du({ 3, 3 }, { 1, 0, -1, 2, 0, -2, 1, 0, -1 });
	Mat_<double> dv({ 3, 3 }, { 1, 2, 1, 0, 0, 0, -1, -2, -1 });
#else
	double du_array[9] = { 1, 0, -1, 2, 0, -2, 1, 0, -1 };
	double dv_array[9] = { 1, 2, 1, 0, 0, 0, -1, -2, -1 };
	Mat du(3, 3, CV_64F, du_array);
	Mat dv(3, 3, CV_64F, dv_array);
#endif

	// compute image derivatives (for principal axes estimation)
	filter2D(img, img_du, -1, du, Point(-1, -1), 0, BORDER_REFLECT);
	filter2D(img, img_dv, -1, dv, Point(-1, -1), 0, BORDER_REFLECT);
	img_angle.create(img.size(), img.type());
	img_weight.create(img.size(), img.type());
	if (!img_du.isContinuous()) {
		Mat tmp = img_du.clone();
		std::swap(tmp, img_du);
	}
	if (!img_dv.isContinuous()) {
		Mat tmp = img_dv.clone();
		std::swap(tmp, img_dv);
	}
	if (!img_angle.isContinuous()) {
		Mat tmp = img_angle.clone();
		std::swap(tmp, img_angle);
	}
	if (!img_weight.isContinuous()) {
		Mat tmp = img_weight.clone();
		std::swap(tmp, img_weight);
	}
	hal::fastAtan64f((const double*)img_dv.data, (const double*)img_du.data,
		(double*)img_angle.data, img.rows * img.cols, false);
	img_angle.forEach<double>([](double& pixel, const int* pos) -> void {
		pixel = pixel >= M_PI ? pixel - M_PI : pixel;
	});
	img_weight.forEach<double>([&img_du, &img_dv](double& pixel, const int* pos) -> void {
		int u = pos[1];
		int v = pos[0];
		pixel = std::sqrt(
			img_du.at<double>(v, u) * img_du.at<double>(v, u) + img_dv.at<double>(v, u) * img_dv.at<double>(v, u));
	});

	// scale input image
	double img_min = 0, img_max = 1;
	minMaxLoc(img, &img_min, &img_max);
	img = (img - img_min) / (img_max - img_min);
}


}//namespace

