#include <math.h>
#include <stdio.h>
#include "config.h"
#include "imgPatch.h"
#include "plot_corners.h"
namespace ReallinkCB {
static void non_maximum_suppression(const Mat& img, int n, double tau, int margin, Corner& corners) {
		Mat choose_img = Mat::zeros(img.size(), CV_8U);
		parallel_for_(Range(1, std::floor((img.rows - 2 * margin) / (n + 1)) + 1), [&](const Range& range) -> void {
			for (int j = range.start * (n + 1) + margin - 1; j < range.end * (n + 1) + margin - 1; j += n + 1) {
				for (int i = n + margin; i < img.cols - n - margin; i += n + 1) {
					int maxi = i, maxj = j;
					double maxval = img.at<double>(j, i);

					for (int j2 = j; j2 <= j + n; ++j2) {
						for (int i2 = i; i2 <= i + n; ++i2) {
							if (img.at<double>(j2, i2) > maxval) {
								maxi = i2;
								maxj = j2;
								maxval = img.at<double>(j2, i2);
							}
						}
					}

					// maximum
					for (int j2 = maxj - n; j2 <= std::min(maxj + n, img.rows - 1 - margin); ++j2) {
						for (int i2 = maxi - n; i2 <= std::min(maxi + n, img.cols - 1 - margin); ++i2) {
							if (img.at<double>(j2, i2) > maxval) {
								goto GOTO_FAILED;
							}
						}
					}

					if (maxval > tau) {
						choose_img.at<uint8_t>(maxj, maxi) = 1;
					}
				GOTO_FAILED:;
				}
			}
		});

		for (int j = margin; j < img.rows - margin; ++j) {
			for (int i = margin; i < img.cols - margin; ++i) {
				if (choose_img.at<uint8_t>(j, i) == 1) {
					corners.p.emplace_back(Point2d(i, j));
					corners.r.emplace_back(margin);
				}
			}
		}
	}

static void non_maximum_suppression_sparse(Corner& corners, int n, Size img_size, const Params& params) {
		Mat img_score = Mat::zeros(img_size, CV_64F);
		Mat used = Mat::ones(img_size, CV_32S) * -1;
		for (int i = 0; i < corners.p.size(); ++i) {
			int u = std::round(corners.p[i].x);
			int v = std::round(corners.p[i].y);
			if (img_score.at<double>(v, u) < corners.score[i]) {
				img_score.at<double>(v, u) = corners.score[i];
				used.at<int>(v, u) = i;
			}
		}
		std::vector<Point2d> corners_out_p, corners_out_v1, corners_out_v2, corners_out_v3;
		std::vector<double> corners_out_score;
		std::vector<int> corners_out_r;
		for (int i = 0; i < corners.p.size(); ++i) {
			int u = std::round(corners.p[i].x);
			int v = std::round(corners.p[i].y);
			double score = corners.score[i];
			if (used.at<int>(v, u) != i) {
				continue;
			}
			for (int j2 = v - n; j2 <= v + n; ++j2) {
				for (int i2 = u - n; i2 <= u + n; ++i2) {
					if (j2 < 0 || j2 >= img_size.height || i2 < 0 || i2 >= img_size.height) {
						continue;
					}
					if (img_score.at<double>(j2, i2) > score && (i2 != u || j2 != v)) {
						goto GOTO_FAILED;
					}
				}
			}
			corners_out_p.emplace_back(corners.p[i]);
			corners_out_r.emplace_back(corners.r[i]);
			corners_out_v1.emplace_back(corners.v1[i]);
			corners_out_v2.emplace_back(corners.v2[i]);
			corners_out_score.emplace_back(corners.score[i]);
		GOTO_FAILED:;
		}
		corners.p = std::move(corners_out_p);
		corners.r = std::move(corners_out_r);
		corners.v1 = std::move(corners_out_v1);
		corners.v2 = std::move(corners_out_v2);
		corners.score = std::move(corners_out_score);
	}

static void get_init_location(const Mat& img, const Mat& img_du, const Mat& img_dv,
                       Corner& corners, const Params& params) {
    Mat gauss_img;
    GaussianBlur(img, gauss_img, Size(7, 7), 1.5, 1.5);
    Mat hessian_img;
    hessian_response(gauss_img, hessian_img);
    double mn = 0, mx = 0;
    minMaxIdx(hessian_img, &mn, &mx, NULL, NULL);
    hessian_img = abs(hessian_img);
    double thr  = std::abs(mn * params.init_loc_thr);
    for(const auto& r : params.radius) {
      non_maximum_suppression(hessian_img, r, thr, r, corners);
    }
  // location refinement
  int width = img.cols, height = img.rows;
  parallel_for_(Range(0, corners.p.size()), [&](const Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      double u = corners.p[i].x;
      double v = corners.p[i].y;
      int r    = corners.r[i];

      Mat G = Mat::zeros(2, 2, CV_64F);
      Mat b = Mat::zeros(2, 1, CV_64F);

      // get subpixel gradiant
      Mat img_du_sub, img_dv_sub;
      if(u - r < 0 || u + r >= width - 1 || v - r < 0 || v + r >= height - 1) {
        break;
      }
      get_image_patch(img_du, u, v, r, img_du_sub);
      get_image_patch(img_dv, u, v, r, img_dv_sub);

      for(int j2 = 0; j2 < 2 * r + 1; ++j2) {
        for(int i2 = 0; i2 < 2 * r + 1; ++i2) {
          // pixel orientation vector
          double o_du   = img_du_sub.at<double>(j2, i2);
          double o_dv   = img_dv_sub.at<double>(j2, i2);
          double o_norm = std::sqrt(o_du * o_du + o_dv * o_dv);
          if(o_norm < 0.1) {
            continue;
          }

          // do not consider center pixel
          if(i2 == r && j2 == r) {
            continue;
          }
          G.at<double>(0, 0) += o_du * o_du;
          G.at<double>(0, 1) += o_du * o_dv;
          G.at<double>(1, 0) += o_du * o_dv;
          G.at<double>(1, 1) += o_dv * o_dv;
          b.at<double>(0, 0) += o_du * o_du * (i2 - r + u) + o_du * o_dv * (j2 - r + v);
          b.at<double>(1, 0) += o_du * o_dv * (i2 - r + u) + o_dv * o_dv * (j2 - r + v);
        }
      }

      Mat new_pos = G.inv() * b;
      if(std::abs(new_pos.at<double>(0, 0) - corners.p[i].x) +
             std::abs(new_pos.at<double>(1, 0) - corners.p[i].y) <
         corners.r[i] * 2) {
        corners.p[i].x = new_pos.at<double>(0, 0);
        corners.p[i].y = new_pos.at<double>(1, 0);
      }
    }
  });
}

static double corner_correlation_score(const Mat& img, const Mat& img_weight,
	const Point2d& v1, const Point2d& v2) {
	//compute gradient filter kernel (bandwith = 3 px)
	double center = (img.cols - 1) / 2;
	Mat img_filter = Mat::ones(img.size(), CV_64F) * -1;
	for (int u = 0; u < img.cols; ++u) {
		for (int v = 0; v < img.rows; ++v) {
			Point2d p1{ u - center, v - center };
			Point2d p2{ (p1.x * v1.x + p1.y * v1.y) * v1.x, (p1.x * v1.x + p1.y * v1.y) * v1.y };
			Point2d p3{ (p1.x * v2.x + p1.y * v2.y) * v2.x, (p1.x * v2.x + p1.y * v2.y) * v2.y };
			if (norm(p1 - p2) <= 1.5 || norm(p1 - p3) <= 1.5) {
				img_filter.at<double>(v, u) = 1;
			}
		}
	}

	// normalize
	Scalar mean, std;
	meanStdDev(img_filter, mean, std);
	img_filter = (img_filter - mean[0]) / std[0];
	meanStdDev(img_weight, mean, std);
	Mat img_weight_norm = (img_weight - mean[0]) / std[0];

	// compute gradient score
	double score_gradient = sum(img_weight_norm.mul(img_filter))[0];
	score_gradient = std::max(score_gradient / (img.cols * img.rows - 1), 0.);

	// create intensity filter kernel
	std::vector<Mat> template_kernel(4); // a1, a2, b1, b2
	create_correlation_patch(template_kernel, std::atan2(v1.y, v1.x), std::atan2(v2.y, v2.x), (img.cols - 1) / 2);

	// checkerboard responses
	double a1 = sum(img.mul(template_kernel[0]))[0];
	double a2 = sum(img.mul(template_kernel[1]))[0];
	double b1 = sum(img.mul(template_kernel[2]))[0];
	double b2 = sum(img.mul(template_kernel[3]))[0];

	// mean
	double mu = (a1 + a2 + b1 + b2) / 4;
	// case 1: a=white, b=black
	double s1 = std::min(std::min(a1, a2) - mu, mu - std::min(b1, b2));
	// case 2: b=white, a=black
	double s2 = std::min(mu - std::min(a1, a2), std::min(b1, b2) - mu);
	double score_intensity = std::max(std::max(s1, s2), 0.);
	// final score: product of gradient and intensity score
	return score_gradient * score_intensity;
}

static void sorce_corners(const Mat& img, const Mat& img_weight, Corner& corners, const Params& params) {
	corners.score.resize(corners.p.size());
	int width = img.cols, height = img.rows;
	auto mask = weight_mask(params.radius);

	// for all corners do
	parallel_for_(Range(0, corners.p.size()), [&](const Range& range) -> void {
		for (int i = range.start; i < range.end; ++i) {
			// corner location
			double u = corners.p[i].x;
			double v = corners.p[i].y;
			int r = corners.r[i];

			if (u - r < 0 || u + r >= width - 1 || v - r < 0 || v + r >= height - 1) {
				corners.score[i] = 0.;
				continue;
			}
			Mat img_sub, img_weight_sub;
			get_image_patch(img, u, v, r, img_sub);
			get_image_patch(img_weight, u, v, r, img_weight_sub);
			img_weight_sub = img_weight_sub.mul(mask[r]);
			corners.score[i] = corner_correlation_score(img_sub, img_weight_sub, corners.v1[i], corners.v2[i]);
		}
	});
}

static void remove_low_scoring_corners(double tau, Corner& corners, const Params& params) {
	std::vector<Point2d> corners_out_p, corners_out_v1, corners_out_v2, corners_out_v3;
	std::vector<double> corners_out_score;
	std::vector<int> corners_out_r;
	for (int i = 0; i < corners.p.size(); ++i) {
		if (corners.score[i] > tau) {
			corners_out_p.emplace_back(corners.p[i]);
			corners_out_r.emplace_back(corners.r[i]);
			corners_out_v1.emplace_back(corners.v1[i]);
			corners_out_v2.emplace_back(corners.v2[i]);
			corners_out_score.emplace_back(corners.score[i]);
		}
	}
	corners.p = std::move(corners_out_p);
	corners.r = std::move(corners_out_r);
	corners.v1 = std::move(corners_out_v1);
	corners.v2 = std::move(corners_out_v2);
	corners.score = std::move(corners_out_score);
}

static void filter_corners(const Mat& img, const Mat& img_angle, const Mat& img_weight,
                    Corner& corners, const Params& params) {
  int width = img.cols, height = img.rows;
  std::vector<Point2d> corners_out_p;
  std::vector<int> corners_out_r;
  std::vector<int> choose(corners.p.size(), 0);
  std::vector<double> cos_v(32), sin_v(32);
  for(int i = 0; i < 32; ++i) {
    cos_v[i] = std::cos(i * 2.0 * M_PI / 31.0);
    sin_v[i] = std::sin(i * 2.0 * M_PI / 31.0);
  }
  auto mask = weight_mask(params.radius);

  parallel_for_(Range(0, corners.p.size()), [&](const Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      int num_crossings = 0, num_modes = 0;
      int center_u = std::round(corners.p[i].x);
      int center_v = std::round(corners.p[i].y);
      int r        = corners.r[i];
      if(center_u - r < 0 || center_u + r >= width - 1 || center_v - r < 0 || center_v + r >= height - 1) {
        continue;
      }

      // extract circle locations and its value
      std::vector<double> c(32);
      for(int j = 0; j < 32; ++j) {
        int circle_u = static_cast<int>(std::round(center_u + 0.75 * r * cos_v[j]));
        int circle_v = static_cast<int>(std::round(center_v + 0.75 * r * sin_v[j]));
        circle_u     = std::min(std::max(circle_u, 0), width - 1);
        circle_v     = std::min(std::max(circle_v, 0), height - 1);
        c[j]         = img.at<double>(circle_v, circle_u);
      }
      auto minmax  = std::minmax_element(c.begin(), c.end());
      double min_c = *minmax.first, max_c = *minmax.second;
      for(int j = 0; j < 32; ++j) {
        c[j] = c[j] - min_c - (max_c - min_c) / 2;
      }

      // count number of zero-crossings
      int fisrt_cross_index = 0;
      for(int j = 0; j < 32; ++j) {
        if((c[j] > 0) ^ (c[(j + 1) % 32] > 0)) {
          fisrt_cross_index = (j + 1) % 32;
          break;
        }
      }
      for(int j = fisrt_cross_index, count = 1; j < 32 + fisrt_cross_index; ++j, ++count) {
        if((c[j % 32] > 0) ^ (c[(j + 1) % 32] > 0)) {
          if(count >= 3) {
            ++num_crossings;
          }
          count = 1;
        }
      }

      int top_left_u         = std::max(center_u - r, 0);
      int top_left_v         = std::max(center_v - r, 0);
      int bottom_right_u     = std::min(center_u + r, width - 1);
      int bottom_right_v     = std::min(center_v + r, height - 1);
      Mat img_weight_sub = Mat::zeros(2 * r + 1, 2 * r + 1, CV_64F);
      img_weight.rowRange(top_left_v, bottom_right_v + 1).colRange(top_left_u, bottom_right_u + 1).copyTo(img_weight_sub(Range(top_left_v - center_v + r, bottom_right_v - center_v + r + 1), Range(top_left_u - center_u + r, bottom_right_u - center_u + r + 1)));
      img_weight_sub    = img_weight_sub.mul(mask[r]);
      double tmp_maxval = 0;
	  minMaxLoc(img_weight_sub, NULL, &tmp_maxval);
      img_weight_sub.forEach<double>([&tmp_maxval](double& val, const int* pos) -> void {
        val = val < 0.5 * tmp_maxval ? 0 : val;
      });

      // create histogram
      std::vector<double> angle_hist(32, 0);
      for(int j2 = top_left_v; j2 <= bottom_right_v; ++j2) {
        for(int i2 = top_left_u; i2 <= bottom_right_u; ++i2) {
          int bin = static_cast<int>(floor(img_angle.at<double>(j2, i2) / (M_PI / 32.))) % 32;
          angle_hist[bin] += img_weight_sub.at<double>(j2 - center_v + r, i2 - center_u + r);
        }
      }

      auto modes = find_modes_meanshift(angle_hist, 1.5);
      for(const auto& j : modes) {
        if(2 * j.second > modes[0].second) {
          ++num_modes;
        }
      }

      if(num_crossings >= 2 && num_modes >= 1) {
        choose[i] = 1;
      }
    }
  });

  for(int i = 0; i < corners.p.size(); ++i) {
    if(choose[i] == 1) {
      corners_out_p.emplace_back(Point2d(corners.p[i].x, corners.p[i].y));
      corners_out_r.emplace_back(corners.r[i]);
    }
  }
  corners.p = std::move(corners_out_p);
  corners.r = std::move(corners_out_r);
}

static std::vector<std::vector<double>> edge_orientations(Mat& img_angle, Mat& img_weight) {
  // number of bins (histogram parameter)
  int n = 32;

  // convert angles from normals to directions
  img_angle.forEach<double>([](double& val, const int* pos) -> void {
    val += M_PI / 2;
    val = val >= M_PI ? val - M_PI : val;
  });

  // create histogram
  std::vector<double> angle_hist(n, 0);
  for(int i = 0; i < img_angle.cols; ++i) {
    for(int j = 0; j < img_angle.rows; ++j) {
      int bin = static_cast<int>(std::floor(img_angle.at<double>(j, i) / (M_PI / n)));
      angle_hist[bin] += img_weight.at<double>(j, i);
    }
  }

  // find modes of smoothed histogram
  auto modes = find_modes_meanshift(angle_hist, 1.5);

  // if only one or no mode => return invalid corner
  if(modes.size() <= 1) {
    return std::vector<std::vector<double>>();
  }

  // compute orientation at modes
  // extract 2 strongest modes and sort by angle
  double angle_1 = modes[0].first * M_PI / n + M_PI / n / 2;
  double angle_2 = modes[1].first * M_PI / n + M_PI / n / 2;
  if(angle_1 > angle_2) {
    std::swap(angle_1, angle_2);
  }

  // compute angle between modes
  double delta_angle = std::min(angle_2 - angle_1, angle_1 + M_PI - angle_2);

  // if angle too small => return invalid corner
  if(delta_angle <= 0.3) {
    return std::vector<std::vector<double>>();
  }

  // set statistics: orientations
  std::vector<std::vector<double>> v(2, std::vector<double>(2));
  v[0][0] = std::cos(angle_1);
  v[0][1] = std::sin(angle_1);
  v[1][0] = std::cos(angle_2);
  v[1][1] = std::sin(angle_2);
  return v;
}

static void refine_corners(const Mat& img_du, const Mat& img_dv, const Mat& img_angle, const Mat& img_weight,
                    Corner& corners, const Params& params) {
  // maximum iterations and precision
  int max_iteration     = 5;
  double eps            = 0.01;
  int width = img_du.cols, height = img_du.rows;
  std::vector<Point2d> corners_out_p, corners_out_v1, corners_out_v2, corners_out_v3;
  std::vector<int> corners_out_r;
  std::vector<int> choose(corners.p.size(), 0);
  corners.v1.resize(corners.p.size());
  corners.v2.resize(corners.p.size());
  auto mask = weight_mask(params.radius);

  // for all corners do
  parallel_for_(Range(0, corners.p.size()), [&](const Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      // extract current corner location
      int ui        = std::round(corners.p[i].x);
      int vi        = std::round(corners.p[i].y);
      double u_init = corners.p[i].x;
      double v_init = corners.p[i].y;
      int r         = corners.r[i];

      // estimate edge orientations (continue, if too close to border)
      if(ui - r < 0 || ui + r >= width - 1 || vi - r < 0 || vi + r >= height - 1) {
        continue;
      }
      Mat img_angle_sub, img_weight_sub;
      get_image_patch(img_angle, ui, vi, r, img_angle_sub);
      get_image_patch(img_weight, ui, vi, r, img_weight_sub);
      img_weight_sub = img_weight_sub.mul(mask[r]);
      auto v         = edge_orientations(img_angle_sub, img_weight_sub);

      // continue, if invalid edge orientations
      if(v.empty()) {
        continue;
      }

      //corner orientation refinement
      Mat A1 = Mat::zeros(2, 2, CV_64F);
      Mat A2 = Mat::zeros(2, 2, CV_64F);
      Mat A3 = Mat::zeros(2, 2, CV_64F);
      for(int j2 = vi - r; j2 <= vi + r; ++j2) {
        for(int i2 = ui - r; i2 <= ui + r; ++i2) {
          //pixel orientation vector
          double o_du   = img_du.at<double>(j2, i2);
          double o_dv   = img_dv.at<double>(j2, i2);
          double o_norm = std::sqrt(o_du * o_du + o_dv * o_dv);
          if(o_norm < 0.1) {
            continue;
          }
          double o_du_norm = o_du / o_norm;
          double o_dv_norm = o_dv / o_norm;

          // robust refinement of orientation 1
          if(std::abs(o_du_norm * v[0][0] + o_dv_norm * v[0][1]) < 0.25) {
            A1.at<double>(0, 0) += o_du * o_du;
            A1.at<double>(0, 1) += o_du * o_dv;
            A1.at<double>(1, 0) += o_du * o_dv;
            A1.at<double>(1, 1) += o_dv * o_dv;
          }

          // robust refinement of orientation 2
          if(std::abs(o_du_norm * v[1][0] + o_dv_norm * v[1][1]) < 0.25) {
            A2.at<double>(0, 0) += o_du * o_du;
            A2.at<double>(0, 1) += o_du * o_dv;
            A2.at<double>(1, 0) += o_du * o_dv;
            A2.at<double>(1, 1) += o_dv * o_dv;
          }
        }
      }

      // set new corner orientation
      Mat eig_tmp1, eig_tmp2;
      eigen(A1, eig_tmp1, eig_tmp2);
      v[0][0] = eig_tmp2.at<double>(1, 0);
      v[0][1] = eig_tmp2.at<double>(1, 1);
      eigen(A2, eig_tmp1, eig_tmp2);
      v[1][0] = eig_tmp2.at<double>(1, 0);
      v[1][1] = eig_tmp2.at<double>(1, 1);

      std::sort(v.begin(), v.end(), [](const auto& a1, const auto& a2) {
        return a1[0] * a2[1] - a1[1] * a2[0] > 0;
      });
      v[v.size() - 1][0] = -v[v.size() - 1][0];
      v[v.size() - 1][1] = -v[v.size() - 1][1];
      std::sort(v.begin(), v.end(), [](const auto& a1, const auto& a2) {
        return a1[0] * a2[1] - a1[1] * a2[0] > 0;
      });

      if(params.polynomial_fit) {
        choose[i]     = 1;
        corners.v1[i] = Point2d(v[0][0], v[0][1]);
        corners.v2[i] = Point2d(v[1][0], v[1][1]);
        continue;
      }

      // corner location refinement
      double u_cur = u_init, v_cur = v_init, u_last = u_cur, v_last = v_cur;
      for(int num_it = 0; num_it < max_iteration; ++num_it) {
        Mat G = Mat::zeros(2, 2, CV_64F);
        Mat b = Mat::zeros(2, 1, CV_64F);

        // get subpixel gradiant
        Mat img_du_sub, img_dv_sub;
        if(u_cur - r < 0 || u_cur + r >= width || v_cur - r < 0 || v_cur + r >= height) {
          break;
        }
        get_image_patch(img_du, u_cur, v_cur, r, img_du_sub);
        get_image_patch(img_dv, u_cur, v_cur, r, img_dv_sub);

        for(int j2 = 0; j2 < 2 * r + 1; ++j2) {
          for(int i2 = 0; i2 < 2 * r + 1; ++i2) {
            // pixel orientation vector
            double o_du   = img_du_sub.at<double>(j2, i2);
            double o_dv   = img_dv_sub.at<double>(j2, i2);
            double o_norm = std::sqrt(o_du * o_du + o_dv * o_dv);
            if(o_norm < 0.1) {
              continue;
            }
            double o_du_norm = o_du / o_norm;
            double o_dv_norm = o_dv / o_norm;

            // do not consider center pixel
            if(i2 == r && j2 == r) {
              continue;
            }

            // robust subpixel corner estimation
            // compute rel. position of pixel and distance to vectors
            double w_u = i2 - r - ((i2 - r) * v[0][0] + (j2 - r) * v[0][1]) * v[0][0];
            double v_u = j2 - r - ((i2 - r) * v[0][0] + (j2 - r) * v[0][1]) * v[0][1];
            double d1  = std::sqrt(w_u * w_u + v_u * v_u);
            w_u        = i2 - r - ((i2 - r) * v[1][0] + (j2 - r) * v[1][1]) * v[1][0];
            v_u        = j2 - r - ((i2 - r) * v[1][0] + (j2 - r) * v[1][1]) * v[1][1];
            double d2  = std::sqrt(w_u * w_u + v_u * v_u);

            // if pixel corresponds with either of the vectors / directions
            if((d1 < 3 && std::abs(o_du_norm * v[0][0] + o_dv_norm * v[0][1]) < 0.25) ||
               (d2 < 3 && std::abs(o_du_norm * v[1][0] + o_dv_norm * v[1][1]) < 0.25)) {
              G.at<double>(0, 0) += o_du * o_du;
              G.at<double>(0, 1) += o_du * o_dv;
              G.at<double>(1, 0) += o_du * o_dv;
              G.at<double>(1, 1) += o_dv * o_dv;
              b.at<double>(0, 0) += o_du * o_du * (i2 - r + u_cur) + o_du * o_dv * (j2 - r + v_cur);
              b.at<double>(1, 0) += o_du * o_dv * (i2 - r + u_cur) + o_dv * o_dv * (j2 - r + v_cur);
            }
          }
        }

        // set new corner location if G has full rank
        Mat new_pos = G.inv() * b;
        u_last          = u_cur;
        v_last          = v_cur;
        u_cur           = new_pos.at<double>(0, 0);
        v_cur           = new_pos.at<double>(1, 0);
        double dist     = std::sqrt((u_cur - u_last) * (u_cur - u_last) + (v_cur - v_last) * (v_cur - v_last));
        if(dist >= 3) {
          u_cur = u_last;
          v_cur = v_last;
          break;
        }
        if(dist <= eps) {
          break;
        }
      }

      // add to corners
      if(std::sqrt((u_cur - u_init) * (u_cur - u_init) + (v_cur - v_init) * (v_cur - v_init)) < std::max(r / 2, 3)) {
        choose[i]     = 1;
        corners.p[i]  = Point2d(u_cur, v_cur);
        corners.v1[i] = Point2d(v[0][0], v[0][1]);
        corners.v2[i] = Point2d(v[1][0], v[1][1]);
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

static void find_corners_reiszed(const Mat& img, Corner& corners, const Params& params) {
  Mat img_resized, img_norm;
  Corner corners_resized;

  // resize image
  double scale = 0.3;
  /*if (img.rows < 640 || img.cols < 480) {
    scale = 2.0;
  } else if(img.rows >= 640 || img.cols >= 480) {
    scale = 0.3;
  } else {
    return;
  }*/
  resize(img, img_resized, Size(img.cols * scale, img.rows * scale), 0, 0, INTER_LINEAR);

  if(img_resized.channels() == 3) {
#if CV_VERSION_MAJOR >= 4
    cvtColor(img_resized, img_norm, COLOR_BGR2GRAY);
#else
    cvtColor(img_resized, img_norm, CV_BGR2GRAY);
#endif
    img_norm.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  } else {
    img_resized.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  }

  // normalize image and calculate gradients
  Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);

  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners_resized, params);
  if(corners_resized.p.empty()) {
    return;
  }

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners_resized, params);

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners_resized, params);

  // merge corners
  std::for_each(corners_resized.p.begin(), corners_resized.p.end(), [&scale](auto& p) { p /= scale; });
  // std::for_each(corners_resized.r.begin(), corners_resized.r.end(), [&scale](auto &r) { r = (double) r / scale; });
  double min_dist_thr = scale > 1 ? 3 : 5;
  for(int i = 0; i < corners_resized.p.size(); ++i) {
    double min_dist = DBL_MAX;
    Point2d& p2 = corners_resized.p[i];
    for(int j = 0; j < corners.p.size(); ++j) {
      Point2d& p1 = corners.p[j];
      double dist     = norm(p2 - p1);
      min_dist        = dist < min_dist ? dist : min_dist;
    }
    if(min_dist > min_dist_thr) {
      corners.p.emplace_back(corners_resized.p[i]);
      corners.r.emplace_back(corners_resized.r[i]);
      corners.v1.emplace_back(corners_resized.v1[i]);
      corners.v2.emplace_back(corners_resized.v2[i]);
    }
  }
}

void find_corners(const Mat& img, Corner& corners, const Params& params) {
  // clear old data
  corners.p.clear();
  corners.r.clear();
  corners.v1.clear();
  corners.v2.clear();
  corners.v3.clear();
  corners.score.clear();

  // convert to double grayscale image
  Mat img_norm;
  if(img.channels() == 3) {
#if CV_VERSION_MAJOR >= 4
    cvtColor(img, img_norm, COLOR_BGR2GRAY);
#else
    cvtColor(img, img_norm, CV_BGR2GRAY);
#endif
    img_norm.convertTo(img_norm, CV_64F, 1. / 255., 0);
  } else {
    img.convertTo(img_norm, CV_64F, 1. / 255., 0);
  }

  // normalize image and calculate gradients
  Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);
  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners, params);
  if(corners.p.empty()) {
    return;
  }
  //plot_corners(img, corners.p, "init location");
  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners, params);
  //plot_corners(img, corners.p, "filter corners");
  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners, params);

  // resize image to detect more corners
  //find_corners_reiszed(img, corners, params);

  // polynomial fit
  if(params.polynomial_fit) {
    polynomial_fit(img_norm, corners, params);
  }

  // score corners
  sorce_corners(img_norm, img_weight, corners, params);

  // remove low scoring corners
  remove_low_scoring_corners(params.score_thr, corners, params);

  // non maximum suppression
  non_maximum_suppression_sparse(corners, 3, img.size(), params);
}

} // namespace
