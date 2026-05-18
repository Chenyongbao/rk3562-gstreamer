
#include "config.h"
#include "plot_corners.h"

// Optional GUI visualization: requires OpenCV highgui.
#if defined(__has_include)
#  if __has_include(<opencv2/highgui.hpp>)
#    include <opencv2/highgui.hpp>
#    define REALLINKCB_HAVE_HIGHGUI 1
#  elif __has_include(<opencv2/highgui/highgui.hpp>)
#    include <opencv2/highgui/highgui.hpp>
#    define REALLINKCB_HAVE_HIGHGUI 1
#  else
#    define REALLINKCB_HAVE_HIGHGUI 0
#  endif
#else
#  define REALLINKCB_HAVE_HIGHGUI 0
#endif

namespace ReallinkCB {

void plot_corners(const cv::Mat& img, const std::vector<cv::Point2d>& corners, const char* str) {
  cv::Mat img_show;
  if(img.channels() != 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
#else
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
#endif
  } else {
    img_show = img.clone();
  }
  for(int i = 0; i < corners.size(); ++i) {
    cv::circle(img_show, corners[i], 2, cv::Scalar(0, 0, 255), -1);
  }
#if REALLINKCB_HAVE_HIGHGUI
  cv::namedWindow(str, cv::WINDOW_NORMAL);
  cv::imshow(str, img_show);
  cv::waitKey();
#else
  (void)str;
#endif
}

void plot_corners(const cv::Mat& img, const Corner& corners) {
  cv::Mat img_show;
  if(img.channels() != 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
#else
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
#endif
  } else {
    img_show = img.clone();
  }
  for(int i = 0; i < corners.p.size(); ++i) {
    cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v1[i], cv::Scalar(255, 0, 0), 2);
    cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v2[i], cv::Scalar(0, 255, 0), 2);
    if(!corners.v3.empty()) {
      cv::line(img_show, corners.p[i], corners.p[i] + 20 * corners.v3[i], cv::Scalar(0, 0, 255), 2);
    }
    cv::circle(img_show, corners.p[i], 3, cv::Scalar(0, 0, 255), -1);
    cv::putText(img_show, std::to_string(i), cv::Point2i(corners.p[i].x - 12, corners.p[i].y - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  }
#if REALLINKCB_HAVE_HIGHGUI
  cv::namedWindow("corners_img", cv::WINDOW_NORMAL);
  cv::imshow("corners_img", img_show);
#endif
  // cv::imwrite("corners_img.png", img_show);
}

} // namespace cbdetect
