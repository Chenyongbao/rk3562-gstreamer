
#include "config.h"
#include "plot_boards.h"

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

void plot_boards(const cv::Mat& img, const Corner& corners,
                 const std::vector<Board>& boards, const Params& params) {
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

  for(int n = 0; n < boards.size(); ++n) {
    const auto& board = boards[n];

    for(int i = 1; i < board.idx.size() - 1; ++i) {
      for(int j = 1; j < board.idx[i].size() - 1; ++j) {
        if(board.idx[i][j] < 0) {
          continue;
        }
        // plot lines in color
        if(board.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i][j + 1]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        
        if(board.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j]],
                   cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        // plot lines in white
        if(board.idx[i][j + 1] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i][j + 1]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        
        if(board.idx[i + 1][j] >= 0) {
          cv::line(img_show, corners.p[board.idx[i][j]], corners.p[board.idx[i + 1][j]],
                   cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
      }
    }

    // plot coordinate system
    for(int i = 1; i < board.idx.size() * board.idx[0].size(); ++i) {
      int row = i / board.idx[0].size();
      int col = i % board.idx[0].size();
      if(board.idx[row][col] < 0 || col == board.idx[0].size() - 1 ||
         board.idx[row][col + 1] < 0 || board.idx[row + 1][col] < 0) {
        continue;
      }
      cv::line(img_show, corners.p[board.idx[row][col]], corners.p[board.idx[row][col + 1]],
               cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
      cv::line(img_show, corners.p[board.idx[row][col]], corners.p[board.idx[row + 1][col]],
               cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      break;
    }

    // plot numbers
    cv::Point2d mean(0.0, 0.0);
    for(int i = 1; i < board.idx.size() - 1; ++i) {
      for(int j = 1; j < board.idx[i].size() - 1; ++j) {
        if(board.idx[i][j] < 0) {
          continue;
        }
        mean += corners.p[board.idx[i][j]];
      }
    }
    mean /= (double)(board.num);
    mean.x -= 10;
    mean.y += 10;
    cv::putText(img_show, std::to_string(n), mean,
                cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(196, 196, 0), 2);
  }
  // imwrite("boards_img.jpg", img_show);
#if REALLINKCB_HAVE_HIGHGUI
  cv::namedWindow("boards_img", cv::WINDOW_NORMAL);
  cv::imshow("boards_img", img_show);
  cv::waitKey();
#else
  (void)params;
#endif
}

} // namespace cbdetect
