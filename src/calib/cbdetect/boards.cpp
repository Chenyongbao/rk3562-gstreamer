#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.h"
#include "growBoard.h"

namespace ReallinkCB {

static int directional_neighbor(const Corner& corners, const std::vector<int>& used,
                         int idx, const Point2d& v, double& min_dist) {
  std::vector<double> dists(corners.p.size(), 1e10);

  // distances
  for(int i = 0; i < corners.p.size(); ++i) {
    if(used[i]) {
      continue;
    }
    Point2d dir   = corners.p[i] - corners.p[idx];
    double dist_point = dir.x * v.x + dir.y * v.y;
    dir               = dir - dist_point * v;
    double dist_edge  = norm(dir);
    double dist       = dist_point + 5 * dist_edge;
    if(dist_point >= 0) {
      dists[i] = dist;
    }
  }

  // find best neighbor
  int neighbor_idx = std::min_element(dists.begin(), dists.end()) - dists.begin();
  min_dist         = dists[neighbor_idx];
  return neighbor_idx;
}

static bool init_board(const Corner& corners, std::vector<int>& used, Board& board, int idx) {
  board.idx.clear();
  // return if not enough corners
  if(corners.p.size() < 9) {
    return false;
  }

  // init chessboard hypothesis
  board.idx = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // extract feature index and orientation (central element)
  const Point2d& v1 = corners.v1[idx];
  const Point2d& v2 = corners.v3.empty() ? corners.v2[idx] : corners.v3[idx];
  board.idx[1][1]       = idx;
  used[idx]             = 1;
  double min_dist[8];

  // find left/right/top/bottom neighbors
  board.idx[1][0]       = directional_neighbor(corners, used, idx, -v1, min_dist[0]);
  used[board.idx[1][0]] = 1;
  board.idx[1][2]       = directional_neighbor(corners, used, idx, v1, min_dist[1]);
  used[board.idx[1][2]] = 1;
  board.idx[0][1]       = directional_neighbor(corners, used, idx, -v2, min_dist[2]);
  used[board.idx[0][1]] = 1;
  board.idx[2][1]       = directional_neighbor(corners, used, idx, v2, min_dist[3]);
  used[board.idx[2][1]] = 1;

  // find top-left/top-right/bottom-left/bottom-right neighbors
  int tmp1, tmp2;
  double d1, d2, min_dist_tmp1, min_dist_tmp2;
  tmp1 = directional_neighbor(corners, used, board.idx[1][0], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[0][1], -v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(norm(corners.p[tmp1] - corners.p[board.idx[1][0]]) -
                  norm(corners.p[tmp1] - corners.p[board.idx[0][1]]));
    d2 = std::abs(norm(corners.p[tmp2] - corners.p[board.idx[1][0]]) -
                  norm(corners.p[tmp2] - corners.p[board.idx[0][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[0][0] = tmp1;
  min_dist[4]     = min_dist_tmp1;
  used[tmp1]      = 1;

  tmp1 = directional_neighbor(corners, used, board.idx[1][2], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[0][1], v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(norm(corners.p[tmp1] - corners.p[board.idx[1][2]]) -
                  norm(corners.p[tmp1] - corners.p[board.idx[0][1]]));
    d2 = std::abs(norm(corners.p[tmp2] - corners.p[board.idx[1][2]]) -
                  norm(corners.p[tmp2] - corners.p[board.idx[0][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[0][2] = tmp1;
  min_dist[5]     = min_dist_tmp1;
  used[tmp1]      = 1;

  tmp1 = directional_neighbor(corners, used, board.idx[1][0], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[2][1], -v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(norm(corners.p[tmp1] - corners.p[board.idx[1][0]]) -
                  norm(corners.p[tmp1] - corners.p[board.idx[2][1]]));
    d2 = std::abs(norm(corners.p[tmp2] - corners.p[board.idx[1][0]]) -
                  norm(corners.p[tmp2] - corners.p[board.idx[2][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[2][0] = tmp1;
  min_dist[6]     = min_dist_tmp1;
  used[tmp1]      = 1;

  tmp1 = directional_neighbor(corners, used, board.idx[1][2], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[2][1], v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(norm(corners.p[tmp1] - corners.p[board.idx[1][2]]) -
                  norm(corners.p[tmp1] - corners.p[board.idx[2][1]]));
    d2 = std::abs(norm(corners.p[tmp2] - corners.p[board.idx[1][2]]) -
                  norm(corners.p[tmp2] - corners.p[board.idx[2][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[2][2] = tmp1;
  min_dist[7]     = min_dist_tmp1;
  used[tmp1]      = 1;

  // initialization must be homogenously distributed
  for(int i = 0; i < 8; ++i) {
    if(std::abs(min_dist[i] - 1e10) < 1) {
      for(int jj = 0; jj < 3; ++jj) {
        for(int ii = 0; ii < 3; ++ii) {
          used[board.idx[jj][ii]] = 0;
        }
      }
      board.idx.clear();
      return false;
    }
  }

  board.num    = 9;
  board.energy = std::move(
      std::vector<std::vector<std::vector<double>>>(3,
                                                    std::vector<std::vector<double>>(3,
                                                                                     std::vector<double>(3, DBL_MAX))));
  return true;
}

static Point3i board_energy(const Corner& corners, Board& board, const Params& params) {
  // energy: number of corners
  double E_corners = -1.0 * board.num;

  // energy: structure
  double max_E_structure = std::numeric_limits<double>::min();
  int res_x = 0, res_y = 0, res_z = 0;

  // walk through v1
  for(int i = 0; i < board.idx.size(); ++i) {
    for(int j = 0; j < board.idx[i].size() - 2; ++j) {
      int idx1 = board.idx[i][j];
      int idx2 = board.idx[i][j + 1];
      int idx3 = board.idx[i][j + 2];
      if(idx1 >= 0 && idx2 >= 0 && idx3 >= 0) {
        const Point2d& x1 = corners.p[idx1];
        const Point2d& x2 = corners.p[idx2];
        const Point2d& x3 = corners.p[idx3];
        double E_structure    = norm(x1 + x3 - 2 * x2) / norm(x1 - x3);
        board.energy[i][j][0] = E_corners * (1 - E_structure);
        if(E_structure > max_E_structure) {
          max_E_structure = E_structure;
          res_x           = j;
          res_y           = i;
          res_z           = 0;
        }
      }
    }
  }
  // walk through v3
  for(int i = 0; i < board.idx.size() - 2; ++i) {
    for(int j = 0; j < board.idx[i].size(); ++j) {
      int idx1 = board.idx[i][j];
      int idx2 = board.idx[i + 1][j];
      int idx3 = board.idx[i + 2][j];
      if(idx1 >= 0 && idx2 >= 0 && idx3 >= 0) {
        const Point2d& x1 = corners.p[idx1];
        const Point2d& x2 = corners.p[idx2];
        const Point2d& x3 = corners.p[idx3];
        double E_structure    = norm(x1 + x3 - 2 * x2) / norm(x1 - x3);
        board.energy[i][j][2] = E_corners * (1 - E_structure);
        if(E_structure > max_E_structure) {
          max_E_structure = E_structure;
          res_x           = j;
          res_y           = i;
          res_z           = 2;
        }
      }
    }
  }

  // final energy
  return {res_x, res_y, res_z};
}

static double find_minE(const Board& board, const Point2i& p) {
	double minE = std::min(std::min(board.energy[p.y][p.x][0], board.energy[p.y][p.x][1]),
		board.energy[p.y][p.x][2]);
	if (p.x - 1 >= 0) {
		minE = std::min(minE, board.energy[p.y][p.x - 1][0]);
	}
	if (p.x - 1 >= 0 && p.y - 1 >= 0) {
		minE = std::min(minE, board.energy[p.y - 1][p.x - 1][1]);
	}
	if (p.y - 1 >= 0) {
		minE = std::min(minE, board.energy[p.y - 1][p.x][2]);
	}
	if (p.x - 2 >= 0) {
		minE = std::min(minE, board.energy[p.y][p.x - 2][0]);
	}
	if (p.x - 2 >= 0 && p.y - 2 >= 0) {
		minE = std::min(minE, board.energy[p.y - 2][p.x - 2][1]);
	}
	if (p.y - 2 >= 0) {
		minE = std::min(minE, board.energy[p.y - 2][p.x][2]);
	}
	return minE;
}

static void filter_board(const Corner& corners, std::vector<int>& used, Board& board,
	std::vector<Point2i>& proposal, double& energy, const Params& params) {
	// erase wrong corners
	while (!proposal.empty()) {
		Point3i maxE_pos = board_energy(corners, board, params);
		double p_energy = board.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
		if (p_energy <= energy) {
			energy = p_energy;
			break;
		}
		if (!params.occlusion) {
			for (const auto& p : proposal) {
				used[board.idx[p.y][p.x]] = 0;
				board.idx[p.y][p.x] = -2;
				--board.num;
			}
			return;
		}

		// find the wrongest corner
		Point2i p[3];
		p[0] = { maxE_pos.x, maxE_pos.y };
		switch (maxE_pos.z) {
		case 0: {
			p[1] = { maxE_pos.x + 1, maxE_pos.y };
			p[2] = { maxE_pos.x + 2, maxE_pos.y };
			break;
		}
		case 1: {
			p[1] = { maxE_pos.x + 1, maxE_pos.y + 1 };
			p[2] = { maxE_pos.x + 2, maxE_pos.y + 2 };
			break;
		}
		case 2: {
			p[1] = { maxE_pos.x, maxE_pos.y + 1 };
			p[2] = { maxE_pos.x, maxE_pos.y + 2 };
			break;
		}
		default:
			break;
		}
		double minE_wrong[3];
		minE_wrong[0] = find_minE(board, p[0]);
		minE_wrong[1] = find_minE(board, p[1]);
		minE_wrong[2] = find_minE(board, p[2]);

		double minE = -DBL_MAX;
		int iter = 0;
		for (auto it = proposal.begin(); it < proposal.end(); ++it) {
			if (it->x == p[0].x && it->y == p[0].y && minE_wrong[0] > minE) {
				minE = minE_wrong[0];
				maxE_pos.x = it->x;
				maxE_pos.y = it->y;
				iter = it - proposal.begin();
			}
			if (it->x == p[1].x && it->y == p[1].y && minE_wrong[1] > minE) {
				minE = minE_wrong[1];
				maxE_pos.x = it->x;
				maxE_pos.y = it->y;
				iter = it - proposal.begin();
			}
			if (it->x == p[2].x && it->y == p[2].y && minE_wrong[2] > minE) {
				minE = minE_wrong[2];
				maxE_pos.x = it->x;
				maxE_pos.y = it->y;
				iter = it - proposal.begin();
			}
		}

		proposal.erase(proposal.begin() + iter);
		used[board.idx[maxE_pos.y][maxE_pos.x]] = 0;
		board.idx[maxE_pos.y][maxE_pos.x] = -2;
		--board.num;
	}
}

void boards_from_corners(const Mat& img, const Corner& corners, std::vector<Board>& boards, const Params& params) {
  // intialize boards
  boards.clear();
  if (corners.p.size() < 4) {
    cout << "Corners too few" << endl;
    return;
  }
  Board board;
  std::vector<int> used(corners.p.size(), 0);

  int start = 0;
  if(!params.overlay) {
    // start from random index
    std::default_random_engine e;
    auto time = std::chrono::system_clock::now().time_since_epoch();
    e.seed(static_cast<unsigned long>(time.count()));
    start = e() % corners.p.size();
  }

  // for all seed corners do
  int n = 0;
  while(n++ < corners.p.size()) {
    // init 3x3 board from seed i
    int i = (n + start) % corners.p.size();
    if(used[i] == 1 || !init_board(corners, used, board, i)) {
      continue;
    }

    // check if this is a useful initial guess
    Point3i maxE_pos = board_energy(corners, board, params);
    double energy        = board.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if(energy > -6.0) {
      for(int jj = 0; jj < 3; ++jj) {
        for(int ii = 0; ii < 3; ++ii) {
          used[board.idx[jj][ii]] = 0;
        }
      }
      continue;
    }

    // grow boards
    while(1) {
      int num_corners = board.num;

      for(int j = 0; j < 4; ++j) {
        std::vector<Point2i> proposal;
		BOARD_GROW_TYPE grow_type = grow_board(corners, used, board, proposal, j, params);
        if(grow_type == BOARD_GROW_FAIL) {
          continue;
        }

        filter_board(corners, used, board, proposal, energy, params);

        if(grow_type == BOARD_GROW_INSIDE) {
          --j;
        }
      }

      // exit loop
      if(board.num == num_corners) {
        break;
      }
    }

    if(!params.overlay) {
      boards.emplace_back(board);
      continue;
    }

    std::vector<std::pair<int, double>> overlap;
    for(int j = 0; j < boards.size(); ++j) {
      // check if new chessboard proposal overlaps with existing chessboards
      for(int k1 = 0; k1 < board.idx.size(); ++k1) {
        for(int k2 = 0; k2 < board.idx[0].size(); ++k2) {
          for(int l1 = 0; l1 < boards[j].idx.size(); ++l1) {
            for(int l2 = 0; l2 < boards[j].idx[0].size(); ++l2) {
              if(board.idx[k1][k2] != -1 && board.idx[k1][k2] != -2 && board.idx[k1][k2] == boards[j].idx[l1][l2]) {
                Point3i maxE_pos_tmp = board_energy(corners, boards[j], params);
                overlap.emplace_back(std::make_pair(j, boards[j].energy[maxE_pos_tmp.y][maxE_pos_tmp.x][maxE_pos_tmp.z]));
                goto GOTO_BREAK;
              }
            }
          }
        }
      }
    }
  GOTO_BREAK:;

    if(overlap.empty()) {
      boards.emplace_back(board);
    } else {
      bool is_better = true;
      for(int j = 0; j < overlap.size(); ++j) {
        if(overlap[j].second <= energy) {
          is_better = false;
          break;
        }
      }
      if(is_better) {
        std::vector<Board> tmp;
        for(int j = 0, k = 0; j < boards.size(); ++j) {
          if(overlap[k].first == j) {
            continue;
            ++k;
          }
          tmp.emplace_back(boards[j]);
        }
        std::swap(tmp, boards);
        boards.emplace_back(board);
      }
    }
    std::fill(used.begin(), used.end(), 0);
    n += 2;
  }
}

}//namespace