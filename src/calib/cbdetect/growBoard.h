#ifndef _CBDETECT_GROW_BOARD_H_
#define _CBDETECT_GROW_BOARD_H_

#include <vector>

#include "config.h"

namespace ReallinkCB {

enum BOARD_GROW_TYPE {
	BOARD_GROW_FAIL = 0,
	BOARD_GROW_INSIDE,
	BOARD_GROW_BOUNDARY,
};

BOARD_GROW_TYPE grow_board(const Corner& corners, std::vector<int>& used, Board& board,
                                         std::vector<Point2i>& proposal, int direction, const Params& params);

}

#endif 
