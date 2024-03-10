#ifndef ROBOT_H
#define ROBOT_H

#include "global_struct.h"
#include "Point.h"

#include <vector>
using std::vector;

class Robot {
public:
    int x, y, goods;
    int status;
    int mbx, mby;
    int nextX, nextY;  // 下一个位置
    int tarX, tarY; // 目标位置
    int lastX, lastY;   // 上一个位置
    int goodsVal;   // 货物价值

    int idxInPth = 0;   // 当前走到路径的第几个点
    vector<Point> pathPoint;    // 存储路径点序列
    vector<int> pathDir;    // 存储路径方向序列
    BOT_MOVE_STATE botMoveState;
    BOT_TARGET_STATE botTarState;
    BOT_PATH_STATE botPathState;
    BOT_AVOID_STATE botAvoidState;

    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
};

#endif // ROBOT_H
