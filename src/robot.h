#ifndef ROBOT_H
#define ROBOT_H

#include "global_struct.h"
#include "Point.h"

#include <vector>
using std::vector;

class Robot {
public:
    int curX, curY;     // 当前位置
    int carryGoods;     // 是否携带货物
    int robotStatus;         // 0表示恢复状态，1表示正常运行状态
    int tarX, tarY;     // 目标位置
    int lastX, lastY;   // 上一个位置
    int goodsVal;       // 货物价值
    int idxInPth = 0;   // 当前走到路径的第几个点
    const int jamDetectBufferLen = 6;   // 堵塞检测缓冲区的长度
    int avoidPriority;  // 自身当前的避让优先级
    int avoidBotID;     // 当前正在避让的robot的ID，没有避让的robot时，值为-1，当前仅允许robot同时只能有一个避让robot
    int tmpTarX, tmpTarY;   // 中途点，作为避让路径的终点
    bool findToBerthFlag;

    vector<Point> pathPoint;    // 存储路径点序列
    vector<int> pathDir;        // 存储路径方向序列
    BOT_MOVE_STATE botMoveState;    // robot的移动状态
    BOT_TARGET_STATE botTarState;   // robot的是否有目标状态（货物目标或泊位目标）
    BOT_PATH_STATE botPathState;    // robot的是否是否找到移动路径状态
    BOT_AVOID_STATE botAvoidState;  // robot的是否正在避让状态
    int jamDetectBuffer[6];   // 堵塞检测缓冲区，存储的是robot的路径的点在map的序号（二维化一维）
    bool availableBerth[10];

    Robot() {}
    Robot(int startX, int startY) {
        curX = startX;
        curY = startY;
    }
};

#endif // ROBOT_H
