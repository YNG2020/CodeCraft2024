#ifndef ROBOT_H
#define ROBOT_H

#include "global_struct.h"
#include "constants.h"

#include <vector>
using std::vector;

class Robot
{
public:
    int id;                          // 机器人ID
    int curX, curY;                   // 当前位置
    int carryGoods;                   // 是否携带货物
    int robotStatus;                  // 0表示恢复状态，1表示正常运行状态
    int tarX, tarY;                   // 目标位置
    int lastX, lastY;                 // 上一个位置
    int goodsVal;                     // 货物价值
    int idxInPth = 0;                 // 当前走到路径的第几个点
    const int jamDetectBufferLen = JAM_BUFFER_SIZE; // 堵塞检测缓冲区的长度
    int avoidPriority;                // 自身当前的避让优先级
    int avoidBotID;                   // 当前正在避让的robot的ID，没有避让的robot时，值为-1，当前仅允许robot同时只能有一个避让robot
    int tmpTarX, tmpTarY;             // 中途点，作为避让路径的终点
    double curPropotion;              // 当前目标的分数
    double meanPropotion;             // 机器人运送货物性价比的历史平均值
    double sumPropotion;              // 机器人运送货物性价比的历史平求和值
    int cntPropotion;                 // 机器人运送货物的性价比被改变的总次数
    int pullBerthID;                  // 机器人正在卸货的泊位ID

    vector<SimplePoint> pathPoint;       // 存储路径点序列
    vector<int> pathDir;           // 存储路径方向序列
    BOT_MOVE_STATE botMoveState;   // robot的移动状态
    BOT_TARGET_STATE botTarState;  // robot的是否有目标状态（货物目标或泊位目标）
    BOT_PATH_STATE botPathState;   // robot的是否是否找到移动路径状态
    BOT_AVOID_STATE botAvoidState; // robot的是否正在避让状态
    int jamDetectBuffer[JAM_BUFFER_SIZE];        // 堵塞检测缓冲区，存储的是robot的路径的点在map的序号（二维化一维）

    Robot(int startX = 0, int startY = 0) :
        curX(startX), curY(startY), carryGoods(0), robotStatus(0), tarX(0), tarY(0),
        lastX(0), lastY(0), goodsVal(0), idxInPth(0), avoidPriority(0), avoidBotID(-1),
        tmpTarX(0), tmpTarY(0), curPropotion(0), meanPropotion(0), sumPropotion(0),
        cntPropotion(0), botMoveState(WAITING), botTarState(NO_TARGET),
        botPathState(NO_PATH), botAvoidState(NO_AVOIDING), pullBerthID(-1)
    {}
};

#endif // ROBOT_H
