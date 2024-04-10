#ifndef BOAT_H
#define BOAT_H
#include "global_struct.h"
#include "constants.h"
#include <vector>
using std::vector;
class Boat
{
public:
    int id;
    int numBoatGoods; // 船目前装载的货物量
    int valBoatGoods; // 船目前装载的货物价值
    int tarBerthID;   // 船的目标泊位
    int boatStatus;   // 正常行驶状态（状态 0）,恢复状态（状态1）,装载状态（状态 2）
    int capacity;     // 船的最大装载量

    int tarX, tarY;   // 目标位置
    int curX, curY;   // 当前位置
    int lastX, lastY; // 上一个位置

    int dire;                        // 船的方向
    vector<int> pathDir;             // 存储路径方向序列
    int idxInPth = 0;                // 当前走到路径的第几个点
    BOAT_MOVE_STATE boatMoveState;   // boat的移动状态
    BOAT_PATH_STATE boatPathState;   // boat的是否是否找到移动路径状态
    BOAT_TARGET_STATE boatTarState;  // boat的是否有目标的状态
    BOAT_FLASH_STATE boatFlashState; // boat的是否在闪现的状态
    vector<BoatPoint> pathPoint;     // 存储路径点序列
    bool findPathFlag;               // 标识上一次是否成功找到路

    int jamDetectBufferLen = BOAT_JAM_BUFFER_SIZE;   // 堵塞检测缓冲区的长度
    int nearerJamdetectLen = BOAT_NEARER_JAM_BUFFER_SIZE;   // 采取避让-等待方式进行避让的检测长度
    int avoidPriority;                               // 自身当前的避让优先级
    int avoidBoatID;                                 // 当前正在避让的boat的ID，没有避让的robot时，值为-1，当前仅允许boat同时只能有一个避让boat
    int tmpTarX, tmpTarY;                            // 中途点，作为避让路径的终点
    BOAT_AVOID_STATE boatAvoidState;                 // boat的是否正在避让状态
    BoatPoint jamDetectBuffer[BOAT_JAM_BUFFER_SIZE]; // 堵塞检测缓冲区，存储的是boat的路径点
    bool setMove;                                    // 记录前一帧是否有下达移动指令
    int jamTime;                                     // 堵塞时长计数

    Boat(int c) : numBoatGoods(0), tarBerthID(-2), boatStatus(1), capacity(c), tarX(0), tarY(0), curX(0), curY(0),
                  lastX(0), lastY(0), dire(0), idxInPth(0), boatMoveState(BOAT_WAITING), boatPathState(BOAT_NO_PATH),
                  boatTarState(BOAT_NO_TARGET), boatFlashState(BOAT_NO_FLASH), findPathFlag(true), avoidPriority(0), 
                  avoidBoatID(-1), tmpTarX(0), tmpTarY(0), boatAvoidState(BOAT_NO_AVOIDING), setMove(false), jamTime(0)
    {
    }
};

#endif // BOAT_H
