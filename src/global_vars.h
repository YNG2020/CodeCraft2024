#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "robot.h"
#include "berth.h"
#include "boat.h"
#include <vector>
#include <utility> // for std::pair

extern Robot robot[ROBOT_NUM];
extern Berth berth[BERTH_NUM];
extern Boat boat[BOAT_NUM];

extern int money, boatCapacity, frameId, frame, K, numCurGoods;
extern double limToTryChangeGoods, limToChangeGoods;
extern int extraSearchTime;
extern int blockBerthTime;
extern int gainForSameBerth;
extern int meanGoodsValue;
extern char map[N][N];
extern int goodsInMap[N][N];
extern int goodsLeftTime[N][N]; // 记录货物的剩余存在时间
extern int nearBerthDis[N][N];  // 记录每个点到最近的泊位的距离
extern int nearBerthID[N][N];	// 记录每个点到最近的泊位的ID
extern int dx[4];
extern int dy[4];
extern vector<std::vector<std::pair<int, int>>> goodsState; // 记录第i帧时的货物位置信息
extern int goodsNum;                                       // 分析用，地图生成的货物总量
extern int pickGoodsNum;                                  // 分析用，机器人捡起的货物总量
extern int shipGoodsNum;                                  // 分析用，船运走的货物总量

#endif // GLOBAL_VARS_H
