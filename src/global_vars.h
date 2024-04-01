#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "robot.h"
#include "berth.h"
#include "boat.h"
#include <vector>
#include <utility> // for std::pair
#include <unordered_map>

extern int robotNum;
extern int boatNum;
extern int berthNum;
extern vector<Robot> robot;
extern vector<Boat> boat;
extern vector<Berth> berth;

extern int money, boatCapacity, frameId, frame, K, numCurGoods;
extern char oriMap[N][N];
extern int goodsInMap[N][N];
extern vector<std::unordered_map<int, int>> goodsInfo; // 存储全体货物在地图中的位置和剩余时间信息，两个维度，第一个维度表示货物出现的时刻（对1000取模），第二个维度是一个unordered_map，键为货物的位置，值为货物的剩余存在时间
extern int goodsLeftTime[N][N];                        // 记录货物的剩余存在时间
extern int nearBerthDis[N][N];                         // 记录每个点到最近的泊位的距离
extern int nearBerthID[N][N];                          // 记录每个点到最近的泊位的ID
extern int goodsIDInBerthZone[N][N];                   // 记录对应点上的货物在最近邻泊位管理区内的ID
extern int boatTimeForDifDir[4][N][N];                 // 记录四个方向上，此点走船需要的时间，不能走则为0，能走时间为1或2
extern int dx[4];
extern int dy[4];
extern int dirBoatDx[3][4]; // 第一个index：0顺时针 1逆时针 2前进；第二个index表示原本的方向
extern int dirBoatDy[3][4];
extern int clockWiseDir[2][4]; // 0顺时针,1逆时针映射
extern int goodsNum;           // 分析用，地图生成的货物总量
extern int pickGoodsNum;       // 分析用，机器人捡起的货物总量
extern int shipGoodsNum;       // 分析用，船运走的货物总量

#endif // GLOBAL_VARS_H
