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
extern int tradeNum;
extern vector<Robot> robot;
extern vector<Boat> boat;
extern vector<Berth> berth;
extern vector<vector<int>> berthTradeDis;

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
extern int dirBoatDxRev[3][4]; // 第一个index：0顺时针 1逆时针 2前进；第二个index表示原本的方向
extern int dirBoatDyRev[3][4];
extern int clockWiseDirRev[2][4]; // 0顺时针,1逆时针映射
extern int DirRev[4];
/* 分析用 */
extern int goodsNum;                            // 分析用，地图生成的货物总量
extern int pickGoodsNum;                        // 分析用，机器人捡起的货物总量
extern int shipGoodsNum;                        // 分析用，船运走的货物总量
extern vector<int> goods_val;                   // 记录每个货物的价值
extern vector<int> goods_frame;                 // 记录每个货物出现的时间
extern vector<int> goods_region;                // 记录每个货物所在的区域
extern vector<int> goods_pull_frame;            // 记录每个货物被装载的时间
extern vector<int> goods_pull_value;            // 记录每个货物被装载的价值
extern vector<int> goods_pull_region;           // 记录每个货物被装载的区域
extern vector<vector<int>> goods_num_inBerth;			// 记录每个泊位对应的区域的货物信息
extern vector<vector<int>> goods_totVal_inBerth;		// 记录每个泊位对应的区域的货物总价值
extern vector<vector<int>> robot_num_inBerth;		// 记录每一帧为对应的泊位进行服务的机器人的数量
/* 分析用 */
extern int berthDis[20][4][MAP_SIZE][MAP_SIZE]; // 记录每个点到泊位的距离（海上）暂定20上限
extern int tradeDis[10][4][MAP_SIZE][MAP_SIZE]; // 记录每个点到交货点的距离（海上）暂定10上限
#endif                                          // GLOBAL_VARS_H
