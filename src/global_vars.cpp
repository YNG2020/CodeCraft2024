#include "global_vars.h"

vector<Robot> robot;
vector<Boat> boat;
vector<Berth> berth;

int robotNum;
int boatNum;
int berthNum;

int money, boatCapacity, frameId, frame, K, numCurGoods = 0;
char oriMap[N][N];
int goodsInMap[N][N];
vector<std::unordered_map<int, int>> goodsInfo(1000);
int boatTimeForDifDir[4][N][N];
int goodsLeftTime[N][N];
int nearBerthDis[N][N];
int nearBerthID[N][N];
int goodsIDInBerthZone[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int dirBoatDx[3][4] = {{0, 0, -2, 2}, {1, -1, -1, 1}, {0, 0, -1, 1}}; // 第一个index：0顺时针 1逆时针 2前进；第二个index表示原本的方向
int dirBoatDy[3][4] = {{2, -2, 0, 0}, {1, -1, 1, -1}, {1, -1, 0, 0}};
int clockWiseDir[2][4] = {{3, 2, 0, 1}, {2, 3, 1, 0}}; // 0顺时针方向映射 1逆时针方向映射
int goodsNum;
int pickGoodsNum;
int shipGoodsNum;