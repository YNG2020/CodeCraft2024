#include "global_vars.h"

Robot robot[ROBOT_NUM];
Berth berth[BERTH_NUM];
Boat boat[BOAT_NUM];


int money, boatCapacity, frameId, frame, K, numCurGoods = 0;
double limToTryChangeGoods, limToChangeGoods;
int extraSearchTime;
int blockBerthTime;
int meanGoodsValue;
int gainForSameBerth;
char map[N][N];
int goodsInMap[N][N];
int goodsLeftTime[N][N];
int nearBerthDis[N][N];
int nearBerthID[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int goodsNum;
int pickGoodsNum;
int shipGoodsNum;