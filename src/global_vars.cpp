#include "global_vars.h"

vector<Robot> robot;
vector<Boat> boat;
vector<Berth> berth;
int robotNum;
int boatNum;
int berthNum;
int tradeNum;

int money, boatCapacity, frameId, frame, K, numCurGoods = 0;
char oriMap[N][N];
int goodsInMap[N][N];
vector<std::unordered_map<int, int>> goodsInfo(1000);
int boatTimeForDifDir[4][N][N];
vector<vector<int>> berthTradeDis;       // 泊位到泊位，泊位到交易点距离，0,berthNum-1为泊位ID，berthNum到berthNum+tradeNum-1为交货点ID
int berthDis[20][4][MAP_SIZE][MAP_SIZE]; // 记录每个点到不同泊位的距离（海上），20是泊位数目上限
int tradeDis[10][4][MAP_SIZE][MAP_SIZE]; // 记录每个点到不同交货点的距离（海上）10是交货点目上限
int goodsLeftTime[N][N];
int nearBerthDis[N][N];
int nearBerthID[N][N];
int goodsIDInBerthZone[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int dirBoatDx[3][4] = {{0, 0, -2, 2}, {1, -1, -1, 1}, {0, 0, -1, 1}}; // 第一个index：0顺时针 1逆时针 2前进；第二个index表示原本的方向
int dirBoatDy[3][4] = {{2, -2, 0, 0}, {1, -1, 1, -1}, {1, -1, 0, 0}};
int clockWiseDir[2][4] = {{3, 2, 0, 1}, {2, 3, 1, 0}}; // 0顺时针方向映射 1逆时针方向映射

int dirBoatDxRev[3][4] = {{1, -1, 1, -1}, {-2, 2, 0, 0}, {0, 0, -1, 1}}; // 第一个index：0顺时针 1逆时针 2前进；第二个index表示原本的方向
int dirBoatDyRev[3][4] = {{-1, 1, 1, -1}, {0, 0, -2, 2}, {1, -1, 0, 0}};
int clockWiseDirRev[2][4] = {{3, 2, 0, 1}, {2, 3, 1, 0}}; // 0顺时针,1逆时针映射
int DirRev[4] = {1, 0, 3, 2};
vector<int> robotType;
int goods_num;
int pick_goods_num;
int ship_goods_num;
int tot_goods_val;

vector<int> goods_val;
vector<int> goods_frame;
vector<int> goods_region;
vector<int> goods_pull_frame;
vector<int> goods_pull_value;
vector<int> goods_pull_region;
vector<vector<int>> goods_num_inBerth;
vector<vector<int>> goods_totVal_inBerth;
vector<vector<int>> robot_num_inBerth;
vector<double> s_p;