#include "global_vars.h"

Robot robot[robot_num];
Berth berth[berth_num];
Boat boat[boat_num];

int money, boat_capacity, id, frame;
char map[N][N];
int goodsInMap[N][N];
int goodsLeftTime[N][N];
int nearBerthDis[N][N];
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int goods_num;
int pick_goods_num;
int ship_goods_num;