#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "robot.h"
#include "berth.h"
#include "boat.h"
#include <vector>
#include <utility> // for std::pair

const int mapSize = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;
const int distExRecoverBot = 25; // �����ڻָ�״̬�Ļ������ų���·��������Χ����С����
const int GoodsValueMax = 200;
extern Robot robot[robot_num];
extern Berth berth[berth_num];
extern Boat boat[boat_num];

extern int money, boat_capacity, id, frame;
extern char map[N][N];
extern int goodsInMap[N][N];
extern int goodsLeftTime[N][N]; // ��¼�����ʣ�����ʱ��
extern int nearBerthDis[N][N];  // ��¼ÿ���㵽����Ĳ�λ�ľ���
extern int dx[4];
extern int dy[4];
extern vector<std::vector<std::pair<int, int>>> goodsState; // ��¼��i֡ʱ�Ļ���λ����Ϣ
extern int goods_num;                                       // �����ã���ͼ���ɵĻ�������
extern int pick_goods_num;                                  // �����ã������˼���Ļ�������
extern int ship_goods_num;                                  // �����ã������ߵĻ�������

#endif // GLOBAL_VARS_H
