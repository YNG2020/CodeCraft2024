#ifndef ROBOT_H
#define ROBOT_H

#include "global_struct.h"
#include "Point.h"

#include <vector>
using std::vector;

class Robot {
public:
    int curX, curY;     // ��ǰλ��
    int carryGoods;     // �Ƿ�Я������
    int status;         // 0��ʾ�ָ�״̬��1��ʾ��������״̬
    int tarX, tarY;     // Ŀ��λ��
    int lastX, lastY;   // ��һ��λ��
    int goodsVal;       // �����ֵ
    int idxInPth = 0;   // ��ǰ�ߵ�·���ĵڼ�����
    const int jamDetectBufferLen = 2;   // ������⻺�����ĳ���
    int avoidPriority;  // ����ǰ�ı������ȼ�
    int avoidBotID;     // ��ǰ���ڱ��õ�robot��ID��û�б��õ�robotʱ��ֵΪ-1����ǰ������robotͬʱֻ����һ������robot
    int tmpTarX, tmpTarY;   // ��;�㣬��Ϊ����·�����յ�

    vector<Point> pathPoint;    // �洢·��������
    vector<int> pathDir;        // �洢·����������
    BOT_MOVE_STATE botMoveState;    // robot���ƶ�״̬
    BOT_TARGET_STATE botTarState;   // robot���Ƿ���Ŀ��״̬������Ŀ���λĿ�꣩
    BOT_PATH_STATE botPathState;    // robot���Ƿ��Ƿ��ҵ��ƶ�·��״̬
    BOT_AVOID_STATE botAvoidState;  // robot���Ƿ����ڱ���״̬
    int jamDetectBuffer[2];   // ������⻺�������洢����robot��·���ĵ���map����ţ���ά��һά��

    Robot() {}
    Robot(int startX, int startY) {
        curX = startX;
        curY = startY;
    }
};

#endif // ROBOT_H
