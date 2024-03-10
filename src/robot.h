#ifndef ROBOT_H
#define ROBOT_H

#include "global_struct.h"
#include "Point.h"

#include <vector>
using std::vector;

class Robot {
public:
    int x, y, goods;
    int status;
    int mbx, mby;
    int nextX, nextY;  // ��һ��λ��
    int tarX, tarY; // Ŀ��λ��
    int lastX, lastY;   // ��һ��λ��
    int goodsVal;   // �����ֵ

    int idxInPth = 0;   // ��ǰ�ߵ�·���ĵڼ�����
    vector<Point> pathPoint;    // �洢·��������
    vector<int> pathDir;    // �洢·����������
    BOT_MOVE_STATE botMoveState;
    BOT_TARGET_STATE botTarState;
    BOT_PATH_STATE botPathState;
    BOT_AVOID_STATE botAvoidState;

    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
};

#endif // ROBOT_H
