#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include "robot.h"
#include "berth.h"
#include "boat.h"
#include "global_vars.h"
#include "decisionmaker.h"
#include <string>

using namespace std;
std::ifstream myCin;
bool Debug = false;

void Init()
{
    if (Debug)
        myCin.open("output.txt"); // ���ļ�

    for (int i = 0; i < n; i++)
    {
        if (Debug)
            myCin >> map[i];
        else
            cin >> map[i];
    }
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        if (Debug)
        {
            myCin >> id;
            myCin >> berth[id].x >> berth[id].y >> berth[id].transport_time >> berth[id].loading_speed;
        }
        else
        {
            cin >> id;
            cin >> berth[id].x >> berth[id].y >> berth[id].transport_time >> berth[id].loading_speed;
        }
    }
    for (int i = 0; i < robot_num; ++i)
    {
        robot[i].botMoveState = WAITING;
        robot[i].botTarState = NO_TARGET;
        robot[i].botPathState = NO_PATH;
        robot[i].botAvoidState = NO_AVOIDING;
    }
    if (Debug)
        myCin >> boat_capacity;
    else
        cin >> boat_capacity;
    for (int i = 0; i < boat_num; i++)
    {
        boat[i].num = 0;
        boat[i].status = 1;
        boat[i].pos = -1;
        boat[i].capacity = boat_capacity;
    }
    string okk;
    if (Debug)
        myCin >> okk;
    else
        cin >> okk;
    cout << "OK" << endl;
    cout.flush();
}

int Input()
{
    if (Debug)
        myCin >> id >> money;
    else
        cin >> id >> money;
    int num;
    if (Debug)
        myCin >> num;
    else
        cin >> num;
    // ����ǰ���ȸ��»�����Ϣ
    int tmpFrame = (frame - 1) % 1000;
    int tmpNum = goodsState[tmpFrame][10].first;
    for (int i = 0; i < tmpNum; ++i)
    {
        goodsInMap[goodsState[tmpFrame][i].first][goodsState[tmpFrame][i].second] = 0;
    }

    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        if (Debug)
            myCin >> x >> y >> val;
        else
            cin >> x >> y >> val;
        goodsInMap[x][y] = val;
        goodsState[(frame - 1) % 1000][i - 1] = make_pair(x, y);
        goodsState[(frame - 1) % 1000][10] = make_pair(num, 0); // ���һ�����ڴ洢�ж��ٸ�����
    }
    for (int i = 0; i < robot_num; i++)
    {
        if (Debug)
            myCin >> robot[i].goods >> robot[i].x >> robot[i].y >> robot[i].status;
        else
            cin >> robot[i].goods >> robot[i].x >> robot[i].y >> robot[i].status;
    }
    for (int i = 0; i < 5; i++)
        if (Debug)
            myCin >> boat[i].status >> boat[i].pos;
        else
            cin >> boat[i].status >> boat[i].pos;
    string okk;
    if (Debug)
        myCin >> okk;
    else
        cin >> okk;

    return id;
}
int main()
{
    srand((unsigned int)time(nullptr)); // Seed for random number generation
    DecisionMaker decisionMaker;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        cout << "OK" << endl;
        cout.flush();
    }
    return 0;
}
