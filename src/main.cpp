#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include "robot.h"
#include "berth.h"
#include "boat.h"
#include "global_vars.h"
#include "decision_maker.h"
#include <string>
#include <ctime>
#include <algorithm>


using namespace std;
std::ifstream myCin;
bool Debug = false;
DecisionMaker decisionMaker;

void Init()
{

    std::srand(1234); // 这里的1234可以是任何你喜欢的整数
    if (Debug)
        myCin.open("output.txt");

    for (int i = 0; i < MAP_SIZE; i++)
    {
        if (Debug)
            myCin >> map[i];
        else
            cin >> map[i];
    }
    for (int i = 0; i < BERTH_NUM; i++)
    {
        int id;
        if (Debug)
        {
            myCin >> id;
            myCin >> berth[id].x >> berth[id].y >> berth[id].transportTime >> berth[id].loadingSpeed;
        }
        else
        {
            cin >> id;
            cin >> berth[id].x >> berth[id].y >> berth[id].transportTime >> berth[id].loadingSpeed;
        }
    }
    if (Debug)
        myCin >> boatCapacity;
    else
        cin >> boatCapacity;


    if (map[0][0] == '*' && map[0][MAP_SIZE - 1] == '*' && map[MAP_SIZE - 1][0] == '*' && map[MAP_SIZE - 1][MAP_SIZE - 1] == '*' && map[14][43] == 'A')
    {
        decisionMaker.setParams(0.4, 1.4, 1000, 1000, 100, 4.0);
    }
    else if (map[0][0] == '.' && map[0][MAP_SIZE - 1] == '.' && map[MAP_SIZE - 1][0] == '.' && map[MAP_SIZE - 1][MAP_SIZE - 1] == '.')
    {
        decisionMaker.setParams(0.5, 1.4, 1000, 1000, 0, 4.0);
    }
    else
    {
        decisionMaker.setParams(0.5, 1.4, 1000, 1000, 0, 4.0);
    }

    for (int i = 0; i < BOAT_NUM; ++i)
    {
        boat[i].setCapacity(boatCapacity);
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
        myCin >> frameId >> money;
    else
        cin >> frameId >> money;
    if (Debug)
        myCin >> K;
    else
        cin >> K;
    numCurGoods += K;
    goodsNum += K;
    for (int i = 1; i <= K; i++)
    {
        int x, y, val;
        if (Debug)
            myCin >> x >> y >> val;
        else
            cin >> x >> y >> val;
        goodsInMap[x][y] = val;
        goodsLeftTime[x][y] = 1000;
        if (nearBerthDis[x][y] == 0)
            decisionMaker.getNearBerthDis(x, y);
    }
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        if (Debug)
            myCin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
        else
            cin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
    }
    for (int i = 0; i < BOAT_NUM; i++)
        if (Debug)
            myCin >> boat[i].boatStatus >> boat[i].tarPos;
        else
            cin >> boat[i].boatStatus >> boat[i].tarPos;
    string okk;
    if (Debug)
        myCin >> okk;
    else
        cin >> okk;

    return frameId;
}
int main()
{
    srand((unsigned int)time(nullptr)); // Seed for random number generation
    // ofstream outputFile("data.csv");
    // outputFile << "goodsNum, pickGoodsNum, shipGoodsNum" << endl;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        cout << "OK" << endl;
        cout.flush();
        // outputFile << goodsNum << ", " << pickGoodsNum << ", " << shipGoodsNum << endl;

        for (int i = 0; i < MAP_SIZE; ++i)
        { // 维护货物的剩余存在时间
            for (int j = 0; j < MAP_SIZE; ++j)
            {
                if (goodsInMap[i][j] != 0) // 说明该位置有货物存在，该值等于0时，不必维护该信息
                {
                    --goodsLeftTime[i][j];
                    if (goodsLeftTime[i][j] == 0) // 货物消失，货物价值归零
                    {
                        --numCurGoods;
                        goodsInMap[i][j] = 0;
                    }
                }    

            }
        }
    }
    // outputFile.close();
    return 0;
}
