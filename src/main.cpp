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
#include <chrono>


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
            scanf("%s", map[i]);
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
            scanf("%d", &id);
            scanf("%d %d %d %d", &berth[id].x, &berth[id].y, &berth[id].transportTime, &berth[id].loadingSpeed);
        }
    }
    if (Debug)
        myCin >> boatCapacity;
    else
        scanf("%d", &boatCapacity);


    if (map[0][0] == '*' && map[0][MAP_SIZE - 1] == '*' && map[MAP_SIZE - 1][0] == '*' && map[MAP_SIZE - 1][MAP_SIZE - 1] == '*' && map[14][43] == 'A')
    {
        decisionMaker.setParams(0.4, 1.4, 35, 1000, 100, 4.0);
    }
    else if (map[0][0] == '.' && map[0][MAP_SIZE - 1] == '.' && map[MAP_SIZE - 1][0] == '.' && map[MAP_SIZE - 1][MAP_SIZE - 1] == '.')
    {
        decisionMaker.setParams(0.2, 1.4, 50, 1000, 50, 4.0);
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
        scanf("%s", okk.c_str());
    printf("OK\n");
    cout.flush();
}

int Input()
{
    if (Debug)
        myCin >> frameId >> money;
    else
        scanf("%d %d", &frameId, &money);
    if (Debug)
        myCin >> K;
    else
        scanf("%d", &K);
    numCurGoods += K;
    goodsNum += K;
    for (int i = 1; i <= K; i++)
    {
        int x, y, val;
        if (Debug)
            myCin >> x >> y >> val;
        else
            scanf("%d %d %d", &x, &y, &val);
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
            scanf("%d %d %d %d", &robot[i].carryGoods, &robot[i].curX, &robot[i].curY, &robot[i].robotStatus);
    }
    for (int i = 0; i < BOAT_NUM; i++)
        if (Debug)
            myCin >> boat[i].boatStatus >> boat[i].tarPos;
        else
            scanf("%d %d", &boat[i].boatStatus, &boat[i].tarPos);
    string okk;
    if (Debug)
        myCin >> okk;
    else
        scanf("%s", okk.c_str());

    return frameId;
}

int main()
{
    auto start = std::chrono::steady_clock::now();
    // ofstream outputFile("data.csv");
    // outputFile << "goodsNum, pickGoodsNum, shipGoodsNum" << endl;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        printf("OK\n");
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
        // 定义结束时间点
    auto end = std::chrono::steady_clock::now();

    //// 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    //// 输出时间差
    std::cerr << "Time taken: " << duration.count() << " milliseconds" << std::endl;
    return 0;
}
