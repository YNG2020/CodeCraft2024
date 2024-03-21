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

void robotInit();
void berthInit();
void boatInit();

using namespace std;
std::ifstream myCin;
bool Debug = false;
DecisionMaker decisionMaker;

void Init()
{
    std::srand(1234); // 这里的1234可以是任何你喜欢的整数
    if (Debug)
        myCin.open("output.txt");

    for (int i = 0; i < mapSize; i++)
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
            myCin >> berth[id].x >> berth[id].y >> berth[id].transportTime >> berth[id].loadingSpeed;
        }
        else
        {
            cin >> id;
            cin >> berth[id].x >> berth[id].y >> berth[id].transportTime >> berth[id].loadingSpeed;
        }
    }
    if (Debug)
        myCin >> boat_capacity;
    else
        cin >> boat_capacity;

    robotInit();
    berthInit();
    boatInit();

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
        myCin >> frame_id >> money;
    else
        cin >> frame_id >> money;
    int num;
    if (Debug)
        myCin >> num;
    else
        cin >> num;
    goods_num += num;
    for (int i = 1; i <= num; i++)
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
    for (int i = 0; i < robot_num; i++)
    {
        if (Debug)
            myCin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
        else
            cin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
    }
    for (int i = 0; i < boat_num; i++)
        if (Debug)
            myCin >> boat[i].boatStatus >> boat[i].tarPos;
        else
            cin >> boat[i].boatStatus >> boat[i].tarPos;
    string okk;
    if (Debug)
        myCin >> okk;
    else
        cin >> okk;

    return frame_id;
}
int main()
{
    srand((unsigned int)time(nullptr)); // Seed for random number generation
    // ofstream outputFile("data.csv");
    // outputFile << "goods_num, pick_goods_num, ship_goods_num" << endl;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        cout << "OK" << endl;
        cout.flush();
        // outputFile << goods_num << ", " << pick_goods_num << ", " << ship_goods_num << endl;

        for (int i = 0; i < mapSize; ++i)
        { // 维护货物的剩余存在时间
            for (int j = 0; j < mapSize; ++j)
            {
                if (goodsInMap[i][j] != 0) // 说明该位置有货物存在，该值等于0时，不必维护该信息
                    --goodsLeftTime[i][j];
                if (goodsLeftTime[i][j] == 0) // 货物消失，货物价值归零
                    goodsInMap[i][j] = 0;
            }
        }
    }
    // outputFile.close();
    return 0;
}

// robot参数的初始化
void robotInit()
{
    for (int i = 0; i < robot_num; ++i)
    {
        robot[i].botMoveState = WAITING;
        robot[i].botTarState = NO_TARGET;
        robot[i].botPathState = NO_PATH;
        robot[i].botAvoidState = NO_AVOIDING;
        robot[i].avoidBotID = -1;
        robot[i].findToBerthFlag = true;
        for (int j = 0; j < berth_num; ++j)
            robot[i].availableBerth[j] = false;
    }
}

// berth参数的初始化
void berthInit()
{
    for (int i = 0; i < berth_num; ++i)
    {
        berth[i].numBerthGoods = 0;
        berth[i].boatIDInBerth = -1;
        berth[i].boatIDToBerth = -1;
        berth[i].timeOfGoodsToBerth = 100.0; // 这个值先初始化为100.0先
        berth[i].lastTimeGetGoods = 0;
        berth[i].totGetGoodsGap = 0;
        berth[i].numGetGoods = 0;
        berth[i].isBlocked = false;
    }
}

// boat参数的初始化
void boatInit()
{
    for (int i = 0; i < boat_num; ++i)
    {
        boat[i].numBoatGoods = 0;
        boat[i].boatStatus = 1;
        boat[i].tarPos = -1;
        boat[i].capacity = boat_capacity;
    }
}
