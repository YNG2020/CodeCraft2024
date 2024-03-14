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
        myCin >> id >> money;
    else
        cin >> id >> money;
    int num;
    if (Debug)
        myCin >> num;
    else
        cin >> num;
    // 输入前，先更新货物信息
    int tmpFrame = (frame - 1) % 1000;
    int tmpNum = goodsState[tmpFrame][10].first;
    for (int i = 0; i < tmpNum; ++i)
    {
        goodsInMap[goodsState[tmpFrame][i].first][goodsState[tmpFrame][i].second] = 0;
    }
    goods_num += num;
    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        if (Debug)
            myCin >> x >> y >> val;
        else
            cin >> x >> y >> val;
        goodsInMap[x][y] = val;
        goodsState[(frame - 1) % 1000][i - 1] = make_pair(x, y);
        goodsState[(frame - 1) % 1000][10] = make_pair(num, 0); // 最后一列用于存储有多少个货物
        if (nearBerthDis[x][y] == 0)
            decisionMaker.getNearBerthDis(x, y);
    }
    for (int i = 0; i < robot_num; i++)
    {
        if (Debug)
            myCin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
        else
            cin >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY >> robot[i].robotStatus;
        if (robot[i].robotStatus == 0)
            int a = 1;
    }
    for (int i = 0; i < 5; i++)
        if (Debug)
            myCin >> boat[i].boatStatus >> boat[i].tarPos;
        else
            cin >> boat[i].boatStatus >> boat[i].tarPos;
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
    ofstream outputFile("data.csv");
    outputFile << "goods_num, pick_goods_num, ship_goods_num" << endl;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        cout << "OK" << endl;
        cout.flush();
        outputFile << goods_num << ", " << pick_goods_num << ", " << ship_goods_num << endl;
    }
    outputFile.close();
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
        berth[i].timeOfGoodsToBerth = 100.0;    // 这个值先初始化为100.0先
        berth[i].lastTimeGetGoods = 0;
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
