#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include "robot.h"
#include "berth.h"
#include "boat.h"
#include "global_vars.h"
#include "global_struct.h"
#include "decision_maker.h"
#include <string>
#include <ctime>
#include <algorithm>
// #include <chrono>
using namespace std;

std::ifstream myCin;
bool Debug = false;
bool Record = false;
void printData();
DecisionMaker decisionMaker;

void Init()
{
    if (Debug)
        myCin.open("output.txt");

    for (int i = 0; i < MAP_SIZE; i++)
    {
        if (Debug)
            myCin >> oriMap[i];
        else
            scanf("%s", oriMap[i]);
    }

    // 读入泊位信息
    if (Debug)
        myCin >> berthNum;
    else
        scanf("%d", &berthNum);
    berth.resize(berthNum);
    for (int i = 0; i < berthNum; i++)
    {
        int id;
        if (Debug)
        {
            myCin >> id;
            myCin >> berth[id].x >> berth[id].y >> berth[id].loadingSpeed;
        }
        else
        {
            scanf("%d", &id);
            scanf("%d %d %d", &berth[id].x, &berth[id].y, &berth[id].loadingSpeed);
        }
    }
    // 必须在读入所有泊位信息后调用
    decisionMaker.analyzeMap();

    for (int i = 0; i < berthNum; ++i)
    { // 获得泊位间的连通性信息
        decisionMaker.getConnectedBerth(i);
    }

    if (Debug)
        myCin >> boatCapacity;
    else
        scanf("%d", &boatCapacity);

    // if (oriMap[100][101] == 'T')
    // {
    // decisionMaker.setParams(0.4, 1.5, 100, 500, 0, 4.0, 1, 15, 5.5);
    // }
    // else
    // {
    // decisionMaker.setParams(0.4, 1.5, 100, 500, 0, 4.0, 2, 16, 5.5);
    // }

    // ifstream paramFile("param.txt");
    // double param;
    // for (int i = 0; i < 6; ++i)
    //{
    //    paramFile >> param;
    //    s_p.push_back(param);
    //}
    //decisionMaker.setParams(s_p[0], s_p[1], s_p[2], s_p[3], s_p[4], 2, 16, s_p[5]);
    decisionMaker.setParams(0.4, 1.5, 100, 4.5, 4.0, 2, 16, 5.3, 1);

    string okk;
    if (Debug)
        myCin >> okk;
    else
        scanf("%s", okk.c_str());
    printf("OK\n");
    fflush(stdout);

    goods_num_inBerth.resize(berthNum);
    goods_totVal_inBerth.resize(berthNum);
    robot_num_inBerth.resize(berthNum);
    for (int i = 0; i < berthNum; ++i)
    {
        goods_num_inBerth[i].resize(15000);
        goods_totVal_inBerth[i].resize(15000);
        robot_num_inBerth[i].resize(15000);
    }
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
    goodsNum += K;
    // numCurGoods += K;
    int frameModIdx = (frameId - 1) % 1000;
    goodsInfo[frameModIdx].clear(); // 先把对应时刻的全部货物信息清空
    for (int i = 1; i <= K; i++)
    {
        int x, y, val;
        if (Debug)
            myCin >> x >> y >> val;
        else
            scanf("%d %d %d", &x, &y, &val);
        if (val == 0) // 货物消失或被取走
            continue;
        totGoodsVal += val;
        goodsInMap[x][y] = val;
        goodsLeftTime[x][y] = 1000;
        if (nearBerthDis[x][y] == 0)
            decisionMaker.getNearBerthDis(x, y);
        else
        {
            int berthID = nearBerthID[x][y];
            ++berth[berthID].totGoodsInBerthZone;
            goodsIDInBerthZone[x][y] = berth[berthID].totGoodsInBerthZone;
            berth[berthID].goodsInBerthInfo.emplace(berth[berthID].totGoodsInBerthZone, singleGoodsInfo(goodsInMap[x][y], 2 * nearBerthDis[x][y], x, y));
            ++numCurGoods;
        }
        /* 分析用 */
        goods_frame.push_back(frameId);
        goods_val.push_back(val);
        goods_region.push_back(nearBerthID[x][y]);
        goodsInfo[frameModIdx].emplace(x * MAP_SIZE + y, 1000);
        /* 分析用 */
    }
    /* 分析用 */
    for (int i = 0; i < berthNum; ++i)
    {
        goods_num_inBerth[i][frameId - 1] = berth[i].goodsInBerthInfo.size();
        int tmpSum = 0;
        for (auto iter = berth[i].goodsInBerthInfo.begin(); iter != berth[i].goodsInBerthInfo.end(); ++iter)
            tmpSum += iter->second.goodsVal;
        goods_totVal_inBerth[i][frameId - 1] = tmpSum;
        robot_num_inBerth[i][frameId - 1] = berth[i].numServingRobot;
    }

    /* 分析用 */
    if (Debug)
        myCin >> robotNum;
    else
        scanf("%d", &robotNum);
    robot.resize(robotNum);
    for (int i = 0; i < robotNum; i++)
    {
        if (Debug)
            myCin >> robot[i].id >> robot[i].carryGoods >> robot[i].curX >> robot[i].curY;
        else
            scanf("%d %d %d %d", &robot[i].id, &robot[i].carryGoods, &robot[i].curX, &robot[i].curY);
    }
    if (Debug)
        myCin >> boatNum;
    else
        scanf("%d", &boatNum);
    boat.resize(boatNum, Boat(boatCapacity));
    for (int i = 0; i < boatNum; i++)
    {
        if (Debug)
            myCin >> boat[i].id >> boat[i].numBoatGoods >> boat[i].curX >> boat[i].curY >> boat[i].dire >> boat[i].boatStatus;
        else
        {
            scanf("%d %d %d %d %d %d", &boat[i].id, &boat[i].numBoatGoods, &boat[i].curX, &boat[i].curY, &boat[i].dire, &boat[i].boatStatus);
        }
    }
    string okk;
    if (Debug)
        myCin >> okk;
    else
        scanf("%s", okk.c_str());
    // 清空机器人为泊位的服务情况
    for (int i = 0; i < berthNum; ++i)
        berth[i].numServingRobot = 0;
    return frameId;
}

int main()
{
    // auto start = std::chrono::steady_clock::now();
    //  ofstream outputFile("data.csv");
    // outputFile << "goodsNum, pickGoodsNum, shipGoodsNum" << endl;
    Init();
    for (frame = 1; frame <= 15000; frame++)
    {
        int id = Input();
        decisionMaker.makeDecision();
        if (frame == 15000)
            printData();
        printf("OK\n");
        fflush(stdout);
        // outputFile << goodsNum << ", " << pickGoodsNum << ", " << shipGoodsNum << endl;

        for (int i = 0, x = 0, y = 0, idx = 0; i < 1000; ++i)
        { // 维护货物的剩余存在时间
            for (auto iter = goodsInfo[i].begin(); iter != goodsInfo[i].end(); ++iter)
            {
                if (iter->second > 0)
                {
                    --iter->second;
                    idx = iter->first;
                    x = idx / MAP_SIZE;
                    y = idx - x * MAP_SIZE;
                    if (goodsInMap[x][y] != 0) // 说明该位置在此刻有货物存在（货物生成了，且暂时没被机器人取走）
                    {
                        goodsLeftTime[x][y] = iter->second;
                        if (goodsLeftTime[x][y] == 0)
                        { // 货物消失，货物价值归零
                            if (goodsInMap[x][y] > 0)
                            { // 该货物未被机器人选定为目标（选定为目标的话，场上货物数目已经减去一次1了）
                                if (nearBerthDis[x][y] != 0)
                                { // if条件不成立，说明该货物对于所有泊位都不可达
                                    --numCurGoods;
                                    int berthID = nearBerthID[x][y];
                                    int goodsID = goodsIDInBerthZone[x][y];
                                    berth[berthID].goodsInBerthInfo.erase(goodsID);
                                }
                            }
                            goodsInMap[x][y] = 0;
                        }
                    }
                    if (goodsInMap[x][y] == 0)
                    { // 货物在当前帧被机器人拿走或剩余存在时间刚好被归零，直接将货物剩余存在时间归零
                        iter->second = 0;
                        goodsLeftTime[x][y] = iter->second;
                    }
                }
            }
        }
    }
    // outputFile.close();
    // 定义结束时间点
    // auto end = std::chrono::steady_clock::now();

    //// 计算时间差
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    //// 输出时间差
    // std::cerr << "Time taken: " << duration.count() << " milliseconds" << std::endl;
    return 0;
}

void printData()
{
    if (Record)
    {
        int sum = 0;
        for (int i = 0; i < robotNum; i++)
        {
            cerr << "robot[" << i << "].total_goods_val = " << robot[i].total_goods_val << endl;
            sum += robot[i].total_goods_val;
        }
        cerr << "theory score = " << sum - boatNum * 8000 - robotNum * 2000 + 25000 << endl;
        sum = 0;
        for (int i = 0; i < berthNum; i++)
        {
            cerr << "berth[" << i << "].numBerthGoods = " << berth[i].numBerthGoods << endl;
            sum += berth[i].getBerthGoodsValueOfNum(berth[i].numBerthGoods, 0, 0);
        }
        cerr << "sum = " << sum << endl;
        cerr << "totGoodsVal = " << totGoodsVal << endl;

        ofstream outputFile("pullInfo.csv");
        outputFile << "goodsValue,goodsRegion,Frame" << endl;
        for (int i = 0; i < goods_pull_value.size(); i++)
        {
            outputFile << goods_pull_value[i] << "," << goods_pull_region[i] << "," << goods_pull_frame[i] << endl;
        }
        outputFile.close();
        ofstream outputFile2("goodsInfo.csv");
        outputFile2 << "goodsValue,goodsRegion,Frame" << endl;
        for (int i = 0; i < goods_val.size(); i++)
        {
            outputFile2 << goods_val[i] << "," << goods_region[i] << "," << goods_frame[i] << endl;
        }
        outputFile2.close();
        ofstream outputFile3("goods_num_inBerth.csv");
        for (const auto &row : goods_num_inBerth)
        {
            for (auto iter = row.begin(); iter != row.end(); ++iter)
            {
                outputFile3 << *iter;
                if (iter != row.end() - 1) // not the last element
                    outputFile3 << ",";
            }
            outputFile3 << std::endl;
        }
        outputFile3.close();
        ofstream outputFile4("goods_totVal_inBerth.csv");
        for (const auto &row : goods_totVal_inBerth)
        {
            for (auto iter = row.begin(); iter != row.end(); ++iter)
            {
                outputFile4 << *iter;
                if (iter != row.end() - 1) // not the last element
                    outputFile4 << ",";
            }
            outputFile4 << std::endl;
        }
        outputFile4.close();
        ofstream outputFile5("robot_num_inBerth.csv");
        for (const auto &row : robot_num_inBerth)
        {
            for (auto iter = row.begin(); iter != row.end(); ++iter)
            {
                outputFile5 << *iter;
                if (iter != row.end() - 1) // not the last element
                    outputFile5 << ",";
            }
            outputFile5 << std::endl;
        }
        outputFile5.close();
    }
}