#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <iostream>
#include <cmath>

using namespace std;

void DecisionMaker::shipDecision()
{

    for (int boatID = 0; boatID < boat_num; ++boatID)
    {
        int berthID = boat[boatID].tarPos;
        // 最终装载时间到，直接去虚拟点（不管它目前是什么状态）
        if (berthID != -1 && (frame >= 15000 - berth[berthID].transportTime))
        {
            std::cout << "go " << boatID << endl;
            ship_goods_num += boat[boatID].numBoatGoods;
            berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
            berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
            continue;
        }
        switch (boat[boatID].boatStatus)
        {
        case 0: // 移动（运输）中
            break;
        case 1: // 装货状态或运输完成状态（在虚拟点）
            if (berthID == -1)
            { // 在虚拟点，上一次运输已完成。选择泊位航向，货物量置0
                boat[boatID].numBoatGoods = 0;
                berth_select(boatID, -1); // 这里需要手动置oriLocation参数，因为还没发出ship指令
            }
            else
            {   // 在泊位装货
                berth[boat[boatID].tarPos].boatIDInBerth = boatID;
                if (boat[boatID].numBoatGoods == boat[boatID].capacity)
                {   // 如果装满了，去虚拟点
                    int newBerth = berth_select(boatID, boat[boatID].tarPos);
                    if (newBerth != boat[boatID].tarPos)
                    {   // 有可能先去其它泊位，再去虚拟点的时间更短
                        berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
                        berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
                        berth[newBerth].boatIDToBerth = boatID;        // 更新泊位被指向的情况
                        break;
                    }
                    cout << "go " << boatID << endl;
                    ship_goods_num += boat[boatID].numBoatGoods;
                    berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
                    berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
                }
                else
                { // 没有满则继续装
                    int loadNum = berth[berthID].load(boat[boatID].capacity - boat[boatID].numBoatGoods);
                    boat[boatID].numBoatGoods += loadNum;
                    int newBerth = boat[boatID].tarPos;
                    if (loadNum == 0) // 跑去别的泊位去装货
                        newBerth = berth_select(boatID, boat[boatID].tarPos);
                    if (newBerth != boat[boatID].tarPos)
                    {
                        berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
                        berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
                        berth[newBerth].boatIDToBerth = boatID;        // 更新泊位被指向的情况
                    }
                }
            }
            break;
        case 2: // 在泊位外等待的状态
            //berth_select(boatID, boat[boatID].tarPos);
            break;
        default:
            break;
        }
    }
}

int DecisionMaker::berth_select(int boatID, int oriLocation)
{

    // 为当前的boat选择泊位ID，目的是平均到每一帧的收益最大
    int moveTimeToBerth;                                                      // 到泊位的时间
    int moveTimeToVir;                                                        // 从虚拟点到泊位的时间
    int moveTimeFromBerth = 500;                                              // 从泊位到另外一个泊位的时间
    double loadGoodsTime1;                                                    // 预计的装载货物的时间（装满的情况，假设泊位上的货物充足）
    double loadGoodsTime2;                                                    // 预计的装载货物的时间（装满的情况，假设泊位上的货物不充足）
    double loadGoodsTime;                                                     // 实际的装货时间
    int numNeedGoods = (boat[boatID].capacity - boat[boatID].numBoatGoods); // robot剩余的要装的货物的数量
    int numRemainGoods;                                                       // berth此刻剩余的货物的数目
    double numAddGoods;                                                       // 在boat驶向泊位期间，泊位增加的货物数
    // int minTime = 100000000;
    double MaxMeanGetValue = 0; // 平均每帧得到最大的价值，初始化为0
    int minIdx = oriLocation;             // 存储将要被选择的泊位ID
    double timeToGetMoney;      // 到预计拿到资金的时间
    int Money;
    int leftTime = frame;   // 预计的robot选择该berth之后，离开该berth的时间
    int gapTime = 0;    // 预计的泊位空闲的时间
    int boatIDLastLeft; // 临时记录各个泊位预计的最后驶离的boat的ID

    for (int berthID = 0; berthID < berth_num; ++berthID)
    {
        moveTimeToVir = berth[berthID].transportTime;
        if (oriLocation == -1) 
        {   // 说明boat是从虚拟点过来的
            moveTimeToBerth = berth[berthID].transportTime;
            if (frame + 2 * moveTimeToBerth >= 15000)
            {   // 这时候一定不选择该泊位，因为时间上来不及再去虚拟点
                continue;
            }
        }
        else
        {                               // 从泊位过来
            if (berthID == oriLocation) // 从原有泊位来
                moveTimeToBerth = 0;
            else // 从别的泊位来
            {
                moveTimeToBerth = moveTimeFromBerth;
                if (frame + moveTimeToBerth + berth[berthID].transportTime >= 15000)
                {   // 这时候一定不选择转移，因为时间上来不及再去虚拟点
                    continue;
                }
            }
        }

        boatIDLastLeft = berth[berthID].boatIDLastLeft;
        gapTime = frame + moveTimeToBerth + 1 - berth[berthID].boatLeftTime[boatIDLastLeft];
        if (gapTime <= 0 && berthID != oriLocation)   // 本应保证gapTime > 0
            continue;
        if (gapTime <= 0)
            gapTime = 0;
        if (frame > berth[berthID].boatLeftTime[boatIDLastLeft])    // 如果当前帧大于该泊位最后驶离的船所对应的帧数
            numRemainGoods = berth[berthID].numBerthGoods;
        else
            numRemainGoods = 0;

        numAddGoods = gapTime / berth[berthID].timeOfGoodsToBerth;
        loadGoodsTime1 = (double)numNeedGoods / (double)berth[berthID].loadingSpeed;
        loadGoodsTime2 = ((double)numRemainGoods + (double)numAddGoods) / (double)berth[berthID].loadingSpeed + // 泊位上能以最高效率给boat装载货物的时间
            (numNeedGoods - (numRemainGoods + numAddGoods)) * berth[berthID].timeOfGoodsToBerth;   // 需要等robot给泊位送货物的时间

        if (numRemainGoods + numAddGoods > numNeedGoods)
        { // （泊位新增的货物 + 泊位剩余的货物） > boat剩下要装的货物的话，说明货物确实充足
            loadGoodsTime = loadGoodsTime1;
        }
        else
        { // 否则
            loadGoodsTime = loadGoodsTime2;
        }

        if (oriLocation == 0) // 说明boat是从虚拟点过来的
            timeToGetMoney = moveTimeToVir + moveTimeToVir + loadGoodsTime;
        else // 从泊位过来
            timeToGetMoney = moveTimeToBerth + moveTimeToVir + loadGoodsTime;
        // 计算可获得的价值
        Money = berth[berthID].getBerthGoodsValueOfNum(numNeedGoods, 0, GoodsValueMax / 2) + 1; // +1是为了Money为0时，仍能在时间上进行比较（适用于通过泊位间的转移来再去虚拟点的情况）
        double MeanGetValue = (double)Money / (timeToGetMoney == 0 ? 1 : timeToGetMoney);
        if ((MeanGetValue > MaxMeanGetValue))   // 取消泊位不被占据的限制
        {
            MaxMeanGetValue = MeanGetValue;
            minIdx = berthID;
            leftTime = frame + moveTimeToBerth + (numRemainGoods + numAddGoods) / berth[berthID].timeOfGoodsToBerth + 1; // 当前帧 + 驶向该泊位的时间 + 装载货物的时间 + 1
        }
    }

    boatIDLastLeft = berth[minIdx].boatIDLastLeft;
    
    if (minIdx == oriLocation)
    {
        if (minIdx == -1)
            return minIdx;
        berth[minIdx].boatLeftTime[boatID] += 1;
        if (berth[minIdx].boatLeftTime[boatID] > berth[minIdx].boatLeftTime[boatIDLastLeft])
            berth[minIdx].boatIDLastLeft = boatID;
        return minIdx;
    }
    
    berth[minIdx].boatLeftTime[boatID] = leftTime;
    if (berth[minIdx].boatLeftTime[boatID] > berth[minIdx].boatLeftTime[boatIDLastLeft])
        berth[minIdx].boatIDLastLeft = boatID;

    berth[minIdx].boatIDToBerth = boatID;
    std::cout << "ship " << boatID << " " << minIdx << endl;
    return minIdx;
}
