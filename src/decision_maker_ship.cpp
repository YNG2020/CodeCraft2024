#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <iostream>

using namespace std;

void DecisionMaker::shipDecision()
{

    for (int boatID = 0; boatID < boatNum; ++boatID)
    {
        int berthID = boat[boatID].tarPos;
        // 最终装载时间到，直接去虚拟点（不管它目前是什么状态）
        if (berthID != -1 && (frameId >= 15000 - berth[berthID].transportTime))
        {
            printf("go %d\n", boatID);
            shipGoodsNum += boat[boatID].numBoatGoods;
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
            { // 在泊位装货
                berth[boat[boatID].tarPos].boatIDInBerth = boatID;
                if (boat[boatID].numBoatGoods == boat[boatID].capacity)
                { // 如果装满了，去虚拟点
                    int newBerth = berth_select(boatID, boat[boatID].tarPos);
                    if (newBerth != boat[boatID].tarPos)
                    {                                                  // 有可能先去其它泊位，再去虚拟点的时间更短
                        berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
                        berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
                        berth[newBerth].boatIDToBerth = boatID;        // 更新泊位被指向的情况
                        break;
                    }
                    printf("go %d\n", boatID);
                    shipGoodsNum += boat[boatID].numBoatGoods;
                    berth[boat[boatID].tarPos].boatIDInBerth = -1; // 更新泊位被占用的情况
                    berth[boat[boatID].tarPos].boatIDToBerth = -1; // 更新泊位被指向的情况
                }
                else
                { // 没有满则继续装
                    int loadNum = berth[berthID].load(boat[boatID].capacity - boat[boatID].numBoatGoods);
                    boat[boatID].numBoatGoods += loadNum;
                    int newBerth = boat[boatID].tarPos;
                    if (loadNum == 0) // 尝试比较跑去别的泊位去装货的性价比
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
            berth_select(boatID, boat[boatID].tarPos);
            break;
        default:
            break;
        }
    }
}

int DecisionMaker::berth_select(int boatID, int oriLocation)
{

    // 为当前的boat选择泊位ID，目的是平均到每一帧的收益最大
    int moveTimeToBerth;                                                    // 到泊位的时间
    int moveTimeToVir;                                                      // 从虚拟点到泊位的时间
    int moveTimeFromBerth = 500;                                            // 从泊位到另外一个泊位的时间
    double loadGoodsTime1;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物充足）
    double loadGoodsTime2;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物不充足）
    double loadGoodsTime;                                                   // 实际的装货时间
    int numNeedGoods = (boat[boatID].capacity - boat[boatID].numBoatGoods); // robot剩余的要装的货物的数量
    int numRemainGoods;                                                     // berth此刻剩余的货物的数目
    double numAddGoods;                                                     // 在boat驶向泊位期间，泊位增加的货物数
    // int minTime = 100000000;
    double MaxMeanGetValue = 0; // 平均每帧得到最大的价值，初始化为0
    int minIdx = oriLocation;   // 存储将要被选择的泊位ID
    double timeToGetMoney;      // 到预计拿到资金的时间
    int Money;

    for (int berthID = 0; berthID < berthNum; ++berthID)
    {
        moveTimeToVir = berth[berthID].transportTime;
        if (oriLocation == -1)
        { // 说明boat是从虚拟点过来的
            moveTimeToBerth = berth[berthID].transportTime;
            if (frameId + 2 * moveTimeToBerth >= 15000)
            { // 这时候一定不选择该泊位，因为时间上来不及再去虚拟点
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
                if (frameId + moveTimeToBerth + berth[berthID].transportTime >= 15000)
                { // 这时候一定不选择转移，因为时间上来不及再去虚拟点
                    continue;
                }
            }
        }

        numRemainGoods = berth[berthID].numBerthGoods;
        numAddGoods = moveTimeToBerth / berth[berthID].timeOfGoodsToBerth;
        loadGoodsTime1 = (double)numNeedGoods / (double)berth[berthID].loadingSpeed;
        loadGoodsTime2 = ((double)numRemainGoods + (double)numAddGoods) / (double)berth[berthID].loadingSpeed + // 泊位上能以最高效率给boat装载货物的时间
                         (numNeedGoods - (numRemainGoods + numAddGoods)) * berth[berthID].timeOfGoodsToBerth;   // 需要等robot给泊位送货物的时间

        if (berth[berthID].isBlocked)
            numAddGoods = 0;

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
        Money = berth[berthID].getBerthGoodsValueOfNum(numNeedGoods, 0, meanGoodsValue) + 1; // +1是为了Money为0时，仍能在时间上进行比较（适用于通过泊位间的转移来再去虚拟点的情况）
        double MeanGetValue = (double)Money / (timeToGetMoney == 0 ? 1 : timeToGetMoney);
        if ((MeanGetValue > MaxMeanGetValue) && (berth[berthID].boatIDToBerth == -1 || berthID == oriLocation)) // 如果泊位有空位
        {
            MaxMeanGetValue = MeanGetValue;
            minIdx = berthID;
        }
    }
    if (minIdx == oriLocation)
        return minIdx;

    berth[minIdx].boatIDToBerth = boatID;
    printf("ship %d %d\n", boatID, minIdx);
    return minIdx;
}
