#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <iostream>

using namespace std;

void DecisionMaker::shipDecision()
{   
    for (int boatID = 0; boatID < boat_num; ++boatID)
    {
        int berthID = boat[boatID].tarPos;
        // 最终装载时间到，直接去虚拟点（不管它目前是什么状态）
        if (berthID != -1 && frame == 15000 - berth[berthID].transportTime)
        {
            cout << "go " << boatID << endl;
            continue;
        }
        switch (boat[boatID].boatStatus)
        {
        case 0: // 移动（运输）中
            break;
        case 1: // 装货状态或运输完成状态（在虚拟点）
            if (boat[boatID].tarPos == -1)
            {   // 在虚拟点，上一次运输已完成。选择泊位航向，货物量置0
                boat[boatID].numBoatGoods = 0;
                berth_select(boatID, 0);     
            }
            else
            {   
                berth[boat[boatID].tarPos].boatIDInBerth = boatID;
                // 在泊位装货
                if (boat[boatID].numBoatGoods == boat[boatID].capacity)
                {   // 如果装满了，去虚拟点
                    cout << "go " << boatID << endl;
                    berth[boat[boatID].tarPos].boatIDInBerth = -1;  // 更新泊位被占用的情况
                    berth[boat[boatID].tarPos].boatIDToBerth = -1;  // 更新泊位被指向的情况
                }
                else
                {   // 没有满则继续装
                    //berth_select(boatID, 1);
                    boat[boatID].numBoatGoods += berth[berthID].load(boat[boatID].capacity - boat[boatID].numBoatGoods);
                }
            }
            break;
        case 2: // 在泊位外等待的状态
            berth_select(boatID, 1);
            break;
        default:
            break;
        }

    }
}

void DecisionMaker::berth_select(int boat_id, int oriLocation)
{

    // 为当前的boat选择泊位ID，目的是平均到每一帧的收益最大
    int moveTimeToBerth;   // 到泊位的时间
    int moveTimeToVir;   // 从虚拟点到泊位的时间
    int moveTimeFromBerth = 500;   // 从泊位到另外一个泊位的时间
    int loadGoodsTime1;  // 预计的装载货物的时间（装满的情况，假设泊位上的货物充足）
    int loadGoodsTime2;  // 预计的装载货物的时间（装满的情况，假设泊位上的货物不充足）
    int loadGoodsTime;   // 实际的装货时间
    int numNeedGoods = (boat[boat_id].capacity - boat[boat_id].numBoatGoods);    // robot剩余的要装的货物的数量
    int numRemainGoods; // berth此刻剩余的货物的数目
    int numAddGoods;    // 在boat驶向泊位期间，泊位增加的货物数
    int minTime = 100000000;
    int minIdx = 0; // 存储将要被选择的泊位ID
    int timeToGetMoney;     // 到预计拿到资金的时间

    for (int berthID = 0; berthID < berth_num; ++berthID)
    {
        //if (berthID == 1 || berthID == 4 || berthID == 6 || berthID == 8 || berthID == 9)
        //    continue;

        moveTimeToVir = berth[berthID].transportTime;
        if (oriLocation == 0)   // 说明boat是从虚拟点过来的
            moveTimeToBerth = berth[berthID].transportTime;
        else // 从泊位过来
            moveTimeToBerth = moveTimeFromBerth;

        numRemainGoods = berth[berthID].numBerthGoods;
        numAddGoods = moveTimeToBerth / berth[berthID].timeOfGoodsToBerth;

        loadGoodsTime1 = numNeedGoods / berth[berthID].loadingSpeed;
        loadGoodsTime2 = (numRemainGoods + numAddGoods) / berth[berthID].loadingSpeed + // 泊位上能以最高效率给boat装载货物的时间
            (numNeedGoods - (numRemainGoods + numAddGoods)) * berth[berthID].timeOfGoodsToBerth;    // 需要等robot给泊位送货物的时间
            
        if (numRemainGoods + numAddGoods > numNeedGoods)
        {   // （泊位新增的货物 + 泊位剩余的货物） > boat剩下要装的货物的话，说明货物确实充足
            loadGoodsTime = loadGoodsTime1;
        }
        else
        {   // 否则
            loadGoodsTime = loadGoodsTime2;
        }

        if (oriLocation == 0)   // 说明boat是从虚拟点过来的
            timeToGetMoney = moveTimeToVir + moveTimeToVir + loadGoodsTime;
        else // 从泊位过来
            timeToGetMoney = moveTimeFromBerth + moveTimeToVir + loadGoodsTime;

        //timeToGetMoney = -(berth[berthID].numBerthGoods + 800 / berth[berthID].timeOfGoodsToBerth);

        if ((timeToGetMoney < minTime) && berth[berthID].boatIDToBerth == -1)
        {
            minTime = timeToGetMoney;
            minIdx = berthID;
        }
    }
    berth[minIdx].boatIDToBerth = boat_id;
    cout << "ship " << boat_id << " " << minIdx << endl;
}
