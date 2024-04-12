#ifndef BERTH_H
#define BERTH_H
#include "global_struct.h"
#include "constants.h"
#include <queue>
#include <unordered_map>
using std::unordered_map;
using std::deque;
class Berth
{
public:
    int x, y;                  // 泊位的左上角的位置
    int loadingSpeed;          // 每帧可以装载的货物数
    int numBerthGoods;         // 泊位当前拥有的货物量
    int boatIDInBerth;         // 当前停泊在该泊位的船的ID
    int boatIDToBerth;         // 当前目标泊位为该泊位的船的ID
    int lastTimeGetGoods;      // 上一次获得货物的时刻
    int totGetGoodsGap;        // 所有获得货物的时间间隔之和
    int numGetGoods;           // 获得的货物的总量
    int totGoodsInBerthZone;   // 落在泊位管理区内的货物总量
    int nearestBerth;          // 最近邻的泊位ID
    int servingRobot[20];      // 正在为该泊位服务的机器人ID，每一帧更新一次
    int numServingRobot;       // 正在为该泊位服务的机器人数量，每一帧更新一次
    int transportTime;         // 轮船运输到虚拟点的时间
    int transportTarget;           // 该泊位对应的交易点ID
    int nearestBerthTime;      // 到最近的泊位（自身不算）的时间
    int nearestBerthID;        // 最近的泊位（自身不算）的ID
    int nearestRobotShop;      // 最近的机器人租赁点的ID
    int nearestRobotShopDis = 0x7fffffff;   // 最近的机器人租赁点的距离

    double timeOfGoodsToBerth;         // 期望的robot把货物运送到berth的时间（帧/个），需要动态维护
    double meanGetGoodsRatio;          // 接收到的货物的平均性价比
    double totGetGoodsRatio;           // 接收到的货物的总性价比
    double connectedBerthMeanGoodsRatio;    // 邻接泊位的接收的货物的平均性价比
    double meanInZoneGoodsRatio;       // 当前处于该泊位管理区内的货物的平均性价比

    bool isBlocked;             // 泊位对robot屏蔽的标识
    bool connectedBerth[TEMP_BERTH_NUM];   // 与某个泊位是否相连
    unordered_map<int, singleGoodsInfo> goodsInBerthInfo;    // 存储落在该泊位管理区内的货物信息，键是标识货物的ID，值是一些货物信息
    deque<int> berthGoodsValueList; // 泊位货物价值队列，与load(),pull同步更新

    int load(int boatCapacityRemain, int &loadValue)
    {
        int loadNum = numBerthGoods > loadingSpeed ? loadingSpeed : numBerthGoods;
        loadNum = loadNum > boatCapacityRemain ? boatCapacityRemain : loadNum;
        numBerthGoods -= loadNum;
        loadValue = 0;
        int i = 0;
        while (i < loadNum) // load以后货物价值出队
        {
            loadValue += berthGoodsValueList.front();
            berthGoodsValueList.pop_front();
            ++i;
        }
        return loadNum;
    }

    int getBerthGoodsValueOfNum(int num, int start) // 计算从下标start（从0开始）开始的num个货物的总价值
    {
        int BerthGoodsValueOfNum = 0;
        int count = 0;
        for (auto value = berthGoodsValueList.begin(); value < berthGoodsValueList.end(); ++value)
        {
            if (count >= start && count < start + num)
                BerthGoodsValueOfNum += *value;
            else
                break;
            count++;
        }
        return BerthGoodsValueOfNum;
    }

    Berth() : transportTime(0), numBerthGoods(0), boatIDInBerth(-1), boatIDToBerth(-1), timeOfGoodsToBerth(100.0), lastTimeGetGoods(0),
        totGetGoodsGap(0), numGetGoods(0), isBlocked(false), meanGetGoodsRatio(0.0), totGetGoodsRatio(0.0), numServingRobot(0), nearestBerth(-1), 
        totGoodsInBerthZone(0), nearestRobotShop(-1)
    {
        goodsInBerthInfo.clear();
    }
};

#endif // BERTH_H