#ifndef BERTH_H
#define BERTH_H
#include "global_struct.h"
#include "constants.h"
#include <queue>
#include <unordered_map>
using std::deque;
class Berth
{
public:
    std::unordered_map<int, singleGoodsInfo> goodsInBerthInfo;    // 存储落在该泊位管理区内的货物信息，键是标识货物的ID，值是一些货物信息
    int x, y;                  // 泊位的左上角的位置
    int transportTime;         // 轮船运输到虚拟点的时间
    int loadingSpeed;          // 每帧可以装载的货物数
    int numBerthGoods;         // 泊位当前拥有的货物量
    int boatIDInBerth;         // 当前停泊在该泊位的船的ID
    int boatIDToBerth;         // 当前目标泊位为该泊位的船的ID
    double timeOfGoodsToBerth; // 期望的robot把货物运送到berth的时间（帧/个），需要动态维护
    int lastTimeGetGoods;      // 上一次获得货物的时刻
    int totGetGoodsGap;        // 所有获得货物的时间间隔之和
    int numGetGoods;           // 获得的货物的总量
    bool isBlocked;            // 泊位对robot屏蔽的标识
    double meanGetGoodsRatio;     // 接收到的货物的平均性价比
    double totGetGoodsRatio;      // 接收到的货物的总性价比
    int totGoodsInBerthZone;   // 落在泊位管理区内的货物总量
    // TODO
    bool connectedBerth[TEMP_BERTH_NUM];
    //
    int nearestBerth;           // 最近邻的泊位ID
    double connectedBerthMeanGoodsRatio;    // 邻接泊位的接收的货物的平均性价比
    int servingRobot[20];   // 正在为该泊位服务的机器人ID，每一帧更新一次
    int numServingRobot;    // 正在为该泊位服务的机器人数量，每一帧更新一次
    double meanInZoneGoodsRatio;    // 当前处于该泊位管理区内的货物的平均性价比

    deque<int> berthGoodsValueList; // 泊位货物价值队列，与load(),pull同步更新
    int load(int boatCapacityRemain)
    {
        int loadNum = numBerthGoods > loadingSpeed ? loadingSpeed : numBerthGoods;
        loadNum = loadNum > boatCapacityRemain ? boatCapacityRemain : loadNum;
        numBerthGoods -= loadNum;
        int i = 0;
        while (i < loadNum) // load以后货物价值出队
        {
            berthGoodsValueList.pop_front();
            ++i;
        }
        return loadNum;
    }
    int getBerthGoodsValueOfNum(int num, int start, int GoodsValueMean) // 计算从下标start（从0开始）开始的num个货物的总价值
    {

        int BerthGoodsValueOfNum = 0;
        int count = 0;
        for (auto value = berthGoodsValueList.begin(); value < berthGoodsValueList.end(); ++value)
        {
            if (count >= start && count < start + num)
                BerthGoodsValueOfNum += *value;
            count++;
        }
        while (count < start + num)
        {
            if (count >= start)
                BerthGoodsValueOfNum += GoodsValueMean;
            count++;
        }
        return BerthGoodsValueOfNum;
    }
    void Init()
    {
        transportTime = 0;
        numBerthGoods = 0;
        boatIDInBerth = -1;
        boatIDToBerth = -1;
        timeOfGoodsToBerth = 100.0;
        lastTimeGetGoods = 0;
        totGetGoodsGap = 0;
        numGetGoods = 0;
        isBlocked = false;
        meanGetGoodsRatio = 0.0;
        totGetGoodsRatio = 0.0;
        numServingRobot = 0;
        nearestBerth = 0;   // 其实是为了防止越界，正常的话，该值能被变更为正确值
        totGoodsInBerthZone = 0;
        goodsInBerthInfo.clear();
    }
    Berth()
    {
        Init();
    }
    Berth(int newX, int newY, int newTransportTime, int newLoadingSpeed)
    {
        x = newX;
        y = newY;
        transportTime = newTransportTime;
        loadingSpeed = newLoadingSpeed;
        Init();
    }
};

#endif // BERTH_H
