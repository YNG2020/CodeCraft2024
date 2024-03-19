#ifndef BERTH_H
#define BERTH_H
#include <queue>
using std::queue;
class Berth
{
public:
    int x, y;                  // 泊位的左上角的位置
    int transportTime;         // 轮船运输到虚拟点的时间
    int loadingSpeed;          // 每帧可以装载的货物数
    int numBerthGoods;         // 泊位当前拥有的货物量
    int boatIDInBerth;         // 当前停泊在该泊位的船的ID
    int boatIDToBerth;         // 当前目标泊位为该泊位的船的ID
    double timeOfGoodsToBerth; // 期望的robot把货物运送到berth的时间（帧/个），需要动态维护
    int lastTimeGetGoods;   // 上一次获得货物的时刻
    int totGetGoodsGap; // 所有获得货物的时间间隔之和
    int numGetGoods;    // 获得的货物的总量
    bool isBlcoked;
    queue<int> berthGoodsValueList; // 泊位货物价值队列，与load(),pull同步更新
    int load(int boatCapacityRemain)
    {
        int loadNum = numBerthGoods > loadingSpeed ? loadingSpeed : numBerthGoods;
        loadNum = loadNum > boatCapacityRemain ? boatCapacityRemain : loadNum;
        numBerthGoods -= loadNum;
        int i = 0;
        while (i < loadNum) // load以后货物价值出队
        {
            berthGoodsValueList.pop();
            ++i;
        }
        return loadNum;
    }
    int getBerthGoodsValueOfNum(int num, int start, int GoodsValueMean) // 计算从下标start（从0开始）开始的num个货物的总价值
    {
        queue<int> berthGoodsValueListCopy = berthGoodsValueList; //
        int BerthGoodsValueOfNum = 0;
        while (!berthGoodsValueListCopy.empty() && start > 0)
        { // 如果从start开始计算货物，需要先让start个货物出队，接下来才是要计算的部分
            berthGoodsValueListCopy.pop();
            --start;
        }
        for (int i = 0; i < num; ++i)
        {
            if (berthGoodsValueListCopy.empty())
                BerthGoodsValueOfNum += GoodsValueMean; // 没有货，则加上物品价值期望
            else
            {
                BerthGoodsValueOfNum += berthGoodsValueListCopy.front();
                berthGoodsValueListCopy.pop();
            }
        }
        return BerthGoodsValueOfNum;
    }
    Berth() {}
    Berth(int newX, int newY, int newTransportTime, int newLoadingSpeed)
    {
        x = newX;
        y = newY;
        transportTime = newTransportTime;
        loadingSpeed = newLoadingSpeed;
        numBerthGoods = 0;
    }
};

#endif // BERTH_H
