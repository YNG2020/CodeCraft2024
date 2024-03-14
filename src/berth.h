#ifndef BERTH_H
#define BERTH_H

class Berth
{
public:
    int x, y;           // 泊位的左上角的位置
    int transportTime;  // 轮船运输到虚拟点的时间
    int loadingSpeed;   // 每帧可以装载的货物数
    int numBerthGoods;  // 泊位当前拥有的货物量
    int boatIDInBerth;  // 当前停泊在该泊位的船的ID
    int boatIDToBerth;  // 当前目标泊位为该泊位的船的ID
    double timeOfGoodsToBerth;   // 期望的robot把货物运送到berth的时间（帧/个），需要动态维护
    int lastTimeGetGoods;

    int load(int boatCapacityRemain)
    {
        int loadNum = numBerthGoods > loadingSpeed ? loadingSpeed : numBerthGoods;
        loadNum = loadNum > boatCapacityRemain ? boatCapacityRemain : loadNum;
        numBerthGoods -= loadNum;
        return loadNum;
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
