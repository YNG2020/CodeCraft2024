#ifndef BERTH_H
#define BERTH_H

class Berth
{
public:
    int x, y;
    int transport_time;
    int loading_speed;
    int goodsNum;

    int load(int boat_capacity_remain)
    {
        int loadNum = goodsNum > loading_speed ? loading_speed : goodsNum;
        loadNum = loadNum > boat_capacity_remain ? boat_capacity_remain : loadNum;
        goodsNum -= loadNum;
        return loadNum;
    }

    Berth() {}
    Berth(int newX, int newY, int newTransportTime, int newLoadingSpeed)
    {
        x = newX;
        y = newY;
        transport_time = newTransportTime;
        loading_speed = newLoadingSpeed;
        goodsNum;
    }
};

#endif // BERTH_H
