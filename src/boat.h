#ifndef BOAT_H
#define BOAT_H

class Boat
{
public:
    int numBoatGoods; // 船目前装载的货物量
    int tarPos;       // 船的目标泊位，如果目标泊位是虚拟点，则为-1（系统维护）
    int boatStatus;   // 0表示移动（运输）中，1 表示正常运行状态(即装货状态或运输完成状态)，2表示泊位外等待状态
    int capacity;     // 船的最大装载量
    Boat() {}
};

#endif // BOAT_H
