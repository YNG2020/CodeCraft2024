#ifndef BOAT_H
#define BOAT_H
#include "global_struct.h"
#include "constants.h"
#include <vector>
using std::vector;
class Boat
{
public:
    int id;
    int numBoatGoods;              // 船目前装载的货物量
    int tarPos;                    // 船的目标泊位，如果目标泊位是虚拟点，则为-1（系统维护）
    int boatStatus;                // 0表示移动（运输）中，1 表示正常运行状态(即装货状态或运输完成状态)，2表示泊位外等待状态
    int capacity;                  // 船的最大装载量
    int curX, curY;                // 当前位置
    int dire;                      // 船的方向
    vector<SimplePoint> pathPoint; // 存储路径点序列
    vector<int> pathDir;           // 存储路径方向序列
    int idxInPth = 0;              // 当前走到路径的第几个点
    Boat(int c) : numBoatGoods(0), tarPos(-1), boatStatus(1), capacity(c) {}
};

#endif // BOAT_H
