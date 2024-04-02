#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <iostream>
#include <cstring>
#include <cmath>
#include <string>

void DecisionMaker::shipDecision()
{

    for (int i = 0; i < boatNum; ++i)
    {
        Boat& bot = boat[i];
        refreshBoatState(i);

        if (bot.boatMoveState == BOAT_ARRIVEBERTH)
        {
            printf("berth %d\n", i);
            bot.boatMoveState = BOAT_WAITING;   // 手动更新为原地等待的状态（等路径分配）
            bot.boatPathState = BOAT_NO_PATH;
            bot.boatTarState = BOAT_NO_TARGET;
            bot.idxInPth = 0;
        }
        if (bot.boatMoveState == BOAT_ARRIVETRADE)
        {
            bot.boatMoveState = BOAT_WAITING;   // 手动更新为原地等待的状态（等路径分配）
            bot.boatPathState = BOAT_NO_PATH;
            bot.boatTarState = BOAT_NO_TARGET;
            bot.idxInPth = 0;
            bot.numBoatGoods = 0;
        }

        switch (bot.boatStatus)
        {
        case 0: // 正常行驶状态（状态 0）            if (bot.boatTarState == BOAT_HAVE_TARGET && bot.boatPathState == BOAT_NO_PATH)            {   // 有目标，无路，分配路                bool findPathFlag = getBoatPathBFS(i, bot.tarX, bot.tarY, boat[i].pathPoint, boat[i].pathDir);                if (findPathFlag)                    bot.boatPathState = BOAT_HAVE_PATH;            }            else if (bot.boatTarState == BOAT_NO_TARGET)            {   // 无目标，直接在路径搜索的同时分配目标            }            break;
        case 1: // 恢复状态（状态1）
            break;
        case 2: // 装载状态（状态 2）
            berth[bot.tarBerthID].boatIDInBerth = i;
            if (bot.numBoatGoods == bot.capacity)
            { // 如果装满了，去虚拟点
                //printf("go %d\n", boatID);
                //shipGoodsNum += bot.numBoatGoods;
                //berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                //berth[bot.tarBerthID].boatIDToBerth = -1; // 更新泊位被指向的情况
            }
            else
            { // 没有满则继续装
                int loadNum = berth[bot.tarBerthID].load(bot.capacity - bot.numBoatGoods);
                bot.numBoatGoods += loadNum;
                int newBerth = bot.tarBerthID;
                if (loadNum == 0) // 尝试比较跑去别的泊位去装货的性价比
                    newBerth = berth_select(i, bot.tarBerthID);
                if (newBerth != bot.tarBerthID)
                {
                    //berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                    //berth[bot.tarBerthID].boatIDToBerth = -1; // 更新泊位被指向的情况
                    //berth[newBerth].boatIDToBerth = i;        // 更新泊位被指向的情况
                }
            }
            break;
        default:
            break;
        }

        if (boat[i].capacity == boat[i].numBoatGoods)
            getBoatPathBFS(i, tradePoint[0].x, tradePoint[0].y, boat[i].pathPoint, boat[i].pathDir);
    }
}

void DecisionMaker::refreshBoatState(int boatID)
{
    Boat& bot = boat[boatID];
    if (((bot.curX != bot.lastX) || ((bot.curY != bot.lastY))))
    { // 发现变更了位置（旋转操作也必定变换位置）
        ++bot.idxInPth;
        bot.lastX = bot.curX;
        bot.lastY = bot.curY;
    }

    if (getBerthId(bot.curX, bot.curY) == bot.tarBerthID && bot.boatMoveState == BOAT_TOBERTH)
    {   // 如果抵达了目标泊位所覆盖的范围
        bot.boatMoveState = BOAT_ARRIVEBERTH;
    }

    if (gridMap[bot.curX][bot.curY] == TRADE && bot.numBoatGoods > 0)
    {   // 抵达交易点，且船上有货物
        bot.boatMoveState = BOAT_ARRIVETRADE;
    }
}

// 得到船去目标点的序列(BFS)
bool DecisionMaker::getBoatPathBFS(int boatID, int tarx, int tary, vector<SimplePoint> &pathPoint, vector<int> &pathDir)
{
    int queueCount = 0;
    int queueIndex = 0;
    int firstDis = 0;
    Node *now = &nodes[queueCount++];
    Node *target = nullptr; // 用于存储找到的目标节点
    Node *child = nullptr;
    now->setNode(boat[boatID].curX, boat[boatID].curY, 0, nullptr, boat[boatID].dire);

    memset(visBoat, 0, sizeof(visBoat));
    visBoat[boat[boatID].dire][boat[boatID].curX][boat[boatID].curY] = true;

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];
        if (now->x == tarx && now->y == tary)
        {
            target = now;
            break;
        }
        for (int i = 0; i < 3; i++) // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
        {
            int nx = now->x + dirBoatDx[i][now->dir];
            int ny = now->y + dirBoatDy[i][now->dir];

            int curDir = i == 2 ? now->dir : clockWiseDir[i][now->dir];
            if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                continue;
            visBoat[curDir][nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, 0, now, curDir);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    vector<int>().swap(pathDir);           // 清空
    vector<SimplePoint>().swap(pathPoint); // 清空
    if (target != nullptr)
    {
        pathPoint.push_back(SimplePoint(target->x, target->y));
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][now->dir] && p->y == p->parent->y + dirBoatDx[i][now->dir])
                {
                    pathDir.push_back(i);
                    pathPoint.push_back(SimplePoint(p->parent->x, p->parent->y));
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(pathPoint.begin(), pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
}
// 得到船去目标点的序列,Dijkstra
bool DecisionMaker::getBoatPathDijkstra(int boatID, int tarx, int tary, vector<SimplePoint> &pathPoint, vector<int> &pathDir)
{
    memset(visBoat, 0, sizeof(visBoat)); // 这里visBoat起确定最短路径集合的作用
    priority_queue<Node> candidate;
    int queueCount = 0;
    int firstDis = 0;
    Node *now = &nodes[queueCount++];
    Node *target = nullptr;
    now->setNode(boat[boatID].curX, boat[boatID].curY, 0, nullptr, boat[boatID].dire);
    candidate.push(*now);
    while (!target || !candidate.empty()) // 如果没找到目标或者优先级队列不为空
    {
        now = &nodes[queueCount++];
        *now = candidate.top(); // 取出最短的节点now
        candidate.pop();
        visBoat[now->dir][now->x][now->y] = 1; // 确定为最短路径集合
        if (now->x == tarx && now->y == tary)
        {
            target = now;
            break;
        }
        for (int i = 0; i < 3; i++) // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
        {
            int nx = now->x + dirBoatDx[i][now->dir];
            int ny = now->y + dirBoatDy[i][now->dir];

            int curDir = i == 2 ? now->dir : clockWiseDir[i][now->dir];
            if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                continue;
            now = &nodes[queueCount++]; // 这里只是为了用申请的空间
            now->setNode(nx, ny, boatTimeForDifDir[curDir][nx][ny] + now->dis, now, curDir);
            candidate.push(*now);
        }
    }
    vector<int>().swap(pathDir);           // 清空
    vector<SimplePoint>().swap(pathPoint); // 清空
    if (target != nullptr)
    {
        pathPoint.push_back(SimplePoint(target->x, target->y));
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][now->dir] && p->y == p->parent->y + dirBoatDx[i][now->dir])
                {
                    pathDir.push_back(i);
                    pathPoint.push_back(SimplePoint(p->parent->x, p->parent->y));
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(pathPoint.begin(), pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
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
