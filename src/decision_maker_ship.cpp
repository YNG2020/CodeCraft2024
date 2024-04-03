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
        Boat &bot = boat[i];
        refreshBoatState(i);

        if (bot.boatMoveState == BOAT_ARRIVEBERTH)
        {
            printf("berth %d\n", i);
            if (bot.boatStatus == 0) // 保险处理（其实不保险，要保证berth指令能成功执行）
            {
                bot.boatFlashState = BOAT_FLASHING; // 进入闪现状态
            }

            bot.boatMoveState = BOAT_WAITING; // 手动更新为原地等待的状态（等路径分配）
            bot.boatPathState = BOAT_NO_PATH;
            bot.boatTarState = BOAT_NO_TARGET;
            bot.boatStatus = 1; // 输入了指令berth，手动将boatStatus置1
            bot.idxInPth = 0;
            vector<int>().swap(bot.pathDir);         // 清空
            vector<BoatPoint>().swap(bot.pathPoint); // 清空
        }
        if (bot.boatMoveState == BOAT_ARRIVETRADE)
        {
            bot.boatMoveState = BOAT_WAITING; // 手动更新为原地等待的状态（等路径分配）
            bot.boatPathState = BOAT_NO_PATH;
            bot.boatTarState = BOAT_NO_TARGET;
            bot.numBoatGoods = 0; // 该值是系统更新的，但这里也手动更新一下
            bot.idxInPth = 0;
            vector<int>().swap(bot.pathDir);         // 清空
            vector<BoatPoint>().swap(bot.pathPoint); // 清空
        }
        int threshold;

        switch (bot.boatStatus)
        {
        case 0: // 正常行驶状态（状态 0）
            // if (bot.boatTarState == BOAT_HAVE_TARGET && bot.boatPathState == BOAT_NO_PATH)
            //{   // 有目标，无路，分配路（应该迟早会适用于避让处理）
            //     bool findPathFlag = getBoatPathBFS(i, bot.tarX, bot.tarY, bot.pathPoint, bot.pathDir);
            //     if (findPathFlag)
            //     {
            //         bot.boatPathState = BOAT_HAVE_PATH;
            //         bot.lastX = bot.curX;
            //         bot.lastY = bot.curY;
            //         bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
            //         bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
            //     }
            //     else
            //     {
            //         bot.idxInPth = 0;
            //         vector<int>().swap(bot.pathDir);     // 清空
            //         vector<BoatPoint>().swap(bot.pathPoint); // 清空
            //         bot.tarX = -1;
            //         bot.tarY = -1;
            //     }
            // }
            if (bot.boatTarState == BOAT_NO_TARGET && bot.numBoatGoods == 0) // 没货物时，找泊位
            {                                                                // 无目标，直接在路径搜索的同时分配目标（目前是强行设置泊位作为目标了）
                int tarBerthID = berthSelect(i);
                bool findPathFlag = getBoatPathDijkstra(i, berth[tarBerthID].x, berth[tarBerthID].y, bot.pathPoint, bot.pathDir);
                if (findPathFlag)
                {
                    bot.boatPathState = BOAT_HAVE_PATH;
                    bot.boatTarState = BOAT_HAVE_TARGET;
                    bot.boatMoveState = BOAT_TOBERTH;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    bot.tarBerthID = tarBerthID;
                }
                else
                {
                    bot.boatPathState = BOAT_NO_PATH;
                    bot.boatTarState = BOAT_NO_TARGET;
                    bot.boatMoveState = BOAT_WAITING;
                    bot.idxInPth = 0;
                    vector<int>().swap(bot.pathDir);         // 清空
                    vector<BoatPoint>().swap(bot.pathPoint); // 清空
                    bot.tarX = -1;
                    bot.tarY = -1;
                    bot.tarBerthID = -2;
                }
            }
            break;
        case 1: // 恢复状态（状态1）
            break;
        case 2: // 装载状态（状态 2）
            berth[bot.tarBerthID].boatIDInBerth = i;

            if (phase == 0)
                threshold = 40;
            else
                threshold = boatCapacity * 0.8;
            if (bot.numBoatGoods >= threshold)
            { // 如果装满了，去虚拟点
                bool findPathFlag = getBoatPathDijkstra(i, tradePoint[0].x, tradePoint[0].y, bot.pathPoint, bot.pathDir);
                if (findPathFlag)
                {
                    berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                    bot.boatPathState = BOAT_HAVE_PATH;
                    bot.boatTarState = BOAT_HAVE_TARGET;
                    bot.boatMoveState = BOAT_TOTRADE;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    bot.tarBerthID = -2;
                }
                else
                {
                    bot.boatPathState = BOAT_NO_PATH;
                    bot.boatTarState = BOAT_NO_TARGET;
                    bot.boatMoveState = BOAT_WAITING;
                    bot.idxInPth = 0;
                    vector<int>().swap(bot.pathDir);         // 清空
                    vector<BoatPoint>().swap(bot.pathPoint); // 清空
                    bot.tarX = -1;
                    bot.tarY = -1;
                    bot.tarBerthID = -2;
                }
                // shipGoodsNum += bot.numBoatGoods;
                // berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                // berth[bot.tarBerthID].boatIDToBerth = -1; // 更新泊位被指向的情况
            }
            else
            { // 没有满则继续装
                int loadNum = berth[bot.tarBerthID].load(bot.capacity - bot.numBoatGoods);
                bot.numBoatGoods += loadNum;
                int newBerth = bot.tarBerthID;
                // if (loadNum == 0) // 尝试比较跑去别的泊位去装货的性价比
                //     newBerth = berth_select(i, bot.tarBerthID);
                // if (newBerth != bot.tarBerthID)
                //{
                //     //berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                //     //berth[bot.tarBerthID].boatIDToBerth = -1; // 更新泊位被指向的情况
                //     //berth[newBerth].boatIDToBerth = i;        // 更新泊位被指向的情况
                // }
            }
            break;
        default:
            break;
        }
    }
    boatMoveControl();
}

void DecisionMaker::refreshBoatState(int boatID)
{
    Boat &bot = boat[boatID];
    if (((bot.curX != bot.lastX) || ((bot.curY != bot.lastY))))
    { // 发现变更了位置（旋转操作也必定变换位置）
        if (bot.boatPathState == BOAT_HAVE_PATH)
            ++bot.idxInPth;
        bot.lastX = bot.curX;
        bot.lastY = bot.curY;
    }

    if (getBerthId(bot.curX, bot.curY) == bot.tarBerthID && bot.boatMoveState == BOAT_TOBERTH && bot.boatStatus == 0)
    { // 如果抵达了目标泊位所覆盖的范围
        bot.boatMoveState = BOAT_ARRIVEBERTH;
        berth[bot.tarBerthID].boatIDInBerth = boatID;
        berth[bot.tarBerthID].boatIDToBerth = -1;
    }

    if (gridMap[bot.curX][bot.curY] == TRADE && bot.boatMoveState == BOAT_TOTRADE)
    { // 抵达交易点，且船上有货物
        bot.boatMoveState = BOAT_ARRIVETRADE;
    }

    if (bot.boatStatus != 1)
    { // 及时解除闪现状态
        bot.boatFlashState = BOAT_NO_FLASH;
    }
}

// 得到船去目标点的序列(BFS)
bool DecisionMaker::getBoatPathBFS(int boatID, int tarX, int tarY, vector<BoatPoint> &pathPoint, vector<int> &pathDir)
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
        if (now->x == tarX && now->y == tarY)
        {
            target = now;
            boat[boatID].idxInPth = 0;
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

    vector<int>().swap(pathDir);         // 清空
    vector<BoatPoint>().swap(pathPoint); // 清空
    if (target != nullptr)
    {
        pathPoint.push_back(BoatPoint(target->x, target->y, target->dir));
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][p->parent->dir] && p->y == p->parent->y + dirBoatDy[i][p->parent->dir])
                {
                    pathDir.push_back(i);
                    pathPoint.push_back(BoatPoint(p->parent->x, p->parent->y, p->parent->dir));
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(pathPoint.begin(), pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
}

// 得到船去目标点的序列(Dijkstra)
bool DecisionMaker::getBoatPathDijkstra(int boatID, int tarX, int tarY, vector<BoatPoint> &pathPoint, vector<int> &pathDir)
{
    memset(visBoat, 0, sizeof(visBoat)); // 这里visBoat起确定最短路径集合的作用
    priority_queue<Node> candidate;
    int queueCount = 0;
    int firstDis = 0;
    Node *now = &nodes[queueCount++];
    Node *target = nullptr;
    Node *child = nullptr;
    now->setNode(boat[boatID].curX, boat[boatID].curY, 0, nullptr, boat[boatID].dire);
    candidate.push(*now);
    while (!target && !candidate.empty()) // 如果没找到目标并且优先级队列不为空
    {
        now = &nodes[queueCount++];
        *now = candidate.top(); // 取出最短的节点now
        candidate.pop();
        if (visBoat[now->dir][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
            continue;
        visBoat[now->dir][now->x][now->y] = 1; // 确定为最短路径集合
        if (now->x == tarX && now->y == tarY)
        {
            target = now;
            boat[boatID].idxInPth = 0;
            break;
        }
        for (int i = 0; i < 3; i++) // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
        {
            int nx = now->x + dirBoatDx[i][now->dir];
            int ny = now->y + dirBoatDy[i][now->dir];

            int curDir = i == 2 ? now->dir : clockWiseDir[i][now->dir];
            if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                continue;
            child = &nodes[queueCount++]; // 这里只是为了用申请的空间
            child->setNode(nx, ny, boatTimeForDifDir[curDir][nx][ny] + now->dis, now, curDir);
            candidate.push(*child);
        }
    }
    if (target == nullptr) // 找不到路直接返回
        return false;
    vector<int>().swap(pathDir);         // 清空
    vector<BoatPoint>().swap(pathPoint); // 清空
    if (target != nullptr)
    {
        pathPoint.push_back(BoatPoint(target->x, target->y, target->dir));
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][p->parent->dir] && p->y == p->parent->y + dirBoatDy[i][p->parent->dir])
                {
                    pathDir.push_back(i);
                    pathPoint.push_back(BoatPoint(p->parent->x, p->parent->y, p->parent->dir));
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(pathPoint.begin(), pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
}

void DecisionMaker::boatMoveControl()
{
    boatJamControl();
    boatJamControl(); // 先这样吧（在同一帧内，前面的robot无法及时读取到后面的robot的可能已更新的堵塞检测缓冲区的信息，导致潜在的堵塞风险）
    for (int i = 0; i < boatNum; ++i)
    {
        Boat &bot = boat[i];

        if (bot.boatPathState == BOAT_NO_PATH || bot.boatAvoidState == BOAT_AVOIDED)
            continue;
        if (bot.pathDir.size() > 0)
        {
            int para = bot.pathDir[bot.idxInPth];
            if (para == 2)
                printf("ship %d\n", i);
            else
                printf("rot %d %d\n", i, para);
        }
    }
}

int DecisionMaker::berthSelect(int boatID)
{

    // 为当前的boat选择泊位ID，目的是平均到每一帧的收益最大
    int moveTimeToBerth; // 到泊位的时间
    int moveTimeToVir;   // 从虚拟点到泊位的时间
    // TODO 计算泊位间距离
    int moveTimeFromBerth = 500;                                            // 从泊位到另外一个泊位的时间
    double loadGoodsTime1;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物充足）
    double loadGoodsTime2;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物不充足）
    double loadGoodsTime;                                                   // 实际的装货时间
    int numNeedGoods = (boat[boatID].capacity - boat[boatID].numBoatGoods); // robot剩余的要装的货物的数量
    int numRemainGoods;                                                     // berth此刻剩余的货物的数目
    double numAddGoods;                                                     // 在boat驶向泊位期间，泊位增加的货物数
    // int minTime = 100000000;
    double MaxMeanGetValue = 0;                                         // 平均每帧得到最大的价值，初始化为0
    int oriLocation = getBerthId(boat[boatID].curX, boat[boatID].curY); // boat当前所在的泊位ID
    int minIdx = oriLocation;                                           // 存储将要被选择的泊位ID
    double timeToGetMoney;                                              // 到预计拿到资金的时间
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

        if (oriLocation == -1) // 说明boat是从虚拟点过来的
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
    {
        // 暂时没有更新boatIDTo/InBerth，做一个特殊处理
        if (minIdx == -1)
            return 0;
        return minIdx;
    }

    berth[minIdx].boatIDToBerth = boatID;
    printf("ship %d %d\n", boatID, minIdx);
    return minIdx;
}
