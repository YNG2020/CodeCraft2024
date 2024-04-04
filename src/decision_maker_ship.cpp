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
            bot.boatFlashState = BOAT_FLASHING; // 进入闪现状态
            bot.boatMoveState = BOAT_WAITING; // 手动更新为原地等待的状态（等路径分配）
            bot.boatPathState = BOAT_NO_PATH;
            bot.boatTarState = BOAT_NO_TARGET;
            bot.boatStatus = 1; // 输入了指令berth，手动将boatStatus置1
            bot.idxInPth = 0;
            vector<int>().swap(bot.pathDir);         // 清空
            vector<BoatPoint>().swap(bot.pathPoint); // 清空
            boatRefreshJamBuffer(i);
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
            boatRefreshJamBuffer(i);
        }
    
        //{   // 用于在复赛练习赛的图1中进防堵塞解决失败的测试代码，正常情况下应注释
        //    if (bot.boatStatus == 2)
        //    {
        //        if (i == 0)
        //            if (boat[1].boatStatus != 2)
        //                continue;
        //        if (i == 1)
        //            if (boat[0].boatStatus != 2)
        //                continue;
        //        int berthID;
        //        if (getBerthId(bot.curX, bot.curY) == 3)
        //            berthID = 0;
        //        else
        //            berthID = 3;
        //        bool findPathFlag = getBoatPathDijkstra(i, berth[berthID].x, berth[berthID].y, bot.pathPoint, bot.pathDir);
        //        bot.boatPathState = BOAT_HAVE_PATH;
        //        bot.boatTarState = BOAT_HAVE_TARGET;
        //        bot.boatMoveState = BOAT_TOBERTH;
        //        bot.lastX = bot.curX;
        //        bot.lastY = bot.curY;
        //        bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
        //        bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
        //        berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
        //        bot.tarBerthID = berthID;
        //        boatRefreshJamBuffer(i);
        //        continue;
        //    }
        //    if (bot.boatTarState == BOAT_NO_TARGET && bot.boatStatus == 0) // 找泊位
        //    {
        //        int tarBerthID = berthSelect(i);
        //        if (i == 0)
        //            tarBerthID = 0;
        //        else
        //            tarBerthID = 3;
        //        bool findPathFlag = getBoatPathDijkstra(i, berth[tarBerthID].x, berth[tarBerthID].y, bot.pathPoint, bot.pathDir);
        //        if (findPathFlag)
        //        {
        //            bot.boatPathState = BOAT_HAVE_PATH;
        //            bot.boatTarState = BOAT_HAVE_TARGET;
        //            bot.boatMoveState = BOAT_TOBERTH;
        //            bot.lastX = bot.curX;
        //            bot.lastY = bot.curY;
        //            bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
        //            bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
        //            bot.tarBerthID = tarBerthID;
        //            boatRefreshJamBuffer(i);
        //        }
        //    }
        //}

        int threshold;
        switch (bot.boatStatus)
        {
        case 0: // 正常行驶状态（状态 0）
            if (bot.boatTarState == BOAT_HAVE_TARGET && bot.boatPathState == BOAT_NO_PATH)
            {   // 有目标，无路，分配路（适用于在前往目的地时中途闪现的情况）
                 bool findPathFlag = getBoatPathDijkstra(i, bot.tarX, bot.tarY, bot.pathPoint, bot.pathDir);
                 if (findPathFlag)
                 {
                     bot.boatPathState = BOAT_HAVE_PATH;
                     bot.lastX = bot.curX;
                     bot.lastY = bot.curY;
                     boatRefreshJamBuffer(i);
                 }
                 else
                 {
                     bot.idxInPth = 0;
                     vector<int>().swap(bot.pathDir);     // 清空
                     vector<BoatPoint>().swap(bot.pathPoint); // 清空
                     boatRefreshJamBuffer(i);
                     bot.tarX = -1;
                     bot.tarY = -1;
                 }
            }
            if (bot.boatTarState == BOAT_NO_TARGET && bot.numBoatGoods == 0) // 没货物时，找泊位
            {
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
                    berth[tarBerthID].boatIDToBerth = i;
                    boatRefreshJamBuffer(i);
                }
                else
                {
                    bot.boatPathState = BOAT_NO_PATH;
                    bot.boatTarState = BOAT_NO_TARGET;
                    bot.boatMoveState = BOAT_WAITING;
                    bot.idxInPth = 0;
                    vector<int>().swap(bot.pathDir);         // 清空
                    vector<BoatPoint>().swap(bot.pathPoint); // 清空
                    boatRefreshJamBuffer(i);
                    bot.tarX = -1;
                    bot.tarY = -1;
                    bot.tarBerthID = -2;
                }
            }
            break;
        case 1: // 恢复状态（状态1）
            break;
        case 2: // 装载状态（状态 2）
            if (phase == 0)
                threshold = 40;
            else
                threshold = boatCapacity * 0.8;
            if (bot.numBoatGoods >= threshold)
            { // 如果装满了，去虚拟点
                int nearestTrade = 0, minMoveTimeToTrade = 0x7fffffff;
                for (int i = 0; i < tradeNum; ++i)
                {
                    if (berthTradeDis[bot.tarBerthID][berthNum + i] < minMoveTimeToTrade)
                    {
                        minMoveTimeToTrade = berthTradeDis[bot.tarBerthID][berthNum + i];
                        nearestTrade = i;
                    }
                }
                    
                bool findPathFlag = getBoatPathDijkstra(i, tradePoint[nearestTrade].x, tradePoint[nearestTrade].y, bot.pathPoint, bot.pathDir);
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
                    boatRefreshJamBuffer(i);
                }
                else
                {
                    bot.boatPathState = BOAT_NO_PATH;
                    bot.boatTarState = BOAT_NO_TARGET;
                    bot.boatMoveState = BOAT_WAITING;
                    bot.idxInPth = 0;
                    vector<int>().swap(bot.pathDir);         // 清空
                    vector<BoatPoint>().swap(bot.pathPoint); // 清空
                    boatRefreshJamBuffer(i);
                    bot.tarX = -1;
                    bot.tarY = -1;
                    bot.tarBerthID = -2;
                }
                // shipGoodsNum += bot.numBoatGoods;
            }
            else
            { // 没有满则继续装
                int loadNum = berth[bot.tarBerthID].load(bot.capacity - bot.numBoatGoods);
                bot.numBoatGoods += loadNum;
                int newBerth = bot.tarBerthID;
                if (loadNum == 0) // 尝试比较跑去别的泊位去装货的性价比
                    newBerth = berthSelect(i);
                if (newBerth != bot.tarBerthID)
                {
                    bool findPathFlag = getBoatPathDijkstra(i, berth[newBerth].x, berth[newBerth].y, bot.pathPoint, bot.pathDir);
                    if (findPathFlag)
                    {
                        berth[bot.tarBerthID].boatIDInBerth = -1; // 更新泊位被占用的情况
                        bot.boatPathState = BOAT_HAVE_PATH;
                        bot.boatTarState = BOAT_HAVE_TARGET;
                        bot.boatMoveState = BOAT_TOBERTH;
                        bot.lastX = bot.curX;
                        bot.lastY = bot.curY;
                        bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                        bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                        bot.tarBerthID = newBerth;
                        berth[newBerth].boatIDToBerth = i;
                        boatRefreshJamBuffer(i);
                    }
                }
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
        { // 多一层对闪现状态的处理
            ++bot.idxInPth;
            boatRefreshJamBuffer(boatID);
        }
        bot.jamTime = 0;
        bot.lastX = bot.curX;
        bot.lastY = bot.curY;
    }
    else if (bot.setMove && bot.boatStatus == 0)
    {   // 发现没变更位置，且前一帧下达了移动指令，使用dept指令防卡死
        if (bot.jamTime >= BOAT_JAM_TOLERANCE_TIME)
        {
            if (bot.avoidBoatID >= 0)
            {   // 适用于由于两条船挤在一起导致的卡死
                bot.boatAvoidState = BOAT_NO_AVOIDING;
                bot.boatPathState = BOAT_NO_PATH;
                bot.avoidBoatID = -1;
                bot.idxInPth = 0;
                vector<int>().swap(bot.pathDir);     // 清空
                vector<BoatPoint>().swap(bot.pathPoint);    // 清空
                boatRefreshJamBuffer(boatID);           // 修改了路径，需要更新碰撞检测缓冲区
                bot.boatFlashState = BOAT_FLASHING;     // 进入闪现状态
                printf("dept %d\n", boatID);
                bot.boatStatus = 1;      // 手动置为恢复状态
            }
            else if (bot.jamTime >= 2 * BOAT_JAM_TOLERANCE_TIME)
            {   // 适用于其它原因导致的卡死
                bot.boatAvoidState = BOAT_NO_AVOIDING;
                bot.boatPathState = BOAT_NO_PATH;
                bot.avoidBoatID = -1;
                bot.idxInPth = 0;
                vector<int>().swap(bot.pathDir);     // 清空
                vector<BoatPoint>().swap(bot.pathPoint);    // 清空
                boatRefreshJamBuffer(boatID);           // 修改了路径，需要更新碰撞检测缓冲区
                bot.boatFlashState = BOAT_FLASHING;     // 进入闪现状态
                printf("dept %d\n", boatID);
                bot.boatStatus = 1;      // 手动置为恢复状态
            }
            else
                ++bot.jamTime;
        }
        else
            ++bot.jamTime;
    }
    bot.setMove = false;

    if (getBerthId(bot.curX, bot.curY) == bot.tarBerthID && bot.boatMoveState == BOAT_TOBERTH && bot.boatStatus == 0 && berth[bot.tarBerthID].boatIDInBerth == -1)
    {   // 如果抵达了目标泊位所覆盖的范围，且当前处于移动状态，且泊位未被占据，确保该指令能被执行
        bot.boatMoveState = BOAT_ARRIVEBERTH;
        berth[bot.tarBerthID].boatIDInBerth = boatID;
        berth[bot.tarBerthID].boatIDToBerth = -1;
    }

    if (gridMap[bot.curX][bot.curY] == TRADE && bot.boatMoveState == BOAT_TOTRADE)
    { // 抵达交易点
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
            child->setNode(nx, ny, boatTimeForDifDir[now->dir][now->x][now->y] + now->dis, now, curDir);
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
            if (bot.idxInPth < bot.pathDir.size() && bot.boatStatus != 1)
            {
                int para = bot.pathDir[bot.idxInPth];
                if (para == 2)
                    printf("ship %d\n", i);
                else
                    printf("rot %d %d\n", i, para);
                bot.setMove = true;
            }
        }
    }
}

int DecisionMaker::berthSelect(int boatID)
{
    // 为当前的boat选择泊位ID，目的是平均到每一帧的收益最大
    int moveTimeToBerth; // 到泊位的时间
    int moveTimeToTrade = 0x7fffffff;           // 某个泊位到其最近的交货点的时间
    double loadGoodsTime1;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物充足）
    double loadGoodsTime2;                                                  // 预计的装载货物的时间（装满的情况，假设泊位上的货物不充足）
    double loadGoodsTime;                                                   // 实际的装货时间
    int numNeedGoods = (boat[boatID].capacity - boat[boatID].numBoatGoods); // robot剩余的要装的货物的数量
    int numRemainGoods;                                                     // berth此刻剩余的货物的数目
    double numAddGoods;                                                     // 在boat驶向泊位期间，泊位增加的货物数
    // int minTime = 100000000;
    double MaxMeanGetValue = 0;                                         // 平均每帧得到最大的价值，初始化为0
    int oriLocation = getBerthId(boat[boatID].curX, boat[boatID].curY); // boat当前所在的泊位ID
    if (oriLocation < 0)
    {
        if (gridMap[boat[boatID].curX][boat[boatID].curY] == TRADE)
            oriLocation = -1;   // 代表交货点
        else if (gridMap[boat[boatID].curX][boat[boatID].curY] == BOAT_SHOP)
            oriLocation = -2;   // 代表船出生点
        else
            oriLocation = -3;   // 代表其它点
    }
    int minIdx = 0;                             // 存储将要被选择的泊位ID
    int timeToGetMoney;                         // 到预计拿到资金的时间
    int Money;

    for (int berthID = 0; berthID < berthNum; ++berthID)
    {
        for (int i = 0; i < tradeNum; ++i)
            moveTimeToTrade = std::min(berthTradeDis[berthID][berthNum + i], moveTimeToTrade);

        moveTimeToBerth = berthDis[berthID][boat[boatID].curX][boat[boatID].curY];
        timeToGetMoney = moveTimeToBerth + moveTimeToTrade;
        if (frameId + timeToGetMoney >= 15000)
        { // 这时候一定不选择该泊位，因为时间上来不及再去交货点
            continue;
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
        timeToGetMoney += loadGoodsTime;
   
        // 计算可获得的价值
        Money = berth[berthID].getBerthGoodsValueOfNum(numNeedGoods, 0, meanGoodsValue) + 1; // +1是为了Money为0时，仍能在时间上进行比较（适用于通过泊位间的转移来再去海上的情况）
        double MeanGetValue = (double)Money / (timeToGetMoney == 0 ? 1 : timeToGetMoney);
        if ((MeanGetValue > MaxMeanGetValue) && ((berth[berthID].boatIDToBerth == -1/* && berth[berthID].boatIDInBerth == -1*/) || berthID == oriLocation)) // 如果泊位有空位，不考虑泊位当前是否被占据
        {
            MaxMeanGetValue = MeanGetValue;
            minIdx = berthID;
        }
    }
    return minIdx;
}
