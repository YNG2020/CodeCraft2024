#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <cstring>
#include <cmath>

void DecisionMaker::robotDecision()
{
    refreshBerthState();

    for (int i = 0; i < robotNum; i++)
    {
        Robot &bot = robot[i];
        refreshRobotState(i); // 自动更新robot的状态

        if (bot.botMoveState == ARRIVEGOODS)
        {
            printf("get %d\n", i);
            ++bot.carryGoods;          // 手动更新为持有货物的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.botPathState = NO_PATH;
            bot.idxInPth = 0;
            if (goodsInMap[bot.curX][bot.curY] > 0)
            { // 说明取走的是无主货物
                goodsInMap[bot.curX][bot.curY] = 0;
                goodsInMap[bot.tarX][bot.tarY] = bot.goodsVal;
            }
            else
                goodsInMap[bot.tarX][bot.tarY] = 0;
            pick_goods_num++;
            bot.lastX = -1;
            bot.lastY = -1;
            bot.sumGoodsVal += bot.goodsVal;
        }
        else if (bot.botMoveState == ARRIVEBERTH)
        {
            printf("pull %d\n", i);

            int berthID = getBerthId(bot.curX, bot.curY);
            if (bot.carryGoods == 1)
            {
                berth[berthID].numBerthGoods++;
                berth[berthID].berthGoodsValueList.push_back(bot.goodsVal);
                berth[berthID].totGetGoodsGap += (frameId - berth[berthID].lastTimeGetGoods);
                berth[berthID].lastTimeGetGoods = frameId;
                ++berth[berthID].numGetGoods;
                berth[berthID].timeOfGoodsToBerth = berth[berthID].totGetGoodsGap / berth[berthID].numGetGoods;
                berth[berthID].totGetGoodsRatio += robot[i].curPropotion;
                berth[berthID].meanGetGoodsRatio = berth[berthID].totGetGoodsRatio / berth[berthID].numGetGoods;
            }
            else
            {
                berth[berthID].numBerthGoods += 2;
                berth[berthID].berthGoodsValueList.push_back(bot.goodsVal);
                berth[berthID].berthGoodsValueList.push_back(bot.sumGoodsVal - bot.goodsVal);
                berth[berthID].totGetGoodsGap += (frameId - berth[berthID].lastTimeGetGoods);
                berth[berthID].lastTimeGetGoods = frameId;
                berth[berthID].numGetGoods += 2;
                berth[berthID].timeOfGoodsToBerth = berth[berthID].totGetGoodsGap / berth[berthID].numGetGoods;
                berth[berthID].totGetGoodsRatio += robot[i].curPropotion;
                berth[berthID].meanGetGoodsRatio = berth[berthID].totGetGoodsRatio / berth[berthID].numGetGoods;
            }
                
            /* */
            goods_pull_frame.push_back(frameId);
            goods_pull_value.push_back(bot.goodsVal);
            goods_pull_region.push_back(berthID);
            /* */
            bot.total_goods_val += bot.sumGoodsVal;
            bot.curPropotion = -1;      
            bot.goodsVal = 0;            // 将目前所拥有的或准备拥有的货物价值清0
            bot.carryGoods = 0;          // 手动更新为不持有货物的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.botPathState = NO_PATH;
            bot.idxInPth = 0;
            bot.lastX = -1;
            bot.lastY = -1;
            bot.sumGoodsVal = 0;
        }
        else if (bot.botMoveState == TOGOODS)
        {   
            int lastBerthID = getBerthId(bot.pathPoint[0].x, bot.pathPoint[0].y);
            int curBerthID = nearBerthID[bot.tarX][bot.tarY];
            if (lastBerthID != curBerthID && bot.curPropotion < limToTryChangeGoods * bot.meanPropotion)
            {
                // 尝试放弃当前的低性价比目标，并去找高性价比目标
                int oriTarX = bot.tarX, oriTarY = bot.tarY, oriGoodsVal = bot.goodsVal;
                bool changePathFlag = getNearestGoods(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i, true, -1, -1);
                if (changePathFlag)
                {
                    goodsInMap[oriTarX][oriTarY] = oriGoodsVal;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;

                    int berthID = nearBerthID[bot.tarX][bot.tarY];
                    int goodsID = goodsIDInBerthZone[bot.tarX][bot.tarY];
                    berth[berthID].goodsInBerthInfo.erase(goodsID);

                    berthID = nearBerthID[oriTarX][oriTarY];
                    goodsID = goodsIDInBerthZone[oriTarX][oriTarY];
                    berth[berthID].goodsInBerthInfo.emplace(goodsID, singleGoodsInfo(goodsInMap[oriTarX][oriTarY], 2 * nearBerthDis[oriTarX][oriTarY], oriTarX, oriTarY));

                    refreshJamBuffer(i);
                }
            }
            continue;
        }

        int callingBerthID = -1;
        if (bot.pullBerthID != -1)
        {   // 探测最近的泊位是否有召唤需求（假设当前泊位是A，则A最近的泊位设为B，而以A作为最近泊位的泊位设为C，B和C都应被探测）
            double factor = berthCallingFactor;
            int berthIDA = bot.pullBerthID;
            int berthIDB = berth[berthIDA].nearestBerth, berthIDC = -1;
            int excessValueA = 0, excessValueB = 0, excessValueC = 0;
            double limit = berth[berthIDA].meanInZoneGoodsRatio;
            limit = 0;
            if (berthIDB != -1 && !berth[berthIDB].isBlocked)
            {   // 存在最近的泊位且泊位未被封锁
                //if (berth[berthIDB].numServingRobot == 0)
                {   // 该泊位上无robot为其服务
                    for (auto it = berth[berthIDA].goodsInBerthInfo.begin(); it != berth[berthIDA].goodsInBerthInfo.end(); ++it)
                        if (it->second.propotion >= limit)
                            excessValueA += it->second.goodsVal;
                    for (auto it = berth[berthIDB].goodsInBerthInfo.begin(); it != berth[berthIDB].goodsInBerthInfo.end(); ++it)
                        if (it->second.propotion >= limit)
                            excessValueB += it->second.goodsVal;
                    if (excessValueB / std::max(1, berth[berthIDB].numServingRobot) > 
                        factor * excessValueA / std::max(1, berth[berthIDA].numServingRobot))
                        callingBerthID = berthIDB;
                }
            }
            if (callingBerthID == -1)
            {
                for (int j = 0; j < berthNum; ++j)
                    if (berth[j].nearestBerth == berthIDA)
                        berthIDC = j;

                if (berthIDC != -1 && !berth[berthIDC].isBlocked)
                {   // 泊位未被封锁
                    //if (berth[berthIDC].numServingRobot == 0)
                    {   // 该泊位上无robot为其服务
                        for (auto it = berth[berthIDA].goodsInBerthInfo.begin(); it != berth[berthIDA].goodsInBerthInfo.end(); ++it)
                            if (it->second.propotion >= limit)
                                excessValueA += it->second.goodsVal;
                        for (auto it = berth[berthIDC].goodsInBerthInfo.begin(); it != berth[berthIDC].goodsInBerthInfo.end(); ++it)
                            if (it->second.propotion >= limit)
                                excessValueC += it->second.goodsVal;
                        if (excessValueC / std::max(1, berth[berthIDC].numServingRobot) >
                            factor * excessValueA / std::max(1, berth[berthIDA].numServingRobot))
                            callingBerthID = berthIDC;
                    }
                }
            }
        }

        // callingBerthID = -1;
        int callingGoodsID = -1;
        //if (callingBerthID == -1 && bot.pullBerthID != -1)
        //{   // 探测是否有货物是即将消失的，如果时间允许，且其价值够高，则优先去运这个即将消失的货物
        //    for (auto it = berth[bot.pullBerthID].goodsInBerthInfo.begin(); it != berth[bot.pullBerthID].goodsInBerthInfo.end(); ++it)
        //        if (it->second.doubleDis >= goodsLeftTime[it->second.x][it->second.y] 
        //            && it->second.propotion > berth[bot.pullBerthID].meanInZoneGoodsRatio)
        //        {
        //            callingGoodsID = it->second.x * MAP_SIZE + it->second.y;
        //            break;
        //        }

        //}

        if (bot.botMoveState == WAITING || bot.botPathState == NO_PATH)
        { // 没有目标，分配目标，之前没找到路，更新路（适用于中途变更路径，但失败的情况）
            if (bot.carryGoods == 0 || (robotType[i] == 1 && bot.carryGoods == 1))
            { // 未持有货物
                bool findPathFlag = getNearestGoods(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i, false, callingBerthID, callingGoodsID);
                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    bot.botMoveState = TOGOODS;
                    bot.botPathState = HAVE_PATH;

                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    refreshJamBuffer(i);

                    --numCurGoods;
                    int berthID = nearBerthID[bot.tarX][bot.tarY];
                    int goodsID = goodsIDInBerthZone[bot.tarX][bot.tarY];
                    berth[berthID].goodsInBerthInfo.erase(goodsID);
                }
                else
                { // 没找到路，也更新一些状态变量
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.idxInPth = 0;
                    bot.curPropotion = -1;
                    vector<int>().swap(bot.pathDir);     // 清空
                    vector<SimplePoint>().swap(bot.pathPoint); // 清空
                    bot.tarX = -1;
                    bot.tarY = -1;
                    refreshJamBuffer(i);
                }
            }
            else
            { // 持有货物
                bool findPathFlag = getNearestBerth(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                {
                    bot.botMoveState = TOBERTH;
                    bot.botPathState = HAVE_PATH;
                    //bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    refreshJamBuffer(i);
                }
                else
                {
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.idxInPth = 0;
                    vector<int>().swap(bot.pathDir);     // 清空
                    vector<SimplePoint>().swap(bot.pathPoint); // 清空
                    bot.tarX = -1;
                    bot.tarY = -1;
                    refreshJamBuffer(i);
                }
            }
        }
    }
    moveControl();
}

void DecisionMaker::refreshRobotState(int botID)
{
    Robot &bot = robot[botID];
    if (((bot.curX != bot.lastX) || ((bot.curY != bot.lastY))))
    { // 发现变更了位置
        ++bot.idxInPth;
        refreshJamBuffer(botID);
        bot.lastX = bot.curX;
        bot.lastY = bot.curY;
    }

    if ((/*goodsInMap[bot.curX][bot.curY] > 0 || */ goodsInMap[bot.curX][bot.curY] == -(botID + 1)) && bot.carryGoods <= 2)
    { // 自己本身没有货物，且遇上了货物（自己的目标或无主货物）
        bot.botMoveState = ARRIVEGOODS;
    }

    if (inBerth(bot.curX, bot.curY))
    { // 自己本身有货物，且抵达到泊位，不必考虑这个泊位是否就是目标泊位
        if (bot.carryGoods > 0)
        {   // 刚好抵达泊位
            bot.botMoveState = ARRIVEBERTH;
            bot.pullBerthID = getBerthId(bot.curX, bot.curY);
        }
        else if (bot.botMoveState == WAITING)   // 在泊位中等待
            bot.pullBerthID = getBerthId(bot.curX, bot.curY);
        else
            bot.pullBerthID = -1;
    }
    else
        bot.pullBerthID = -1;

    if (bot.botMoveState == TOGOODS && goodsInMap[bot.tarX][bot.tarY] == 0)
    {                                // 货物消失，及时更新自身状态
        bot.botMoveState = WAITING;
        bot.botPathState = NO_PATH;
        bot.curPropotion = -1;
        bot.goodsVal = 0; // 当前货物价值清零
    }

}

void DecisionMaker::refreshBerthState()
{
    for (int i = 0; i < robotNum; ++i)
    {
        Robot& bot = robot[i];
        int servingBerthID;
        if (bot.botMoveState == TOGOODS)
            servingBerthID = nearBerthID[bot.tarX][bot.tarY];
        else if (bot.botMoveState == TOBERTH)
            servingBerthID = getBerthId(bot.tarX, bot.tarY);
        else
            servingBerthID = -1;
        if (servingBerthID != -1)
            berth[servingBerthID].servingRobot[berth[servingBerthID].numServingRobot++] = i;
    }


    // 计算全局接收到的货物的性价比
    double tmpSum = 0.0;
    int tmpCnt = 0;
    for (int i = 0; i < berthNum; ++i)
    {
        tmpSum += berth[i].totGetGoodsRatio;
        tmpCnt += berth[i].numGetGoods;
    }
    if (tmpCnt > 0)
        globalMeanGoodsRatio = tmpSum / tmpCnt;

    // 计算各个泊位的连通泊位接收到的货物的性价比
    // for (int i = 0; i < berthNum; ++i)
    // {
    //     tmpSum = 0.0;
    //     tmpCnt = 0;
    //     for (int j = 0; j < berthNum; ++j)
    //     {
    //         if (berth[i].connectedBerth[j] && i != j)
    //         {
    //             tmpSum += berth[j].totGetGoodsRatio;
    //             tmpCnt += berth[j].numGetGoods;
    //         }
    //     }
    //     berth[i].connectedBerthMeanGoodsRatio = tmpSum / tmpCnt;
    // }

    // 计算各个泊位当前管理区上的货物的平均性价比
    for (int i = 0; i < berthNum; ++i)
    {
        tmpSum = 0.0;
        tmpCnt = berth[i].goodsInBerthInfo.size();
        for (auto it = berth[i].goodsInBerthInfo.begin(); it != berth[i].goodsInBerthInfo.end(); ++it)
        {
            tmpSum += it->second.propotion;
        }
        berth[i].meanInZoneGoodsRatio = tmpSum / tmpCnt;
    }
}

void DecisionMaker::moveControl()
{
    jamControl();
    jamControl(); // 先这样吧（在同一帧内，前面的robot无法及时读取到后面的robot的可能已更新的堵塞检测缓冲区的信息，导致潜在的堵塞风险）
    jamControl();
    for (int i = 0; i < robotNum; ++i)
    {
        Robot &bot = robot[i];

        if (bot.botPathState == NO_PATH || bot.botAvoidState == AVOIDED)
            continue;
        if (bot.pathDir.size() > 0)
        {
            // if (bot.pathDir[bot.idxInPth] < 0 || bot.pathDir[bot.idxInPth] > 3)
                // continue;
             printf("move %d %d\n", i, bot.pathDir[bot.idxInPth]);
        }
    }
}

bool DecisionMaker::getNearestGoods(int x, int y, vector<SimplePoint>& pathPoint, vector<int>& pathDir, int botID, bool tryChangePath, int callingBerthID, int callingGoodsID)
{
    int queueCount = 0;
    int queueIndex = 0;
    if (numCurGoods <= 0)
        return false;
    double propotion = 0;
    if (tryChangePath)
        propotion = robot[botID].curPropotion;
    int firstDis = 0;
    // int cnt = 0;

    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr);

    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robotNum; i++)
    {
        if (robot[i].robotStatus == 0 && (abs(robot[i].curX - x) + abs(robot[i].curY - y) < BOT_EXRECOVER_DIST))
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;
    if (robot[botID].avoidBotID != -1) // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象
        vis[robot[robot[botID].avoidBotID].curX][robot[robot[botID].avoidBotID].curY] = true;

    int botInberthID = getBerthId(x, y);
    double factor = 2.0;
    double gainForCalling = 100.0;

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

        if (goodsInMap[now->x][now->y] > 0)
        {
            if (now->dis < goodsLeftTime[now->x][now->y])
            { // 赶得及在货物消失之前把货物运走
                int goodsNearBerthID = nearBerthID[now->x][now->y];
                if (botInberthID == goodsNearBerthID)
                    if (callingGoodsID == (now->x * MAP_SIZE + now->y))
                        factor = gainForCalling;
                    else
                        factor = gainForSameBerth;
                else if (callingBerthID == goodsNearBerthID)
                    factor = gainForCalling;
                else
                    factor = 1.0;

                if (phase == 2 && (frame + now->dis + nearBerthDis[now->x][now->y] + berth[goodsNearBerthID].transportTime >= 15000))
                    factor = 0.00001;

                if (firstDis == 0)
                    // if (cnt == 0)
                { // 第一次找到货物
                    if (!tryChangePath)
                        propotion = factor * (double)goodsInMap[now->x][now->y] / (now->dis + nearBerthDis[now->x][now->y]);
                    else // 尝试变更要搬运的货物的目标
                        propotion = factor * (double)goodsInMap[now->x][now->y] / (robot[botID].idxInPth + now->dis + nearBerthDis[now->x][now->y]);
                    target = now;
                    firstDis = now->dis;
                    //++cnt;
                }
                else
                { // 尝试寻找性价比更高的货物
                    double newPropotion;
                    if (!tryChangePath)
                        newPropotion = factor * (double)goodsInMap[now->x][now->y] / (now->dis + nearBerthDis[now->x][now->y]);
                    else // 尝试变更要搬运的货物的目标
                        newPropotion = factor * (double)goodsInMap[now->x][now->y] / (robot[botID].idxInPth + now->dis + nearBerthDis[now->x][now->y]);

                    if (newPropotion > propotion)
                    {
                        propotion = newPropotion;
                        target = now;
                    }
                }
            }
        }
        if (firstDis > 0)
            // if (cnt > 0)
        {
            if (now->dis - firstDis > extraSearchTime) // 最多额外搜索extraSearchTime步
                // if (cnt > extraSearchTime) // 最多额外搜索extraSearchTime步
                break;
            //++cnt;
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }

    if (target)
    {
        int goodsNearBerthID = nearBerthID[target->x][target->y];
        if (botInberthID == goodsNearBerthID)
            propotion = propotion / gainForSameBerth;
        if (callingBerthID == goodsNearBerthID)
            propotion = propotion / gainForCalling;
        if (callingGoodsID == (target->x * MAP_SIZE + target->y))
            propotion = propotion / gainForCalling;
    }

    if (propotion <= limToChangeGoods * robot[botID].curPropotion)
        target = nullptr;

    if (target == nullptr) // 找不到路直接返回
        return false;

    robot[botID].goodsVal = goodsInMap[target->x][target->y];                           // 先存储
    robot[botID].idxInPth = 0;                                                          // 更新路径点序列
    robot[botID].curPropotion = propotion;                                              // 更新性价比
    robot[botID].sumPropotion += propotion;                                             // 更新历史性价比之和
    ++robot[botID].cntPropotion;                                                        // 更新性价比被改变的总次数
    robot[botID].meanPropotion = robot[botID].sumPropotion / robot[botID].cntPropotion; // 更新历史平均性价比
    goodsInMap[target->x][target->y] = -(botID + 1);                                    // 打上标记

    vector<int>().swap(pathDir); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 4; i++)
            {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i])
                {
                    pathDir.push_back(i);
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end()); // 反转路径，使其从起始节点开始

        pathPoint.resize(pathDir.size() + 1);
        int curX = x, curY = y;
        pathPoint[0] = SimplePoint(curX, curY);
        for (int i = 0; i < pathDir.size(); ++i)
        {
            curX += dx[pathDir[i]];
            curY += dy[pathDir[i]];
            pathPoint[i + 1] = SimplePoint(curX, curY);
        }
    }

    if (robot[botID].avoidBotID != -1)
    { // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象，货物消失也会进入这个条件
        robot[botID].botAvoidState = NO_AVOIDING;
        robot[botID].avoidBotID = -1;
    }
    return true;
}

bool DecisionMaker::getNearestBerth(int x, int y, vector<SimplePoint>& pathPoint, vector<int>& pathDir, int botID)
{
    int queueCount = 0;
    int queueIndex = 0;
    bool haveBerthFlag = false;
    for (int i = 0; i < berthNum; ++i)
        if (frameId < 10000 || (!berth[i].isBlocked))
        { // 没被封锁或泊位路径可达
            haveBerthFlag = true;
            break;
        }

    if (!haveBerthFlag)
        return false;

    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr);

    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robotNum; i++)
    {
        if (robot[i].robotStatus == 0)
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;
    if (robot[botID].avoidBotID != -1) // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象
        vis[robot[robot[botID].avoidBotID].curX][robot[robot[botID].avoidBotID].curY] = true;

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

        if (inBerth(now->x, now->y))
        {
            int berthID = getBerthId(now->x, now->y);
            if (!berth[berthID].isBlocked)
            {
                target = now;              // 找到目标
                robot[botID].idxInPth = 0; // 更新路径点序列
                break;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    vector<int>().swap(pathDir); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 4; i++)
            {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i])
                {
                    pathDir.push_back(i);
                    break;
                }
            }
        }
        reverse(pathDir.begin(), pathDir.end()); // 反转路径，使其从起始节点开始

        pathPoint.resize(pathDir.size() + 1);
        int curX = x, curY = y;
        pathPoint[0] = SimplePoint(curX, curY);
        for (int i = 0; i < pathDir.size(); ++i)
        {
            curX += dx[pathDir[i]];
            curY += dy[pathDir[i]];
            pathPoint[i + 1] = SimplePoint(curX, curY);
        }
    }

    if (robot[botID].avoidBotID != -1)
    { // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象，中途捡到货物也会进入到这里
        robot[botID].botAvoidState = NO_AVOIDING;
        robot[botID].avoidBotID = -1;
    }
    return true;
}

// robot[botID]寻找到robot[botID]的目标位置的路径
bool DecisionMaker::getToTarPath(int botID, bool calFromJam)
{
    int queueCount = 0;
    int queueIndex = 0;
    if (robot[botID].botMoveState == WAITING)
        return false;

    Robot &bot = robot[botID];
    int x = bot.curX, y = bot.curY;
    int tarX = bot.tarX, tarY = bot.tarY;

    Node *now = &nodes[queueCount++];
    Node *target = nullptr; // 用于存储找到的目标节点
    Node *child = nullptr;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));

    for (int i = 0; i < robotNum; i++)
    {
        if (robot[i].robotStatus == 0 && (abs(robot[i].curX - x) + abs(robot[i].curY - y) < BOT_EXRECOVER_DIST))
            vis[robot[i].curX][robot[i].curY] = true;
        if (calFromJam && robot[i].botAvoidState == AVOIDED)
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

        if (now->x == tarX && now->y == tarY)
        {
            target = now;     // 找到目标
            bot.idxInPth = 0; // 更新路径点序列
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    bot.pathDir.clear(); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 4; i++)
            {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i])
                {
                    bot.pathDir.push_back(i);
                    break;
                }
            }
        }
        reverse(bot.pathDir.begin(), bot.pathDir.end()); // 反转路径，使其从起始节点开始

        bot.pathPoint.resize(bot.pathDir.size() + 1);
        int curX = x, curY = y;
        bot.pathPoint[0] = SimplePoint(curX, curY);
        for (int i = 0; i < bot.pathDir.size(); ++i)
        {
            curX += dx[bot.pathDir[i]];
            curY += dy[bot.pathDir[i]];
            bot.pathPoint[i + 1] = SimplePoint(curX, curY);
        }
    }
    return true;
}
