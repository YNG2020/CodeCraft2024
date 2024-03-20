#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <cstring>
#include <cmath>

bool DecisionMaker::getNearestGoods(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID)
{
    queue<Node *> q;
    vector<Node *> rest;
    double propotion = 0;
    int cnt = 0;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].robotStatus == 0)
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;
    if (robot[botID].avoidBotID != -1) // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象
        vis[robot[robot[botID].avoidBotID].curX][robot[robot[botID].avoidBotID].curY] = true;

    Node *target = nullptr; // 用于存储找到的目标节点

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

        if (goodsInMap[now->x][now->y] == -(botID + 1))
        {
            target = now; // 找到目标或找到自身目标
            break;
        }
        if (goodsInMap[now->x][now->y] > 0)
        {
            if (now->dis < goodsLeftTime[now->x][now->y])
            { // 赶得及在货物消失之前把货物运走
                if (cnt == 0)
                { // 第一次找到货物

                    propotion = pow((double)goodsInMap[now->x][now->y], 2) / (now->dis + nearBerthDis[now->x][now->y]);
                    target = now;
                    cnt++;
                }
                else
                { // 尝试寻找性价比更高的货物
                    double newPropotion = pow((double)goodsInMap[now->x][now->y], 2) / (now->dis + nearBerthDis[now->x][now->y]);
                    if (newPropotion > propotion)
                    {
                        propotion = newPropotion;
                        target = now;
                    }
                }
            }
        }
        if (cnt > 0)
        {
            cnt++;
            if (cnt == 1000)
            { // 最多额外搜索1000轮
                break;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now, now->dis + 1)); // 使用父节点指针
        }
        rest.push_back(now);
    }

    if (target == nullptr) // 找不到路直接返回
        return false;
    robot[botID].goodsVal = goodsInMap[target->x][target->y]; // 先存储
    robot[botID].idxInPth = 0;                                // 更新路径点序列
    goodsInMap[target->x][target->y] = -(botID + 1);          // 打上标记

    vector<int>().swap(pathDir); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
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
        pathPoint[0] = Point(curX, curY);
        for (int i = 0; i < pathDir.size(); ++i)
        {
            curX += dx[pathDir[i]];
            curY += dy[pathDir[i]];
            pathPoint[i + 1] = Point(curX, curY);
        }
    }
    while (!rest.empty())
    {
        delete rest.back();
        rest.pop_back();
    }
    while (!q.empty())
    {
        delete q.front();
        q.pop();
    }
    if (robot[botID].avoidBotID != -1)
    { // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象，货物消失也会进入这个条件
        robot[botID].botAvoidState = NO_AVOIDING;
        robot[botID].avoidBotID = -1;
    }
    return true;
}

bool DecisionMaker::getNearestBerth(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID)
{
    bool haveBerthFlag = false;
    for (int i = 0; i < berth_num; ++i)
        if (frame_id < 10000 || (!berth[i].isBlcoked && robot[botID].availableBerth[i]))
        { // 没被封锁或泊位路径可达
            haveBerthFlag = true;
            break;
        }

    if (!haveBerthFlag)
        return false;

    if (!robot[botID].findToBerthFlag)
        return false;
    queue<Node *> q;
    vector<Node *> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].robotStatus == 0)
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;
    if (robot[botID].avoidBotID != -1) // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象
        vis[robot[robot[botID].avoidBotID].curX][robot[robot[botID].avoidBotID].curY] = true;

    Node *target = nullptr; // 用于存储找到的目标节点

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

        if (inBerth(now->x, now->y))
        {
            int berthID = getBerthId(now->x, now->y);
            robot[botID].availableBerth[berthID] = true;
            if (!berth[berthID].isBlcoked)
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
            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now)); // 使用父节点指针
        }
        rest.push_back(now);
    }

    if (target == nullptr) // 找不到路直接返回
    {
        robot[botID].findToBerthFlag = false;
        return false;
    }

    vector<int>().swap(pathDir); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
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
        pathPoint[0] = Point(curX, curY);
        for (int i = 0; i < pathDir.size(); ++i)
        {
            curX += dx[pathDir[i]];
            curY += dy[pathDir[i]];
            pathPoint[i + 1] = Point(curX, curY);
        }
    }
    while (!rest.empty())
    {
        delete rest.back();
        rest.pop_back();
    }
    while (!q.empty())
    {
        delete q.front();
        q.pop();
    }
    if (robot[botID].avoidBotID != -1)
    { // 避让状态中找不到路，找新路时则避免路过曾经要避让但避让失败的对象，中途捡到货物也会进入到这里
        robot[botID].botAvoidState = NO_AVOIDING;
        robot[botID].avoidBotID = -1;
    }
    return true;
}

void DecisionMaker::robotDecision()
{
    for (int i = 0; i < berth_num; ++i)
        if (frame_id + 2 * 500 + berth[i].transportTime >= 15000 && (berth[i].boatIDToBerth == -1 && berth[i].boatIDInBerth == -1))
            berth[i].isBlcoked = true;
        else
            berth[i].isBlcoked = false;

    for (int i = 0; i < robot_num; i++)
    {

        Robot &bot = robot[i];
        refreshRobotState(i); // 自动更新robot的状态

        if (bot.botMoveState == ARRIVEGOODS)
        {
            cout << "get " << i << endl;
            bot.carryGoods = 1;          // 手动更新为持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.botPathState = NO_PATH;
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
        }
        if (bot.botMoveState == ARRIVEBERTH)
        {
            cout << "pull " << i << endl;
            berth[getBerthId(bot.curX, bot.curY)].numBerthGoods++;
            berth[getBerthId(bot.curX, bot.curY)].berthGoodsValueList.push(bot.goodsVal);
            berth[getBerthId(bot.curX, bot.curY)].totGetGoodsGap += (frame_id - berth[getBerthId(bot.curX, bot.curY)].lastTimeGetGoods);
            berth[getBerthId(bot.curX, bot.curY)].lastTimeGetGoods = frame_id;
            berth[getBerthId(bot.curX, bot.curY)].numGetGoods += 1;
            berth[getBerthId(bot.curX, bot.curY)].timeOfGoodsToBerth = 1 * berth[getBerthId(bot.curX, bot.curY)].totGetGoodsGap / berth[getBerthId(bot.curX, bot.curY)].numGetGoods + 0 * (frame_id - berth[getBerthId(bot.curX, bot.curY)].lastTimeGetGoods);
            bot.goodsVal = 0;            // 将目前所拥有的或准备拥有的货物价值清0
            bot.carryGoods = 0;          // 手动更新为不持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.botPathState = NO_PATH;
            bot.lastX = -1;
            bot.lastY = -1;
        }

        if (bot.botTarState == NO_TARGET || bot.botPathState == NO_PATH)
        { // 没有目标，分配目标（目前通过寻路在分配目标，需要改进），之前没找到路，更新路（适用于中途变更路径，但失败的情况）
            if (bot.carryGoods == 0)
            { // 未持有货物
                bool findPathFlag = getNearestGoods(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    bot.botMoveState = TOGOODS;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    refreshJamBuffer(i);
                }
                else
                { // 没找到路，也更新一些状态变量
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.botTarState = NO_TARGET;
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
                    bot.botTarState = HAVE_TARGET;
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
                    bot.botTarState = NO_TARGET;
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

    if ((/*goodsInMap[bot.curX][bot.curY] > 0 || */ goodsInMap[bot.curX][bot.curY] == -(botID + 1)) && bot.carryGoods == 0)
    { // 自己本身没有货物，且遇上了货物（自己的目标或无主货物）
        bot.botMoveState = ARRIVEGOODS;
    }
    if (inBerth(bot.curX, bot.curY) && bot.carryGoods > 0)
    { // 自己本身有货物，且抵达到泊位，不必考虑这个货物是否就是目标泊位
        bot.botMoveState = ARRIVEBERTH;
    }
    if (bot.botMoveState == TOGOODS && goodsInMap[bot.tarX][bot.tarY] == 0)
    {                                // 货物消失，及时更新自身状态
        bot.carryGoods = 0;          // 手动更新为不持有货物的状态
        bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
        bot.botMoveState = WAITING;
    }
}

void DecisionMaker::moveControl()
{
    jamControl();
    jamControl(); // 先这样吧（在同一帧内，前面的robot无法及时读取到后面的robot的可能已更新的堵塞检测缓冲区的信息，导致潜在的堵塞风险）
    for (int i = 0; i < robot_num; ++i)
    {
        Robot &bot = robot[i];

        if (bot.botPathState == NO_PATH || bot.botAvoidState == AVOIDED)
            continue;
        if (bot.pathDir.size() > 0)
        {
            if (bot.pathDir[bot.idxInPth] < 0 || bot.pathDir[bot.idxInPth] > 3)
                continue;
            cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
        }
    }
}

// 更新robot的避让优先级
void DecisionMaker::setPriority()
{
    // for (int i = 0; i < robot_num; ++i) {
    //     priorityFactor[i][0] = i;   // 第一列放编号
    //     priorityFactor[i][1] = robot[i].goodsVal;   // 第二列放货物的价值
    // }
    for (int i = 0; i < 10; ++i)
    {
        // robot[i].avoidPriority = i;
        if (robot[i].carryGoods == 0)
            robot[i].avoidPriority = i;
        else
            robot[i].avoidPriority = 10 + robot[i].goodsVal;
    }
    // 使用 lambda 表达式定义比较函数并对索引进行排序
    // std::sort(priority.begin(), priority.end(), [&](int a, int b) {
    //    return priorityFactor[a][0] != priorityFactor[b][0] ? priorityFactor[a][0] < priorityFactor[b][0] : priorityFactor[a][1] < priorityFactor[b][1];
    //});
}

// 更新堵塞检测缓冲区
void DecisionMaker::refreshJamBuffer(int botID)
{
    Robot &bot = robot[botID];
    int i = 0;
    if (bot.botPathState == HAVE_PATH)
    { // 有路，可以正常更新缓冲区
        for (; i < bot.jamDetectBufferLen && (bot.idxInPth + i) < bot.pathPoint.size(); ++i)
        {
            bot.jamDetectBuffer[i] = bot.pathPoint[bot.idxInPth + i].x * mapSize + bot.pathPoint[bot.idxInPth + i].y;
        }
        for (; i < bot.jamDetectBufferLen; ++i)
        {
            bot.jamDetectBuffer[i] = bot.jamDetectBuffer[i - 1];
        }
    }
    else
    { // 没路，缓冲区全都存储为自己本身所在的位置
        for (; i < bot.jamDetectBufferLen; ++i)
        {
            bot.jamDetectBuffer[i] = bot.curX * mapSize + bot.curY;
        }
    }
}

// 对每对robot进行堵塞检测
bool DecisionMaker::jamDetect(int botID1, int botID2)
{
    if (botID1 == botID2)
        return false;
    if ((robot[botID1].robotStatus == 0 || robot[botID1].botAvoidState == AVOIDED) && (robot[botID2].robotStatus == 0 || robot[botID2].botAvoidState == AVOIDED)) // 此刻双方都不动
        return false;
    if (robot[botID2].robotStatus == 0 || robot[botID2].botPathState == NO_PATH || robot[botID2].botAvoidState == AVOIDED) // // 此刻robto[botID2]停止不动
        if (robot[botID1].jamDetectBuffer[1] == (robot[botID2].curX * mapSize + robot[botID2].curY))
            return true;
        else
            return false;
    if (robot[botID1].robotStatus == 0 || robot[botID1].botPathState == NO_PATH || robot[botID1].botAvoidState == AVOIDED) // // 此刻robto[botID1]停止不动
        if (robot[botID2].jamDetectBuffer[1] == (robot[botID1].curX * mapSize + robot[botID1].curY))
            return true;
        else
            return false;

    bool jamFlag = false; // 标识可能发生的堵塞情况
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {
        if (robot[botID2].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i])
        {
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tarX * mapSize + robot[botID2].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tmpTarX * mapSize + robot[botID2].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID1].jamDetectBuffer[i])
        {
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tarX * mapSize + robot[botID1].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tmpTarX * mapSize + robot[botID1].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }

        // 只需逐对检测是否有可能发生冲突即可
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i + 1]) // 二者下一个目标位置都相同
            jamFlag = true;
        else if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i] && robot[botID1].jamDetectBuffer[i] == robot[botID2].jamDetectBuffer[i + 1]) // 对撞
            jamFlag = true;
    }
    return jamFlag;
}

// 对robot1和robot2进行是否可以接触避让状态的检测，默认此刻botID1正在避让botID2
bool DecisionMaker::unJamDetect(int botID1, int botID2)
{
    bool jamFlag = false;                                                     // 标识可能发生的堵塞情况
    if (robot[botID2].jamDetectBuffer[1] == robot[botID1].jamDetectBuffer[0]) // bto2撞到robot[botID1]停下的位置上
        jamFlag = true;
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {
        if (robot[botID2].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i])
        {
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tarX * mapSize + robot[botID2].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tmpTarX * mapSize + robot[botID2].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID1].jamDetectBuffer[i])
        {
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tarX * mapSize + robot[botID1].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tmpTarX * mapSize + robot[botID1].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }
        // 只需逐对检测是否有可能发生冲突即可
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i + 1]) // 二者下一个目标位置都相同
            jamFlag = true;
        else if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i] && robot[botID1].jamDetectBuffer[i] == robot[botID2].jamDetectBuffer[i + 1]) // 对撞
            jamFlag = true;
    }
    return jamFlag;
}

// 堵塞控制
void DecisionMaker::jamControl()
{
    unJam();
    setPriority(); // 计算每一个机器人的移动优先级
    for (int i = 0; i < robot_num; ++i)
    {
        for (int j = i + 1; j < robot_num; ++j)
        {
            if (jamDetect(i, j))
            {                               // 预计会发生碰撞，robot_j去寻找避让点（将考虑robot_i的堵塞检测缓冲区），robot_i则保持原来的轨迹
                int botID1 = i, botID2 = j; // 默认botID1的优先级高于botID2
                if (robot[botID1].avoidPriority < robot[botID2].avoidPriority)
                { // 如果不是默认情况，人为调转序号
                    int tmp = botID1;
                    botID1 = botID2;
                    botID2 = tmp;
                }
                if (robot[i].robotStatus == 0 && robot[j].robotStatus == 0)
                    continue;
                else if (robot[i].robotStatus == 0)
                {
                    botID1 = i;
                    botID2 = j;
                }
                else if (robot[j].robotStatus == 0)
                {
                    botID1 = j;
                    botID2 = i;
                }
                if (robot[botID2].avoidBotID == -1 || robot[botID2].avoidBotID == botID1)
                { // botID2没有避让对象，或避让对象就是botID1
                    jamResolve(botID1, botID2);
                }
                else
                { // botID2有避让对象
                    jamResolve(botID2, botID1);
                }
            }
        }
    }
}

// 堵塞消解，默认是botID2去寻找避让botID1的路径，且默认botID2没有避让对象
void DecisionMaker::jamResolve(int botID1, int botID2)
{
    bool findPathFlag;
    if (robot[botID1].botMoveState == AVOIDED)
    {                                        // 此时botID2不可能也出于AVOIDED的状态，因为此时jamDetect将返回false的结果
        findPathFlag = getToTarPath(botID2); // botID2直接不进行避让动作，而是找路去既定目标
        if (findPathFlag)
        {
            robot[botID2].avoidBotID = -1;
            robot[botID2].botAvoidState = NO_AVOIDING;
            robot[botID2].botPathState = HAVE_PATH;
            refreshJamBuffer(botID2); // 修改了路径，需要更新碰撞检测缓冲区
        }
    }

    findPathFlag = getAvoidPath(botID1, botID2);
    if (findPathFlag)
    { // 成功找到路，及时更新状态变量
        robot[botID2].avoidBotID = botID1;
        robot[botID2].botAvoidState = AVOIDING;
        robot[botID2].botPathState = HAVE_PATH;
        robot[botID2].tmpTarX = robot[botID2].pathPoint[robot[botID2].pathPoint.size() - 1].x;
        robot[botID2].tmpTarY = robot[botID2].pathPoint[robot[botID2].pathPoint.size() - 1].y;
        refreshJamBuffer(botID2); // 修改了路径，需要更新碰撞检测缓冲区
    }
    else
    { // 调转优先级，再寻一次路
        findPathFlag = getAvoidPath(botID2, botID1);
        if (findPathFlag)
        {
            robot[botID1].avoidBotID = botID2;
            robot[botID1].botAvoidState = AVOIDING;
            robot[botID1].botPathState = HAVE_PATH;
            robot[botID1].tmpTarX = robot[botID1].pathPoint[robot[botID1].pathPoint.size() - 1].x;
            robot[botID1].tmpTarY = robot[botID1].pathPoint[robot[botID1].pathPoint.size() - 1].y;
            refreshJamBuffer(botID1); // 修改了路径，需要更新碰撞检测缓冲区
        }
        else
        { // 还是找不到路，则在当前帧不动，且放弃当前的目标货物（如果有），在下一帧中寻找新的目标，直到能找到为止
            robot[botID1].botAvoidState = NO_AVOIDING;
            robot[botID1].botPathState = NO_PATH;
            robot[botID1].botTarState = NO_TARGET;
            if (robot[botID1].botMoveState == TOGOODS)
                goodsInMap[robot[botID1].tarX][robot[botID1].tarY] = robot[botID1].goodsVal;
            robot[botID1].idxInPth = 0;
            refreshJamBuffer(botID1); // 修改了路径，需要更新碰撞检测缓冲区
            // 这个变量留给寻找新路的时候用
            // robot[botID1].avoidBotID = -1;

            robot[botID2].botAvoidState = NO_AVOIDING;
            robot[botID2].botPathState = NO_PATH;
            robot[botID2].botTarState = NO_TARGET;
            if (robot[botID2].botMoveState == TOGOODS)
                goodsInMap[robot[botID2].tarX][robot[botID2].tarY] = robot[botID2].goodsVal;
            robot[botID2].idxInPth = 0;
            refreshJamBuffer(botID2); // 修改了路径，需要更新碰撞检测缓冲区
            // 这个变量留给寻找新路的时候用
            // robot[botID2].avoidBotID = -1;
        }
    }
}

// 寻找避让路径，默认是botID2去寻找避让botID1的路径
bool DecisionMaker::getAvoidPath(int botID1, int botID2)
{
    if (robot[botID2].robotStatus == 0)
        return false;
    int x = robot[botID2].curX, y = robot[botID2].curY;
    queue<Node *> q;
    vector<Node *> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].robotStatus == 0 && (abs(robot[i].curX - x) + abs(robot[i].curY - y) < distExRecoverBot))
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;
    vis[robot[botID1].curX][robot[botID1].curY] = true; // 不经过要避让的robot此刻所在的位置
    int tmpX, tmpY;
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {                                                                                 // 构建寻路屏障，不让避让路径与botID1的路径有冲突
        if (robot[botID2].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i]) // robot[botID2]将在此处停下，这一点仍能被路径搜索（也就是让它动起来）
            continue;
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i + 1])
        { // 二者下一个目标位置都相同
            tmpX = robot[botID2].jamDetectBuffer[i + 1] / mapSize;
            tmpY = robot[botID2].jamDetectBuffer[i + 1] % mapSize;
            vis[tmpX][tmpY] = true;
        }
        else if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i] && robot[botID1].jamDetectBuffer[i] == robot[botID2].jamDetectBuffer[i + 1])
        { // 对撞
            tmpX = robot[botID1].jamDetectBuffer[i] / mapSize;
            tmpY = robot[botID1].jamDetectBuffer[i] % mapSize;
            vis[tmpX][tmpY] = true;
        }
    }

    Node *target = nullptr;     // 用于存储找到的目标节点
    bool pointAvailable = true; // 用于标识找到的避让点是否可行

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

        pointAvailable = true;
        for (int i = robot[botID1].idxInPth; i < robot[botID1].pathPoint.size(); ++i)
        { // 检查在robot[botID1]接下来的全体路径点
            if (now->x == robot[botID1].pathPoint[i].x && now->y == robot[botID1].pathPoint[i].y)
            {
                pointAvailable = false;
                break;
            }
        }

        if (pointAvailable)
            for (int i = 0; i < robot_num; ++i) // 检查该避让点是否已经被别人所占据
                if (robot[i].botAvoidState != NO_AVOIDING)
                    if (now->x == robot[i].tmpTarX && now->y == robot[i].tmpTarY)
                    {
                        pointAvailable = false;
                        break;
                    }

        if (pointAvailable)
        {
            for (int i = 0; i < robot_num; ++i)
            { // 检查该避让点是否已经被别人所占据
                if (robot[i].botAvoidState != NO_AVOIDING)
                {
                    if (now->x == robot[i].tmpTarX && now->y == robot[i].tmpTarY)
                    {
                        pointAvailable = true;
                    }
                }
            }
            target = now;               // 找到避让点
            robot[botID2].idxInPth = 0; // 更新路径点序列
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now)); // 使用父节点指针
        }
        rest.push_back(now);
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    robot[botID2].pathDir.clear(); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node *p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 4; i++)
            {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i])
                {
                    robot[botID2].pathDir.push_back(i);
                    break;
                }
            }
        }
        reverse(robot[botID2].pathDir.begin(), robot[botID2].pathDir.end()); // 反转路径，使其从起始节点开始

        robot[botID2].pathPoint.resize(robot[botID2].pathDir.size() + 1);
        int curX = x, curY = y;
        robot[botID2].pathPoint[0] = Point(curX, curY);
        for (int i = 0; i < robot[botID2].pathDir.size(); ++i)
        {
            curX += dx[robot[botID2].pathDir[i]];
            curY += dy[robot[botID2].pathDir[i]];
            robot[botID2].pathPoint[i + 1] = Point(curX, curY);
        }
    }
    while (!rest.empty())
    {
        delete rest.back();
        rest.pop_back();
    }
    while (!q.empty())
    {
        delete q.front();
        q.pop();
    }
    return true;
}

// 检测是否可以解除堵塞状态
void DecisionMaker::unJam()
{
    for (int i = 0; i < robot_num; ++i)
    {
        if (robot[i].botAvoidState == AVOIDING)
        { // 正处于避让状态
            if (robot[i].curX == robot[i].tmpTarX && robot[i].curY == robot[i].tmpTarY)
            {
                robot[i].botAvoidState = AVOIDED;
                bool findPathFlag = getToTarPath(i);
                if (!findPathFlag)
                { // 应该只用处理找不到的情况，找到路的话，状态变量似乎没有什么需要特地更新的
                    // 还是找不到路，则在当前帧不动，且放弃当前的目标货物（如果有），在下一帧中寻找新的目标，直到能找到为止
                    robot[i].botPathState = NO_PATH;
                    robot[i].botTarState = NO_TARGET;
                    robot[i].botMoveState = WAITING;
                    if (robot[i].botMoveState == TOGOODS)
                        goodsInMap[robot[i].tarX][robot[i].tarY] = robot[i].goodsVal;
                    robot[i].idxInPth = 0;
                }
                refreshJamBuffer(i); // 修改了路径，需要更新碰撞检测缓冲区
            }
            else
                continue;
        }
        if (robot[i].botAvoidState == AVOIDED)
        { // 正处于避让结束，原地等待的状态
            if (!unJamDetect(i, robot[i].avoidBotID))
            {
                robot[i].botAvoidState = NO_AVOIDING;
                robot[i].avoidBotID = -1;
            }
        }
    }
}

// robot[botID]寻找到robot[botID]的目标位置的路径
bool DecisionMaker::getToTarPath(int botID)
{
    queue<Node *> q;
    vector<Node *> rest;
    Robot &bot = robot[botID];
    int x = bot.curX, y = bot.curY;
    int tarX = bot.tarX, tarY = bot.tarY;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].robotStatus == 0 && (abs(robot[i].curX - x) + abs(robot[i].curY - y) < distExRecoverBot))
            vis[robot[i].curX][robot[i].curY] = true;
        if (robot[i].botAvoidState == AVOIDED)
            vis[robot[i].curX][robot[i].curY] = true;
    }
    vis[x][y] = true;

    Node *target = nullptr; // 用于存储找到的目标节点

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

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
            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now)); // 使用父节点指针
        }
        rest.push_back(now);
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
        bot.pathPoint[0] = Point(curX, curY);
        for (int i = 0; i < bot.pathDir.size(); ++i)
        {
            curX += dx[bot.pathDir[i]];
            curY += dy[bot.pathDir[i]];
            bot.pathPoint[i + 1] = Point(curX, curY);
        }
    }
    while (!rest.empty())
    {
        delete rest.back();
        rest.pop_back();
    }
    while (!q.empty())
    {
        delete q.front();
        q.pop();
    }
    return true;
}
