#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <cstring>
#include <cmath>

// 更新robot的避让优先级
void DecisionMaker::setPriority()
{
    // for (int i = 0; i < ROBOT_NUM; ++i) {
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
    Robot& bot = robot[botID];
    int i = 0;
    if (bot.botPathState == HAVE_PATH)
    { // 有路，可以正常更新缓冲区
        for (; i < bot.jamDetectBufferLen && (bot.idxInPth + i) < bot.pathPoint.size(); ++i)
        {
            bot.jamDetectBuffer[i] = bot.pathPoint[bot.idxInPth + i].x * MAP_SIZE + bot.pathPoint[bot.idxInPth + i].y;
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
            bot.jamDetectBuffer[i] = bot.curX * MAP_SIZE + bot.curY;
        }
    }
}

// 对每对robot进行堵塞检测
bool DecisionMaker::jamDetect(int botID1, int botID2)
{
    if (botID1 == botID2)
        return false;
    if ((robot[botID1].robotStatus == 0 || robot[botID1].botPathState == NO_PATH || robot[botID1].botAvoidState == AVOIDED) &&
        (robot[botID2].robotStatus == 0 || robot[botID2].botPathState == NO_PATH || robot[botID2].botAvoidState == AVOIDED)) // 此刻双方都不动
        return false;
    if (robot[botID2].robotStatus == 0 || robot[botID2].botPathState == NO_PATH || robot[botID2].botAvoidState == AVOIDED) // // 此刻robto[botID2]停止不动
        if (robot[botID1].jamDetectBuffer[1] == (robot[botID2].curX * MAP_SIZE + robot[botID2].curY))
            return true;
        else
            return false;
    if (robot[botID1].robotStatus == 0 || robot[botID1].botPathState == NO_PATH || robot[botID1].botAvoidState == AVOIDED) // // 此刻robto[botID1]停止不动
        if (robot[botID2].jamDetectBuffer[1] == (robot[botID1].curX * MAP_SIZE + robot[botID1].curY))
            return true;
        else
            return false;

    bool jamFlag = false; // 标识可能发生的堵塞情况
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {
        if (robot[botID2].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i])
        {
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tarX * MAP_SIZE + robot[botID2].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tmpTarX * MAP_SIZE + robot[botID2].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID1].jamDetectBuffer[i])
        {
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tarX * MAP_SIZE + robot[botID1].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tmpTarX * MAP_SIZE + robot[botID1].tmpTarY))
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

// 对robot1和robot2进行是否可以解除避让状态的检测，默认此刻botID1正在避让botID2，即botID1是Avoided状态，对robot可能出现的停止状态，交给jamdetect判断
bool DecisionMaker::unJamDetect(int botID1, int botID2)
{
    bool jamFlag = false; // 标识可能发生的堵塞情况
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {
        if (robot[botID2].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i])
        {
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tarX * MAP_SIZE + robot[botID2].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID2].jamDetectBuffer[i + 1] == (robot[botID2].tmpTarX * MAP_SIZE + robot[botID2].tmpTarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
        }
        if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID1].jamDetectBuffer[i])
        {
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tarX * MAP_SIZE + robot[botID1].tarY))
                break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
            if (robot[botID1].jamDetectBuffer[i + 1] == (robot[botID1].tmpTarX * MAP_SIZE + robot[botID1].tmpTarY))
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
    for (int i = 0; i < ROBOT_NUM; ++i)
    {
        for (int j = i + 1; j < ROBOT_NUM; ++j)
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
                if (robot[botID1].botPathState == NO_PATH)
                { // 不可能二者同为该状态，否则在jamDetect的时候就pass掉，此时让没路的避让有路的
                    int tmp = botID1;
                    botID1 = botID2;
                    botID2 = tmp;
                }
                if (robot[i].robotStatus == 0 && robot[j].robotStatus == 0)
                    continue;
                else if (robot[i].robotStatus == 0)
                { // 意外情况
                    botID1 = i;
                    botID2 = j;
                }
                else if (robot[j].robotStatus == 0)
                { // 意外情况
                    botID1 = j;
                    botID2 = i;
                }
                if (robot[botID2].avoidBotID == -1 || robot[botID2].avoidBotID == botID1)
                { // botID2没有避让对象，或避让对象就是botID1
                    jamResolve(botID1, botID2);
                }
                else
                { // botID2有避让对象
                    if (robot[botID1].avoidBotID == -1)
                        jamResolve(botID2, botID1);
                    else if (robot[botID1].avoidPriority > robot[robot[botID2].avoidBotID].avoidPriority)
                        jamResolve(botID1, botID2);
                    else
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
    {                                        // 此时botID2不可能也处于AVOIDED的状态，因为此时jamDetect将返回false的结果
        findPathFlag = getToTarPath(botID2, true); // botID2直接不进行避让动作，而是找路去既定目标
        if (findPathFlag)
        {
            robot[botID2].avoidBotID = -1;
            robot[botID2].botAvoidState = NO_AVOIDING;
            robot[botID2].botPathState = HAVE_PATH;
            refreshJamBuffer(botID2); // 修改了路径，需要更新碰撞检测缓冲区
            return;
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
            //robot[botID1].botTarState = NO_TARGET;
            if (robot[botID1].botMoveState == TOGOODS)
            {
                goodsInMap[robot[botID1].tarX][robot[botID1].tarY] = robot[botID1].goodsVal;
                robot[botID1].curPropotion = -1;
                ++numCurGoods;

                int x = robot[botID1].tarX, y = robot[botID1].tarY;
                int berthID = nearBerthID[x][y];
                int goodsID = goodsIDInBerthZone[x][y];
                berth[berthID].goodsInBerthInfo.emplace(goodsID, singleGoodsInfo(goodsInMap[x][y], 2 * nearBerthDis[x][y], x, y));
            }
            robot[botID1].botMoveState = WAITING;

            robot[botID1].idxInPth = 0;
            vector<int>().swap(robot[botID1].pathDir);     // 清空
            vector<Point>().swap(robot[botID1].pathPoint); // 清空
            refreshJamBuffer(botID1);                      // 修改了路径，需要更新碰撞检测缓冲区
            // 这个变量留给寻找新路的时候用
            // robot[botID1].avoidBotID = -1;

            robot[botID2].botAvoidState = NO_AVOIDING;
            robot[botID2].botPathState = NO_PATH;
            //robot[botID2].botTarState = NO_TARGET;
            if (robot[botID2].botMoveState == TOGOODS)
            {
                goodsInMap[robot[botID2].tarX][robot[botID2].tarY] = robot[botID2].goodsVal;
                robot[botID2].curPropotion = -1;
                ++numCurGoods;

                int x = robot[botID2].tarX, y = robot[botID2].tarY;
                int berthID = nearBerthID[x][y];
                int goodsID = goodsIDInBerthZone[x][y];
                berth[berthID].goodsInBerthInfo.emplace(goodsID, singleGoodsInfo(goodsInMap[x][y], 2 * nearBerthDis[x][y], x, y));
            }
            robot[botID2].botMoveState = WAITING;

            robot[botID2].idxInPth = 0;
            vector<int>().swap(robot[botID2].pathDir);     // 清空
            vector<Point>().swap(robot[botID2].pathPoint); // 清空
            refreshJamBuffer(botID2);                      // 修改了路径，需要更新碰撞检测缓冲区
            // 这个变量留给寻找新路的时候用
            // robot[botID2].avoidBotID = -1;
        }
    }
}

// 寻找避让路径，默认是botID2去寻找避让botID1的路径
bool DecisionMaker::getAvoidPath(int botID1, int botID2)
{
    int queueCount = 0;
    int queueIndex = 0;
    if (robot[botID2].robotStatus == 0)
        return false;
    int x = robot[botID2].curX, y = robot[botID2].curY;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        if (robot[i].robotStatus == 0 && (abs(robot[i].curX - x) + abs(robot[i].curY - y) < BOT_EXRECOVER_DIST))
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
            tmpX = robot[botID2].jamDetectBuffer[i + 1] / MAP_SIZE;
            tmpY = robot[botID2].jamDetectBuffer[i + 1] % MAP_SIZE;
            vis[tmpX][tmpY] = true;
        }
        else if (robot[botID1].jamDetectBuffer[i + 1] == robot[botID2].jamDetectBuffer[i] && robot[botID1].jamDetectBuffer[i] == robot[botID2].jamDetectBuffer[i + 1])
        { // 对撞
            tmpX = robot[botID1].jamDetectBuffer[i] / MAP_SIZE;
            tmpY = robot[botID1].jamDetectBuffer[i] % MAP_SIZE;
            vis[tmpX][tmpY] = true;
        }
    }

    bool pointAvailable = true; // 用于标识找到的避让点是否可行

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

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
            for (int i = 0; i < ROBOT_NUM; ++i) // 检查该避让点是否已经被别人所占据
                if (robot[i].botAvoidState != NO_AVOIDING)
                    if (now->x == robot[i].tmpTarX && now->y == robot[i].tmpTarY)
                    {
                        pointAvailable = false;
                        break;
                    }

        if (pointAvailable)
        {
            for (int i = 0; i < ROBOT_NUM; ++i)
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
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    robot[botID2].pathDir.clear(); // 清空
    if (target != nullptr)
    {
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent)
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
    return true;
}

// 检测是否可以解除堵塞状态
void DecisionMaker::unJam()
{
    for (int i = 0; i < ROBOT_NUM; ++i)
    {
        if (robot[i].botAvoidState == AVOIDING)
        { // 正处于避让状态
            if (robot[i].curX == robot[i].tmpTarX && robot[i].curY == robot[i].tmpTarY)
            {
                robot[i].botAvoidState = AVOIDED;
                bool findPathFlag = getToTarPath(i, true);
                if (!findPathFlag)
                { // 应该只用处理找不到的情况，找到路的话，状态变量似乎没有什么需要特地更新的
                    // 还是找不到路，则在当前帧不动，且放弃当前的目标货物（如果有），在下一帧中寻找新的目标，直到能找到为止
                    robot[i].botPathState = NO_PATH;
                    //robot[i].botTarState = NO_TARGET;
                    vector<int>().swap(robot[i].pathDir);     // 清空
                    vector<Point>().swap(robot[i].pathPoint); // 清空
                    if (robot[i].botMoveState == TOGOODS)
                    {
                        ++numCurGoods;
                        goodsInMap[robot[i].tarX][robot[i].tarY] = robot[i].goodsVal;

                        int x = robot[i].tarX, y = robot[i].tarY;
                        int berthID = nearBerthID[x][y];
                        int goodsID = goodsIDInBerthZone[x][y];
                        berth[berthID].goodsInBerthInfo.emplace(goodsID, singleGoodsInfo(goodsInMap[x][y], 2 * nearBerthDis[x][y], x, y));
                    }
                    robot[i].botMoveState = WAITING;
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
