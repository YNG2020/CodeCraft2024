#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <cstring>

bool DecisionMaker::willCollide(int robotId, int direction)
{
    int nx = robot[robotId].curX + dx[direction];
    int ny = robot[robotId].curY + dy[direction];
    // 检查目标位置是否有其他机器人
    for (int i = 0; i < robot_num; i++)
    {
        if (i != robotId && robot[i].curX == nx && robot[i].curY == ny)
        {
            return true; // 发现潜在碰撞
        }
    }
    return false; // 无碰撞风险
}

bool DecisionMaker::getNearestGoods(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID)
{
    queue<Node *> q;
    vector<Node *> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        vis[robot[i].curX][robot[i].curY] = true;
    }

    Node *target = nullptr; // 用于存储找到的目标节点

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

        if (goodsInMap[now->x][now->y] > 0 || goodsInMap[now->x][now->y] == -(botID + 1))
        {
            target = now;                                       // 找到目标或找到自身目标
            robot[botID].goodsVal = goodsInMap[now->x][now->y]; // 先存储
            robot[botID].idxInPth = 0;                          // 更新路径点序列
            goodsInMap[now->x][now->y] = -(botID + 1);          // 打上标记
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
    return true;
}

bool DecisionMaker::getNearestBerth(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID)
{
    queue<Node *> q;
    vector<Node *> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++)
    {
        vis[robot[i].curX][robot[i].curY] = true;
    }

    Node *target = nullptr; // 用于存储找到的目标节点

    while (!q.empty())
    {
        Node *now = q.front();
        q.pop();

        if (inBerth(now->x, now->y))
        {
            target = now;              // 找到目标
            robot[botID].idxInPth = 0; // 更新路径点序列
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
    return true;
}

void DecisionMaker::robotDecision()
{

    for (int i = 0; i < robot_num; i++)
    {

        Robot& bot = robot[i];
        refreshState(i); // 自动更新robot的状态

        if (bot.botMoveState == ARRIVEGOODS)
        {
            cout << "get " << i << endl;
            goodsInMap[bot.curX][bot.curY] = 0;
            bot.carryGoods = 1;               // 手动更新为持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            if (goodsInMap[bot.curX][bot.curY] > 0) {   // 说明取走的是无主货物
                goodsInMap[bot.curX][bot.curY] = 0;
                goodsInMap[bot.tarX][bot.tarY] = bot.goodsVal;
            }
            else
                goodsInMap[bot.curX][bot.curX] = 0;
            bot.lastX = -1;
            bot.lastY = -1;
        }
        if (bot.botMoveState == ARRIVEBERTH)
        {
            cout << "pull " << i << endl;
            berth[getBerthId(bot.curX, bot.curY)].goodsNum++;
            bot.carryGoods = 0;               // 手动更新为不持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
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
                    bot.idxInPth = 0;
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

void DecisionMaker::refreshState(int botID)
{

    Robot& bot = robot[botID];
    if (((bot.curX != bot.lastX) || ((bot.curY != bot.lastY))))
    { // 发现变更了位置
        ++bot.idxInPth;
        refreshJamBuffer(botID);
        bot.lastX = bot.curX;
        bot.lastY = bot.curY;
    }

    if ((goodsInMap[bot.curX][bot.curY] > 0 || goodsInMap[bot.curX][bot.curY] == -(botID + 1)) && bot.carryGoods == 0)
    { // 自己本身没有货物，且遇上了货物（自己的目标或无主货物）
        bot.botMoveState = ARRIVEGOODS;
    }
    if (inBerth(bot.curX, bot.curY) && bot.carryGoods > 0)
    { // 自己本身有货物，且抵达到泊位，不必考虑这个货物是否就是目标泊位
        bot.botMoveState = ARRIVEBERTH;
    }
    if (bot.botMoveState == TOGOODS && goodsInMap[bot.tarX][bot.tarY] == 0)
    {                                // 货物消失，及时更新自身状态
        bot.carryGoods = 0;               // 手动更新为不持有货物的状态
        bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
        bot.botMoveState = WAITING;
    }
}

void DecisionMaker::moveControl()
{

    char oriMap[robot_num]; // 保存被robot修改的地图信息
    vector<bool> block(robot_num, false);
    vector<bool> isChangePath(robot_num, false);
    calPriority(); // 计算每一个机器人的移动优先级

    for (int i = 0; i < robot_num; ++i)
    {
        for (int j = 0; j < robot_num; ++j)
        {
            if (i == j) // 自己不对自己进行避让
                continue;
            if (jamDetect(priority[i], priority[j]))
            { // 预计会发生碰撞
                if (priority[i] > priority[j])
                {
                    oriMap[priority[j]] = map[robot[priority[j]].curX][robot[priority[j]].curY];
                    map[robot[priority[j]].curX][robot[priority[j]].curY] = '#';
                    block[priority[j]] = true;
                    isChangePath[priority[i]] = true;
                }
            }
        }
    }

    for (int i = 0; i < robot_num; ++i)
    {

        Robot& bot = robot[i];
        if (block[i])
        {
            bot.botAvoidState = AVOIDED;
            refreshJamBuffer(i);
            continue;
        }
        else
        {
            bot.botAvoidState = NO_AVOIDING;
        }
        if (isChangePath[i])
        {
            if (bot.carryGoods > 0)
            { // 原本是要去港口的
                bool findPathFlag = getNearestBerth(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    if (bot.pathDir.size() > 0)
                        cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
                    bot.botMoveState = TOBERTH;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                }
                else
                { // 没找到路，也更新一些状态变量
                    bot.idxInPth = 0;
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.tarX = -1;
                    bot.tarY = -1;
                }
                continue;
            }
            else
            { // 原本是要取货物的
                bool findPathFlag = getNearestGoods(bot.curX, bot.curY, bot.pathPoint, bot.pathDir, i);

                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    if (bot.pathDir.size() > 0)
                        cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
                    bot.botMoveState = TOGOODS;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.curX;
                    bot.lastY = bot.curY;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                }
                else
                { // 没找到路，也更新一些状态变量
                    bot.idxInPth = 0;
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.tarX = -1;
                    bot.tarY = -1;
                }
                continue;
            }
        }
        else
        {
            if (bot.botPathState == NO_PATH)
                continue;
            if (bot.pathDir.size() > 0)
                cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
        }
    }

    for (int i = 0; i < robot_num; ++i)
    {
        if (block[i])
        {
            map[robot[i].curX][robot[i].curY] = oriMap[i]; // 恢复这一个格子
        }
    }
}

void DecisionMaker::calPriority()
{
    // for (int i = 0; i < robot_num; ++i) {
    //     priorityFactor[i][0] = i;   // 第一列放编号
    //     priorityFactor[i][1] = robot[i].goodsVal;   // 第二列放货物的价值
    // }
    for (int i = 0; i < 10; ++i)
    { // 目前仅以编号作为唯一判据，可不受突发情况影响
        priority[i] = i;
    }
    //// 使用 lambda 表达式定义比较函数并对索引进行排序
    // std::sort(priority.begin(), priority.end(), [&](int a, int b) {
    //     return priorityFactor[a][0] != priorityFactor[b][0] ? priorityFactor[a][0] < priorityFactor[b][0] : priorityFactor[a][1] < priorityFactor[b][1];
    // });
}

// 更新堵塞检测缓冲区
void DecisionMaker::refreshJamBuffer(int botID)
{
    Robot& bot = robot[botID];
    int i = 0;
    if (bot.botPathState == HAVE_PATH)
    {   // 有路，可以正常更新缓冲区
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
    {   // 没路，缓冲区全都存储为自己本身所在的位置
        for (; i < bot.jamDetectBufferLen; ++i)
        {
            bot.jamDetectBuffer[i] = bot.curX * mapSize + bot.curY;
        }
    }

}

// 对每对robot进行堵塞检测
bool DecisionMaker::jamDetect(int botID1, int botID2) {

    Robot& bot1 = robot[botID1], bot2 = robot[botID2];
    for (int i = 0; i < robot[botID1].jamDetectBufferLen - 1; ++i)
    {    // 只需逐对检测是否有可能发生冲突即可
        if (bot1.jamDetectBuffer[i + 1] == bot2.jamDetectBuffer[i + 1]) // 二者下一个目标位置都相同
            return true;
        if (bot2.status == 0 || bot2.botPathState == NO_PATH)   
        {   // bot2不动 
            if (bot1.jamDetectBuffer[i + 1] == bot2.jamDetectBuffer[i]) // bto1撞到bot2停下的位置上
                return true;
        }
        if (bot1.jamDetectBuffer[i + 1] == bot2.jamDetectBuffer[i] && bot1.jamDetectBuffer[i] == bot2.jamDetectBuffer[i + 1])   // 对撞
            return true;
    }
    return false;
}
