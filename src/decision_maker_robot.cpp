#include "decision_maker.h"
#include "global_vars.h"
#include <algorithm>
#include <cstring>

bool DecisionMaker::willCollide(int robotId, int direction)
{
    int nx = robot[robotId].x + dx[direction];
    int ny = robot[robotId].y + dy[direction];
    // 检查目标位置是否有其他机器人
    for (int i = 0; i < robot_num; i++)
    {
        if (i != robotId && robot[i].x == nx && robot[i].y == ny)
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
        vis[robot[i].x][robot[i].y] = true;
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
            if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
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
        vis[robot[i].x][robot[i].y] = true;
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
            if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
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

void DecisionMaker::moveControl()
{

    char oriMap[robot_num]; // 保存被robot修改的地图信息
    vector<bool> block(robot_num, false);
    vector<bool> isChangePath(robot_num, false);
    calPriority(); // 计算每一个机器人的移动优先级

    if (frame == 2586)
        int a = 1;

    for (int i = 0; i < robot_num; ++i)
    {
        for (int j = 0; j < robot_num; ++j)
        {
            if (i == j) // 自己不对自己进行避让
                continue;
            if (robot[priority[i]].nextX == robot[priority[j]].nextX && robot[priority[i]].nextY == robot[priority[j]].nextY ||
                robot[priority[i]].nextX == robot[priority[j]].x && robot[priority[i]].nextY == robot[priority[j]].y)
            { // 预计会发生碰撞
                if (priority[i] > priority[j])
                {
                    oriMap[priority[j]] = map[robot[priority[j]].x][robot[priority[j]].y];
                    map[robot[priority[j]].x][robot[priority[j]].y] = '#';
                    robot[priority[j]].nextX = robot[priority[j]].x;
                    robot[priority[j]].nextY = robot[priority[j]].y;
                    block[priority[j]] = true;
                    isChangePath[priority[i]] = true;
                }
            }
        }
    }

    for (int i = 0; i < robot_num; ++i)
    {

        Robot &bot = robot[i];
        if (block[i])
        {
            bot.botAvoidState = AVOIDING;
            if (bot.idxInPth + 1 < bot.pathPoint.size())
            { // 更新堵塞检测缓冲区
                bot.nextX = bot.pathPoint[bot.idxInPth + 1].x;
                bot.nextY = bot.pathPoint[bot.idxInPth + 1].y;
            }
            continue;
        }
        else
        {
            bot.botAvoidState = NO_AVOIDING;
        }
        if (isChangePath[i])
        {
            if (bot.goods > 0)
            { // 原本是要去港口的
                bool findPathFlag = getNearestBerth(bot.x, bot.y, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
                    bot.botMoveState = TOBERTH;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.x;
                    bot.lastY = bot.y;
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
                bool findPathFlag = getNearestGoods(bot.x, bot.y, bot.pathPoint, bot.pathDir, i);

                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
                    bot.botMoveState = TOGOODS;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.x;
                    bot.lastY = bot.y;
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
            cout << "move " << i << " " << bot.pathDir[bot.idxInPth] << endl;
        }
    }

    for (int i = 0; i < robot_num; ++i)
    {
        if (block[i])
        {
            map[robot[i].x][robot[i].y] = oriMap[i]; // 恢复这一个格子
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
