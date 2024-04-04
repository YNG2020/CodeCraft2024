#include "decision_maker.h"
#include "global_vars.h"
#include "global_struct.h"
#include <cstring>
#include <map>
#include <fstream>

DecisionMaker::DecisionMaker() : priority(robotNum, 0)
{
    memset(vis, 0, sizeof(vis));
    memset(berthMap, -1, sizeof(berthMap));
    nodes = new Node[100 * MAP_SIZE * MAP_SIZE];
    boatNumLimit = 2;
    robotNumLimit = 13;
}

void DecisionMaker::makeDecision()
{
    phaseDecision();
    purchaseDecision();
    robotDecision();
    shipDecision();
}

bool DecisionMaker::invalidForBoat(int x, int y)
{
    return true;
}

bool DecisionMaker::invalidForRobot(int x, int y)
{
    return x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE || gridMap[x][y] > ROAD_MIX;
    // return x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE || oriMap[x][y] == '#' || oriMap[x][y] == '*' || oriMap[x][y] == '~'
    // || oriMap[x][y] == 'S' || oriMap[x][y] == 'K' || oriMap[x][y] == 'T';
}

bool DecisionMaker::inBerth(int x, int y)
{
    return gridMap[x][y] == BERTH;
}

int DecisionMaker::getBerthId(int x, int y)
{
    return berthMap[x][y];
}

void DecisionMaker::getNearBerthDis(int x, int y)
{
    int queueCount = 0;
    int queueIndex = 0;
    Node *now = &nodes[queueCount++];
    Node *target = nullptr; // 用于存储找到的目标节点
    Node *child = nullptr;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            if (inBerth(nx, ny))
            {
                nearBerthDis[x][y] = now->dis + 1;
                int berthID = getBerthId(nx, ny);
                nearBerthID[x][y] = berthID;
                ++berth[berthID].totGoodsInBerthZone;
                goodsIDInBerthZone[x][y] = berth[berthID].totGoodsInBerthZone;
                berth[berthID].goodsInBerthInfo.emplace(berth[berthID].totGoodsInBerthZone, singleGoodsInfo(goodsInMap[x][y], 2 * nearBerthDis[x][y], x, y));
                // cerr << "nearBerthDis[" << x << "][" << y << "] = " << now.dis << endl;
                ++numCurGoods;
                return;
            }
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }
}

void DecisionMaker::getConnectedBerth(int berthID)
{
    int nearestBerthDis = 1000000000; // 最近邻泊位的距离
    int queueCount = 0;
    int queueIndex = 0;
    Node *now = &nodes[queueCount++];
    Node *target = nullptr; // 用于存储找到的目标节点
    Node *child = nullptr;
    int x = berth[berthID].x, y = berth[berthID].y;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));
    int numFoundedBerth = 0;

    while (queueCount > queueIndex && numFoundedBerth < berthNum)
    {
        now = &nodes[queueIndex++];

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (invalidForRobot(nx, ny) || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            if (inBerth(nx, ny))
            {
                int connectedBerthID = getBerthId(nx, ny);
                if (!berth[berthID].connectedBerth[connectedBerthID])
                    ++numFoundedBerth;
                berth[berthID].connectedBerth[connectedBerthID] = true;
                if (connectedBerthID != berthID && now->dis + 1 < nearestBerthDis)
                {
                    nearestBerthDis = now->dis + 1;
                    berth[berthID].nearestBerth = connectedBerthID;
                }
            }

            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }
}

void DecisionMaker::setParams(double limToTryChangeGoods, double limToChangeGoods, int extraSearchTime, int blockBerthTime, int meanGoodsValue, double gainForSameBerth)
{
    this->limToTryChangeGoods = limToTryChangeGoods;
    this->limToChangeGoods = limToChangeGoods;
    this->extraSearchTime = extraSearchTime;
    this->blockBerthTime = blockBerthTime;
    this->meanGoodsValue = meanGoodsValue;
    this->gainForSameBerth = gainForSameBerth;
}

void DecisionMaker::paintBerth(int berthID)
{
    queue<SimplePoint> q;
    int x = berth[berthID].x, y = berth[berthID].y;
    q.push(SimplePoint(x, y));
    berthMap[x][y] = berthID;
    while (!q.empty())
    {
        SimplePoint now = q.front();
        q.pop();
        for (int i = 0; i < 4; i++)
        {
            int nx = now.x + dx[i];
            int ny = now.y + dy[i];
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || (gridMap[nx][ny] != BERTH && gridMap[nx][ny] != ANCHORAGE) || berthMap[nx][ny] != -1)
                continue;
            q.push(SimplePoint(nx, ny));
            berthMap[nx][ny] = berthID;
        }
    }
}

void DecisionMaker::findTrade(int berthID)
{
    int x = berth[berthID].x, y = berth[berthID].y;
    int minDis = 0x7fffffff;
    int minTradeID = -1;
    for (int i = 0; i < tradePoint.size(); i++)
    {
        int dis = abs(tradePoint[i].x - x) + abs(tradePoint[i].y - y);
        if (dis < minDis)
        {
            minDis = dis;
            minTradeID = i;
        }
    }
    berth[berthID].transportTime = minDis;
    berth[berthID].transportTarget = minTradeID;
}

void DecisionMaker::analyzeMap()
{

    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            if (oriMap[i][j] == 'R')
            {
                robotShop.emplace_back(i, j);
            }
            if (oriMap[i][j] == 'S')
            {
                boatShop.emplace_back(i, j);
            }
            if (oriMap[i][j] == 'T')
            {
                tradePoint.emplace_back(i, j);
            }
            switch (oriMap[i][j])
            {
            case 'B':
                gridMap[i][j] = BERTH;
                break;
            case 'K':
                gridMap[i][j] = ANCHORAGE;
                break;
            case 'S':
                gridMap[i][j] = BOAT_SHOP;
                break;
            case 'T':
                gridMap[i][j] = TRADE;
                break;
            case 'R':
                gridMap[i][j] = ROBOT_SHOP;
                break;
            case '#':
                gridMap[i][j] = BLOCK;
                break;
            case '*':
                gridMap[i][j] = WATER;
                break;
            case '~':
                gridMap[i][j] = ROAD_WATER;
                break;
            case '.':
                gridMap[i][j] = LAND;
                break;
            case '>':
                gridMap[i][j] = ROAD_LAND;
                break;
            case 'C':
                gridMap[i][j] = MIX;
                break;
            case 'c':
                gridMap[i][j] = ROAD_MIX;
                break;
            default:
                break;
            }
        }
    }
    tradeNum = tradePoint.size();
    for (int i = 0; i < berthNum; i++)
    {
        paintBerth(i);
        findTrade(i);
    }
    getMapInfoBoat();        // 得到船运动的地图信息
    getMapDisBerth();        // 得到泊位的海上距离map
    getMapDisTrade();        // 得到交货点的海上距离map
    getNearBerthInfo();      // 得到地图上的点最近泊位
    getNearTradeInfo();      // 得到地图上的点最近交货点
    generateBerthTradeDis(); // 生成泊位交货点距离邻接矩阵
    // test_print();
}

// 得到船运动的地图信息
void DecisionMaker::getMapInfoBoat()
{
    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            char s = oriMap[i][j];
            if (s == '.' || s == '>' || s == '#' || s == 'R') // 不在船能走的区域
                continue;
            else
                for (int dir = 0; dir < 4; ++dir)
                    boatTimeForDifDir[dir][i][j] = BoatAvailable(i, j, dir);
        }
    }
}

void DecisionMaker::generateBerthTradeDis()
{
    // 调整berthTradeDis大小
    berthTradeDis.resize(berthNum + tradeNum);
    for (auto &row : berthTradeDis)
    {
        row.resize(berthNum + tradeNum);
    }
    for (int i = 0; i < berthNum; ++i)
    {
        for (int j = i + 1; j < berthNum; ++j)
        {
            berthTradeDis[i][j] = berthDis[i][berth[j].x][berth[j].y];
            berthTradeDis[j][i] = berthDis[j][berth[i].x][berth[i].y];
        }
    }
    for (int i = 0; i < tradeNum; ++i)
    {
        for (int j = 0; j < berthNum; ++j)
        {
            berthTradeDis[berthNum + i][j] = tradeDis[i][berth[j].x][berth[j].y];
            berthTradeDis[j][berthNum + i] = tradeDis[i][berth[j].x][berth[j].y];
        }
    }
}

void DecisionMaker::test_print()
{
    fstream f;
    f.open("berthDis.txt");
    int temp = berthNum + tradeNum;
    // int temp = MAP_SIZE;
    for (int i = 0; i < temp; ++i)
    {
        for (int j = 0; j < temp; ++j)
            f << berthTradeDis[i][j] << " ";
        f << endl;
    }
    f.close();
}

void DecisionMaker::getMapDisBerth()
{
    for (int i = 0; i < berthNum; ++i)
    {
        for (int dir = 0; dir < 4; ++dir)
        {
            memset(visBoat, 0, sizeof(visBoat)); // 这里visBoat起确定最短路径集合的作用
            priority_queue<Node> candidate;
            int queueCount = 0;
            int firstDis = 0;
            Node *now = &nodes[queueCount++];
            Node *target = nullptr;
            Node *child = nullptr;
            now->setNode(berth[i].x, berth[i].y, 0, nullptr, dir);
            candidate.push(*now);
            while (!candidate.empty()) // 如果没找到目标并且优先级队列不为空
            {
                now = &nodes[queueCount++];
                *now = candidate.top(); // 取出最短的节点now
                candidate.pop();
                if (visBoat[now->dir][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
                    continue;
                berthDis[i][now->x][now->y] = max(now->dis, berthDis[i][now->x][now->y]);
                visBoat[now->dir][now->x][now->y] = 1; // 确定为最短路径集合
                for (int j = 0; j < 3; j++)            // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
                {
                    int nx = now->x + dirBoatDx[j][now->dir];
                    int ny = now->y + dirBoatDy[j][now->dir];

                    int curDir = j == 2 ? now->dir : clockWiseDir[j][now->dir];
                    if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                        continue;
                    child = &nodes[queueCount++]; // 这里只是为了用申请的空间
                    child->setNode(nx, ny, boatTimeForDifDir[curDir][nx][ny] + now->dis, now, curDir);
                    candidate.push(*child);
                }
            }
        }
    }
}

void DecisionMaker::getMapDisTrade()
{
    for (int i = 0; i < tradeNum; ++i)
    {
        for (int dir = 0; dir < 4; ++dir)
        {
            memset(visBoat, 0, sizeof(visBoat)); // 这里visBoat起确定最短路径集合的作用
            priority_queue<Node> candidate;
            int queueCount = 0;
            int firstDis = 0;
            Node *now = &nodes[queueCount++];
            Node *target = nullptr;
            Node *child = nullptr;
            now->setNode(tradePoint[i].x, tradePoint[i].y, 0, nullptr, dir);
            candidate.push(*now);
            while (!candidate.empty()) // 如果没找到目标并且优先级队列不为空
            {
                now = &nodes[queueCount++];
                *now = candidate.top(); // 取出最短的节点now
                candidate.pop();
                if (visBoat[now->dir][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
                    continue;
                tradeDis[i][now->x][now->y] = max(now->dis, tradeDis[i][now->x][now->y]);
                visBoat[now->dir][now->x][now->y] = 1; // 确定为最短路径集合
                for (int j = 0; j < 3; j++)            // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
                {
                    int nx = now->x + dirBoatDx[j][now->dir];
                    int ny = now->y + dirBoatDy[j][now->dir];

                    int curDir = j == 2 ? now->dir : clockWiseDir[j][now->dir];
                    if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                        continue;
                    child = &nodes[queueCount++]; // 这里只是为了用申请的空间
                    child->setNode(nx, ny, boatTimeForDifDir[curDir][nx][ny] + now->dis, now, curDir);
                    candidate.push(*child);
                }
            }
        }
    }
}

void DecisionMaker::getNearBerthInfo()
{
    for (int x = 0; x < MAP_SIZE; ++x)
    {
        for (int y = 0; y < MAP_SIZE; ++y)
        {
            int curID = -1;
            int curDis = -1;
            if (boatTimeForDifDir[0][x][y] || boatTimeForDifDir[1][x][y] || boatTimeForDifDir[2][x][y] || boatTimeForDifDir[3][x][y])
            { // 如果船在此点的某个方向可以容下，计算此点的最近泊位ID
                for (int i = 0; i < berthNum; ++i)
                {
                    if (curID == -1 && berthDis[i][x][y] != 0)
                    {
                        curID = i;
                        curDis = berthDis[i][x][y];
                    }
                    else if (curDis > berthDis[i][x][y] && berthDis[i][x][y] != 0)
                    {
                        curID = i;
                        curDis = berthDis[i][x][y];
                    }
                }
                berthMapSea[x][y] = curID; // 使用前要判定合法boatTimeForDifDir
            }
            else
                berthMapSea[x][y] = -1;
        }
    }
}

void DecisionMaker::getNearTradeInfo()
{
    for (int x = 0; x < MAP_SIZE; ++x)
    {
        for (int y = 0; y < MAP_SIZE; ++y)
        {
            int curID = -1;
            int curDis = -1;
            if (boatTimeForDifDir[0][x][y] || boatTimeForDifDir[1][x][y] || boatTimeForDifDir[2][x][y] || boatTimeForDifDir[3][x][y])
            { // 如果船在此点的某个方向可以容下，计算此点的最近交货点
                for (int i = 0; i < tradeNum; ++i)
                {
                    if (curID == -1 && tradeDis[i][x][y] != 0)
                    {
                        curID = i;
                        curDis = tradeDis[i][x][y];
                    }
                    else if (curDis > tradeDis[i][x][y] && tradeDis[i][x][y] != 0)
                    {
                        curID = i;
                        curDis = tradeDis[i][x][y];
                    }
                }
                tradeMapSea[x][y] = curID; // 使用前要判定合法boatTimeForDifDir
            }
            else
                tradeMapSea[x][y] = -1;
        }
    }
}

int DecisionMaker::BoatAvailable(int x, int y, int dir) // 返回船在x,y处移动时间（1或2），不能放船则为0
{
    int row = 0, col = 0;
    int revX = 1, revY = 1;
    if (dir == 0 || dir == 1) // 0右  1左
    {
        row = 1;
        col = 2;
        if (dir == 0)
        {
            revX = 1;
            revY = 1;
        }
        else
        {
            revX = -1;
            revY = -1;
        }
    }
    else if (dir == 2 || dir == 3) // 2上 3下
    {
        row = 2;
        col = 1;
        if (dir == 2)
        {
            revX = -1;
            revY = 1;
        }
        else
        {
            revX = 1;
            revY = -1;
        }
    }
    int time = 1; // 默认需要1帧
    for (int i = 0; i <= row; ++i)
    {
        for (int j = 0; j <= col; ++j)
        {
            int xx = x + i * revX;
            int yy = y + j * revY;
            if (xx < 0 || xx >= MAP_SIZE || yy < 0 || yy >= MAP_SIZE)
                return 0;
            char s = oriMap[xx][yy];
            if (s == '.' || s == '>' || s == '#' || s == 'R') // 在陆地上，返回零
                return 0;
            else if (s != 'C' && s != '*') // 如果有某一部分进入了主航道、靠泊区、泊位等，则时间为2
            {                              // 对于交货点暂时认为时间为2
                time = 2;
            }
        }
    }
    return time;
}

void DecisionMaker::phaseDecision()
{
    if (robotNum < robotNumLimit)
    {
        phase = 0;
    }
    else
    {
        phase = 1;
    }
}

void DecisionMaker::purchaseDecision()
{
    if (phase == 0)
    {
        for (int i = 0; i < robotShop.size(); i++)
        {
            for (int j = 0; j < 2 && (robotNum + i * 2 + j < robotNumLimit); ++j)
                printf("lbot %d %d\n", robotShop[i].x, robotShop[i].y);
        }
        // TODO 买船
    }
    // 第一帧有两种策略，买1/2艘船，具体效果待观察
    if (frame == 1)
    {
        printf("lboat %d %d\n", boatShop[0].x, boatShop[0].y);
    }
    if (robotNum >= robotNumLimit && boatNum < boatNumLimit)
    {
        printf("lboat %d %d\n", boatShop[0].x, boatShop[0].y);
    }
}
