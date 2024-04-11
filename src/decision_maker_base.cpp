#include "decision_maker.h"
#include "global_vars.h"
#include "global_struct.h"
#include <cstring>
#include <map>
#include <cmath>
#include <fstream>

DecisionMaker::DecisionMaker() : priority(robotNum, 0)
{
    memset(vis, 0, sizeof(vis));
    memset(berthMap, -1, sizeof(berthMap));
    memset(nearBerthID, -1, sizeof(nearBerthID));
    nodes = new Node[100 * MAP_SIZE * MAP_SIZE];
    boatNumLimit = 2;
    robotNumLimit = 16;
    efficientBerthID = 0;
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

bool DecisionMaker::inBerthSea(int x, int y)
{
    return gridMap[x][y] == BERTH;
}

int DecisionMaker::getBerthIdSea(int x, int y)
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

void DecisionMaker::setParams(double limToTryChangeGoods, double limToChangeGoods, 
    int extraSearchTime, double lastTimeFactor, double gainForSameBerth,
    int boatNumLimit, int robotNumLimit, double berthCallingFactor, int recursionDepthInBerthSelect)
{
    this->limToTryChangeGoods = limToTryChangeGoods;
    this->limToChangeGoods = limToChangeGoods;
    this->extraSearchTime = extraSearchTime;
    this->lastTimeFactor = lastTimeFactor;
    this->gainForSameBerth = gainForSameBerth;
    this->boatNumLimit = boatNumLimit;
    this->robotNumLimit = robotNumLimit;
    this->berthCallingFactor = berthCallingFactor;
    this->recursionDepthInBerthSelect = recursionDepthInBerthSelect;
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
    int minDis = 0x7fffffff;
    int minTradeID = -1;
    for (int i = 0; i < tradeNum; ++i)
    {
        int dis = berthTradeDis[berthID][berthNum + i];
        if (dis < minDis)
        {
            minDis = dis;
            minTradeID = i;
        }
    }
    berth[berthID].transportTime = minDis;
    berth[berthID].transportTarget = minTradeID;
}

// 找到与泊位berthID最近的泊位（自身不算）
void DecisionMaker::findNearestBerth(int berthID)
{
    int minDis = 0x7fffffff;
    int minBerthID = berthID;
    for (int i = 0; i < berthNum; ++i)
    {
        if (i == berthID)
            continue;
        int dis = berthTradeDis[berthID][i];
        if (dis < minDis)
        {
            minDis = dis;
            minBerthID = i;
        }
    }
    berth[berthID].nearestBerthTime = minDis;
    berth[berthID].nearestBerthID = minBerthID;
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

    getMapInfoBoat();        // 得到船运动的地图信息
    getMapDisBerth();        // 得到泊位的海上距离map
    getMapDisTrade();        // 得到交货点的海上距离map
    getNearBerthInfo();      // 得到地图上的点最近泊位
    getNearTradeInfo();      // 得到地图上的点最近交货点
    tradeAvailable();        // 判定船购买点是否为封闭区域，如果不可达则删除此购买点
    berthAvailable();        // 泊位是否能到交易点，不能则封禁该泊位
    generateBerthTradeDis(); // 生成泊位交货点距离邻接矩阵
    // test_print();
    for (int i = 0; i < berthNum; i++)
    {
        paintBerth(i);
        findTrade(i);
        findNearestBerth(i);
    }
    int minTransportTime = 0x7fffffff;
    for (int i = 0; i < berthNum; ++i)
    {
        if (berth[i].transportTime < minTransportTime)
        {
            efficientBerthID = i;
            minTransportTime = berth[i].transportTime;
        }
    }

    // 依据泊位到交货点的距离对泊位ID进行升序排序
    sortBerthsByTransportTime.resize(berthNum);
    for (int i = 0; i < berthNum; ++i)
    {
        sortBerthsByTransportTime[i] = i; // 初始化序号为0, 1, 2, ..., berthNum-1
    }
    std::sort(sortBerthsByTransportTime.begin(), sortBerthsByTransportTime.end(), [&](int a, int b)
              { return berth[a].transportTime < berth[b].transportTime; });
    
    vector<bool> findBerthFlag(robotShop.size(), false);
    for (int i = 0; i < robotShop.size(); ++i)
    {
        findBerthFlag[i] = getNearRobotShop(i);
    }

    // 检验机器人租赁点有无可达的泊位，没有就erase掉
    for (int i = 0; i < robotShop.size(); ++i)
    {
        if (!findBerthFlag[i])
        {
            robotShop.erase(robotShop.begin() + i);
            findBerthFlag.erase(findBerthFlag.begin() + i);
            --i;
        }
    }
}

void DecisionMaker::tradeAvailable()
{
    for (int i = 0; i < boatShop.size(); ++i)
    {
        int dir;
        for (dir = 0; dir < 4; ++dir)
        {
            if (berthMapSea[dir][boatShop[i].x][boatShop[i].y] >= 0)
                break;
        }
        if (dir == 4)
        {
            boatShop.erase(boatShop.begin() + i);
            --i;
        }
    }
}

void DecisionMaker::berthAvailable()
{
    for (int i = 0; i < berthNum; ++i)
    {
        int tradeID = -1;
        for (int dir = 0; dir < 4; ++dir)
            tradeID = tradeID == -1 ? tradeMapSea[dir][berth[i].x][berth[i].y] : tradeID;
        if (tradeID == -1) // 如果这个泊位找不到相连的交货点，跳过
        {
            berth[i].isBlocked = true;
        }
    }
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
            for (int dir = 0; dir < 4; ++dir)
            {
                berthTradeDis[i][j] = max(berthTradeDis[i][j], berthDis[i][dir][berth[j].x][berth[j].y]);
                berthTradeDis[j][i] = max(berthTradeDis[j][i], berthDis[j][dir][berth[i].x][berth[i].y]);
            }
        }
    }
    for (int i = 0; i < tradeNum; ++i)
    {
        for (int j = 0; j < berthNum; ++j)
        {
            for (int dir = 0; dir < 4; ++dir)
            {
                berthTradeDis[berthNum + i][j] = max(berthTradeDis[berthNum + i][j], tradeDis[i][dir][berth[j].x][berth[j].y]);
                berthTradeDis[j][berthNum + i] = max(berthTradeDis[j][berthNum + i], tradeDis[i][dir][berth[j].x][berth[j].y]);
            }
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

void DecisionMaker::getMapDisBerth() // 此处船从泊位处开始倒着走
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
                if (visBoat[DirRev[now->dir]][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
                    continue;
                berthDis[i][DirRev[now->dir]][now->x][now->y] = max(now->dis, berthDis[i][DirRev[now->dir]][now->x][now->y]);
                visBoat[DirRev[now->dir]][now->x][now->y] = 1; // 确定为最短路径集合
                for (int j = 0; j < 3; j++)                    // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
                {
                    int nx = now->x + dirBoatDxRev[j][now->dir];
                    int ny = now->y + dirBoatDyRev[j][now->dir];

                    int curDir = j == 2 ? now->dir : clockWiseDirRev[j][now->dir];
                    if (boatTimeForDifDir[DirRev[curDir]][nx][ny] == 0 || visBoat[DirRev[curDir]][nx][ny])
                        continue;
                    child = &nodes[queueCount++]; // 这里只是为了用申请的空间
                    child->setNode(nx, ny, boatTimeForDifDir[DirRev[curDir]][nx][ny] + now->dis, now, curDir);
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
                if (visBoat[DirRev[now->dir]][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
                    continue;
                tradeDis[i][DirRev[now->dir]][now->x][now->y] = max(now->dis, tradeDis[i][DirRev[now->dir]][now->x][now->y]);
                visBoat[DirRev[now->dir]][now->x][now->y] = 1; // 确定为最短路径集合
                for (int j = 0; j < 3; j++)                    // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
                {
                    int nx = now->x + dirBoatDxRev[j][now->dir];
                    int ny = now->y + dirBoatDyRev[j][now->dir];

                    int curDir = j == 2 ? now->dir : clockWiseDirRev[j][now->dir];
                    if (boatTimeForDifDir[DirRev[curDir]][nx][ny] == 0 || visBoat[DirRev[curDir]][nx][ny])
                        continue;
                    child = &nodes[queueCount++]; // 这里只是为了用申请的空间
                    child->setNode(nx, ny, boatTimeForDifDir[DirRev[curDir]][nx][ny] + now->dis, now, curDir);
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
            for (int dir = 0; dir < 4; ++dir)
            {
                int curID = -1;
                int curDis = -1;
                for (int i = 0; i < berthNum; ++i)
                {
                    if (curID == -1 && berthDis[i][dir][x][y] != 0)
                    {
                        curID = i;
                        curDis = berthDis[i][dir][x][y];
                    }
                    else if (curDis > berthDis[i][dir][x][y] && berthDis[i][dir][x][y] != 0)
                    {
                        curID = i;
                        curDis = berthDis[i][dir][x][y];
                    }
                }
                berthMapSea[dir][x][y] = curID; // 使用前要判定合法boatTimeForDifDir
            }
        }
    }
}

void DecisionMaker::getNearTradeInfo()
{
    for (int x = 0; x < MAP_SIZE; ++x)
    {
        for (int y = 0; y < MAP_SIZE; ++y)
        {
            for (int dir = 0; dir < 4; ++dir)
            {
                int curID = -1;
                int curDis = -1;
                for (int i = 0; i < berthNum; ++i)
                {
                    if (curID == -1 && tradeDis[i][dir][x][y] != 0)
                    {
                        curID = i;
                        curDis = tradeDis[i][dir][x][y];
                    }
                    else if (curDis > tradeDis[i][dir][x][y] && tradeDis[i][dir][x][y] != 0)
                    {
                        curID = i;
                        curDis = tradeDis[i][dir][x][y];
                    }
                }
                tradeMapSea[dir][x][y] = curID; // 使用前要判定合法boatTimeForDifDir
            }
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

    if (frame + lastTimeFactor * berth[efficientBerthID].transportTime > 15000 && phase <= 1)
    {
        phase = 2;
        for (int j = 0; j < berthNum; ++j)
        {
            if (!berth[j].isBlocked)
            {
                int tradeID = -1;
                for (int dir = 0; dir < 4; ++dir)
                    tradeID = tradeID == -1 ? tradeMapSea[dir][berth[j].x][berth[j].y] : tradeID;
                if (tradeID == -1) // 如果这个泊位找不到相连的交货点，跳过
                {
                    berth[j].isBlocked = true;
                    continue;
                }
                // 检验该泊位对所有船的可达性
                berth[j].isBlocked = true;
                for (int i = 0; i < boatNum; ++i)
                {
                    int curBerth = getBerthIdSea(boat[i].curX, boat[i].curY);
                    int moveTimeToBerth = berthDis[j][boat[i].dire][boat[i].curX][boat[i].curY];
                    if (moveTimeToBerth != 0 || j == curBerth)
                    {
                        berth[j].isBlocked = false;
                        break;
                    }
                }
            }
        }
        // 选出boatNum个泊位来作为最后时刻的泊位
        int OKBerthNum = berthNum;
        for (int i = 0; i < berthNum; ++i)
            if (berth[i].isBlocked)
                --OKBerthNum;

        int counter = berthNum - 1;
        while (OKBerthNum > boatNum && counter >= 0)
        {
            int berthID = sortBerthsByTransportTime[counter];
            if (berth[berthID].isBlocked == false)
            {
                --OKBerthNum;
                berth[berthID].isBlocked = true;
            }
            --counter;
        }
    }
}

void DecisionMaker::purchaseDecision()
{
    if (phase == 0 && money >= 2000)
    {
        for (int i = 0; i < robotShop.size(); i++)
        {
            for (int j = 0; j < 2 && (robotNum + i * 2 + j < robotNumLimit); ++j)
                printf("lbot %d %d\n", robotShop[i].x, robotShop[i].y);
        }

        //int numRobotBuyInFirstTime = std::min(8, robotNumLimit);        // 第一批次购买机器人的数目
        //if (robotNum < numRobotBuyInFirstTime)
        //{
        //    int maxBuyRobotNum = std::min((int)ceil((double)numRobotBuyInFirstTime / robotShop.size()), 2);
        //    for (int i = 0; i < robotShop.size(); ++i)
        //        for (int j = 0; j < maxBuyRobotNum; ++j)
        //            printf("lbot %d %d\n", robotShop[i].x, robotShop[i].y);
        //}
        //else
        //{
        //    int addRobotNum = money / 2000;
        //    vector<int>buyRobotNum(robotShop.size(), 0);
        //    for (int i = 0; i < addRobotNum; ++i)
        //    {
        //        int buyFromRobotShopID = 0;
        //        double maxPriority = 0;
        //        for (int j = 0; j < robotShop.size(); ++j)
        //        {
        //            double thisPriority;
        //            double totGoodsVal = 0;
        //            int totServingRobot = 0;
        //            for (int berthID = 0; berthID < berthNum; ++berthID)
        //            {
        //                if (berth[berthID].nearestRobotShop == j)
        //                    for (auto it = berth[berthID].goodsInBerthInfo.begin(); it != berth[berthID].goodsInBerthInfo.end(); ++it)
        //                        totGoodsVal += it->second.goodsVal;
        //                else
        //                    continue;
        //                for (int l = 0; l < robotNum; ++l)
        //                {
        //                    Robot& bot = robot[l];
        //                    int servingBerthID = -1;
        //                    if (bot.botMoveState == TOGOODS)
        //                        servingBerthID = nearBerthID[bot.tarX][bot.tarY];
        //                    else if (bot.botMoveState == TOBERTH)
        //                        servingBerthID = getBerthId(bot.tarX, bot.tarY);
        //                    if (servingBerthID == berthID)
        //                        ++totServingRobot;
        //                }
        //            }
        //            totServingRobot += buyRobotNum[j];
        //            thisPriority = totGoodsVal / totServingRobot;
        //            if (thisPriority > maxPriority)
        //            {
        //                maxPriority = thisPriority;
        //                buyFromRobotShopID = j;
        //            }
        //        }
        //        ++buyRobotNum[buyFromRobotShopID];
        //        printf("lbot %d %d\n", robotShop[buyFromRobotShopID].x, robotShop[buyFromRobotShopID].y);
        //    }
        //}
    }
    // 第一帧有两种策略，买1/2艘船，具体效果待观察
    if (frame == 1)
    {
        printf("lboat %d %d\n", boatShop[0].x, boatShop[0].y);
    }
    if (robotNum >= robotNumLimit && boatNum < boatNumLimit)
    {
        int tmpSum = 0;
        for (int i = 0; i < berthNum; ++i)
            for (auto iter = berth[i].berthGoodsValueList.begin(); iter != berth[i].berthGoodsValueList.end(); ++iter)
                tmpSum += *iter;
        if (berthNum <= 4)
        {
            boatNumLimit -= 1;
            return;
        }
        printf("lboat %d %d\n", boatShop[0].x, boatShop[0].y);
    }
}

bool DecisionMaker::getNearRobotShop(int robotShopID)
{
    int x = robotShop[robotShopID].x, y = robotShop[robotShopID].y;
    int queueCount = 0;
    int queueIndex = 0;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));
    bool findBerthFlag = false;

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
                int berthID = getBerthId(nx, ny);
                if (berth[berthID].isBlocked)
                    continue;
                findBerthFlag = true;
                if (now->dis + 1 < berth[berthID].nearestRobotShopDis)
                {
                    berth[berthID].nearestRobotShopDis = now->dis + 1;
                    berth[berthID].nearestRobotShop = robotShopID;
                }
            }
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }
    return findBerthFlag;
}
