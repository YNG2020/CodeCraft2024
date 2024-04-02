#include "decision_maker.h"
#include "global_vars.h"
#include "global_struct.h"
#include <cstring>
#include <map>

DecisionMaker::DecisionMaker() : priority(robotNum, 0)
{
    memset(vis, 0, sizeof(vis));
    memset(berthMap, -1, sizeof(berthMap));
    nodes = new Node[MAP_SIZE * MAP_SIZE];
}

void DecisionMaker::makeDecision()
{

    robotDecision();
    shipDecision();
    purchaseDecision();
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
    // return oriMap[x][y] == 'B';
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

void DecisionMaker::paintBerth(int x, int y, int berthID)
{
    queue<SimplePoint> q;
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
            // if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || gridMap[nx][ny] != BERTH || berthMap[nx][ny] != -1) continue;
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || oriMap[nx][ny] != 'B' || berthMap[nx][ny] != -1)
                continue;
            q.push(SimplePoint(nx, ny));
            berthMap[nx][ny] = berthID;
        }
    }
}

void DecisionMaker::analyzeMap()
{

    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            // gridMap[i][j] = mp[oriMap[i][j]];
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
                case 'B': gridMap[i][j] = BERTH; break;
                case 'K': gridMap[i][j] = ANCHORAGE; break;
                case 'S': gridMap[i][j] = BOAT_SHOP; break;
                case 'T': gridMap[i][j] = TRADE; break;
                case 'R': gridMap[i][j] = ROBOT_SHOP; break;
                case '#': gridMap[i][j] = BLOCK; break;
                case '*': gridMap[i][j] = WATER; break;
                case '~': gridMap[i][j] = ROAD_WATER; break;
                case '.': gridMap[i][j] = LAND; break;
                case '>': gridMap[i][j] = ROAD_LAND; break;
                case 'C': gridMap[i][j] = MIX; break;
                case 'c': gridMap[i][j] = ROAD_MIX; break;
                default: break;
            }
        }
    }
    for (int i = 0; i < berthNum; i++)
    {
        paintBerth(berth[i].x, berth[i].y, i);
    }
    getMapInfoBoat();   // 得到船运动的地图信息
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

void DecisionMaker::purchaseDecision()
{
    if (robotNum < 9)
    {
        for (int i = 0; i < robotShop.size(); i++)
        {
            printf("lbot %d %d\n", robotShop[i].x, robotShop[i].y);
        }
    }
    if (frame == 1)
    {
        for (int i = 0; i < boatShop.size(); i++)
        {
            printf("lboat %d %d\n", boatShop[i].x, boatShop[i].y);
        }
    }
}