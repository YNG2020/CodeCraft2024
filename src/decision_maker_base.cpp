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
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
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
    int nearestBerthDis = 1000000000;    // 最近邻泊位的距离
    int queueCount = 0;
    int queueIndex = 0;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
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
    while (!q.empty()) {
        SimplePoint now = q.front();
        q.pop();
        for (int i = 0; i < 4; i++) {
            int nx = now.x + dx[i];
            int ny = now.y + dy[i];
            // if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || gridMap[nx][ny] != BERTH || berthMap[nx][ny] != -1) continue;
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || oriMap[nx][ny] != 'B' || berthMap[nx][ny] != -1) continue;
            q.push(SimplePoint(nx, ny));
            berthMap[nx][ny] = berthID;
        }
    }
}

void DecisionMaker::analyzeMap()
{
    // std::map<char, GRID_TYPE> mp = {
    //     {'#', BLOCK},
    //     {'*', WATER},
    //     {'~', ROAD_WATER},
    //     {'.', LAND},
    //     {'>', ROAD_LAND},
    //     {'R', ROBOT_SHOP},
    //     {'S', BOAT_SHOP},
    //     {'T', TRADE},
    //     {'B', BERTH},
    //     {'K', ANCHORAGE},
    //     {'C', MIX},
    //     {'c', ROAD_MIX}
    // };
    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            if (oriMap[i][j] == 'R') {
                robotShop.emplace_back(i, j);
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
    for (int i = 0; i < berthNum; i++) {
        paintBerth(berth[i].x, berth[i].y, i);
    }
}

void DecisionMaker::purchaseDecision()
{
    if (robotNum < 9) {
        for (int i = 0; i < robotShop.size(); i++)
        {
            printf("lbot %d %d\n", robotShop[i].x, robotShop[i].y);
        }
    }
}