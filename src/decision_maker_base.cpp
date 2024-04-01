#include "decision_maker.h"
#include "global_vars.h"
#include "global_struct.h"
#include <cstring>

namespace decision_maker_base {
    struct DisNode {
        int x, y, dis;
        DisNode(int xx, int yy, int d) : x(xx), y(yy), dis(d) {}
    };
}

DecisionMaker::DecisionMaker() : priority(robotNum, 0) { nodes = new Node[MAP_SIZE * MAP_SIZE]; }

void DecisionMaker::makeDecision()
{
    robotDecision();
    shipDecision();
}

bool DecisionMaker::invalidForBoat(int x, int y)
{
    return true;
}

bool DecisionMaker::invalidForRobot(int x, int y)
{
    return x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE || map[x][y] == '*' || map[x][y] == '#';
}

bool DecisionMaker::inBerth(int x, int y)
{
    for (int i = 0; i < berthNum; i++)
    {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4)
        {
            return true;
        }
    }
    return false;
}

int DecisionMaker::getBerthId(int x, int y)
{
    for (int i = 0; i < berthNum; i++)
    {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4)
        {
            return i;
        }
    }
    return -1;
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