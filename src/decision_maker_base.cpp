#include "decision_maker.h"
#include "global_vars.h"
#include <cstring>

namespace decision_maker_base {
    struct DisNode {
        int x, y, dis;
        DisNode(int xx, int yy, int d) : x(xx), y(yy), dis(d) {}
    };
}

DecisionMaker::DecisionMaker() : priority(ROBOT_NUM, 0) { nodes = new Node[MAP_SIZE * MAP_SIZE]; }

void DecisionMaker::makeDecision()
{
    robotDecision();
    shipDecision();
}

bool DecisionMaker::inBerth(int x, int y)
{
    for (int i = 0; i < BERTH_NUM; i++)
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
    for (int i = 0; i < BERTH_NUM; i++)
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
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            if (inBerth(nx, ny))
            {
                nearBerthDis[x][y] = now->dis;
                nearBerthID[x][y] = getBerthId(nx, ny);
                // cerr << "nearBerthDis[" << x << "][" << y << "] = " << now.dis << endl;
                return;
            }
            child = &nodes[queueCount++];
            child->setNode(nx, ny, now->dis + 1, now);
        }
    }
}

void DecisionMaker::getAvailableBerth(int x, int y, int botID)
{
    int queueCount = 0;
    int queueIndex = 0;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr);
    memset(vis, 0, sizeof(vis));
    int numFoundedBerth = 0;

    while (queueCount > queueIndex && numFoundedBerth < BERTH_NUM)
    {
        now = &nodes[queueIndex++];

        for (int i = 0; i < 4; i++)
        {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            if (inBerth(nx, ny))
            {
                int berthID = getBerthId(nx, ny);
                if (!robot[botID].availableBerth[berthID])
                    ++numFoundedBerth;
                robot[botID].availableBerth[berthID] = true;
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