#include "decision_maker.h"
#include "global_vars.h"
#include <cstring>

namespace decision_maker_base {
    struct DisNode {
        int x, y, dis;
        DisNode(int xx, int yy, int d) : x(xx), y(yy), dis(d) {}
    };
}

DecisionMaker::DecisionMaker() : priority(robot_num, 0) {}

void DecisionMaker::makeDecision()
{
    robotDecision();
    shipDecision();
}

bool DecisionMaker::inBerth(int x, int y)
{
    for (int i = 0; i < berth_num; i++)
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
    for (int i = 0; i < berth_num; i++)
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
    queue<decision_maker_base::DisNode> q;
    memset(vis, 0, sizeof(vis));
    q.push(decision_maker_base::DisNode(x, y, 0));
    while (!q.empty())
    {
        decision_maker_base::DisNode now = q.front();
        q.pop();
        for (int i = 0; i < 4; i++)
        {
            int nx = now.x + dx[i];
            int ny = now.y + dy[i];
            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny])
                continue;
            vis[nx][ny] = true;
            if (inBerth(nx, ny))
            {
                nearBerthDis[x][y] = now.dis;
                nearBerthID[x][y] = getBerthId(nx, ny);
                // cerr << "nearBerthDis[" << x << "][" << y << "] = " << now.dis << endl;
                return;
            }
            q.push(decision_maker_base::DisNode(nx, ny, now.dis + 1));
        }
    }
}
