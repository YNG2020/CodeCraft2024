#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H
#include "global_vars.h"
#include <iostream>
#include <cstring>
#include <queue>
using namespace std;

class DecisionMaker {
private:
    struct Node {
        int x, y, firstStepDir;
        Node(int xx, int yy, int fsd = -1) : x(xx), y(yy), firstStepDir(fsd) {}
    };


    bool vis[N+5][N+5];

    bool inBerth(int x, int y) {
        for (int i = 0; i < berth_num; i++) {
            if (x >= berth[i].x && x < berth[i].x + 4 &&
                y >= berth[i].y && y < berth[i].y + 4) {
                return true;
            }
        }
        return false;
    }

    int getBerthId(int x, int y) {
        for (int i = 0; i < berth_num; i++) {
            if (x >= berth[i].x && x < berth[i].x + 4 &&
                y >= berth[i].y && y < berth[i].y + 4) {
                return i;
            }
        }
        return -1;
    }

public:
    void makeDecision() {
        robotDecision();
        shipDecision();
    }

    void shipDecision() {
        for (int i = 0; i < boat_num; i++) {
            int berthId = boat[i].pos;
            int loadNum = berth[berthId].load();
            boat[i].num += loadNum;
        }
        for (int i = 0; i < boat_num; i++) {
            if (boat[i].status == 0) continue;
            if (boat[i].status == 1) {
                if (boat[i].pos == -1) {
                    cout << "ship " << i << " " << i << endl;
                }
                else {
                    if (boat[i].num >= 100)
                        cout << "go " << i << endl;
                }
            }
            
        }
    }

    void robotDecision() {
        for (int i = 0; i < robot_num; i++) {
            if (goods[robot[i].x][robot[i].y] > 0 && robot[i].goods == 0) {
                cout << "get " << i << endl;
                goods[robot[i].x][robot[i].y] = 0;
            }
            else if (robot[i].goods > 0) {
                if (inBerth(robot[i].x, robot[i].y)) {
                    cout << "pull " << i << endl;
                    berth[getBerthId(robot[i].x, robot[i].y)].goodsNum++;
                }
                else {
                    Node target = getNearestBerth(robot[i].x, robot[i].y);
                    if (target.firstStepDir != -1 && !willCollide(i, target.firstStepDir)) { // 确保找到了有效的移动方向且不会发生碰撞
                        cout << "move " << i << " " << target.firstStepDir << endl;
                    }
                }
            }
            else if (robot[i].goods == 0) {
                Node target = getNearestGoods(robot[i].x, robot[i].y);
                if (target.firstStepDir != -1 && !willCollide(i, target.firstStepDir)) { // 确保找到了有效的移动方向且不会发生碰撞
                    cout << "move " << i << " " << target.firstStepDir << endl;
                }
            }
        }
    }

    bool willCollide(int robotId, int direction) {
        int nx = robot[robotId].x + dx[direction];
        int ny = robot[robotId].y + dy[direction];
        // 检查目标位置是否有其他机器人
        for (int i = 0; i < robot_num; i++) {
            if (i != robotId && robot[i].x == nx && robot[i].y == ny) {
                return true; // 发现潜在碰撞
            }
        }
        return false; // 无碰撞风险
    }

    // 查找最近的货物
    Node getNearestGoods(int x, int y) {
        queue<Node> q;
        q.push(Node(x, y));
        memset(vis, 0, sizeof(vis));
        for (int i = 0; i < robot_num; i++) {
            vis[robot[i].x][robot[i].y] = true;
        }

        while (!q.empty()) {
            Node now = q.front();
            q.pop();

            if (goods[now.x][now.y] > 0) {
                return now; // 返回包含第一步方向的节点
            }

            for (int i = 0; i < 4; i++) {
                int nx = now.x + dx[i];
                int ny = now.y + dy[i];
                if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny]) continue;
                vis[nx][ny] = true;
                int firstStepDir = (now.firstStepDir == -1) ? i : now.firstStepDir;
                q.push(Node(nx, ny, firstStepDir)); // 保持第一步方向不变
            }
        }

        return Node(-1, -1, -1); // 如果没有找到目标，返回一个无效节点
    }

    // 查找最近的泊位
    Node getNearestBerth(int x, int y) {
        queue<Node> q;
        q.push(Node(x, y));
        memset(vis, 0, sizeof(vis));
        vis[x][y] = true;

        while (!q.empty()) {
            Node now = q.front();
            q.pop();

            if (inBerth(now.x, now.y)) {
                return now; // 返回包含第一步方向的节点
            }

            for (int i = 0; i < 4; i++) {
                int nx = now.x + dx[i];
                int ny = now.y + dy[i];
                if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny]) continue;
                vis[nx][ny] = true;
                int firstStepDir = (now.firstStepDir == -1) ? i : now.firstStepDir;
                q.push(Node(nx, ny, firstStepDir)); // 保持第一步方向不变
            }
        }

        return Node(-1, -1, -1); // 如果没有找到泊位，返回一个无效节点
    }

};

#endif // DECISION_MAKER_H
