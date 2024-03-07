#ifndef DECISION_MAKER_HPP
#define DECISION_MAKER_HPP
#include "global_vars.hpp"
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

    bool inBert(int x, int y) {
        for (int i = 0; i < berth_num; i++) {
            if (x >= berth[i].x && x < berth[i].x + 4 &&
                y >= berth[i].y && y < berth[i].y + 4) {
                return true;
            }
        }
        return false;
    }

public:
    void makeDecision() {
        robotDecision();
        shipDecision();
    }

    void shipDecision() {
        for (int i = 0; i < boat_num; i++) {
            if (boat[i].status == 0) continue;
            if (boat[i].status == 1) {
                if (boat[i].pos == -1) {
                    cout << "ship " << i << " " << i << endl;
                }
                else if (boat[i].num == 5) {
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
                if (inBert(robot[i].x, robot[i].y)) {
                    cout << "pull " << i << endl;
                }
                else {
                    Node target = getNearestBerth(robot[i].x, robot[i].y);
                    if (target.firstStepDir != -1) { // 确保找到了有效的移动方向
                        cout << "move " << i << " " << target.firstStepDir << endl;
                    }
                }
            }
            else if (robot[i].goods == 0) {
                Node target = getNearestGoods(robot[i].x, robot[i].y);
                if (target.firstStepDir != -1) { // 确保找到了有效的移动方向
                    cout << "move " << i << " " << target.firstStepDir << endl;
                }
            }
        }
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

            if (inBert(now.x, now.y)) {
                return now; // 返回包含第一步方向的节点
            }

            for (int i = 0; i < 4; i++) {
                int nx = now.x + dx[i];
                int ny = now.y + dy[i];
                if (nx < 1 || nx > n || ny < 1 || ny > n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny]) continue;
                vis[nx][ny] = true;
                int firstStepDir = (now.firstStepDir == -1) ? i : now.firstStepDir;
                q.push(Node(nx, ny, firstStepDir)); // 保持第一步方向不变
            }
        }

        return Node(-1, -1, -1); // 如果没有找到泊位，返回一个无效节点
    }

};

#endif // DECISION_MAKER_H
