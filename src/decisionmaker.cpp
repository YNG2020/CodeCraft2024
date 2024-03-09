#include "decisionmaker.h"
#include <algorithm>
#include <cstring>

DecisionMaker::DecisionMaker() : priority(robot_num, 0) {}

bool DecisionMaker::inBerth(int x, int y) {
    for (int i = 0; i < berth_num; i++) {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4) {
            return true;
        }
    }
    return false;
}

int DecisionMaker::getBerthId(int x, int y) {
    for (int i = 0; i < berth_num; i++) {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4) {
            return i;
        }
    }
    return -1;
}

bool DecisionMaker::willCollide(int robotId, int direction) {
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

vector<int> DecisionMaker::getNearestGoods(int x, int y) {
    queue<Node*> q;
    vector<Node*> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++) {
        vis[robot[i].x][robot[i].y] = true;
    }

    Node* target = nullptr; // 用于存储找到的目标节点

    while (!q.empty()) {
        Node* now = q.front();
        q.pop();

        if (goods[now->x][now->y] > 0) {
            target = now; // 找到目标
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny]) continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now)); // 使用父节点指针
        }
        rest.push_back(now);
    }

    vector<int> path;
    if (target != nullptr) {
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent) {
            for (int i = 0; i < 4; i++) {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i]) {
                    path.push_back(i);
                    break;
                }
            }
        }
        reverse(path.begin(), path.end()); // 反转路径，使其从起始节点开始
    }
    while (!q.empty()) {
        delete q.front();
        q.pop();
    }
    while (!rest.empty()) {
        delete rest.back();
        rest.pop_back();
    }
    return path;
}

vector<int> DecisionMaker::getNearestBerth(int x, int y) {
    queue<Node*> q;
    vector<Node*> rest;
    q.push(new Node(x, y));
    memset(vis, 0, sizeof(vis));
    for (int i = 0; i < robot_num; i++) {
        vis[robot[i].x][robot[i].y] = true;
    }

    Node* target = nullptr; // 用于存储找到的目标节点

    while (!q.empty()) {
        Node* now = q.front();
        q.pop();

        if (inBerth(now->x, now->y)) {
            target = now; // 找到目标
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nx = now->x + dx[i];
            int ny = now->y + dy[i];
            if (nx < 0 || nx >= n || ny < 0 || ny >= n || map[nx][ny] == '*' || map[nx][ny] == '#' || vis[nx][ny]) continue;
            vis[nx][ny] = true;
            q.push(new Node(nx, ny, now)); // 使用父节点指针
        }
        rest.push_back(now);
    }

    vector<int> path;
    if (target != nullptr) {
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent) {
            for (int i = 0; i < 4; i++) {
                if (p->x == p->parent->x + dx[i] && p->y == p->parent->y + dy[i]) {
                    path.push_back(i);
                    break;
                }
            }
        }
        reverse(path.begin(), path.end()); // 反转路径，使其从起始节点开始
    }
    while (!q.empty()) {
        delete q.front();
        q.pop();
    }
    while (!rest.empty()) {
        delete rest.back();
        rest.pop_back();
    }
    return path;
}

void DecisionMaker::moveControl() {
    char oriMap[robot_num]; // 保存被robot修改的地图信息
    vector<bool> block(robot_num, false);
    vector<bool> isChangePath(robot_num, false);
    calPriority();
    // 计算每一个机器人的移动优先级
    for (int i = 0; i < robot_num; ++i) {   // 先从优先级最大的机器人开始
        for (int j = 0; j < robot_num; ++j) {
            if (i == j) // 自己不对自己进行避让
                continue;
            if (robot[priority[i]].nextX == robot[priority[j]].nextX && robot[priority[i]].nextY == robot[priority[j]].nextY ||
                robot[priority[i]].nextX == robot[priority[j]].x && robot[priority[i]].nextY == robot[priority[j]].y) { // 预计会发生碰撞
                if (priority[i] > priority[j]) {
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
    for (int i = 0; i < robot_num; ++i) {
        if (block[i])
            continue;
        if (isChangePath[i]) {
            if (robot[i].goods > 0) {
                vector<int> path = getNearestGoods(robot[i].x, robot[i].y);

                if (path.empty()) {
                    robot[i].nextX = robot[i].x;
                    robot[i].nextY = robot[i].y;
                    robot[i].nextDir = -1;  // 这种情况自己先不去主动移动
                    continue;
                }
                else {
                    robot[i].nextX += dx[path[0]];
                    robot[i].nextY += dy[path[0]];
                    robot[i].nextDir = path[0];
                }
                cout << "move " << i << " " << robot[i].nextDir << endl;
                continue;
            }
            else {
                vector<int> path = getNearestBerth(robot[i].x, robot[i].y);

                if (path.empty()) {
                    robot[i].nextX = robot[i].x;
                    robot[i].nextY = robot[i].y;
                    robot[i].nextDir = -1;  // 这种情况自己先不去主动移动
                    continue;
                }
                else {
                    robot[i].nextX += dx[path[0]];
                    robot[i].nextY += dy[path[0]];
                    robot[i].nextDir = path[0];
                }
                cout << "move " << i << " " << robot[i].nextDir << endl;
                continue;
            }
        }
        else {
            if (robot[i].nextDir == -1)
                continue;
            cout << "move " << i << " " << robot[i].nextDir << endl;
        }

    }

    for (int i = 0; i < robot_num; ++i) {
        if (block[i]) {
            map[robot[i].x][robot[i].y] = oriMap[i];  // 恢复这一个格子
        }
    }
}

void DecisionMaker::calPriority() {
    //for (int i = 0; i < robot_num; ++i) {
    //    priorityFactor[i][0] = i;   // 第一列放编号
    //    priorityFactor[i][1] = robot[i].goodsVal;   // 第二列放货物的价值
    //}
    for (int i = 0; i < 10; ++i) {  // 目前仅以编号作为唯一判据，可不受突发情况影响
        priority[i] = i;
    }
    //// 使用 lambda 表达式定义比较函数并对索引进行排序
    //std::sort(priority.begin(), priority.end(), [&](int a, int b) {
    //    return priorityFactor[a][0] != priorityFactor[b][0] ? priorityFactor[a][0] < priorityFactor[b][0] : priorityFactor[a][1] < priorityFactor[b][1];
    //});
}

void DecisionMaker::makeDecision() {
    robotDecision();
    shipDecision();
}

void DecisionMaker::shipDecision() {
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

void DecisionMaker::robotDecision() {
    for (int i = 0; i < robot_num; i++) {
        if (goods[robot[i].x][robot[i].y] > 0 && robot[i].goods == 0) {
            cout << "get " << i << endl;
            goods[robot[i].x][robot[i].y] = 0;
            robot[i].goods = 1;     // 立刻更新为持有货物的状态
        }
        if (robot[i].goods > 0) {
            if (inBerth(robot[i].x, robot[i].y)) {
                cout << "pull " << i << endl;
                berth[getBerthId(robot[i].x, robot[i].y)].goodsNum++;
                robot[i].goods = 0;
            }
            else {
                vector<int> path = getNearestBerth(robot[i].x, robot[i].y);
                if (path.empty()) {
                    robot[i].nextX = robot[i].x;
                    robot[i].nextY = robot[i].y;
                    robot[i].nextDir = -1;  // 这种情况自己先不去主动移动
                }
                else {
                    robot[i].nextX += dx[path[0]];
                    robot[i].nextY += dy[path[0]];
                    robot[i].nextDir = path[0];
                }
                //if (target.firstStepDir != -1 && !willCollide(i, target.firstStepDir)) { // 确保找到了有效的移动方向且不会发生碰撞
                //    cout << "move " << i << " " << target.firstStepDir << endl;
                //}
            }
        }
        if (robot[i].goods == 0) {
            vector<int> path = getNearestGoods(robot[i].x, robot[i].y);
            if (path.empty()) {
                robot[i].nextX = robot[i].x;
                robot[i].nextY = robot[i].y;
                robot[i].nextDir = -1;  // 这种情况自己先不去主动移动
            }
            else {
                robot[i].nextX += dx[path[0]];
                robot[i].nextY += dy[path[0]];
                robot[i].nextDir = path[0];
            }
            //if (target.firstStepDir != -1 && !willCollide(i, target.firstStepDir)) { // 确保找到了有效的移动方向且不会发生碰撞
            //    cout << "move " << i << " " << target.firstStepDir << endl;
            //}
        }
    }
    moveControl();
}
