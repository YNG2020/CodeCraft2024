#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H
#include "global_vars.h"
#include <iostream>
#include <vector>
#include <queue>
using namespace std;

class DecisionMaker {
private:
    struct Node {
        int x, y;
        Node* parent; // 指向父节点的指针
        Node(int xx, int yy, Node* p = nullptr) : x(xx), y(yy), parent(p) {}
    };

    vector<int> priority;
    bool vis[N][N];

    bool inBerth(int x, int y);
    int getBerthId(int x, int y);
    bool willCollide(int robotId, int direction);
    vector<int> getNearestGoods(int x, int y);
    vector<int> getNearestBerth(int x, int y);
    void moveControl();
    void calPriority();

public:
    DecisionMaker();
    void makeDecision();
    void shipDecision();
    void robotDecision();
};

#endif // DECISION_MAKER_H