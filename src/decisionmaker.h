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
        int x, y, firstStepDir;
        Node(int xx, int yy, int fsd = -1) : x(xx), y(yy), firstStepDir(fsd) {}
    };

    vector<int> priority;
    bool vis[N][N];

    bool inBerth(int x, int y);
    int getBerthId(int x, int y);
    bool willCollide(int robotId, int direction);
    Node getNearestGoods(int x, int y);
    Node getNearestBerth(int x, int y);
    void moveControl();
    void calPriority();

public:
    DecisionMaker();
    void makeDecision();
    void shipDecision();
    void robotDecision();
};

#endif // DECISION_MAKER_H