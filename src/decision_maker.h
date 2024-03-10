#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H
#include "Point.h"
#include <iostream>
#include <vector>
#include <queue>
using namespace std;

class DecisionMaker
{
private:
    struct Node
    {
        int x, y;
        Node *parent; // 指向父节点的指针
        Node(int xx, int yy, Node *p = nullptr) : x(xx), y(yy), parent(p) {}
    };

    vector<int> priority;
    bool vis[210][210];

    bool inBerth(int x, int y);
    int getBerthId(int x, int y);
    bool willCollide(int robotId, int direction);
    bool getNearestGoods(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID);
    bool getNearestBerth(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID);
    void moveControl();
    void calPriority();
    void ship_init();
    void berth_select(int boat_id);

public:
    DecisionMaker();
    void makeDecision();
    void shipDecision();
    void robotDecision();
    void refreshState(int botID);
};

#endif // DECISION_MAKER_H