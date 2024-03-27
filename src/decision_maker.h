#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H
#include "point.h"
#include <iostream>
#include <vector>
#include <queue>
using namespace std;

class DecisionMaker
{
private:
    struct Node
    {
        int x, y, dis;
        Node *parent; // 指向父节点的指针
        Node(int xx = 0, int yy = 0, Node *p = nullptr, int d = 0) : x(xx), y(yy), parent(p), dis(d) {}
        void setNode(int xx, int yy, int d, Node* p)
        {
            x = xx;
            y = yy;
            dis = d;
            parent = p;
        }
    };

    Node* nodes;
    vector<int> priority;
    bool vis[210][210];

    bool inBerth(int x, int y);
    int getBerthId(int x, int y);
    bool getNearestGoods(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID, bool tryChangePath);
    bool getNearestBerth(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID);
    bool getAvoidPath(int botID1, int botID2);
    bool getToTarPath(int botID);
    void moveControl();
    void setPriority();
    int berth_select(int boat_id, int oriLocation);
    void refreshJamBuffer(int botID);
    bool jamDetect(int botID1, int botID2);
    bool unJamDetect(int botID1, int botID2);
    
    void jamControl();
    void jamResolve(int botID1, int botID2);
    void unJam();
public:
    DecisionMaker();
    void makeDecision();
    void shipDecision();
    void robotDecision();
    void refreshRobotState(int botID);
    //void refreshBoatState(int boatID);
    void getNearBerthDis(int x, int y); // 计算点到最近的泊位的距离
};

#endif // DECISION_MAKER_H