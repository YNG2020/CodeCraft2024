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

    double limToTryChangeGoods, limToChangeGoods;   // 尝试选新目标，接受新目标的参数
    int extraSearchTime;        // getNearestGoods中的额外搜索轮次
    int blockBerthTime;         // 提前屏蔽berth的时间
    int meanGoodsValue;         
    double gainForSameBerth;    // 本区增益
    double globalMeanGoodsRatio;// 全场泊位接收的货物的平均性价比

    Node* nodes;
    vector<int> priority;
    bool vis[210][210];

    bool inBerth(int x, int y);
    bool getNearestGoods(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID, bool tryChangePath, int callingBerthID);
    bool getNearestBerth(int x, int y, vector<Point> &pathPoint, vector<int> &pathDir, int botID);
    bool getAvoidPath(int botID1, int botID2);
    bool getToTarPath(int botID, bool calFromJam);
    void moveControl();
    void setPriority();
    int berth_select(int boat_id, int oriLocation);
    void refreshJamBuffer(int botID);
    bool jamDetect(int botID1, int botID2);
    bool unJamDetect(int botID1, int botID2);
    
    void jamControl();
    void jamResolve(int botID1, int botID2);
    void unJam();
    bool invalidForBoat(int x, int y);
    bool invalidForRobot(int x, int y);
public:


    DecisionMaker();
    void setParams(double limToTryChangeGoods, double limToChangeGoods, int extraSearchTime, int blockBerthTime, int meanGoodsValue, double gainForSameBerth);
    void makeDecision();
    void shipDecision();
    void robotDecision();
    void refreshRobotState(int botID);
    void refreshBerthState();
    void getNearBerthDis(int x, int y); // 计算点到最近的泊位的距离
    void getAvailableBerth(int x, int y, int botID);  // 计算机器人可行的泊位
    void getConnectedBerth(int berthID); // 计算相互邻接的泊位的距离
    int getBerthId(int x, int y);
};

#endif // DECISION_MAKER_H