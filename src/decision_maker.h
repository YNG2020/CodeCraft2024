#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H
#include "global_struct.h"
#include "global_vars.h"
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
using namespace std;

class DecisionMaker
{
private:
    struct Node
    {
        int x, y, dis, dir;
        Node *parent; // 指向父节点的指针
        Node(int xx = 0, int yy = 0, Node *p = nullptr, int d = 0) : x(xx), y(yy), parent(p), dis(d) {}
        void setNode(int xx, int yy, int d, Node *p)
        {
            x = xx;
            y = yy;
            dis = d;
            parent = p;
        }
        void setNode(int xx, int yy, int d, Node *p, int direction)
        {
            x = xx;
            y = yy;
            dir = direction;
            dis = d;
            parent = p;
        }

        bool operator<(const Node other) const
        {
            return dis > other.dis; // 这里使用小于号，表示距离越小，优先级越高
        }
    };

    double limToTryChangeGoods, limToChangeGoods; // 尝试选新目标，接受新目标的参数
    int extraSearchTime;                          // getNearestGoods中的额外搜索轮次
    int blockBerthTime;                           // 提前屏蔽berth的时间
    int phase;                                    // 根据租赁情况判断当前阶段
    int boatNumLimit;                             // 船只数量限制
    int robotNumLimit;                            // 机器人数量限制
    double gainForSameBerth;                      // 本区增益
    double globalMeanGoodsRatio;                  // 全场泊位接收的货物的平均性价比
    double berthCallingFactor;                    // 泊位向邻区发出召唤的参数
    int efficientBerthID;                         // 最高效的泊位ID
    vector<int> sortBerthsByTransportTime;        // 依据泊位到交货点的距离对泊位ID进行升序排序
    double lastTimeFactor;
    int recursionDepthInBerthSelect;    // 穷举所有泊位组合时的最大递归深度

    Node *nodes;
    Node boatMapDis[MAP_SIZE][MAP_SIZE];
    vector<int> priority;
    vector<SimplePoint> robotShop;
    vector<SimplePoint> boatShop;
    vector<SimplePoint> tradePoint;
    bool vis[MAP_SIZE][MAP_SIZE];
    bool visBoat[4][MAP_SIZE][MAP_SIZE];
    int berthMap[MAP_SIZE][MAP_SIZE];       // 记录对应的泊位ID
    int berthMapSea[4][MAP_SIZE][MAP_SIZE]; // 记录海上对应的最近泊位ID
    int tradeMapSea[4][MAP_SIZE][MAP_SIZE]; // 记录海上对应的最近交易点ID
    GRID_TYPE gridMap[MAP_SIZE][MAP_SIZE];

    bool inBerth(int x, int y);
    void paintBerth(int berthID);       // 对泊位进行染色
    void findTrade(int berthID);        // 找到与泊位最近的交易点
    void findNearestBerth(int berthID); // 找到与泊位berthID最近的泊位（自身不算）
    bool inBerthSea(int x, int y);
    int getBerthIdSea(int x, int y);
    bool getNearestGoods(int x, int y, vector<SimplePoint> &pathPoint, vector<int> &pathDir, int botID, bool tryChangePath, int callingBerthID, int callingGoodsID);
    bool getNearestBerth(int x, int y, vector<SimplePoint> &pathPoint, vector<int> &pathDir, int botID);
    bool getToTarPath(int botID, bool calFromJam);
    bool getDetourPath(int botID1, int botID2);

    bool getBoatPathBFS(int boatID, int tarX, int tarY, vector<BoatPoint> &pathPoint, vector<int> &pathDir);
    bool getBoatPathDijkstra(int boatID, int tarX, int tarY, vector<BoatPoint> &pathPoint, vector<int> &pathDir);
    bool getBoatNearestBerthDijkstra(int boatID, vector<BoatPoint> &pathPoint, vector<int> &pathDir);
    bool getBoatNearestTradeDijkstra(int boatID, vector<BoatPoint> &pathPoint, vector<int> &pathDir);
    bool getBoatDetourPath(int botID1, int botID2);

    void moveControl();
    void boatMoveControl();

    int berthSelect(int boatID);
    void blockBerth(int berthID);

    void setPriority();
    void refreshJamBuffer(int botID);
    int jamDetect(int botID1, int botID2);
    bool unJamDetect(int botID1, int botID2);
    void jamControl();
    void jamResolve(int botID1, int botID2, int jamPos);
    bool getAvoidPath(int botID1, int botID2);
    void unJam();

    bool invalidForRobot(int x, int y);

    void boatSetPriority();
    void boatRefreshJamBuffer(int boatID);
    int boatJamDetect(int boatID1, int boatID2);
    bool boatUnJamDetect(int boatID1, int boatID2);
    void boatJamControl();
    void boatJamResolve(int boatID1, int boatID2, int jamPos);
    bool boatGetAvoidPath(int boatID1, int boatID2);
    void boatUnJam();
    bool checkOverLap(const BoatPoint &boat1, const BoatPoint &boat2);

    void phaseDecision();
    void purchaseDecision();
    void specialBerthSelect(int boatID, int upperBerthID, int upperTime, int upperGoodsNum, int Level, vector<int>& visitedBerth, int& minTransportTime, stack<int>& curStack, stack<int>& bestStack);
    int calAddGoodsNum(int berthID, int moveTime);

public:
    DecisionMaker();
    void setParams(double limToTryChangeGoods, double limToChangeGoods, int extraSearchTime, double lastTimeFactor, double gainForSameBerth, int boatNumLimit, int robotNumLimit, double berthCallingFactor, int recursionDepthInBerthSelect);
    void analyzeMap();
    void getMapInfoBoat();   // 得到船运动的地图信息
    void getMapDisBerth();   // 得到泊位的海上距离map
    void getMapDisTrade();   // 得到交货点的海上距离map
    void getNearBerthInfo(); // 得到地图上的点最近泊位
    void getNearTradeInfo(); // 得到地图上的点最近交货点
    void generateBerthTradeDis();
    void tradeAvailable(); // 交易点是否能到泊位，不能则删除该交易点
    void berthAvailable(); // 泊位是否能到交易点，不能则封禁该泊位
    bool getNearRobotShop(int robotShopID);    // 为泊位找到其最近的机器人租赁点

    int BoatAvailable(int x, int y, int dir);

    void makeDecision();
    void shipDecision();
    void robotDecision();

    void refreshRobotState(int botID);
    void refreshBerthState();
    void refreshBoatState(int boatID);

    void getNearBerthDis(int x, int y);  // 计算点到最近的泊位的距离
    void getConnectedBerth(int berthID); // 计算相互邻接的泊位的距离
    int getBerthId(int x, int y);
};

#endif // DECISION_MAKER_H
