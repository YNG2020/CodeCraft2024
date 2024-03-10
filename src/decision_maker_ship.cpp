#include "decision_maker.h"
#include "global_vars.h"

void DecisionMaker::shipDecision()
{
    if (id == 1)
    {
        ship_init();
        return;
    }
    for (int boat_id = 0; boat_id < boat_num; ++boat_id)
    {
        int berth_id = boat[boat_id].pos;
        // 最终装载时间到,直接去虚拟点
        if (id == 15000 - berth[berth_id].transport_time)
        {
            cout << "go " << boat_id << endl;
            continue;
        }
        switch (boat[boat_id].status)
        {
        case 0:
            break;
        case 1:

            if (boat[boat_id].pos == -1)
            { // 在虚拟点，选择泊位航向，货物量置0
                berth_select(boat_id);
                boat[boat_id].num = 0;
            }
            else
            {
                if (boat[boat_id].num == boat[boat_id].capacity) // 如果装满了，去虚拟点
                {
                    cout << "go " << boat_id << endl;
                }
                else
                { // 没有满则继续装
                    boat[boat_id].num += berth[berth_id].load(boat[boat_id].capacity - boat[boat_id].num);
                }
            }
            break;
        case 2:
            berth_select(boat_id);
            break;
        default:
            break;
        }
    }
}
void DecisionMaker::ship_init()
{
    for (int boat_id = 0; boat_id < boat_num; ++boat_id)
    {
        cout << "ship " << boat_id << " " << 2 * boat_id << endl;
    }
}
void DecisionMaker::berth_select(int boat_id)
{
    cout << "ship " << boat_id << " " << rand() % 9 << endl;
}

void DecisionMaker::robotDecision()
{

    for (int i = 0; i < robot_num; i++)
    {

        Robot &bot = robot[i];
        refreshState(i); // 自动更新robot的状态

        if (bot.botMoveState == ARRIVEGOODS)
        {
            cout << "get " << i << endl;
            goodsInMap[bot.x][bot.y] = 0;
            bot.goods = 1;               // 手动更新为持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.lastX = -1;
            bot.lastY = -1;
        }
        if (bot.botMoveState == ARRIVEBERTH)
        {
            cout << "pull " << i << endl;
            berth[getBerthId(bot.x, bot.y)].goodsNum++;
            bot.goods = 0;               // 手动更新为不持有货物的状态
            bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
            bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
            bot.lastX = -1;
            bot.lastY = -1;
        }

        if (bot.botTarState == NO_TARGET || bot.botPathState == NO_PATH)
        { // 没有目标，分配目标（目前通过寻路在分配目标，需要改进），之前没找到路，更新路（适用于中途变更路径，但失败的情况）
            if (bot.goods == 0)
            { // 未持有货物
                bool findPathFlag = getNearestGoods(bot.x, bot.y, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                { // 找到路，则更新一些状态变量
                    bot.idxInPth = 0;
                    bot.botMoveState = TOGOODS;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.x;
                    bot.lastY = bot.y;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    if (bot.pathPoint.size() > 1)
                    {
                        bot.nextX = bot.pathPoint[1].x;
                        bot.nextY = bot.pathPoint[1].y;
                    }
                }
                else
                { // 没找到路，也更新一些状态变量
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.botTarState = NO_TARGET;
                    bot.tarX = -1;
                    bot.tarY = -1;
                    if (bot.pathPoint.size() > 1)
                    {
                        bot.nextX = bot.pathPoint[1].x;
                        bot.nextY = bot.pathPoint[1].y;
                    }
                }
            }
            else
            { // 持有货物
                bool findPathFlag = getNearestBerth(bot.x, bot.y, bot.pathPoint, bot.pathDir, i);
                if (findPathFlag)
                {
                    bot.botMoveState = TOBERTH;
                    bot.botPathState = HAVE_PATH;
                    bot.botTarState = HAVE_TARGET;
                    bot.lastX = bot.x;
                    bot.lastY = bot.y;
                    bot.tarX = bot.pathPoint[bot.pathPoint.size() - 1].x;
                    bot.tarY = bot.pathPoint[bot.pathPoint.size() - 1].y;
                    if (bot.pathPoint.size() > 1)
                    {
                        bot.nextX = bot.pathPoint[1].x;
                        bot.nextY = bot.pathPoint[1].y;
                    }
                }
                else
                {
                    bot.botMoveState = WAITING;
                    bot.botPathState = NO_PATH;
                    bot.botTarState = NO_TARGET;
                    bot.tarX = -1;
                    bot.tarY = -1;
                    if (bot.pathPoint.size() > 1)
                    {
                        bot.nextX = bot.pathPoint[1].x;
                        bot.nextY = bot.pathPoint[1].y;
                    }
                }
            }
        }
    }
    moveControl();
}

void DecisionMaker::refreshState(int botID)
{

    Robot &bot = robot[botID];
    if (((bot.x != bot.lastX) || ((bot.y != bot.lastY))))
    { // 发现变更了位置
        ++bot.idxInPth;
        if (bot.idxInPth + 1 < bot.pathPoint.size())
        { // 更新堵塞检测缓冲区
            bot.nextX = bot.pathPoint[bot.idxInPth + 1].x;
            bot.nextY = bot.pathPoint[bot.idxInPth + 1].y;
        }
        bot.lastX = bot.x;
        bot.lastY = bot.y;
    }

    if ((goodsInMap[bot.x][bot.y] > 0 || goodsInMap[bot.x][bot.y] == -(botID + 1)) && bot.goods == 0)
    { // 自己本身没有货物，且遇上了货物（自己的目标或无主货物）
        bot.botMoveState = ARRIVEGOODS;
    }
    if (inBerth(bot.x, bot.y) && bot.goods > 0)
    { // 自己本身有货物，且抵达到泊位，不必考虑这个货物是否就是目标泊位
        bot.botMoveState = ARRIVEBERTH;
    }
    if (bot.botMoveState == TOGOODS && goodsInMap[bot.tarX][bot.tarY] == 0)
    {                                // 货物消失，及时更新自身状态
        bot.goods = 0;               // 手动更新为不持有货物的状态
        bot.botTarState = NO_TARGET; // 手动更新为无目标位置的状态
        bot.botMoveState = WAITING;  // 手动更新为原地等待的状态（等路径分配）
    }
}
