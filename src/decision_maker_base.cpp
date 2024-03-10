#include "decision_maker.h"
#include "global_vars.h"

DecisionMaker::DecisionMaker() : priority(robot_num, 0) {}

void DecisionMaker::makeDecision()
{
    robotDecision();
    shipDecision();
}

bool DecisionMaker::inBerth(int x, int y)
{
    for (int i = 0; i < berth_num; i++)
    {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4)
        {
            return true;
        }
    }
    return false;
}

int DecisionMaker::getBerthId(int x, int y)
{
    for (int i = 0; i < berth_num; i++)
    {
        if (x >= berth[i].x && x < berth[i].x + 4 &&
            y >= berth[i].y && y < berth[i].y + 4)
        {
            return i;
        }
    }
    return -1;
}