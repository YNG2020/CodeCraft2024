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
