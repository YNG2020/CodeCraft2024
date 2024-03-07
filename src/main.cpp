#include <iostream>
#include <cstdlib>
#include <ctime>
#include "robot.hpp"
#include "berth.hpp"
#include "boat.hpp"
#include "global_vars.hpp"
#include "decisionmaker.hpp"

using namespace std;

void Init() {
    for (int i = 0; i < n; i++) {
        cin >> map[i];
    }
    for (int i = 0; i < berth_num; i++) {
        int id;
        cin >> id;
        cin >> berth[id].x >> berth[id].y >> berth[id].transport_time >> berth[id].loading_speed;
    }
    for (int i = 0; i < boat_num; i++) {
        boat[i].num = 0;
        boat[i].status = 1;
        boat[i].pos = -1;
    }
    cin >> boat_capacity;
    string okk;
    cin >> okk;
    cout << "OK" << endl;
    cout.flush();
}

int Input() {
    cin >> id >> money;
    int num;
    cin >> num;
    for (int i = 1; i <= num; i++) {
        int x, y, val;
        cin >> x >> y >> val;
        goods[x][y] = val;
    }
    for (int i = 0; i < robot_num; i++) {
        cin >> robot[i].goods >> robot[i].x >> robot[i].y >> robot[i].status;
    }
    for (int i = 0; i < 5; i++)
        cin >> boat[i].status >> boat[i].pos;
    string okk;
    cin >> okk;
    return id;
}

int main() {
    srand((unsigned int)time(nullptr)); // Seed for random number generation
    DecisionMaker decisionMaker;
    Init();
    for (int zhen = 1; zhen <= 15000; zhen++) {
        int id = Input();
        decisionMaker.makeDecision();
        cout << "OK" << endl;
        cout.flush();
    }
    return 0;
}
