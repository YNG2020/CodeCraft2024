#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

struct Robot {
    int x, y, goods;
    int status;
    int mbx, mby;
    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
}robot[robot_num + 10];

struct Berth {
    int x;
    int y;
    int transport_time;
    int loading_speed;
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
}berth[berth_num + 10];

struct Boat {
    int num, pos, status;
}boat[10];

int money, boat_capacity, id;
char ch[N][N];
int gds[N][N];

void Init() {
    for (int i = 1; i <= n; i++)
        cin >> (ch[i] + 1);
    for (int i = 0; i < berth_num; i++) {
        int id;
        cin >> id;
        cin >> berth[id].x >> berth[id].y >> berth[id].transport_time >> berth[id].loading_speed;
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
    }
    for (int i = 0; i < robot_num; i++) {
        int sts;
        cin >> robot[i].goods >> robot[i].x >> robot[i].y >> sts;
    }
    for (int i = 0; i < 5; i++)
        cin >> boat[i].status >> boat[i].pos;
    string okk;
    cin >> okk;
    return id;
}

int main() {
    srand(time(nullptr)); // Seed for random number generation
    Init();
    for (int zhen = 1; zhen <= 15000; zhen++) {
        int id = Input();
        for (int i = 0; i < robot_num; i++)
            cout << "move " << i << " " << rand() % 4 << endl;
        cout << "OK" << endl;
        cout.flush();
    }
    return 0;
}
