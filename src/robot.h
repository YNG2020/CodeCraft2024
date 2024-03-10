#ifndef ROBOT_H
#define ROBOT_H

class Robot {
public:
    int x, y, goods;
    int status;
    int mbx, mby;
    int nextX, nextY, nextDir;

    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
};

#endif // ROBOT_H
