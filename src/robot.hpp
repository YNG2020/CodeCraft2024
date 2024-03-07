#ifndef ROBOT_HPP
#define ROBOT_HPP

class Robot {
public:
    int x, y, goods;
    int status;
    int mbx, mby;

    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
};

#endif // ROBOT_HPP
