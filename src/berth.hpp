#ifndef BERTH_HPP
#define BERTH_HPP

class Berth {
public:
    int x, y;
    int transport_time;
    int loading_speed;

    Berth() {}
    Berth(int newX, int newY, int newTransportTime, int newLoadingSpeed) {
        x = newX;
        y = newY;
        transport_time = newTransportTime;
        loading_speed = newLoadingSpeed;
    }
};

#endif // BERTH_HPP
