#ifndef BOAT_H
#define BOAT_H

class Boat
{
public:
    int numBoatGoods;   // ��Ŀǰװ�صĻ�����
    int tarPos;     // ����Ŀ�겴λ�����Ŀ�겴λ������㣬��Ϊ-1��ϵͳά����
    int tarPosManual;   // ����Ŀ�겴λ�����Ŀ�겴λ������㣬��Ϊ-1���ֶ�ά����
    int boatStatus; // 0��ʾ�ƶ������䣩�У�1 ��ʾ��������״̬(��װ��״̬���������״̬)��2��ʾ��λ��ȴ�״̬
    int capacity;   // �������װ����
    Boat() {}
};

#endif // BOAT_H
