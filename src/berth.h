#ifndef BERTH_H
#define BERTH_H

class Berth
{
public:
    int x, y;           // ��λ�����Ͻǵ�λ��
    int transportTime;  // �ִ����䵽������ʱ��
    int loadingSpeed;   // ÿ֡����װ�صĻ�����
    int numBerthGoods;  // ��λ��ǰӵ�еĻ�����
    int boatIDInBerth;  // ��ǰͣ���ڸò�λ�Ĵ���ID
    int boatIDToBerth;  // ��ǰĿ�겴λΪ�ò�λ�Ĵ���ID
    double timeOfGoodsToBerth;   // ������robot�ѻ������͵�berth��ʱ�䣨֡/��������Ҫ��̬ά��
    int lastTimeGetGoods;

    int load(int boatCapacityRemain)
    {
        int loadNum = numBerthGoods > loadingSpeed ? loadingSpeed : numBerthGoods;
        loadNum = loadNum > boatCapacityRemain ? boatCapacityRemain : loadNum;
        numBerthGoods -= loadNum;
        return loadNum;
    }

    Berth() {}
    Berth(int newX, int newY, int newTransportTime, int newLoadingSpeed)
    {
        x = newX;
        y = newY;
        transportTime = newTransportTime;
        loadingSpeed = newLoadingSpeed;
        numBerthGoods = 0;
    }
};

#endif // BERTH_H
