#include "decision_maker.h"
#include "global_vars.h"

// 更新boat的避让优先级
void DecisionMaker::boatSetPriority()
{
    for (int i = 0; i < boatNum; ++i)
    {
        // boat[i].avoidPriority = i;
        if (boat[i].numBoatGoods == 0)
            boat[i].avoidPriority = i;
        else
            boat[i].avoidPriority = boatNum + boat[i].numBoatGoods;
    }
}

// 更新堵塞检测缓冲区
void DecisionMaker::boatRefreshJamBuffer(int boatID)
{
    Boat& bot = boat[boatID];
    int i = 0;
    if (bot.boatPathState == BOAT_HAVE_PATH)
    { // 有路，可以正常更新缓冲区
        for (; i < bot.jamDetectBufferLen && (bot.idxInPth + i) < bot.pathPoint.size(); ++i)
        {
            bot.jamDetectBuffer[i] = bot.pathPoint[bot.idxInPth + i];
        }
        for (; i < bot.jamDetectBufferLen; ++i)
        {
            bot.jamDetectBuffer[i] = bot.jamDetectBuffer[i - 1];
        }
    }
    else
    { // 没路，缓冲区全都存储为自己本身所在的位置
        for (; i < bot.jamDetectBufferLen; ++i)
        {
            bot.jamDetectBuffer[i] = BoatPoint(bot.curX, bot.curY, bot.dire);
        }
    }
}

// 对每对boat进行堵塞检测
bool DecisionMaker::boatJamDetect(int boatID1, int boatID2)
{
    if (boatID1 == boatID2)
        return false;
    if (boat[boatID1].boatStatus != 0 || boat[boatID2].boatStatus != 0) // 只要双方其中一个不是移动状态，保证不撞（装货时船只必在主航道，恢复时船只相当于消失）
        return false;
    if ((boat[boatID1].boatPathState == BOAT_NO_PATH || boat[boatID1].boatAvoidState == BOAT_AVOIDED) &&
        (boat[boatID2].boatPathState == BOAT_NO_PATH || boat[boatID2].boatAvoidState == BOAT_AVOIDED)) // 此刻双方都不动
        return false;
    if (boat[boatID2].boatPathState == BOAT_NO_PATH || boat[boatID2].boatAvoidState == BOAT_AVOIDED)  // 此刻boat[boatID2]停止不动
        if (checkOverLap(boat[boatID1].jamDetectBuffer[1], boat[boatID2].jamDetectBuffer[0]))
            return true;
        else
            return false;
    if (boat[boatID1].boatPathState == BOAT_NO_PATH || boat[boatID1].boatAvoidState == BOAT_AVOIDED) // 此刻boat[boatID1]停止不动
		if (checkOverLap(boat[boatID2].jamDetectBuffer[1], boat[boatID1].jamDetectBuffer[0]))
            return true;
        else
            return false;

    bool jamFlag = false; // 标识可能发生的堵塞情况
	for (int i = 0; i < boat[boatID1].jamDetectBufferLen - 1; ++i)
	{
		if (boat[boatID2].jamDetectBuffer[i + 1] == boat[boatID2].jamDetectBuffer[i])
			if (boat[boatID2].jamDetectBuffer[i + 1] == boat[boatID2].pathPoint[boat[boatID2].pathPoint.size() - 1])
				break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
		if (boat[boatID1].jamDetectBuffer[i + 1] == boat[boatID1].jamDetectBuffer[i])
			if (boat[boatID1].jamDetectBuffer[i + 1] == boat[boatID1].pathPoint[boat[boatID1].pathPoint.size() - 1])
				break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
		// 只需逐对检测是否有可能发生冲突即可，应该不用考虑对撞的情况，因为船无论怎么前进和旋转，都不能变动一个身位
		if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], boat[boatID2].jamDetectBuffer[i + 1])) // 二者在下一个目标位置会发生碰撞
			jamFlag = true;
	}
    return jamFlag;
}

// 对boat1和boat2进行是否可以解除避让状态的检测，默认此刻boatID1正在避让boatID2，即boatID1是Avoided状态，对boat可能出现的停止状态，交给boatJamDetect判断
bool DecisionMaker::boatUnJamDetect(int boatID1, int boatID2)
{
	bool jamFlag = false; // 标识可能发生的堵塞情况
	for (int i = 0; i < boat[boatID1].jamDetectBufferLen - 1; ++i)
	{
		if (boat[boatID2].jamDetectBuffer[i + 1] == boat[boatID2].jamDetectBuffer[i])
			if (boat[boatID2].jamDetectBuffer[i + 1] == boat[boatID2].pathPoint[boat[boatID2].pathPoint.size() - 1])
				break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
		if (boat[boatID1].jamDetectBuffer[i + 1] == boat[boatID1].jamDetectBuffer[i])
			if (boat[boatID1].jamDetectBuffer[i + 1] == boat[boatID1].pathPoint[boat[boatID1].pathPoint.size() - 1])
				break; // 这是路径检测缓冲区的有效长度，此刻与之后无须检测
		// 只需逐对检测是否有可能发生冲突即可，应该不用考虑对撞的情况，因为船无论怎么前进和旋转，都不能变动一个身位
		if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], boat[boatID2].jamDetectBuffer[i + 1])) // 二者在下一个目标位置会发生碰撞
			jamFlag = true;
	}
	return jamFlag;
}

// 堵塞控制
void DecisionMaker::boatJamControl()
{
    boatUnJam();
    boatSetPriority(); // 计算每一个机器人的移动优先级
    for (int i = 0; i < berthNum; ++i)
    {
        for (int j = i + 1; j < berthNum; ++j)
        {
            if (boatJamDetect(i, j))
            {                               // 预计会发生碰撞，boat_j去寻找避让点（将考虑boat_i的堵塞检测缓冲区），boat_i则保持原来的轨迹
                int boatID1 = i, boatID2 = j; // 默认boatID1的优先级高于boatID2
                if (boat[boatID1].avoidPriority < boat[boatID2].avoidPriority)
                { // 如果不是默认情况，人为调转序号
                    int tmp = boatID1;
                    boatID1 = boatID2;
                    boatID2 = tmp;
                }
                if (boat[boatID1].boatPathState == BOAT_NO_PATH)
                { // 不可能二者同为该状态，否则在boatJamDetect的时候就pass掉，此时让没路的避让有路的
                    int tmp = boatID1;
                    boatID1 = boatID2;
                    boatID2 = tmp;
                }
                if (boat[boatID2].avoidBoatID == -1 || boat[boatID2].avoidBoatID == boatID1)
                { // boatID2没有避让对象，或避让对象就是boatID1
                    boatJamResolve(boatID1, boatID2);
                }
                else
                { // boatID2有避让对象
                    if (boat[boatID1].avoidBoatID == -1)
                        boatJamResolve(boatID2, boatID1);
                    else if (boat[boatID1].avoidPriority > boat[boat[boatID2].avoidBoatID].avoidPriority)
                        boatJamResolve(boatID1, boatID2);
                    else
                        boatJamResolve(boatID2, boatID1);
                }
            }
        }
    }
}

// 堵塞消解，默认是boatID2去寻找避让boatID1的路径，且默认boatID2没有避让对象
void DecisionMaker::boatJamResolve(int boatID1, int boatID2)
{
    bool findPathFlag;

    findPathFlag = boatGetAvoidPath(boatID1, boatID2);
    if (findPathFlag)
    {   // 成功找到路，及时更新状态变量
        boat[boatID2].avoidBoatID = boatID1;
        boat[boatID2].boatAvoidState = BOAT_AVOIDING;
        boat[boatID2].boatPathState = BOAT_HAVE_PATH;
        boat[boatID2].tmpTarX = boat[boatID2].pathPoint[boat[boatID2].pathPoint.size() - 1].x;
        boat[boatID2].tmpTarY = boat[boatID2].pathPoint[boat[boatID2].pathPoint.size() - 1].y;
        boatRefreshJamBuffer(boatID2); // 修改了路径，需要更新碰撞检测缓冲区
    }
    else
    {   // 调转优先级，再寻一次路
        findPathFlag = boatGetAvoidPath(boatID2, boatID1);
        if (findPathFlag)
        {
            boat[boatID1].avoidBoatID = boatID2;
            boat[boatID1].boatAvoidState = BOAT_AVOIDING;
            boat[boatID1].boatPathState = BOAT_HAVE_PATH;
            boat[boatID1].tmpTarX = boat[boatID1].pathPoint[boat[boatID1].pathPoint.size() - 1].x;
            boat[boatID1].tmpTarY = boat[boatID1].pathPoint[boat[boatID1].pathPoint.size() - 1].y;
            boatRefreshJamBuffer(boatID1); // 修改了路径，需要更新碰撞检测缓冲区
        }
        else
        {   // 还是找不到路，让低优先级的执行dept指令，同时手动置状态，但不放弃原有目标
            int superior = boatID1, inferior = boatID2;
            if (boat[superior].avoidPriority < boat[inferior].avoidPriority)
            {   // 如果不是预设情况，交换索引
                int tmp = superior;
                superior = inferior;
                inferior = tmp;
            }
            boat[inferior].boatAvoidState = BOAT_NO_AVOIDING;
            boat[inferior].boatPathState = BOAT_NO_PATH;
            boat[inferior].idxInPth = 0;
            vector<int>().swap(boat[inferior].pathDir);     // 清空
            vector<BoatPoint>().swap(boat[inferior].pathPoint); // 清空
            boatRefreshJamBuffer(inferior);                      // 修改了路径，需要更新碰撞检测缓冲区
            printf("dept %d\n", inferior);
            boat[inferior].boatStatus = 1;      // 手动置为恢复状态（碰撞检测会用到）
        }
    }
}

// 寻找避让路径，默认是botID2去寻找避让boatID1的路径
bool DecisionMaker::boatGetAvoidPath(int boatID1, int boatID2)
{
    return true;
}

// 检测是否可以解除堵塞状态
void DecisionMaker::boatUnJam()
{
    for (int i = 0; i < berthNum; ++i)
    {
        if (boat[i].boatAvoidState == AVOIDING)
        { // 正处于避让状态,TODO
            if (boat[i].idxInPth == boat[i].pathDir.size() - 1)
            {
                boat[i].boatAvoidState = BOAT_AVOIDED;
                bool findPathFlag = getToTarPath(i, true);
                if (!findPathFlag)
                { // 应该只用处理找不到的情况，找到路的话，状态变量似乎没有什么需要特地更新的
                    // 还是找不到路，则在当前帧不动，且放弃当前的目标货物（如果有），在下一帧中寻找新的目标，直到能找到为止
                    boat[i].boatPathState = BOAT_NO_PATH;
                    vector<int>().swap(boat[i].pathDir);     // 清空
                    vector<BoatPoint>().swap(boat[i].pathPoint); // 清空
                    boat[i].idxInPth = 0;
                }
                boatRefreshJamBuffer(i); // 修改了路径，需要更新碰撞检测缓冲区
            }
            else
                continue;
        }
        if (boat[i].boatAvoidState == BOAT_AVOIDED)
        { // 正处于避让结束，原地等待的状态
            if (!boatUnJamDetect(i, boat[i].avoidBoatID))
            {
                boat[i].boatAvoidState = BOAT_NO_AVOIDING;
                boat[i].avoidBoatID = -1;
            }
        }
    }
}

// 检测两条船是否会在主航道以外的区域发生重叠
bool DecisionMaker::checkOverLap(const BoatPoint& boat1, const BoatPoint& boat2)
{
	int minX1, minY1, minX2, minY2, maxX1, maxY1, maxX2, maxY2;
	switch (boat1.dire)
	{
	case 0:	// 右
		minX1 = boat1.x; minY1 = boat1.y;
		maxX1 = boat1.x + 1; maxY1 = boat1.y + 2;
		break;
	case 1:	// 左
		minX1 = boat1.x - 1; minY1 = boat1.y - 2;
		maxX1 = boat1.x; maxY1 = boat1.y;
		break;
	case 2:	// 上
		minX1 = boat1.x - 2; minY1 = boat1.y;
		maxX1 = boat1.x; maxY1 = boat1.y + 1;
		break;
	case 3:	// 下
		minX1 = boat1.x; minY1 = boat1.y - 1;
		maxX1 = boat1.x + 2; maxY1 = boat1.y;
		break;
	default:
		minX1 = 0; minY1 = 0;
		maxX1 = 0; maxY1 = 0;
		break;
	}
	switch (boat2.dire)
	{
	case 0:	// 右
		minX2 = boat2.x; minY2 = boat2.y;
		maxX2 = boat2.x + 1; maxY2 = boat2.y + 2;
		break;
	case 1:	// 左
		minX2 = boat2.x - 1; minY2 = boat2.y - 2;
		maxX2 = boat2.x; maxY2 = boat2.y;
		break;
	case 2:	// 上
		minX2 = boat2.x - 2; minY2 = boat2.y;
		maxX2 = boat2.x; maxY2 = boat2.y + 1;
		break;
	case 3:	// 下
		minX2 = boat2.x; minY2 = boat2.y - 1;
		maxX2 = boat2.x + 2; maxY2 = boat2.y;
		break;
	default:
		minX2 = 0; minY2 = 0;
		maxX2 = 0; maxY2 = 0;
		break;
	}
	bool horizontalOverlap = (minX1 <= maxX2) && (maxX1 >= minX2);
	bool verticalOverlap = (minY1 <= maxY2) && (maxY1 >= minY2);

	if (horizontalOverlap && verticalOverlap)
	{	// 有重叠区域
		int intersectionMinX = std::max(minX1, minX2);
		int intersectionMaxX = std::min(maxX1, maxX2);
		int intersectionMinY = std::max(minY1, minY2);
		int intersectionMaxY = std::min(maxY1, maxY2);
		for (int inX = intersectionMinX; inX <= intersectionMaxX; ++inX)
		{	// 检验重叠区域中，是否有不是主航道的区域，有的话直接返回true
			for (int inY = intersectionMinY; inY <= intersectionMaxY; ++inY)
			{
				if (gridMap[inX][inY] == WATER || gridMap[inX][inY] == MIX)
				{	// 海洋区域只有海洋和海陆立体交通地块不是主航道
					return true;
				}
			}
		}
	}
	return false;
}
