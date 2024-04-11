#include "decision_maker.h"
#include "global_vars.h"
#include <cstring>

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
int DecisionMaker::boatJamDetect(int boatID1, int boatID2)
{
    if (boatID1 == boatID2)
        return 0;
    if (boat[boatID1].boatFlashState == BOAT_FLASHING || boat[boatID2].boatFlashState == BOAT_FLASHING) // 闪现状态，肯定撞不了
        return 0;
    if ((boat[boatID1].boatStatus != 0 || boat[boatID1].boatPathState == BOAT_NO_PATH || boat[boatID1].boatAvoidState == BOAT_AVOIDED) &&
        (boat[boatID2].boatStatus != 0 || boat[boatID2].boatPathState == BOAT_NO_PATH || boat[boatID2].boatAvoidState == BOAT_AVOIDED)) // 此刻双方都不动
        return 0;
    if (boat[boatID2].boatStatus != 0 || boat[boatID2].boatPathState == BOAT_NO_PATH || boat[boatID2].boatAvoidState == BOAT_AVOIDED)  // 此刻boat[boatID2]停止不动
        if (checkOverLap(boat[boatID1].jamDetectBuffer[1], boat[boatID2].jamDetectBuffer[0]))
            return 1;
        else
            return 0;
    if (boat[boatID1].boatStatus != 0 || boat[boatID1].boatPathState == BOAT_NO_PATH || boat[boatID1].boatAvoidState == BOAT_AVOIDED) // 此刻boat[boatID1]停止不动
		if (checkOverLap(boat[boatID2].jamDetectBuffer[1], boat[boatID1].jamDetectBuffer[0]))
            return 1;
        else
            return 0;

    int jamFlag = 0; // 标识可能发生的堵塞情况
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
            return i + 1;
	}
    return jamFlag;
}

// 对boat1和boat2进行是否可以解除避让状态的检测，默认此刻boatID1正在避让boatID2，即boatID1是Avoided状态，对boat可能出现的停止状态，交给boatJamDetect判断
bool DecisionMaker::boatUnJamDetect(int boatID1, int boatID2)
{
	bool jamFlag = false; // 标识可能发生的堵塞情况
    if (boat[boatID1].pathPoint.size() == 0 || boat[boatID2].pathPoint.size() == 0)
        return true;    // 双方其中有一个没有路，这种情况交给boatJamDetect判断
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
    boatSetPriority(); // 计算每一个船只的移动优先级
    for (int i = 0; i < boatNum; ++i)
    {
        for (int j = i + 1; j < boatNum; ++j)
        {
            int jamPos = boatJamDetect(i, j);
            if (jamPos)
            {                               // 预计会发生碰撞，boat_j去寻找避让点（将考虑boat_i的堵塞检测缓冲区），boat_i则保持原来的轨迹
                int boatID1 = i, boatID2 = j; // 默认boatID1的优先级高于boatID2
                if (boat[boatID1].avoidPriority < boat[boatID2].avoidPriority)
                { // 如果不是默认情况，人为调转序号
                    int tmp = boatID1;
                    boatID1 = boatID2;
                    boatID2 = tmp;
                }
                if (boat[boatID1].boatTarState == BOAT_NO_TARGET && boat[boatID2].boatTarState == BOAT_HAVE_TARGET)
                { // 二者同时为该状态，不用管，只有一个是没目标的话，让没目标的避让有目标的
                    int tmp = boatID1;
                    boatID1 = boatID2;
                    boatID2 = tmp;
                }
                if (boat[boatID2].avoidBoatID == -1 || boat[boatID2].avoidBoatID == boatID1)
                { // boatID2没有避让对象，或避让对象就是boatID1
                    boatJamResolve(boatID1, boatID2, jamPos);
                }
                else
                { // boatID2有避让对象
                    if (boat[boatID1].avoidBoatID == -1)
                        boatJamResolve(boatID2, boatID1, jamPos);
                    else if (boat[boatID1].avoidPriority > boat[boat[boatID2].avoidBoatID].avoidPriority)
                        boatJamResolve(boatID1, boatID2, jamPos);
                    else
                        boatJamResolve(boatID2, boatID1, jamPos);
                }
            }
        }
    }
}

// 堵塞消解，默认是boatID2去寻找避让boatID1的路径，且默认boatID2没有避让对象
void DecisionMaker::boatJamResolve(int boatID1, int boatID2, int jamPos)
{
    bool findPathFlag;
    //if (jamPos + 1 >= boat[boatID1].nearerJamdetectLen)
    //{   // 碰撞发生的区域在nearerJamdetectLen之外，采用找新的路径的方式进行避让
    //    findPathFlag = getBoatDetourPath(boatID1, boatID2); // botID2直接不进行避让动作，而是找路去既定目标
    //    if (findPathFlag)
    //    {
    //        boat[boatID2].avoidBoatID = -1;
    //        boat[boatID2].boatAvoidState = BOAT_NO_AVOIDING;
    //        boat[boatID2].boatPathState = BOAT_HAVE_PATH;
    //        boatRefreshJamBuffer(boatID2); // 修改了路径，需要更新碰撞检测缓冲区
    //        return;
    //    }
    //}

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
            if (boat[boatID2].avoidBoatID == boatID1)
            {   // 防止相互之间循环锁死
                boat[boatID2].boatAvoidState = BOAT_NO_AVOIDING;
                boat[boatID2].boatPathState = BOAT_NO_PATH;
                boat[boatID2].avoidBoatID = -1;
                boat[boatID2].idxInPth = 0;
                vector<int>().swap(boat[boatID2].pathDir);     // 清空
                vector<BoatPoint>().swap(boat[boatID2].pathPoint); // 清空
                boatRefreshJamBuffer(boatID2);                      // 修改了路径，需要更新碰撞检测缓冲区
            }
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
            if (boat[inferior].boatStatus == 0)     // 保险处理
            {
                boat[inferior].boatAvoidState = BOAT_NO_AVOIDING;
                boat[inferior].boatPathState = BOAT_NO_PATH;
                boat[inferior].avoidBoatID = -1;
                boat[inferior].idxInPth = 0;
                vector<int>().swap(boat[inferior].pathDir);     // 清空
                vector<BoatPoint>().swap(boat[inferior].pathPoint); // 清空
                boatRefreshJamBuffer(inferior);                      // 修改了路径，需要更新碰撞检测缓冲区
                boat[inferior].boatFlashState = BOAT_FLASHING;    // 进入闪现状态
                printf("dept %d\n", inferior);
                boat[inferior].boatStatus = 1;      // 手动置为恢复状态
            }
        }
    }
}

// 寻找避让路径，默认是botID2去寻找避让boatID1的路径
bool DecisionMaker::boatGetAvoidPath(int boatID1, int boatID2)
{
    int queueCount = 0;
    int queueIndex = 0;
    int x = boat[boatID2].curX, y = boat[boatID2].curY, dire = boat[boatID2].dire;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr, dire);
    memset(visBoat, 0, sizeof(visBoat));
    visBoat[dire][x][y] = true;
    visBoat[boat[boatID1].dire][boat[boatID1].curX][boat[boatID1].curY] = true; // 不经过要避让的boat此刻所在的位置

    int tmpX, tmpY, tmpDire;
    for (int i = 0; i < boat[boatID1].jamDetectBufferLen - 1; ++i)
    {   // 构建寻路屏障，不让避让路径与boatID1的路径有冲突
        if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], boat[boatID2].jamDetectBuffer[i + 1]))
        { // 二者下一个目标位置会发生碰撞
            tmpX = boat[boatID2].jamDetectBuffer[i + 1].x;
            tmpY = boat[boatID2].jamDetectBuffer[i + 1].y;
            for (tmpDire = 0; tmpDire <= 3; ++tmpDire)
            {   // 如果该位置会发生碰撞，那么四个方向都检测一下有无碰撞风险
                if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], BoatPoint(tmpX, tmpY, tmpDire)))
                    visBoat[tmpDire][tmpX][tmpY] = true;
            }
        }
    }

    bool pointAvailable = true; // 用于标识找到的避让点是否可行

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];

        pointAvailable = true;
        for (int i = boat[boatID1].idxInPth; i < boat[boatID1].pathPoint.size(); ++i)
        { // 检查在boat[boatID1]接下来的全体路径点
            if (checkOverLap(boat[boatID1].pathPoint[i], BoatPoint(now->x, now->y, now->dir)))
            {
                pointAvailable = false;
                break;
            }
        }

        if (pointAvailable)
            for (int i = 0; i < boatNum; ++i) // 检查该避让点是否已经被别人所占据
                if (boat[i].boatAvoidState != BOAT_NO_AVOIDING && i != boatID2)
                    if (checkOverLap(boat[i].pathPoint[boat[i].pathPoint.size() - 1], BoatPoint(now->x, now->y, now->dir)))
                    {
                        pointAvailable = false;
                        break;
                    }
        if (pointAvailable)
        {
            target = now;               // 找到避让点
            boat[boatID2].idxInPth = 0; // 更新路径点序列
            break;
        }

        for (int i = 0; i < 3; i++) // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
        {
            int nx = now->x + dirBoatDx[i][now->dir];
            int ny = now->y + dirBoatDy[i][now->dir];

            int curDir = i == 2 ? now->dir : clockWiseDir[i][now->dir];
            if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                continue;
            visBoat[curDir][nx][ny] = true;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, 0, now, curDir);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    vector<int>().swap(boat[boatID2].pathDir);           // 清空
    vector<BoatPoint>().swap(boat[boatID2].pathPoint);  // 清空
    if (target != nullptr)
    {
        boat[boatID2].pathPoint.push_back(BoatPoint(target->x, target->y, target->dir));
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][p->parent->dir] && p->y == p->parent->y + dirBoatDy[i][p->parent->dir])
                {
                    boat[boatID2].pathDir.push_back(i);
                    boat[boatID2].pathPoint.push_back(BoatPoint(p->parent->x, p->parent->y, p->parent->dir));
                    break;
                }
            }
        }
        reverse(boat[boatID2].pathDir.begin(), boat[boatID2].pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(boat[boatID2].pathPoint.begin(), boat[boatID2].pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
}

// 寻找避让路径，默认是botID2去寻找避让boatID1的路径(Dijkstra)
bool DecisionMaker::getBoatDetourPath(int boatID1, int boatID2)
{
    int queueCount = 0;
    int queueIndex = 0;
    int x = boat[boatID2].curX, y = boat[boatID2].curY, dire = boat[boatID2].dire;
    Node* now = &nodes[queueCount++];
    Node* target = nullptr; // 用于存储找到的目标节点
    Node* child = nullptr;
    now->setNode(x, y, 0, nullptr, dire);
    memset(visBoat, 0, sizeof(visBoat));
    priority_queue<Node> candidate;
    candidate.push(*now);
    visBoat[boat[boatID1].dire][boat[boatID1].curX][boat[boatID1].curY] = true; // 不经过要避让的boat此刻所在的位置

    int tmpX, tmpY, tmpDire;
    for (int i = 0; i < boat[boatID1].jamDetectBufferLen - 1; ++i)
    {   // 构建寻路屏障，不让避让路径与boatID1的路径有冲突
        if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], boat[boatID2].jamDetectBuffer[i + 1]))
        { // 二者下一个目标位置会发生碰撞
            tmpX = boat[boatID2].jamDetectBuffer[i + 1].x;
            tmpY = boat[boatID2].jamDetectBuffer[i + 1].y;
            for (tmpDire = 0; tmpDire <= 3; ++tmpDire)
            {   // 如果该位置会发生碰撞，那么四个方向都检测一下有无碰撞风险
                if (checkOverLap(boat[boatID1].jamDetectBuffer[i + 1], BoatPoint(tmpX, tmpY, tmpDire)))
                    visBoat[tmpDire][tmpX][tmpY] = true;
            }
        }
    }

    bool pointAvailable = true; // 用于标识找到的避让点是否可行

    while (queueCount > queueIndex)
    {
        now = &nodes[queueIndex++];
        *now = candidate.top(); // 取出最短的节点now
        candidate.pop();
        if (visBoat[now->dir][now->x][now->y] == 1) // 如果已经是最短路径集合，跳过
            continue;
        visBoat[now->dir][now->x][now->y] = 1; // 确定为最短路径集合

        if (now->dis + boat[boatID2].idxInPth > boat[boatID2].pathDir.size())
            return false;

        if (now->x == boat[boatID2].tarX && now->y == boat[boatID2].tarY)
        {
            target = now;               // 找到避让点
            boat[boatID2].idxInPth = 0; // 更新路径点序列
            break;
        }

        for (int i = 0; i < 3; i++) // 这里轮船只有三个选择，0顺时针转，1逆时针转，2前进
        {
            int nx = now->x + dirBoatDx[i][now->dir];
            int ny = now->y + dirBoatDy[i][now->dir];

            int curDir = i == 2 ? now->dir : clockWiseDir[i][now->dir];
            if (boatTimeForDifDir[curDir][nx][ny] == 0 || visBoat[curDir][nx][ny])
                continue;
            child = &nodes[queueCount++];
            child->setNode(nx, ny, boatTimeForDifDir[now->dir][now->x][now->y] + now->dis, now, curDir);
            candidate.push(*child);
        }
    }

    if (target == nullptr) // 找不到路直接返回
        return false;

    vector<int>().swap(boat[boatID2].pathDir);           // 清空
    vector<BoatPoint>().swap(boat[boatID2].pathPoint);  // 清空
    if (target != nullptr)
    {
        boat[boatID2].pathPoint.push_back(BoatPoint(target->x, target->y, target->dir));
        // 从目标节点回溯到起始节点，构建路径
        for (Node* p = target; p->parent != nullptr; p = p->parent)
        {
            for (int i = 0; i < 3; i++)
            {
                if (p->x == p->parent->x + dirBoatDx[i][p->parent->dir] && p->y == p->parent->y + dirBoatDy[i][p->parent->dir])
                {
                    boat[boatID2].pathDir.push_back(i);
                    boat[boatID2].pathPoint.push_back(BoatPoint(p->parent->x, p->parent->y, p->parent->dir));
                    break;
                }
            }
        }
        reverse(boat[boatID2].pathDir.begin(), boat[boatID2].pathDir.end());     // 反转路径，使其从起始节点开始
        reverse(boat[boatID2].pathPoint.begin(), boat[boatID2].pathPoint.end()); // 反转路径，使其从起始节点开始
    }
    return true;
}

// 检测是否可以解除堵塞状态
void DecisionMaker::boatUnJam()
{
    for (int i = 0; i < boatNum; ++i)
    {
        if (boat[i].boatAvoidState == BOAT_AVOIDING)
        { // 正处于避让状态
            if (boat[i].idxInPth == boat[i].pathDir.size())
            {
                //bool findPathFlag = getBoatPathBFS(i, boat[i].tarX, boat[i].tarY, boat[i].pathPoint, boat[i].pathDir);
                bool findPathFlag = getBoatPathDijkstra(i, boat[i].tarX, boat[i].tarY, boat[i].pathPoint, boat[i].pathDir);
                if (!findPathFlag)
                {   // 找不到路的话，使用dept指令
                    if (boat[i].boatStatus == 0)     // 保险处理
                    {
                        boat[i].boatAvoidState = BOAT_NO_AVOIDING;
                        boat[i].boatPathState = BOAT_NO_PATH;
                        boat[i].avoidBoatID = -1;
                        boat[i].idxInPth = 0;
                        vector<int>().swap(boat[i].pathDir);     // 清空
                        vector<BoatPoint>().swap(boat[i].pathPoint); // 清空
                        boatRefreshJamBuffer(i);                      // 修改了路径，需要更新碰撞检测缓冲区
                        boat[i].boatFlashState = BOAT_FLASHING;    // 进入闪现状态
                        printf("dept %d\n", i);
                        boat[i].boatStatus = 1;      // 手动置为恢复状态
                    }
                }
                else
                {
                    boat[i].boatAvoidState = BOAT_AVOIDED;
                    boatRefreshJamBuffer(i); // 路径状态有变动，需要更新碰撞检测缓冲区
                }
            }
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
