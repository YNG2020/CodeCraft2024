#ifndef CONSTANTS_H
#define CONSTANTS_H

const int MAP_SIZE = 200;
const int N = 210;
const int BOT_EXRECOVER_DIST = 25; // 将处于恢复状态的机器人排除在路径搜索范围的最小距离
const int GOODS_MAX_VALUE = 200;
const int ROBOT_JAM_BUFFER_SIZE = 2;
const int ROBOT_NEARER_JAM_BUFFER_SIZE = 2;	// 采取避让-等待方式进行避让的检测长度
const int BOAT_JAM_BUFFER_SIZE = 4;
const int BOAT_NEARER_JAM_BUFFER_SIZE = 4;	// 采取避让-等待方式进行避让的检测长度
const int BOAT_JAM_TOLERANCE_TIME = 2;	// 堵塞容忍时长

// 用于robot, boat, berth申请数组的临时变量
const int TEMP_ROBOT_NUM = 20; 
const int TEMP_BOAT_NUM = 20;
const int TEMP_BERTH_NUM = 20;

#endif // CONSTANTS_H