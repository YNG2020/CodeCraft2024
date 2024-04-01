#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H
// 声明一些影响全局的结构

/// <summary>
/// bot层的移动状态,
/// </summary>
enum BOT_MOVE_STATE
{
	WAITING,	 // 手动更新为原地等待的状态（等路径分配）
	ARRIVEGOODS, // 抵达货物所在地
	ARRIVEBERTH, // 抵达港口所在地
	TOGOODS,	 // 奔赴货物所在地
	TOBERTH		 // 奔赴泊位所在地
};

/// <summary>
/// bot层的避让状态,
/// </summary>
enum BOT_AVOID_STATE
{
	AVOIDING,	 // 碰撞避免中
	AVOIDED,	 // 避让完成，停止不动
	NO_AVOIDING, // 解除了避让状态
};

/// <summary>
/// bot层的目标状态,
/// </summary>
enum BOT_TARGET_STATE
{
	HAVE_TARGET, // 无目标
	NO_TARGET	 // 有目标
};

/// <summary>
/// bot层的寻路状态,
/// </summary>
enum BOT_PATH_STATE
{
	HAVE_PATH, // 有路径
	NO_PATH	   // 无路径
};

class SimplePoint
{
public:
    SimplePoint(int _x, int _y) :x(_x), y(_y) {}
    SimplePoint() :x(0), y(0) {}
    SimplePoint(const SimplePoint& point) {
        x = point.x;
        y = point.y;
    }
    int x, y;
};

class singleGoodsInfo
{
public:
	double propotion;	// 把货物运到最近泊位的性价比，默认robot也是从最近的泊位出发
	int goodsVal;		// 货物价值
	int doubleDis;		// 从泊位出发，运送货物的往返距离之和
	int x, y;			// 货物坐标
	singleGoodsInfo(int newGoodsVal, int newDoubleDis, int xx, int yy)
	{
		goodsVal = newGoodsVal;
		doubleDis = newDoubleDis;
		propotion = double(newGoodsVal) / newDoubleDis;
		x = xx;
		y = yy;
	}
};

#endif // GLOBAL_STRUCT_H