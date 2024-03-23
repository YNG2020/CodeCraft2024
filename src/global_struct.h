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

#endif // GLOBAL_STRUCT_H