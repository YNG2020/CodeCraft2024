#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H
// 声明一些影响全局的结构

/// <summary>
/// bot层的移动状态,   
/// </summary>
enum BOT_MOVE_STATE {
	WAITING,		// 等待中(用以可能出现的原地等待情况)
	AVOIDING,		// 碰撞避免中(用以可能出现的碰撞避免情况)
	ARRIVEGOODS,	// 抵达货物所在地
	ARRIVEBERTH,	// 抵达港口所在地
	TOGOODS,		// 奔赴货物所在地
	TOBERTH			// 奔赴港口所在地
};

/// <summary>
/// bot层的目标状态, 
/// </summary>
enum BOT_TARGET_STATE {
	HAVE_TARGET,	// 无目标
	NO_TARGET		// 有目标
};

/// <summary>
/// bot层的寻路状态, 
/// </summary>
enum BOT_PATH_STATE {
	HAVE_PATH,	// 有路径
	NO_PATH		// 无路径
};

#endif // GLOBAL_STRUCT_H

