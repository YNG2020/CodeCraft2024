#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H
// ����һЩӰ��ȫ�ֵĽṹ

/// <summary>
/// bot����ƶ�״̬,   
/// </summary>
enum BOT_MOVE_STATE {
	WAITING,		// �ȴ���(���Կ��ܳ��ֵ�ԭ�صȴ����)
	ARRIVEGOODS,	// �ִ�������ڵ�
	ARRIVEBERTH,	// �ִ�ۿ����ڵ�
	TOGOODS,		// �����������ڵ�
	TOBERTH			// ������λ���ڵ�
};

/// <summary>
/// bot��ı���״̬,   
/// </summary>
enum BOT_AVOID_STATE {
	AVOIDING,		// ��ײ������(���Կ��ܳ��ֵ���ײ�������)
	AVOIDED,		// ������ɣ�ֹͣ����
	NO_AVOIDING,	// ����˱���״̬
};

/// <summary>
/// bot���Ŀ��״̬, 
/// </summary>
enum BOT_TARGET_STATE {
	HAVE_TARGET,	// ��Ŀ��
	NO_TARGET		// ��Ŀ��
};

/// <summary>
/// bot���Ѱ·״̬, 
/// </summary>
enum BOT_PATH_STATE {
	HAVE_PATH,	// ��·��
	NO_PATH		// ��·��
};

#endif // GLOBAL_STRUCT_H

