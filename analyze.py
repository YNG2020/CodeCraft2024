import pandas as pd
import matplotlib.pyplot as plt

# 从CSV文件读取数据
df = pd.read_csv('data.csv', sep=',', skipinitialspace=True)

# 绘制图表
plt.plot(df['goods_num'], label='Goods Number')
plt.plot(df['pick_goods_num'], label='Pick Goods Number')
plt.plot(df['ship_goods_num'], label='Ship Goods Number')

# 添加图例
plt.legend()

plt.title('Goods Statistics')
plt.xlabel('Time')
plt.ylabel('Number')

plt.show()

# for goodsInfo.csv
# import pandas as pd
# import matplotlib.pyplot as plt

# # 读取CSV文件
# df = pd.read_csv('goodsInfo.csv')

# # 统计每个参数的商品数量
# goods_value_count = df['goodsValue'].value_counts()
# goods_region_count = df['goodsRegion'].value_counts()
# frame_count = df['Frame'].value_counts()

# # 绘制统计图
# plt.figure(figsize=(12, 8))

# # 商品价值统计图
# plt.subplot(3, 1, 1)
# goods_value_count.plot(kind='bar')
# plt.title('Goods Value Count')
# plt.xlabel('Goods Value')
# plt.ylabel('Count')

# # 商品区域统计图
# plt.subplot(3, 1, 2)
# goods_region_count.plot(kind='bar')
# plt.title('Goods Region Count')
# plt.xlabel('Goods Region')
# plt.ylabel('Count')

# # 帧数统计图
# plt.subplot(3, 1, 3)
# frame_count.plot(kind='bar')
# plt.title('Frame Count')
# plt.xlabel('Frame')
# plt.ylabel('Count')

# plt.tight_layout()
# plt.show()
