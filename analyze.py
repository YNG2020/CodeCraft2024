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
