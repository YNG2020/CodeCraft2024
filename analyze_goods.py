import pandas as pd

# 读取CSV文件
df = pd.read_csv('pullInfo.csv')

# 定义价值区间
value_bins = [0, 30, 60, float('inf')]
value_labels = ['0-30', '31-60', '>60']

# 根据价值区间对数据进行分组
df['ValueRange'] = pd.cut(df['goodsValue'], bins=value_bins, labels=value_labels, right=False)

# 按照Region和ValueRange进行分组统计数量
output = df.groupby(['goodsRegion', 'ValueRange']).size().unstack(fill_value=0).reset_index()

# 输出到CSV文件
output.to_csv('output1.csv', index=False)

df = pd.read_csv('goodsInfo.csv')
df['ValueRange'] = pd.cut(df['goodsValue'], bins=value_bins, labels=value_labels, right=False)
output = df.groupby(['goodsRegion', 'ValueRange']).size().unstack(fill_value=0).reset_index()
output.to_csv('output2.csv', index=False)

import matplotlib.pyplot as plt

# 读取output1.csv和output2.csv
df1 = pd.read_csv('output1.csv')
df2 = pd.read_csv('output2.csv')

# 计算output1每项数据/output2每项数据
result = df1.set_index('goodsRegion').div(df2.set_index('goodsRegion'))

# 重置索引
result = result.reset_index()

# 绘制图表
result.plot(x='goodsRegion', kind='bar', figsize=(10, 6))
plt.title('Ratio of output1 to output2')
plt.ylabel('Ratio')
plt.xlabel('Goods Region')
plt.xticks(rotation=0)
plt.legend(title='Value Range')
plt.tight_layout()
plt.show()
