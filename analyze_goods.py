import pandas as pd
import matplotlib.pyplot as plt

# 定义价值区间
value_bins = [0, 30, 60, float('inf')]
value_labels = ['0-30', '31-60', '>60']

# 读取并处理第一个CSV文件
df1 = pd.read_csv('pullInfo.csv')
df1['ValueRange'] = pd.cut(df1['goodsValue'], bins=value_bins, labels=value_labels, right=False)
output1 = df1.groupby(['goodsRegion', 'ValueRange']).size().unstack(fill_value=0)
sum_output1 = df1.groupby(['goodsRegion', 'ValueRange'])['goodsValue'].sum().unstack(fill_value=0)

# 读取并处理第二个CSV文件
df2 = pd.read_csv('goodsInfo.csv')
df2['ValueRange'] = pd.cut(df2['goodsValue'], bins=value_bins, labels=value_labels, right=False)
output2 = df2.groupby(['goodsRegion', 'ValueRange']).size().unstack(fill_value=0)
sum_output2 = df2.groupby(['goodsRegion', 'ValueRange'])['goodsValue'].sum().unstack(fill_value=0)

# 计算两个输出的比例
result = output1.div(output2)

# 绘制比例图表
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
result.plot(kind='bar', ax=plt.gca())
plt.title('Ratio of pullInfo to goodsInfo')
plt.ylabel('Ratio')
plt.xlabel('Goods Region')
plt.xticks(rotation=0)
plt.legend(title='Value Range')

# 计算两个输出的goodsValue之和的差距
difference = sum_output1.sub(sum_output2).abs()

# 绘制差距的条形图
plt.subplot(1, 2, 2)
difference.plot(kind='bar', ax=plt.gca())
plt.title('Difference in goodsValue Sum between pullInfo and goodsInfo')
plt.ylabel('Difference in Sum of goodsValue')
plt.xlabel('Goods Region')
plt.xticks(rotation=0)
plt.legend(title='Value Range')

plt.tight_layout()
plt.show()
