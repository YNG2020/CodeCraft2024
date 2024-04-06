close all
frame = 1 : 15000;
numColor = 3;

% 读取 .csv 文件
goods_num_inBerth = csvread('goods_num_inBerth.csv');
goods_totVal_inBerth = csvread('goods_totVal_inBerth.csv');
robot_num_inBerth = csvread('robot_num_inBerth.csv');
berthNum = size(goodsRegionInfo, 1);

figure("Name", "goods_num_inBerth")
hold on
for i = 1 : berthNum
    plot(frame, goods_num_inBerth(i, :), '-', 'DisplayName', "berth" + num2str(i-1))
end
legend

RGB_label = label2rgb(1 : numColor,'hsv','w','shuffle');

colors = {'#F00','#F80','#FF0','#0B0','#00F','#50F','#A0F'};

for i = 1 : berthNum
    figure("Name", "goods_totVal_inBerth" + num2str(i-1))
    hold on
    yyaxis left;
    plot(frame, goods_totVal_inBerth(i, :), '-', 'color', RGB_label(1, 1, :), 'DisplayName', "berth" + num2str(i-1))
    ylabel({"泊位" + num2str(i-1) + "对应区域总金额"}, 'fontsize', 17);
    yyaxis right;
    plot(frame, robot_num_inBerth(i, :), ':', 'linewidth', 1.5, 'color', RGB_label(1, numColor, :), 'DisplayName', "berth" + num2str(i-1))
    ylabel({"为泊位" + num2str(i-1) + "服务的机器人数量"}, 'fontsize', 17);
    xticks(0 : 1000 : 15000)
    xlabel({"帧"}, 'fontsize', 17);
end
legend
% 
% figure("Name", "robot_num_inBerth")
% hold on
% for i = 1 : berthNum
%     plot(frame, robot_num_inBerth(i, :) + (i-1) * 2, '.', 'DisplayName', "berth" + num2str(i-1))
% end
% legend
