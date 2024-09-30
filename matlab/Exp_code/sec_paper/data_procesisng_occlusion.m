clc
close all

%% 距离1400 mm
dd1 = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\2024_1_29\1400.txt");
init_point_1 = mean(dd1(1:120,:));

oc_dd1 = dd1(241:840,:);
distances_1 = pdist2(init_point_1,oc_dd1)';
err_1 = abs(100-distances_1);
err_11 = err_1(1:120,:);
err_12 = err_1(121:240,:);
err_13 = err_1(241:360,:);
err_14 = err_1(361:480,:);
err_15 = err_1(481:600,:);

err_11 = rmoutliers(err_11);
err_12 = rmoutliers(err_12);
err_13 = rmoutliers(err_13);
err_14 = rmoutliers(err_14);
err_15 = rmoutliers(err_15);

err_11 = err_11(1:100,:);
err_12 = err_12(1:100,:);
err_13 = err_13(1:100,:);
err_14 = err_14(1:100,:);
err_15 = err_15(1:100,:);

%% 距离1600 mm
dd2 = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\2024_1_29\1600.txt");
init_point_2 = mean(dd2(1:120,:));

oc_dd2 = dd2(121:720,:);
distances_2 = pdist2(init_point_2,oc_dd2)';
err_2 = abs(100-distances_2);
err_21 = err_2(1:120,:);
err_22 = err_2(121:240,:);
err_23 = err_2(241:360,:);
err_24 = err_2(361:480,:);
err_25 = err_2(481:600,:);

err_21 = rmoutliers(err_21);
err_22 = rmoutliers(err_22);
err_23 = rmoutliers(err_23);
err_24 = rmoutliers(err_24);
err_25 = rmoutliers(err_25);

err_21 = err_21(1:100,:);
err_22 = err_22(1:100,:);
err_23 = err_23(1:100,:);
err_24 = err_24(1:100,:);
err_25 = err_25(1:100,:);

%% 距离1800 mm
dd3 = load("C:\Users\Lenovo\Desktop\XXZ\第二篇-CR\【Contact Registration】-V3\实验数据\2024_1_29\1800.txt");
init_point_3 = mean(dd3(1:120,:));

oc_dd3 = dd3(121:720,:);
distances_3 = pdist2(init_point_3,oc_dd3)';
err_3 = abs(100-distances_3);
err_31 = err_3(1:120,:);
err_32 = err_3(121:240,:);
err_33 = err_3(241:360,:);
err_34 = err_3(361:480,:);
err_35 = err_3(481:600,:);

err_31 = rmoutliers(err_31);
err_32 = rmoutliers(err_32);
err_33 = rmoutliers(err_33);
err_34 = rmoutliers(err_34);
err_35 = rmoutliers(err_35);

err_31 = err_31(1:100,:);
err_32 = err_32(1:100,:);
err_33 = err_33(1:100,:);
err_34 = err_34(1:100,:);
err_35 = err_35(1:100,:);
