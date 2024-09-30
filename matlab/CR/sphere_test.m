function [x,y,z] = sphere_test(sphere_centre, radius)
    % 生成球面generate sphere mesh
    [x,y,z] = sphere;
    % 设置半径 set radius
    x = radius*x;
    y = radius*y;
    z = radius*z;
    % 设置球心 set the center of sphere
    a = sphere_centre(1); 
    b = sphere_centre(2); 
    c = sphere_centre(3);
    x = x+a; 
    y = y+b; 
    z = z+c;
end