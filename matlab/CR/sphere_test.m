function [x,y,z] = sphere_test(sphere_centre, radius)
    % ��������generate sphere mesh
    [x,y,z] = sphere;
    % ���ð뾶 set radius
    x = radius*x;
    y = radius*y;
    z = radius*z;
    % �������� set the center of sphere
    a = sphere_centre(1); 
    b = sphere_centre(2); 
    c = sphere_centre(3);
    x = x+a; 
    y = y+b; 
    z = z+c;
end