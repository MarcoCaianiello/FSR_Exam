%%Circle Obstacle
function circle = circle_obstacle (radius, center)
    t = 0:0.01:2*pi;
    
    x = radius*cos(t);
    y = radius*sin(t);
    
    x = x+center(1);
    y = y+center(2);
    
    circle = polyshape(x,y);
end