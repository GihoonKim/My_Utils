function [p,R,R_d,ey_look,ey_look_d,ey,d_]= fcn(h,be,map,x,y,look_ahead,ay)
%#codegen

look_ahead_d = 0.5;
coder.extrinsic('polyfit');
coder.extrinsic('delete');

p = zeros(4,1);
angle = h+be;
rot = [cos(-angle),-sin(-angle),0;sin(-angle),cos(-angle),0;0,0,1];
trans = [1 0 -x;0 1 -y;0 0 1];

transition_mat = rot*trans;

% translate map according to vehicle coordinate
map_trans = transition_mat*map';

[~,index] = min(sqrt(map_trans(1,1:300).^2+map_trans(2,1:300).^2));

plot_data = [map_trans(1,index:index+110);map_trans(2,index:index+110)];

%Fitting line as polynomial formation
p = polyfit(plot_data(1,:),plot_data(2,:),3);

RR = 6*p(1)*look_ahead+2*p(2);
RR_d = 6*p(1)*look_ahead_d+2*p(2);
R=1/RR;
R_d = 1/RR_d;
ey_look = -(p(1)*look_ahead^3+p(2)*look_ahead^2+p(3)*look_ahead+p(4));
ey = -p(4);

ey_look_d = -(p(1)*look_ahead_d^3+p(2)*look_ahead_d^2+p(3)*look_ahead_d+p(4));

kv = 0;%1530*0.58/150000;
    
d__ = -atan((2*2.78*(ey_look))/(look_ahead^2+(ey_look)^2));
% d = d__*180/pi*16;
d_ = (d__+kv*ay)*180/pi*16;

end