clear;clc;close all
%% Colour codes for field
DarkGreen = [0 128 0]/255;
LightGreen = [0 255 0]/255;
%% Edge Coordinates
OuterEdge = [0,0,10400,7400];
InnerEdge = [400,400,9600,6600];
PlayEdge = [700,700,9000,6000];
%% middle Circle
[xcircle, ycircle] = circle(5200,3700,500);
%% Horizontal Line of playground
x1 = linspace(700,9700);
y1 = linspace(3700,3700);
%% Vertical Line of playground
y2 = linspace(700,6700);
x2 = linspace(5200,5200);
%% Defence Area edges
defx0 =linspace(700,1700); defx1 = linspace(1700,1700);  defx2= linspace(1700,700); % 
defx00 =linspace(8700,9700); defx11 = linspace(8700,8700);  defx22= linspace(8700,9700);
defy0 =linspace(2700,2700);defy1 = linspace(2700,4700);  defy2= linspace(4700,4700);
Defx =[ defx0 defx1 defx2]; Defy = [defy0 defy1 defy2];
Defxx = [defx00 defx11 defx22]; Defyy = Defy;
%% Goal
Goalx0 =linspace(700,500); Goalx1 = linspace(500,500);  Goalx2= linspace(500,700);
Goaly0 =linspace(3200,3200); Goaly1 = linspace(3200,4200);  Goaly2= linspace(4200,4200);
Goalx =[ Goalx0 Goalx1 Goalx2]; Goaly = [Goaly0 Goaly1 Goaly2];
Goalx00 =linspace(9700,9900); Goalx11 = linspace(9900,9900);  Goalx22= linspace(9900,9700);
Goalxx =[ Goalx00 Goalx11 Goalx22]; Goalyy = Goaly;

%%
figure(1)
rectangle('Position',OuterEdge,'FaceColor',LightGreen)
hold on
rectangle('Position',InnerEdge,'FaceColor',DarkGreen)
rectangle('Position',PlayEdge,'EdgeColor','w')
plot(xcircle,ycircle,'w')
plot(x1,y1,'w',x2,y2,'w')
plot(Defx,Defy,'w',Defxx,Defyy,'w')
plot(Goalx,Goaly,'r',Goalxx,Goalyy,'r')
xlim([0 10400])
ylim([0 7400])



