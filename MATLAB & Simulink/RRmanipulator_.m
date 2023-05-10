clear
close all
clc
%% Robot Parameters.
N = 2; %Links.
L = [1 1]; %Links long.
m = [1 1]; %Links weight.

%% DH Parameters.
DH = struct('alpha', cell(1,N), 'a', cell(1,N), 'theta', cell(1,N), 'd', cell(1,N),...
    'type', cell(1,N)); %Structure.
DH(1).alpha = 0; DH(1).a = 0; DH(1).d = 0; DH(1).type = 'R';
DH(2).alpha = 0; DH(2).a = L(1); DH(2).d = 0; DH(2).type = 'R';

%% Links Creation.
links = {0};
for  i = 1:N
        links{i} = Link('alpha', DH(i).alpha, 'a', DH(i).a, 'd', ...
            DH(i).d, 'modified', 'm', m(i), 'r', [L(i),0,0], 'G', 1, 'Tc', 1);%'Tc', [1 1]); % Links vector.
        %links{i}.dyn();
end

%% Tool.
tool = transl([L(2), 0, 0]); % Tool offset.

%% Robot Creation.
RRbot = SerialLink([links{:}], 'tool', tool, 'name', 'RRbot');
RRbot.dyn();
RRbotD = RRbot.perturb(0.8); %Disturbance.

%% Robot Teach.
q0=[0 0]; %Inintial angle.
figure();
RRbot.teach(q0);
hold on;
patch([2 0 0 2] , [0 2 2 0], [5 5 -5 -5], 'green');

%% Gains.
g1 = 1;
g2 = 5;
Kp = [g1 0; 0 g2];
Kv = [2*sqrt(g1) 0; 0 2*sqrt(g2)];

g1f = 5e5;
g2f = 5;%10;%0.5;
Kpf = [g1f 0; 0 g2f];
Kvf = [2*sqrt(g1f) 0; 0 2*sqrt(g2f)];
Kpf_ = g1f;
Kvf_ = 2*sqrt(g1f);

Ke = [1e6 0; 0 0];
ke = 1e6;
ike = ke^(-1);

S = [0 0; 0 1];%[1 0; 0 1];
S_ = [1 0; 0 0];

R = [cos(-pi/4) -sin(-pi/4);
    sin(-pi/4) cos(-pi/4)];

RT = R';
