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
patch( [2 0 0 2] , [0 2 2 0], [5 5 -5 -5], 'green')
%patch( [1 -1 -1 1] , [0 0 0 0], [1 1 -1 -1], [1 1 -1 -1])

%% Robot Animation.
tmax = 1; %time max.
ts = 0.05; %Time step.

%Joint space.
qi = [-pi/2 pi/2];
qf = [-pi/3 2*pi/3];
t = 0:ts:tmax;
q = jtraj(qi, qf, t);
%q = mtraj(@lspb, qi, qf, 20);
%q = mtraj(@tpoly, qi, qf, 20);
x = RRbot.fkine(q);

%Cartesian space
Ti = transl(1, -1, 0);
Tf = transl(1, 1, 0);
T = ctraj(Ti, Tf, 21);
q = RRbot.ikine(T,'q0', [-pi/2 pi/2], 'mask',[1 1 0 0 0 0]);
figure;
plot(q(:,1))
hold on;
plot(q(:,2))
x = RRbot.fkine(q).transl;
x = x(:,1:N);
C2 = (x(:,1).^2+x(:,2).^2-L(1)^2-L(2)^2)/(2*prod(L));
S2 = sqrt(1-C2.^2);
q = [atan2(x(:,2), x(:,1))-atan2(L(2)*S2, L(1)+L(2)*C2) atan2(S2, C2)];
plot(q(:,1))
hold on;
plot(q(:,2))
% x = RRbot.fkine([1 2]).transl';
% x = x(1:N);
% size(T)
% xi = [1 -1];
% xf = [1 1];
% t = 0:ts:tmax;
% x = jtraj(xi, xf, t)
% size(x)

%% Robot Dynamics.
%Joint space
q = [1; 1];
qd = [2; -3];
qdd = [0.5; 0.5];

M = RRbot.inertia(q');
Mqdd = [M(1,1)*qdd(1)+M(1,2)*qdd(2); M(2,1)*qdd(1)+M(2,2)*qdd(2)];
C = RRbot.coriolis(q', qd');
Cqd = [C(1,1)*qd(1)+C(1,2)*qd(2); C(2,1)*qd(1)+C(2,2)*qd(2)]; %V = Cqd
F = -RRbot.friction(qd')';

tau = Mqdd + Cqd + F
tau = RRbot.rne(q', qd', qdd')

%Cartesian space
x = [1 -1];
xdd = [0.5; 0.5];

J = RRbot.jacob0(q');
J = J(1:N,1:N)
%J(1:N,1:N)'
iJ = J^(-1);
iJT = J.'^(-1);
%iJ.'

%Form 1.
dJqd = RRbot.jacob_dot(q', qd');
dJqd = dJqd(1:N)

%Form 2.
dJ = [-(sum(qd))*cos(sum(q))*L(2)-qd(1)*cos(q(1))*L(1) -(sum(qd))*cos(sum(q))*L(2);
     -(sum(qd))*sin(sum(q))*L(2)-qd(1)*sin(q(1))*L(1) -(sum(qd))*sin(sum(q))*L(2)];
dJqd = [dJ(1,1)*qd(1)+dJ(1,2)*qd(2); dJ(2,1)*qd(1)+dJ(2,2)*qd(2)]

V = Cqd;
Mx = iJT*M.*iJ;
Mxxdd = [Mx(1,1)*xdd(1)+Mx(1,2)*xdd(2); Mx(2,1)*xdd(1)+Mx(2,2)*xdd(2)];
Vx = M*iJ;
Vx = V - [Vx(1,1)*dJqd(1)+Vx(1,2)*dJqd(2); Vx(2,1)*dJqd(1)+Vx(2,2)*dJqd(2)];
Vx = [iJT(1,1)*Vx(1)+iJT(1,2)*Vx(2); iJT(2,1)*Vx(1)+iJT(2,2)*Vx(2)];
Fx = [iJT(1,1)*F(1)+iJT(1,2)*F(2); iJT(2,1)*F(1)+iJT(2,2)*F(2)];
taux = Mxxdd + Vx + Fx

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

%% Matrices.
J = [-sin(sum(q))*L(2)-sin(q(1))*L(1) -sin(sum(q))*L(2);
    cos(sum(q))*L(2)+cos(q(1))*L(1) cos(sum(q))*L(2)];

dJ = [-(sum(qd))*cos(sum(q))*L(2)-qd(1)*cos(q(1))*L(1) -(sum(qd))*cos(sum(q))*L(2);
    -(sum(qd))*sin(sum(q))*L(2)-qd(1)*sin(q(1))*L(1) -(sum(qd))*sin(sum(q))*L(2)];

M = [L(2)*m(2)*(L(2)+cos(q(2))*L(1))+m(1)*L(1)^2+L(1)*m(2)*(L(1)+cos(q(2))*L(2)) L(2)^2*m(2)+L(1)*m(2)*L(2)*cos(q(2));
    L(2)*m(2)*(L(2)+cos(q(2))*L(1)) L(2)^2*m(2)];

V = [-2*m(2)*L(1)*L(2)*sin(q(2))*prod(qd)-m(2)*L(1)*L(2)*sin(q(2))*qd(2)^2;
    m(2)*L(1)*L(2)*sin(q(2))*qd(1)^2];

G = [0;
    0];

F = [sign(qd(1));
    sign(qd(2))];

C2 = (x(1)^2+x(2)^2-L(1)^2-L(2)^2)/(2*prod(L));
S2 = sqrt(1-C2^2);
q = [atan2(x(2), x(1))-atan2(L(2)*S2, L(1)+L(2)*C2); atan2(S2, C2)]
T = transl(1, -1, 0);
q = RRbot.ikine(T,'q0', [-pi/2 pi/2], 'mask',[1 1 0 0 0 0])

Mqdd = [M(1,1)*qdd(1)+M(1,2)*qdd(2); M(2,1)*qdd(1)+M(2,2)*qdd(2)];
tau = Mqdd + V + F

R = [cos(-pi/4) -sin(-pi/4);
    sin(-pi/4) cos(-pi/4)];
p = [1; 1];
p_ = R*p

%% To Simulink (?)
% N = 2; %Links.
% L1 = 1; %Link 1 long.
% LT = 1; %Link 2 long.
% DH = struct('alpha', cell(1,N), 'a', cell(1,N), 'theta', cell(1,N), 'd', cell(1,N),...
%     'type', cell(1,N)); %Structure.
% DH(1).alpha = 0; DH(1).a = 0; DH(1).d = 0; DH(1).type = 'R';
% DH(2).alpha = 0; DH(2).a = L1; DH(2).d = 0; DH(2).type = 'R';
% links = {0};
% for  i = 1:N
%         links{i} = Link('alpha', DH(i).alpha, 'a', DH(i).a, 'd', ...
%             DH(i).d, 'modified', 'm', 1, 'r', [1,0,0], 'G', 1, 'Tc', 1);%'Tc', [1 1]); % Links vector.
%         %links{i}.dyn();
% end
% tool = transl([LT, 0, 0]); % Tool offset.
% RRbot = SerialLink([links{:}], 'tool', tool, 'name', 'RRbot');

%% Test
A = [1 2; 3 4];
A_ = [2 4; 6 8];
A__ = [1 3; 5 7];
v = [1; 2];

B = A^(-1)
B = inv(A)

C = inv(A')
C = (A')^(-1)

D = A*A_*A__
D = A*A_;
D = D*A__

A*v
