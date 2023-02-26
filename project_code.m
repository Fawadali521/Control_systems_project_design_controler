clear;
clc;
close all;

%======Matrices Decleration==========

A = [3 2 3; 3 2 4; 1 1 3];
B = [4; 0; 3]
C = [1 1 0.5]
D = [0]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Checking if the system is stable or not
%Method 1
% Checking poles of transfer function
[num,denum] = ss2tf(A,B,C,D);
denumRoots = roots(denum);
disp('Poles of Tf are:')
disp(denumRoots);

%Method 2
% Checking eigen values
e = eig(A);
disp('Eigen Values are: ');
disp(e);

%Method 3
% Step response method
step(A,B,C,D)

%Method 4
% Root Locus method
figure;
sys = tf(num,denum)
disp(sys);
rlocus(sys);
fedd=feedback(sys,1);
pid=pidtune(fedd,"pid");
pidc=feedback(pid*sys,1)
%Method 5
%RH table

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
%size of matrix A
oderA= size(A,1);
disp("The order of system is ")
disp(oderA);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%================Controllability test===============

P = ctrb(A,B); %P is controllability matrix
disp('The P matrix is: ')
disp(P);
r = rank(P);
disp('The rank of P matrix is:')
disp(r);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%================Observability test================

Q = obsv(A,C); %Q is Observability matrix
disp('The Q matrix is: ')
disp(Q);
r2 = rank(Q);
disp('The rank of Q matrix is:')
disp(r2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%================Observer Design================
observer_eigenvalues = [-80 -40 -50]%[-10 -20]%
L = place(A',C',observer_eigenvalues)'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%================Controller Design================
controller_eigenvalues = [-16 -8 -10];%[-3 -5]%
K = place(A,B,controller_eigenvalues)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%Thank You%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%observer
%syms l1 l2 l3 s
%I = [1 0 0; 0 1 0; 0 0 1];
%sI = s*I;
%L= [l1;l2;l3];
%LC = L*C;
%A1= A-LC;   %A1= A -LC
%A2= sI - A1 ;  %A2= sI -(A -LC)
%detO = det(A2)

% (s + 10)(s + 40)(s + 50)= s^3 + 100*s^2 + 2900*s + 20000;
%Compare coeffcients
% -l3 + 2.5l2 - 2.5l1 + 1 = 20000
% 4.5l3 - 3.5l2 - 1.5l1 + 8 = 2900
% l3/2 - l2 + l1 - 8 = 100

%%%%%%%%%controller
%syms k1 k2 k3 
%K= [k1 k2 k3];
%BK = B*K;
%B1= A-BK;   %B1= A -BK
%B2= sI - B1 ;  %A2= sI -(A -BK)
%detC = det(B2)

% (s + 2)(s + 8)(s + 10) = s^3 + 20*s^2 + 116*s + 160
%Compare coefcients
% 4k3-29k2+14k1 +1 = 160
% -11k3+24k2-11k1 +8 = 116,
% 3k3+4k1 -8 = 20,
%desired

