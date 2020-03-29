%% Benders Decomposition 示例程序

clear variables
close all
warning off
clc

%% 构建系数矩阵

% 1. 问题的标准形式
% min   cx + dy
% s.t.  Ax + By >= f
%       x >= 0
%       x is continious
%       y is binary

c = [ 1, 1, 1, 1, 1];
d = [ 7, 7, 7, 7, 7];
A = [-1, 0, 0,-1,-1;
      1, 0, 0, 1, 1;
      0,-1, 0, 0,-1;
      0, 1, 0, 0, 1;
      0, 0,-1,-1, 0;
      0, 0, 1, 1, 0;
     -1, 0, 0, 0, 0;
      0,-1, 0, 0, 0;
      0, 0,-1, 0, 0;
      0, 0, 0,-1, 0;
      0, 0, 0, 0,-1];  % 系数矩阵A
B = [ 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0;
      0, 0, 0, 0, 0;
      0, 0, 0, 0, 0;
      0, 0, 0, 0, 0;
      0, 0, 0, 0, 0;
      8, 0, 0, 0, 0;
      0, 3, 0, 0, 0;
      0, 0, 5, 0, 0;
      0, 0, 0, 5, 0;
      0, 0, 0, 0, 3];  % 系数矩阵A
f = [-8; 8;-3; 3;-5; 5; 0; 0; 0; 0; 0];  % 系数矩阵f

% 2. 对于不可行问题，需构造其Feasibility Relaxation，其标准形式为
% min   ez
% s.t.  Ax + By + Cz >= f
%       x >= 0
%       z >= 0
%       x is continious
%       y is binary

e = ones(1, size(A,1)) * 5;  % 系数矩阵e
C = eye(size(A,1));  % 系数矩阵C


%% 采用 yalmip + cplex 直接建模求解

% 1. 变量设置
x = sdpvar(5,1);  % 变量x
y = binvar(5,1);  % 变量y

% 2. 模型构建
obj = c*x + d*y;  % 目标函数
constr = [];  % 约束条件
constr = [constr, A*x + B*y >= f];
constr = [constr, x >= 0];

% 3. 模型求解
opts = sdpsettings('solver','cplex','verbose',2);  % yalmip参数设置
diag = optimize(constr,obj,opts);  % 模型求解
if diag.problem == 0  % 有可行解
    res_cplex = value(obj);  % 记录可行解
    sol_cplex = [value(x),value(y)];
else
    disp('The original problem is infeasible!')
    pause();
end

%% 采用Benders Decomposition求解

% 1. 算法初始化
lb = [];  % 下界
ub = [];  % 上界
sub_coef = [];  % 记录对偶系数
sub_sol  = [];  % 记录子问题中主问题传递的解
sub_obj  = [];  % 记录子问题的目标函数

% 2. 主函数
while true
    % 1) 变量设置
    x = sdpvar(5,1);
    y = binvar(5,1);
    z = sdpvar(size(A,1),1);  % 不可行松弛新增变量
    t = sdpvar(1,1);  % 主问题新增变量
    
    % 2) 求解主问题
    obj = d*y + t;
    constr = [];
    constr = [constr, t >= 0];
    if ~isempty(sub_coef)  % 添加Benders Cut
        for i = 1:size(sub_coef,2)
            constr = [constr, t >= sub_obj(i) - sub_coef(:,i)' * (y - sub_sol(:,i))];
        end
    end
    opts = sdpsettings('solver','cplex','verbose',2);
    optimize(constr,obj,opts);
    y_star = value(y);  % 记录当前y的值
    lb = [lb, value(obj)];  % 更新下界
    
    % 3) 求解子问题
    obj = c*x;
    constr = [];
    constr = [constr, A*x + B*y >= f];
    constr = [constr, x >= 0];
    constr = [constr, y == y_star];  % 将主问题结果应用于子问题
    opts = sdpsettings('solver','cplex','verbose',2,'relax',1);  % 将所有0-1变量松弛
    diag = optimize(constr,obj,opts);
    if diag.problem == 1  % 无可行解，构建Infeasibility Relaxation
        obj = c*x + e*z;
        constr = [];
        constr = [constr, A*x + B*y + C*z >= f];
        constr = [constr, x >= 0];
        constr = [constr, z >= 0];
        constr = [constr, y == y_star];  % 将主问题结果应用于子问题
        opts = sdpsettings('solver','cplex','verbose',2,'relax',1);  % 将所有0-1变量松弛
        optimize(constr,obj,opts);
    end
    sub_coef = [sub_coef, dual(constr(end))];  % 更新对偶系数
    sub_sol  = [sub_sol , y_star];  % 更新子问题中主问题传递的解
    sub_obj  = [sub_obj , value(obj)];  % 更新子问题的目标函数
    ub = [ub, d * y_star + value(obj)];  % 更新上界
    
    % 4) 收敛判断
    gap = (ub(end) - lb(end)) / ub(end);
    if gap <= 1e-5
        res_benders = ub(end);
        sol_benders = [value(x),value(y)];
        break;
    end
end

%% 输出结果

disp('');
disp('cplex得到的问题最优解为：');
sol_cplex
disp(['最优解：', num2str(res_cplex)]);
disp('');
disp('Benders Decomposition得到的问题最优解为：');
sol_benders
disp(['最优解：', num2str(res_benders)]);




