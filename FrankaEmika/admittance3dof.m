function out = admittance3dof(in)
% admittance3dof : 3-DOF Cartesian admittance control
%
% Input struct fields:
%   in.x_r      : filted position
%   in.dx_r     : filted velocity
%   in.ddx_r    : filted acceleration

%   in.xd     : desired position (3×1) this is the original input
%   in.dxd    : desired velocity (3×1) this is the original input
%   in.dt     : timestep
%   in.Fext   : external force (3×1)

%
% NOTE: Admittance computes *motion* from force.
%       Use this as the motion generator before sending position cmds.

    % ---------------------------------------------
    % 1. Desired admittance parameters (tunable)
    % ---------------------------------------------
    Md = diag([20 20 20]);        % virtual mass
    Bd = diag([50 50 50]);     % virtual damping
    Kd = diag([200  200  200]);   % virtual stiffness 
    % ---------------------------------------------
    % 2. Extract input values
    % ---------------------------------------------
    x_r    = in.x_r; %filted reference signal
    dx_r   = in.dx_r; %filted reference signal
    xd   = in.xd;
    dxd  = in.dxd;
    dt   = in.dt;
    Fext = in.Fext;
    if norm(Fext,2)<5
        Fext = 0 * Fext;
    end

    % ---------------------------------------------
    % 3. Compute admittance acceleration
    %    Md * ddx = Fext - Bd*(dx - dxd) - Kd*(x - xd)
    % ---------------------------------------------
    ddx = Md \ ( Bd*(dxd - dx_r) + Kd*(xd - x_r) - Fext);

    % ---------------------------------------------
    % 4. Integrate to update velocity and position
    % ---------------------------------------------
    dx_new = dx_r + ddx * dt;
    x_new  = x_r  + dx_new * dt;

    % ---------------------------------------------
    % 5. Write outputs back into struct
    % ---------------------------------------------
    out = in;
    out.ddx_r = ddx;
    out.dx_r  = dx_new;
    out.x_r   = x_new;
end