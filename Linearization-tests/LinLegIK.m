function [th_s, th_h, th_k] = LinLegIK(XYZ,XYZ0)
    

    x  = XYZ(1);
    y  = XYZ(2);
    z  = XYZ(3);

    x0 = XYZ0(1);
    y0 = XYZ0(2);
    z0 = XYZ0(3);

    %Lengths
    Ls = 40.5;
    Lh = 60;
    Lk = 100;
   
    %Useful vars
    %for Theta_s
    R0 = hypot(y0,z0);
    sgn = x>0;
    y0p = sqrt(y0^2 + z0^2 - Ls^2);

    theta_s_X = 0;
    theta_s_Y =  (z0)/(R0^2) - (Ls*y0)/(R0^2 * sqrt(R0^2 - Ls^2));
    theta_s_Z = -(y0)/(R0^2) - (Ls*z0)/(R0^2 * sqrt(R0^2 - Ls^2));
    theta_s_C = (acos(z0/R0) - acos(Ls/R0)) - theta_s_Y * y0 - theta_s_Z*z0;
    
    %for Theta_h
    %% Phi coefficients
    u_phi = x0 / (x0^2 + y0p^2);
    dphi_dx = -(y0p^2 - x0^2)/((x0^2 + y0p^2)^2 * sqrt(1 - u_phi^2));
    dphi_dy = 2*x0*y0 / ((x0^2 + y0p^2)^2 * sqrt(1 - u_phi^2));
    dphi_dz = 2*x0*z0 / ((x0^2 + y0p^2)^2 * sqrt(1 - u_phi^2));
    phi0 = acos(u_phi);
    
    %% Psi coefficients
    u_psi = (Lh^2 + x0^2 + y0p^2 - Lk^2) / (2*Lh*sqrt(x0^2 + y0p^2));
    dpsi_dx = - x0*(x0^2 + y0p^2 + Lk^2 - Lh^2) / (2*Lh*(x0^2 + y0p^2)^(3/2) * sqrt(1 - u_psi^2));
    dpsi_dy = - y0*(x0^2 + y0p^2 + Lk^2 - Lh^2) / (2*Lh*(x0^2 + y0p^2)^(3/2) * sqrt(1 - u_psi^2));
    dpsi_dz = - z0*(x0^2 + y0p^2 + Lk^2 - Lh^2) / (2*Lh*(x0^2 + y0p^2)^(3/2) * sqrt(1 - u_psi^2));
    psi0 = acos(u_psi);
    
    %% Combine -Psi +- Phi
    theta_h_X = sgn*dphi_dx - dpsi_dx;
    theta_h_Y = sgn*dphi_dy - dpsi_dy;
    theta_h_Z = sgn*dphi_dz - dpsi_dz;
    theta_h_C = -psi0 + sgn*phi0;

    %% Theta_K
    % Compute u0
    u0 = (Lh^2 + Lk^2 - x0^2 - y0p^2) / (2 * Lh * Lk);
    
    % Derivatives
    theta_k_X = x0 / (Lh * Lk * sqrt(1 - u0^2));
    theta_k_Y = y0 / (Lh * Lk * sqrt(1 - u0^2));
    theta_k_Z = z0 / (Lh * Lk * sqrt(1 - u0^2));
    
    % Constant term
    theta_k_C = acos(u0);


    A = zeros(3,3);
    B = zeros(3,1);

    %Build the linear system
    A(1,1) = theta_s_X;
    A(1,2) = theta_s_Y;
    A(1,3) = theta_s_Z;
    B(1)   = theta_s_C;

    A(2,1) = theta_h_X;
    A(2,2) = theta_h_Y;
    A(2,3) = theta_h_Z;
    B(2)   = theta_h_C;

    A(3,1) = theta_k_X;
    A(3,2) = theta_k_Y;
    A(3,3) = theta_k_Z;
    B(3) = theta_k_C;

    disp("Matrix A:") ; disp(A);
    disp("Matrix B:") ; disp(B);



end