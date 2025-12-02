eta = [1,2,3,0.2,0.3,0]';
q_i = [1,2,1.0,1.0]';
n=100;
pred = zeros(6,n);
for i = 1:n
    delta_psi = rand()
    eta_i = eta;
    eta_i(6) = eta_i(6) + delta_psi;
    Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS);
    pred_i = Y_i*theta_fmincon;
    pred(:,i) = pred_i;
end