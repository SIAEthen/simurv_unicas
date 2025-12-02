function [error_benchmark,error_fmincon,error_nn] = cal_performance(theta_benchmark,theta_fmincon,net,eta,q,GironaPARAMS,tau_body_frame)
%CAL_PERFORMANCE Summary of this function goes here
%   Detailed explanation goes here
    num_of_validation = size(eta,2);


    measure = zeros(6,num_of_validation);
    error = zeros(6,num_of_validation);
    error_relative = zeros(6,num_of_validation);
    %fmincon parameters
    for i=1:size(q,2)
        q_i = q(:,i);
        eta_i = eta(:,i);
        Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
        
        meas_i = tau_body_frame(:,i);
        pred_i = Y_i*theta_fmincon;
        error_i = pred_i - meas_i;
        error(:,i) = error_i;
        error_relative(:,i) = error_i./meas_i;
        measure(:,i) = (meas_i);
    end
    error_fmincon = error;
    error_relative_fmincon = error_relative;
    
    error = 0*error;
    error_relative = 0*error_relative;
    % benchmark model
    for i=1:size(q,2)
        q_i = q(:,i);
        eta_i = eta(:,i);
        Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
        
        meas_i = tau_body_frame(:,i);
        pred_i = Y_i*theta_benchmark;
        error_i = pred_i - meas_i;
        error(:,i) = error_i;
        error_relative(:,i) = error_i./meas_i;
        measure(:,i) = (meas_i);
    end
    error_benchmark = error;
    error_relative_benchmark = error_relative;

    
    % nn method
    rpy = 0*eta(1:3,:);
    for i=1:size(rpy,2)
        rpy(:,i) = Quat2Rpy(eta(4:7,i));
    end

    input_all = [rpy(1:2,:); q]';
    output_all = tau_body_frame';
    X_val = input_all;
    Y_val = output_all;
    Y_predict = predict(net,X_val);
    error_nn = Y_val'-Y_predict';
    error_relative_nn = error_nn./tau_body_frame;
    

tittle_list = "Prediction Error Distribution in "+ ["X","Y","Z","Rx","Ry","Rz"];
x_list = ["Error ($N$)","Error ($N$)","Error ($N$)","Error ($Nm$)","Error ($Nm$)","Error ($Nm$)"];
fig_name_list = "Prediction_error_hist_in_"+ ["X","Y","Z","Rx","Ry","Rz"];
for i = 1:6
    %直方图======================================================================

    figure
    hist([error_benchmark(i,:)' error_fmincon(i,:)' error_nn(i,:)'],10)
    
%     h1=histogram(error_benchmark(i,:),10);
%     h1.FaceColor='none';
%     h1.EdgeColor=[0 1 0];
%     h1.LineWidth=1.0;
%     
%     hold on
%     h2=histogram(error_fmincon(i,:),10);
%     h2.FaceColor='none';
%     h2.EdgeColor=[1 0 0];
% %     h2.BinEdges=h2.BinEdges+0.1;
%     h2.LineWidth=1.5;
%     
%     
%     hold on
%     h3=histogram(error_nn(i,:),10);
%     h3.FaceColor='none';
%     h3.EdgeColor=[0 0 1];
% %     h3.BinEdges=h3.BinEdges-0.1;
%     h3.LineWidth=1.0;

%     title(tittle_list(i))
    legend(["Benchmark","Model-based","NN"],"FontSize",8)
% legend("Des","Robust","SMAC","SMACDO","FTAIC","FontSize",4)
    xlabel(x_list(i),'fontsize',8,'Interpreter','latex')
    ylabel("Frequency",'fontsize',8,'Interpreter','latex')
    ax = gcf;
    ax.Units = "centimeters";
    ax.Position = [20,10,8,6];
    print(ax, '-dpng', '-r300', fig_name_list(i)+'.png')

end


fig_name_list = "Relative_error_hist_in_"+ ["X","Y","Z","Rx","Ry","Rz"];
tittle_list = "Prediction Relative Error Distribution in "+ ["X","Y","Z","Rx","Ry","Rz"];
x_list = ["Percentage","Percentage","Percentage","Percentage","Percentage","Percentage"];
error_relative_benchmark(error_relative_benchmark>1) = 1;
error_relative_benchmark(error_relative_benchmark<-1) = -1;
error_relative_fmincon(error_relative_fmincon>1) = 1;
error_relative_fmincon(error_relative_fmincon<-1) = -1;
error_relative_nn(error_relative_nn>1) = 1;
error_relative_nn(error_relative_nn<-1) = -1;

error_relative_benchmark = 100 * error_relative_benchmark;
error_relative_fmincon = 100 * error_relative_fmincon;
error_relative_nn = 100 * error_relative_nn;

for i = 1:6
    %直方图======================================================================
    figure
    hist([error_relative_benchmark(i,:)' error_relative_fmincon(i,:)' error_relative_nn(i,:)'],10)
%     h1=histogram(error_relative(i,:) * 100,10);
%     h1.FaceColor='none';
%     h1.EdgeColor=[0 1 0];
%     h1.LineWidth=1.0;
%   
%     hold on
%     h2=histogram(error_relative_fmincon(i,:) * 100,10);
%     h2.FaceColor='none';
%     h2.EdgeColor=[1 0 0];
% %     h2.BinEdges=h2.BinEdges+0.1;
%     h2.LineWidth=1.5;
%     
%     hold on
%     h3=histogram(error_relative_nn(i,:) * 100,10);
%     h3.FaceColor='none';
%     h3.EdgeColor=[0 0 1];
% %     h3.BinEdges=h3.BinEdges-0.1;
%     h3.LineWidth=1.0;

%     title(tittle_list(i))
    legend(["Benchmark","Model-based","NN"],"FontSize",8)
    xlabel(x_list(i),'fontsize',8,'Interpreter','latex')
    ylabel("Frequency",'fontsize',8,'Interpreter','latex')
    ax = gcf;
    ax.Units = "centimeters";
    ax.Position = [20,10,8,6];
    print(ax, '-dpng', '-r300', fig_name_list(i)+'.png')
end

%% Distribution of 6 DOF output

% x_list = ["X ($N$)","Y ($N$)","Z ($N$)","K ($Nm$)","M ($Nm$)","N ($Nm$)"];
x_list = ["Force ($N$)","Force ($N$)","Force ($N$)","Torque ($Nm$)","Torque ($Nm$)","Torque ($Nm$)"];
fig_name_list = "data_hist_in_"+ ["X","Y","Z","Rx","Ry","Rz"];
for i = 1:6
    %直方图======================================================================
    figure
    hist(tau_body_frame(i,:)' ,10)
    % legend("Des","Robust","SMAC","SMACDO","FTAIC","FontSize",4)
    xlabel(x_list(i),'fontsize',8,'Interpreter','latex')
    ylabel("Frequency",'fontsize',8,'Interpreter','latex')
    ax = gcf;
    ax.Units = "centimeters";
    ax.Position = [20,10,8,6];
    print(ax, '-dpng', '-r300', fig_name_list(i)+'.png')

end










fprintf("Error mse in 6 DOF (Units: N or Nm) \n Benchmark & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ \\\\ \n",...
    mean(sqrt(error_benchmark(1,:).^2)),mean(sqrt(error_benchmark(2,:).^2)),mean(sqrt(error_benchmark(3,:).^2)), ...
    mean(sqrt(error_benchmark(4,:).^2)),mean(sqrt(error_benchmark(5,:).^2)),mean(sqrt(error_benchmark(6,:).^2)));
fprintf("Model-based & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ \\\\ \n",...
    mean(sqrt(error_fmincon(1,:).^2)),mean(sqrt(error_fmincon(2,:).^2)),mean(sqrt(error_fmincon(3,:).^2)), ...
    mean(sqrt(error_fmincon(4,:).^2)),mean(sqrt(error_fmincon(5,:).^2)),mean(sqrt(error_fmincon(6,:).^2)));
fprintf("NN & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ \\\\ \n",...
    mean(sqrt(error_nn(1,:).^2)),mean(sqrt(error_nn(2,:).^2)),mean(sqrt(error_nn(3,:).^2)), ...
    mean(sqrt(error_nn(4,:).^2)),mean(sqrt(error_nn(5,:).^2)),mean(sqrt(error_nn(6,:).^2)));
%     
%     x_list = ["Simulations","Simulations","Simulations","Simulations","Simulations","Simulations"];
%     for i=1:6
%         figure
%         x = 1:size(predict,2);
%         plot(x,predict(i,:), x,measure(i,:));
%         legend(["predict","simulation"]);
%         title(title_list(i))
%         xlabel(x_list(i))
%         ylabel(y_list(i))
%     end
%     
%     title_list = ["X","Y","Z","Rx","Ry","Rz"];
%     x_list = ["Force ($N$)","Force ($N$)","Force ($N$)","Torque ($Nm$)","Torque ($Nm$)","Torque ($Nm$)"];
%     for i=1:6
%         figure
%         data = predict(i,:)-measure(i,:);
%         histogram(data,100)
%         title("Predict error distribution in "+title_list(i))
%         xlabel(x_list(i))
%         ylabel("Number")
%     end
%     
%     title_list = ["X","Y","Z","Rx","Ry","Rz"];
%     for i=1:6
%         figure
%         data = 100 * error_relative(i,:);
%         histogram(data,100)
%         title("Relative error distribution in " + title_list(i))
%         xlabel("Error percentage (%)")
%     end
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 











end

