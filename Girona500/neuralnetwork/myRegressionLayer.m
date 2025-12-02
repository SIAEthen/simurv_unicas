classdef myRegressionLayer < nnet.layer.RegressionLayer
    % 自定义回归层: L1 + 0.5*MSE
    methods
        function loss = forwardLoss(layer, Y, T)
            % Y: 预测输出
            % T: 真实目标
%             loss = mean(abs(T - Y), 'all') + 0.5 * mean((T - Y).^2, 'all');
            loss = mean((T - Y).^2, 'all');
        end
    end
end
