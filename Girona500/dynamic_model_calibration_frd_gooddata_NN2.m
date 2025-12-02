



% 定义网络结构
layers = [
    featureInputLayer(6)
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(6)
    myRegressionLayer
];

options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 64, ...
    'Plots', 'training-progress', ...
    'Verbose', false);

net = trainNetwork(X_train, Y_train, layers, options);

