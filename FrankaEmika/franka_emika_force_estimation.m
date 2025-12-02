% get_emika_dh_params
% get_emika_data
error = [];
mdh = emika_dh_parameters; %MDH
for i=1:40
    q_i = pos(:,i);
    if abs(q_i(2))>0.1
        mdh_i =  [mdh(:,1:3) mdh(:,4)+ q_i];
        J7_i = jacobian_mdh(mdh_i);
        J3_i = jacobian_mdh(mdh_i(1:3,:));
        J3_i_linear = J3_i(1:3,:);
        tau_mea = tau(1:7,i);
        tau_pre = get_emika_gravity(q_i(1:7));
        tau_e = tau_mea - tau_pre;
    
        f_e = mypinv(J3_i_linear',eye(3))*tau_e(1:3);
        if(f_e'*f_e>50)
            q_i(1:7)'
            disp("big error, reason is joint 2 is nearly zero and sigular\n")
        end
        error = [error f_e'*f_e];
    end
     
end

