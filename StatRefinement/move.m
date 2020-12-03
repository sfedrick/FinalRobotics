function move(qfinal, lynx)
% move to q configuration
    lynx.command(qfinal);
    %pause(1)
    reach = 0;
    count=0;
    
    while (reach == 0)
        [q, ~] = lynx.get_state();
        %pause(0.05)
        lynx.command(qfinal)
        count=count+1;                                      %useful when robot gets stuck in gazebo
        if count>30
            break;
        end
        disp("reaching")
%       reachNorm = norm(q(1:5)-qfinal(1:5));
        reachNorm = norm(q - qfinal);                       %compares present state vs desired state of robot
        if reachNorm < 0.1                                  %if L2 norm distance error <0.1 ; reach = 1;
            reach = 1;
        else
            lynx.command(qfinal)                            %if error > 0.1, set_position again
        end
    end
    
    disp("reach complete");
    %pause(0.1)
end