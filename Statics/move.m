function move(qfinal, lynx)
% move to q configuration
    lynx.command(qfinal);
    pause(1)
    reach = 0;
    count=0;
    % reach 50 mm above the object
    while (reach == 0)
        [q, ~] = lynx.get_state();
        pause(0.1)
        lynx.command(qfinal)
        count=count+1;
        if count>30
            break;
        end
        disp("reaching")
%         reachNorm = norm(q(1:5)-qfinal(1:5));
        reachNorm = norm(q - qfinal);
        if reachNorm < 0.1
            reach = 1;
        else
            lynx.command(qfinal)
        end
    end
    
    disp("reach complete");
    pause(0.5)
end