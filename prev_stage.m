%Starts from the current decoded state, takes input as minimum distance to
%reach that state (node value) and previous state And returns previous state
%and decoded information bit corresponding to that state.

function [prev_state,decoded_bit]=prev_stage(curr_state,node_value_prev,branch_metric)
    if(curr_state==1)
        if(node_value_prev(1)+branch_metric(1) <= node_value_prev(5)+branch_metric(1))
            prev_state=1;decoded_bit=-1;
        else
            prev_state=5;decoded_bit=-1;
        end
    end
    
    if(curr_state==2)
        if(node_value_prev(1)+branch_metric(2) <= node_value_prev(5)+branch_metric(2))
            prev_state=1;decoded_bit=1;
        else
            prev_state=5;decoded_bit=1;
        end
    end
    
    if(curr_state==3)
        if(node_value_prev(2)+branch_metric(3) <= node_value_prev(6)+branch_metric(3))
            prev_state=2;decoded_bit=-1;
        else
            prev_state=6;decoded_bit=-1;
        end
    end
    
    if(curr_state==4)
        if(node_value_prev(2)+branch_metric(4) <= node_value_prev(6)+branch_metric(4))
            prev_state=2;decoded_bit=1;
        else
            prev_state=6;decoded_bit=1;
        end
    end
    if(curr_state==5)
        if(node_value_prev(3)+branch_metric(5) <= node_value_prev(7)+branch_metric(5))
            prev_state=3;decoded_bit=-1;
        else
            prev_state=7;decoded_bit=-1;
        end
    end
    if(curr_state==6)
        if(node_value_prev(3)+branch_metric(6) <= node_value_prev(7)+branch_metric(6))
            prev_state=3;decoded_bit=1;
        else
            prev_state=7;decoded_bit=1;
        end
    end
    if(curr_state==7)
        if(node_value_prev(4)+branch_metric(7) <= node_value_prev(8)+branch_metric(7))
            prev_state=4;decoded_bit=-1;
        else
            prev_state=8;decoded_bit=-1;
        end
    end
    if(curr_state==8)
        if(node_value_prev(4)+branch_metric(8) <= node_value_prev(8)+branch_metric(8))
            prev_state=4;decoded_bit=1;
        else
            prev_state=8;decoded_bit=1;
        end
    end    
end