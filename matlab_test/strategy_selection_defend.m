function [strategy_A,strategy_B,strategy_C,count_different, count_accept, count_right] = strategy_selection_defend(times)
compare_result=0;
plot_position=1-compare_result;
if plot_position==1
    times=1;
    position=figure;
else
    compare=figure;
end

payoff_matrix=[5,3,1;3,3,3;1,3,5];
random_payoff=zeros(1,times);
nubot_payoff=zeros(1,times);
final_payoff=zeros(1,times);
count_different=0;
count_accept=0;
count_right=0;
strategy_A=0;
strategy_B=0;
strategy_C=0;

if plot_position==1
    axes1 = axes('Parent',position);
    axis equal;
    ylim(axes1,[-600 600]);
    xlim(axes1,[-900 900]);
    hold(axes1,'on');
    
    img=imread('field.png');
    min_x= -900;
    max_x=900;
    min_y=-600;
    max_y=600;
    imagesc([min_x max_x], [min_y max_y], flip(img,1));
end

for test_time=1:times
    left_points=[];
    right_points=[];
    middle_points=[];
    %CONSERVATIVE 1, BALANCE 2, RADICAL 3
    attack_strategy=randi([1,3],1);
    pos_opponent=zeros(4,2);
    P=zeros(1,3);
    if attack_strategy==1
        pos_opponent_x=randi([-800,400],4,1);
    elseif attack_strategy==2
        pos_opponent_x=randi([-600,600],4,1);
    else
        pos_opponent_x=randi([-400,800],4,1);
    end
    pos_opponent_y=randi([-600,600],4,1);
    pos_opponent(:,1)=pos_opponent_x;
    pos_opponent(:,2)=pos_opponent_y;
    K=convhull(pos_opponent);
    
    for i=1:length(K)-1
        if pos_opponent_x(K(i))>=200
            right_points=[right_points;pos_opponent(K(i),:)];
        elseif pos_opponent_x(K(i))<=-200
            left_points=[left_points;pos_opponent(K(i),:)];
        else
            middle_points=[middle_points;pos_opponent(K(i),:)];
        end
        
        if (pos_opponent_x(K(i))<200 && pos_opponent_x(K(i+1))>=200) || (pos_opponent_x(K(i))>=200 && pos_opponent_x(K(i+1))<200)
            d=(pos_opponent_y(K(i+1))-pos_opponent_y(K(i)))/(pos_opponent_x(K(i+1))-pos_opponent_x(K(i)));
            tmp_x=200;
            tmp_y=pos_opponent_y(K(i))+d*(200-pos_opponent_x(K(i)));
            middle_points=[middle_points;[tmp_x,tmp_y]];
            right_points=[right_points;[tmp_x,tmp_y]];
            if plot_position==1
                plot(tmp_x,tmp_y,'bx','LineWidth',2,'markersize',12);
            end
        end
        
        if (pos_opponent_x(K(i))<-200 && pos_opponent_x(K(i+1))>=-200) || (pos_opponent_x(K(i))>=-200 && pos_opponent_x(K(i+1))<-200)
            d=(pos_opponent_y(K(i+1))-pos_opponent_y(K(i)))/(pos_opponent_x(K(i+1))-pos_opponent_x(K(i)));
            tmp_x=-200;
            tmp_y=pos_opponent_y(K(i))+d*(-200-pos_opponent_x(K(i)));
            left_points=[left_points;[tmp_x,tmp_y]];
            middle_points=[middle_points;[tmp_x,tmp_y]];
            if plot_position==1
                plot(tmp_x,tmp_y,'bx','LineWidth',2,'markersize',12);
            end
        end
    end
    
    if ~isempty(left_points)
        P(1)=polyarea(left_points(:,1),left_points(:,2));
        if plot_position==1
            p1  =patch(left_points(:,1),left_points(:,2),'r','EdgeColor','none');
            p1.FaceVertexAlphaData = 0.5;
            p1.FaceAlpha = 'flat' ;
        end
    else
        P(1)=0;
    end
    if ~isempty(middle_points)
        P(2)=polyarea(middle_points(:,1),middle_points(:,2));
        if plot_position==1
            p2  =patch(middle_points(:,1),middle_points(:,2),'y','EdgeColor','none');
            p2.FaceVertexAlphaData = 0.5;
            p2.FaceAlpha = 'flat' ;
        end
    else
        P(2)=0;
    end
    if ~isempty(right_points)
        P(3)=polyarea(right_points(:,1),right_points(:,2));
        if plot_position==1
            p3  =patch(right_points(:,1),right_points(:,2),'b','EdgeColor','none');
            p3.FaceVertexAlphaData = 0.5;
            p3.FaceAlpha = 'flat' ;
        end
    else
        P(3)=0;
    end
    
    P_norm=P/sum(P);
    
    if plot_position==1
        plot(pos_opponent_x,pos_opponent_y,'ko','markersize',12,'MarkerFaceColor','k');
        plot([-200,-200],[-600,600],'b--');
        plot([200,200],[-600,600],'b--');
    end
    
    E_payoff=P_norm*payoff_matrix;
    [value,robot_strategy]=max(E_payoff);
    
    random_strategy=randi([1,3],1);
    
    if attack_strategy==1
        human_strategy=randsrc(1,1,[[1 2 3]; [0.8 0.1 0.1]]);
    elseif attack_strategy==2
        human_strategy=randsrc(1,1,[[1 2 3]; [0.1 0.8 0.1]]);
    else
        human_strategy=randsrc(1,1,[[1 2 3]; [0.1 0.1 0.8]]);
    end
    
    if robot_strategy~=human_strategy
        count_different=count_different+1;
        if human_strategy==1
            payoff_modify=payoff_matrix*[0.8, 0, 0;0, 0.1, 0;0, 0, 0.1];
        elseif human_strategy==2
            payoff_modify=payoff_matrix*[0.1, 0, 0;0, 0.8, 0;0, 0, 0.1];
        else
            payoff_modify=payoff_matrix*[0.1, 0, 0;0, 0.1, 0;0, 0, 0.8];
        end
        
        E_payoff=P_norm*payoff_modify;
        [value,final_strategy]=max(E_payoff);
        if final_strategy~=robot_strategy
            count_accept=count_accept+1;
            if attack_strategy==final_strategy
                count_right=count_right+1;
            end
        end
    else
        final_strategy=robot_strategy;
    end
    
    if final_strategy==1
        strategy_A=strategy_A+1;
    elseif final_strategy==2
        strategy_B=strategy_B+1;
    else
        strategy_C=strategy_C+1;
    end
    
    if test_time~=1
        random_payoff(test_time)=(random_payoff(test_time-1)*(test_time-1)+payoff_matrix(attack_strategy,random_strategy))/test_time;
        nubot_payoff(test_time)=(nubot_payoff(test_time-1)*(test_time-1)+payoff_matrix(attack_strategy,robot_strategy))/test_time;
        final_payoff(test_time)=(final_payoff(test_time-1)*(test_time-1)+payoff_matrix(attack_strategy,final_strategy))/test_time;
    else
        random_payoff(test_time)=payoff_matrix(attack_strategy,random_strategy);
        nubot_payoff(test_time)=payoff_matrix(attack_strategy,robot_strategy);
        final_payoff(test_time)=payoff_matrix(attack_strategy,final_strategy);
    end
end
if compare_result==1
    t=1:test_time;
    figure(compare);
    plot(t,random_payoff,t,nubot_payoff,t,final_payoff);
    legend('random', 'robot','robot&human');
end
end
