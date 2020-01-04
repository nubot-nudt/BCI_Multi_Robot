clear;
count_robot=0;
count_human=0;
count_robot_human=0;
count_robot_init=0;
count_robot_human_init=0;
count_tmp=zeros(5,1);

loop_times=1000;
right_rate=zeros(5,loop_times);
right_rate_tmp=zeros(5,loop_times);
w_robot= ...
    [0.4  0.2  0.2  0.2;
    0.2  0.4  0.2  0.2;
    0.2  0.2  0.4  0.2;
    0.2  0.2  0.2  0.4];
w_mix= ...
    [0.2    0.1    0.1    0.1    0.5;
    0.1    0.2    0.1    0.1    0.5;
    0.1    0.1    0.2    0.1    0.5;
    0.1    0.1    0.1    0.2    0.5;
    0.125    0.125    0.125    0.125    0.5];

pay_off_table_def=...
    [1 2 5;
    3 5 3;
    5 2 1];
              
pay_off_table_off=...
    [5 2 1;
    3 5 3;
    1 2 5];

w_mix_init=w_mix;
w_robot_init=w_robot;

for loop=1:loop_times
    attack_or_defend=randi([1,2],1);
    if attack_or_defend==1
        our_strategy=randi([1,3],1);
        defend_strategy=randi([1,3],1);
        pos_opponent=randi([-600,600],4,2);
        pos_ball=pos_opponent(1,:);
        pos_regional=[pos_ball;200,400;200,-400;400,0];
        pos_nubot=randi([-50,50],4,2);
        index1=find(pos_nubot>0);
        index2=find(pos_nubot<=0);
        pos_nubot(index1)=pos_nubot(index1)+50;
        pos_nubot(index2)=pos_nubot(index2)-50;
        
        if our_strategy==1
            pos_nubot=pos_nubot+pos_regional;
        elseif our_strategy==2
            pos_nubot=pos_nubot+pos_opponent;
        elseif our_strategy==3
            pos_nubot=pos_nubot+[pos_ball;pos_ball;pos_ball;pos_ball];
        end
    else
        pos_nubot=randi([-600,600],4,2);
        attack_strategy=randi([1,3],1);
        pos_opponent=zeros(4,2);
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
    end
    
    %reference strategy without error
    if attack_or_defend==1
        probability_right=predict_defend_strategy(pos_nubot, pos_opponent);
        [max_value, right_decision]=max(transpose(probability_right)*pay_off_table_def);
    else
        probability_right=predict_attack_strategy(pos_opponent);
        [max_value, right_decision]=max(transpose(probability_right)*pay_off_table_off);
    end
    
    %robot strategy with error
    for ID=1:4
        if ID==2
            error(ID,:)=normrnd(300,150,1,2);
        else
            error(ID,:)=normrnd(0,150,1,2);
        end
    end
    pos_nubot=pos_nubot+error;
    pos_init=pos_opponent;
    tmp_decision=zeros(3,1);
    for ID=1:4
        for i=1:length(pos_opponent)
            pos_opponent(i,:)=pos_init(i,:)+error(ID,:);
        end
        if attack_or_defend==1
            probability(ID,:)=predict_defend_strategy(pos_nubot, pos_opponent);
            [max_value, tmp_decision(ID)]=max(probability(ID,:)*pay_off_table_def);
        else
            probability(ID,:)=predict_attack_strategy(pos_opponent);
            [max_value, tmp_decision(ID)]=max(probability(ID,:)*pay_off_table_off);
        end
    end
    
    %get the consistent decision from robots' strategy with w_robot update
    probability_init=probability;
    for t=1:100
        probability=w_robot*probability;
        %         scatter3(probability(1,1),probability(1,2),probability(1,3),'r');
        %         scatter3(probability(2,1),probability(2,2),probability(2,3),'b');
        %         scatter3(probability(3,1),probability(3,2),probability(3,3),'k');
        %         scatter3(probability(4,1),probability(4,2),probability(4,3),'g');
        %         hold on;
        if probability(1,1)==probability(2,1)&&probability(1,1)==probability(3,1)&&probability(1,1)==probability(4,1)
            break;
        end
    end
    if attack_or_defend==1
        [max_value, robot_decision]=max(probability(1,:)*pay_off_table_def);
    else
        [max_value, robot_decision]=max(probability(1,:)*pay_off_table_off);
    end
    
    %get the consistent decision from robots' strategy with w_robot_init
    probability=probability_init;
    for t=1:100
        probability=w_robot_init*probability;
        %         scatter3(probability(1,1),probability(1,2),probability(1,3),'r');
        %         scatter3(probability(2,1),probability(2,2),probability(2,3),'b');
        %         scatter3(probability(3,1),probability(3,2),probability(3,3),'k');
        %         scatter3(probability(4,1),probability(4,2),probability(4,3),'g');
        %         hold on;
        if probability(1,1)==probability(2,1)&&probability(1,1)==probability(3,1)&&probability(1,1)==probability(4,1)
            break;
        end
    end
    if attack_or_defend==1
        [max_value, robot_decision_init]=max(probability(1,:)*pay_off_table_def);
    else
        [max_value, robot_decision_init]=max(probability(1,:)*pay_off_table_off);
    end
        
    %human choose the strategy correctly with a probability of 80%
    if right_decision==1
        human_decision=randsrc(1,1,[[1 2 3]; [0.8 0.1 0.1]]);
    elseif right_decision==2
        human_decision=randsrc(1,1,[[1 2 3]; [0.1 0.8 0.1]]);
    else
        human_decision=randsrc(1,1,[[1 2 3]; [0.1 0.1 0.8]]);
    end
    
    %human&robot strategy with w_mix update
    if human_decision~=robot_decision        
        %human strategy from BCI
        probability_human=zeros(1,length(probability_right));
        for i=1:length(probability_human)
            probability_human(i)=normrnd(probability_right(i),0.3);
            if probability_human(i)<0
                probability_human(i)=0;
            end
        end
        probability_human=probability_human/sum(probability_human);
        if attack_or_defend==1
            [max_value, human_decision]=max(probability_human);
            human_decision=4-human_decision;
        else
            [max_value, human_decision]=max(probability_human);
        end
        %这并不是真正的人类决策，已经得到了机器人的帮助，其正确率还应该*80%
        
        probability_mix=[probability_init;probability_human];
        %get the consistent decision from probability_mix
        probability_mix_init=probability_mix;
        for t=1:100
            probability_mix=w_mix*probability_mix;
            if probability_mix(1,1)==probability_mix(2,1)&&probability_mix(1,1)==probability_mix(3,1)&&probability_mix(1,1)==probability_mix(4,1)
                break;
            end
        end
        if attack_or_defend==1
            [max_value, robot_human_decision]=max(probability_mix(1,:)*pay_off_table_def);
        else
            [max_value, robot_human_decision]=max(probability_mix(1,:)*pay_off_table_off);
        end
    
        %human&robot strategy with w_mix_init
        probability_mix=[probability_init;probability_human];
        %get the consistent decision from probability_mix
        probability_mix_init=probability_mix;
        for t=1:100
            probability_mix=w_mix_init*probability_mix;
            if probability_mix(1,1)==probability_mix(2,1)&&probability_mix(1,1)==probability_mix(3,1)&&probability_mix(1,1)==probability_mix(4,1)
                break;
            end
        end
        if attack_or_defend==1
            [max_value, robot_human_decision_init]=max(probability_mix(1,:)*pay_off_table_def);
        else
            [max_value, robot_human_decision_init]=max(probability_mix(1,:)*pay_off_table_off);
        end
    else
        human_decision=robot_decision;
        robot_human_decision=robot_decision;
        robot_human_decision_init=robot_decision;
    end
    
    for i=1:length(w_mix)
        w_mix(i,:)=w_mix(i,:)/sum(w_mix(i,:));
    end
    
    for i=1:ID
        if tmp_decision(i)==robot_human_decision
            count_tmp(i)=count_tmp(i)+1;
        end
        right_rate_tmp(i,loop)=count_tmp(i)/loop;
    end
    
    if human_decision==robot_human_decision
        count_tmp(5)=count_tmp(5)+1;
    end
    right_rate_tmp(5,loop)=count_tmp(5)/loop;
    
    if robot_decision==right_decision
        count_robot=count_robot+1;
    end
    if robot_decision_init==right_decision
        count_robot_init=count_robot_init+1;
    end
    if human_decision==right_decision
        count_human=count_human+1;
    end
    if robot_human_decision==right_decision
        count_robot_human=count_robot_human+1;
    end
    if robot_human_decision_init==right_decision
        count_robot_human_init=count_robot_human_init+1;
    end
    right_rate(1,loop)=count_robot/loop;
    right_rate(2,loop)=count_human/loop;
    right_rate(3,loop)=count_robot_human/loop;
    right_rate(4,loop)=count_robot_init/loop;
    right_rate(5,loop)=count_robot_human_init/loop;
    
    if mod(loop,100)==0
        theta=right_rate_tmp(:,loop);
        for i=1:ID
            w_robot(:,i)=w_robot(:,i)*theta(i);
        end
        for i=1:ID
            w_robot(i,:)=w_robot(i,:)/sum(w_robot(i,:));
        end
        
        for i=1:ID+1
            w_mix(:,i)=w_mix(:,i)*theta(i);
        end
        for i=1:ID+1
            w_mix(i,:)=w_mix(i,:)/sum(w_mix(i,:));
        end
    end
end
t=1:loop_times;
plot(t,right_rate(1,:),t,right_rate(2,:),t,right_rate(3,:),t,right_rate(4,:),t,right_rate(5,:));
legend('robot','human','robot&human','robot(without w update)','robot&human(without w update)');
