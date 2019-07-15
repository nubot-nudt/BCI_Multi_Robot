function [strategy_A,strategy_B,strategy_C,count_different, count_accept, count_right] = strategy_selection_attack(times)
position=figure;
%probability=figure;
%compare=figure;
payoff_matrix=[1,3,5;3,3,3;5,3,1];
loss_matrix=[5,1,3;3,3,3;1,3,5];
random_payoff=zeros(1,times);
nubot_payoff=zeros(1,times);
final_payoff=zeros(1,times);
count_different=0;
count_accept=0;
count_right=0;
strategy_A=0;
strategy_B=0;
strategy_C=0;
%Regional 1, Man2Man 2, Focus 3
for test_time=1:times
    
    our_strategy=randi([1,3],1);
    defend_strategy=randi([1,3],1);
    loop_times=20;
    
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
    
    vel_opponent=zeros(4,2);
    label_opponent=zeros(1,4);
    mini_times=zeros(1,4);
    
    DBI_record=zeros(3,loop_times);
    OPP_x_record=zeros(4,loop_times);
    OPP_y_record=zeros(4,loop_times);
    
    axes1 = axes('Parent',position);
    axis equal;
    hold on;
    ylim(axes1,[-600 600]);
    xlim(axes1,[-900 900]);
    hold(axes1,'on');
    
    img=imread('field.png');
    min_x= -900;
    max_x=900;
    min_y=-600;
    max_y=600;
    imagesc([min_x max_x], [min_y max_y], flip(img,1));
    
    plot(pos_nubot(:,1),pos_nubot(:,2),'ko','markersize',12,'MarkerFaceColor','k');
    plot(pos_opponent(:,1),pos_opponent(:,2),'rx','LineWidth',2,'markersize',12,'MarkerFaceColor','r');
    
    if defend_strategy==1
        for i=1:length(pos_regional)
            less_distance=2000;
            tmp_label=0;
            for j=1:length(label_opponent)
                if label_opponent(j)==0
                    distance=norm(pos_regional(i,:)-pos_opponent(j,:));
                    if distance<less_distance
                        less_distance=distance;
                        tmp_label=j;
                    end
                end
                if j==length(label_opponent)
                    label_opponent(tmp_label)=i;
                end
            end
        end
        
        for i=1:length(label_opponent)
            vel_opponent(i,:)=10*(pos_regional(label_opponent(i),:)-pos_opponent(i,:))...
                /norm(pos_regional(label_opponent(i),:)-pos_opponent(i,:));
            mini_times(i)=floor(norm(pos_regional(label_opponent(i),:)-pos_opponent(i,:))/10);
        end
        
    elseif defend_strategy==2
        for i=1:length(pos_nubot)
            less_distance=2000;
            tmp_label=0;
            for j=1:length(label_opponent)
                if label_opponent(j)==0
                    distance=norm(pos_nubot(i,:)-pos_opponent(j,:));
                    if distance<less_distance
                        less_distance=distance;
                        tmp_label=j;
                    end
                end
                if j==length(label_opponent)
                    label_opponent(tmp_label)=i;
                end
            end
        end
        
        for i=1:length(label_opponent)
            vel_opponent(i,:)=10*(pos_nubot(label_opponent(i),:)-pos_opponent(i,:))...
                /norm(pos_nubot(label_opponent(i),:)-pos_opponent(i,:));
            mini_times(i)=floor(norm(pos_nubot(label_opponent(i),:)-pos_opponent(i,:))/10);
        end
        
    elseif defend_strategy==3
        for i=1:length(label_opponent)
            vel_opponent(i,:)=10*(pos_nubot(1,:)-pos_opponent(i,:))/norm(pos_nubot(1,:)-pos_opponent(i,:));
            mini_times(i)=floor(norm(pos_nubot(1,:)-pos_opponent(i,:))/10);
        end
    end
    
    for n=1:loop_times
        for j=1:length(pos_opponent)
            if n<mini_times(j)
                pos_opponent(j,:)=pos_opponent(j,:)+vel_opponent(j,:);
            end
            OPP_x_record(j,n)=pos_opponent(j,1);
            OPP_y_record(j,n)=pos_opponent(j,2);
        end
        % calculate the DBI for regional
        % Si
        less_distance=2000;
        tmp_label=0;
        for j=1:length(pos_opponent)
            distance=norm(pos_nubot(1,:)-pos_opponent(j,:));
            if distance<less_distance
                less_distance=distance;
                tmp_label=j;
            end
        end
        
        A=pos_nubot;B=[];
        for j=1:length(pos_opponent)
            if j~=tmp_label
                B=[B;pos_opponent(j,:)];
            end
        end
        A_center=mean(A);
        B_center=mean(B);
        sum_A=0;sum_B=0;
        
        for i=1:length(A)
            sum_A=sum_A+norm(A(i,:)-A_center);
        end
        S_A=sqrt(sum_A/length(A));
        
        for i=1:length(B)
            sum_B=sum_B+norm(B(i,:)-B_center);
        end
        S_B=sqrt(sum_B/length(B));
        % Mij
        M_AB=norm(A_center-B_center);
        % Rij
        R_AB=(S_A+S_B)/M_AB;
        % Di
        D=R_AB;
        % DB
        DB=D;
        DBI_record(1,n)=DB;
        
        % calculate the DBI for man2man
        % Si
        tmp_opponent=zeros(1,4);
        for i=1:length(pos_nubot)
            less_distance=2000;
            tmp_label=0;
            for j=1:length(tmp_opponent)
                if tmp_opponent(j)==0
                    distance=norm(pos_nubot(i,:)-pos_opponent(j,:));
                    if distance<less_distance
                        less_distance=distance;
                        tmp_label=j;
                    end
                end
                if j==length(tmp_opponent)
                    tmp_opponent(tmp_label)=i;
                end
            end
        end
        
        A=[pos_opponent(1,:);pos_nubot(tmp_opponent(1),:)];
        B=[pos_opponent(2,:);pos_nubot(tmp_opponent(2),:)];
        C=[pos_opponent(3,:);pos_nubot(tmp_opponent(3),:)];
        D=[pos_opponent(4,:);pos_nubot(tmp_opponent(4),:)];
        
        A_center=mean(A);
        B_center=mean(B);
        C_center=mean(C);
        D_center=mean(D);
        sum_A=0;sum_B=0;sum_C=0;sum_D=0;
        
        for i=1:length(A)
            sum_A=sum_A+norm(A(i,:)-A_center);
        end
        S_A=sqrt(sum_A/length(A));
        
        for i=1:length(B)
            sum_B=sum_B+norm(B(i,:)-B_center);
        end
        S_B=sqrt(sum_B/length(B));
        
        for i=1:length(C)
            sum_C=sum_C+norm(C(i,:)-C_center);
        end
        S_C=sqrt(sum_C/length(C));
        
        for i=1:length(D)
            sum_D=sum_D+norm(D(i,:)-D_center);
        end
        S_D=sqrt(sum_D/length(D));
        
        % Mij
        M_AB=norm(A_center-B_center);
        M_AC=norm(A_center-C_center);
        M_AD=norm(A_center-D_center);
        M_BC=norm(B_center-C_center);
        M_BD=norm(B_center-D_center);
        M_CD=norm(C_center-D_center);
        
        % Rij
        R_AB=(S_A+S_B)/M_AB;
        R_AC=(S_A+S_C)/M_AC;
        R_AD=(S_A+S_D)/M_AD;
        R_BC=(S_B+S_C)/M_BC;
        R_BD=(S_B+S_D)/M_BD;
        R_CD=(S_C+S_D)/M_CD;
        % Di
        D_A=max([R_AB,R_AC,R_AD]);
        D_B=max([R_AB,R_BC,R_BD]);
        D_C=max([R_AC,R_BC,R_CD]);
        D_D=max([R_AD,R_BD,R_CD]);
        % DB
        DB=(D_A+D_B+D_C+D_D)/4;
        DBI_record(2,n)=DB;
        %
        
        % calculate the DBI for focus
        % Si
        A=[pos_opponent;pos_nubot(1,:)];
        B=[pos_nubot(2,:)];
        C=[pos_nubot(3,:)];
        D=[pos_nubot(4,:)];
        
        A_center=mean(A);
        B_center=mean(B);
        C_center=mean(C);
        D_center=mean(D);
        sum_A=0;sum_B=0;sum_C=0;sum_D=0;
        
        for i=1:length(A)
            sum_A=sum_A+norm(A(i,:)-A_center);
        end
        S_A=sqrt(sum_A/length(A));
        
        S_B=0;
        S_C=0;
        S_D=0;
        
        % Mij
        M_AB=norm(A_center-B_center);
        M_AC=norm(A_center-C_center);
        M_AD=norm(A_center-D_center);
        M_BC=norm(B_center-C_center);
        M_BD=norm(B_center-D_center);
        M_CD=norm(C_center-D_center);
        
        % Rij
        R_AB=(S_A+S_B)/M_AB;
        R_AC=(S_A+S_C)/M_AC;
        R_AD=(S_A+S_D)/M_AD;
        R_BC=(S_B+S_C)/M_BC;
        R_BD=(S_B+S_D)/M_BD;
        R_CD=(S_C+S_D)/M_CD;
        % Di
        D_A=max([R_AB,R_AC,R_AD]);
        D_B=max([R_AB,R_BC,R_BD]);
        D_C=max([R_AC,R_BC,R_CD]);
        D_D=max([R_AD,R_BD,R_CD]);
        % DB
        DB=(D_A+D_B+D_C+D_D)/4;
        DBI_record(3,n)=DB;
        %
    end
    
    %     for i=1:length(pos_opponent)
    %         scatter(OPP_x_record(i,:),OPP_y_record(i,:),'>');
    %     end
    
    %     figure(probability);
    %     hold on;
    %     t=1:loop_times;
    %     plot(t,DBI_record(1,:),t,DBI_record(2,:),t,DBI_record(3,:));
    %     legend('Regional', 'Man2Man', 'Focus');
    
    P1=max(DBI_record(1,:))-DBI_record(1,loop_times);
    P2=max(DBI_record(2,:))-DBI_record(2,loop_times);
    P3=max(DBI_record(3,:))-DBI_record(3,loop_times);
    theta=1/(P1+P2+P3);
    P1=theta*P1;
    P2=theta*P2;
    P3=theta*P3;
    
    %     E_conservative=P1+2*P2+3*P3;
    %     E_balance=2*P1+2*P2+2*P3;
    %     E_radical=3*P1+2*P2+P3;
    
    E_payoff=[P1 P2 P3]*payoff_matrix;
    [value,robot_strategy]=max(E_payoff);
    
    random_strategy=randi([1,3],1);
    
    if defend_strategy==1
        human_strategy=randsrc(1,1,[[1 2 3]; [0.1 0.1 0.8]]);
    elseif defend_strategy==2
        human_strategy=randsrc(1,1,[[1 2 3]; [0.1 0.8 0.1]]);
    else
        human_strategy=randsrc(1,1,[[1 2 3]; [0.8 0.1 0.1]]);
    end
    
    if robot_strategy~=human_strategy
        count_different=count_different+1;
        %         E_loss=[P1 P2 P3]*loss_matrix;
        %         robot_human=E_payoff(robot_strategy)*E_loss(human_strategy);
        %         human_robot=E_payoff(human_strategy)*E_loss(robot_strategy);
        %
        %         if robot_human>human_robot
        %             final_strategy=robot_strategy;
        %         else
        %             final_strategy=human_strategy;
        %         end
        if human_strategy==1
            payoff_modify=payoff_matrix*[0.8, 0, 0;0, 0.1, 0;0, 0, 0.1];
        elseif human_strategy==2
            payoff_modify=payoff_matrix*[0.1, 0, 0;0, 0.8, 0;0, 0, 0.1];
        else
            payoff_modify=payoff_matrix*[0.1, 0, 0;0, 0.1, 0;0, 0, 0.8];
        end
        
        E_payoff=[P1 P2 P3]*payoff_modify;
        [value,final_strategy]=max(E_payoff);
        if final_strategy~=robot_strategy
            count_accept=count_accept+1;
            if defend_strategy==1&&final_strategy==2
                count_right=count_right+1;
            elseif defend_strategy>1&&final_strategy==1
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
        random_payoff(test_time)=(random_payoff(test_time-1)*(test_time-1)+payoff_matrix(defend_strategy,random_strategy))/test_time;
        nubot_payoff(test_time)=(nubot_payoff(test_time-1)*(test_time-1)+payoff_matrix(defend_strategy,robot_strategy))/test_time;
        final_payoff(test_time)=(final_payoff(test_time-1)*(test_time-1)+payoff_matrix(defend_strategy,final_strategy))/test_time;
    else
        random_payoff(test_time)=payoff_matrix(defend_strategy,random_strategy);
        nubot_payoff(test_time)=payoff_matrix(defend_strategy,robot_strategy);
        final_payoff(test_time)=payoff_matrix(defend_strategy,final_strategy);
    end
end
% t=1:test_time;
% figure(compare);
% plot(t,random_payoff,t,nubot_payoff,t,final_payoff);
% legend('random', 'robot','robot&human');
end

    
    
    
