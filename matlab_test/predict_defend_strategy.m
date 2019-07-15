function probability = predict_defend_strategy(pos_nubot, pos_opponent)
%Regional 1, Man2Man 2, Focus 3
probability=zeros(3,1);

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
probability(1)=DB;

% calculate the DBI for man2man
% Si
tmp_opponent=zeros(1,4);
for i=1:length(pos_nubot)
    less_distance=10000;
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
probability(2)=DB;
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
probability(3)=DB;

if sum(probability)==0
    probability(1)=1/3;
    probability(2)=1/3;
    probability(3)=1/3;
else
    probability=probability/sum(probability);
end
end




