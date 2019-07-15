function probability = predict_attack_strategy(pos_opponent)
%Conservative 1, Balance 2, Radical 3
probability=zeros(3,1);

left_points=[];
right_points=[];
middle_points=[];

K=convhull(pos_opponent);

for i=1:length(K)-1
    if pos_opponent(K(i),1)>=200
        right_points=[right_points;pos_opponent(K(i),:)];
    elseif pos_opponent(K(i),1)<=-200
        left_points=[left_points;pos_opponent(K(i),:)];
    else
        middle_points=[middle_points;pos_opponent(K(i),:)];
    end
    
    if (pos_opponent(K(i),1)<200 && pos_opponent(K(i+1),1)>=200) ||...
       (pos_opponent(K(i),1)>=200 && pos_opponent(K(i+1),1)<200)
        d=(pos_opponent(K(i+1),2)-pos_opponent(K(i),2))/(pos_opponent(K(i+1),1)-pos_opponent(K(i),1));
        tmp_x=200;
        tmp_y=pos_opponent(K(i),2)+d*(200-pos_opponent(K(i),1));
        middle_points=[middle_points;[tmp_x,tmp_y]];
        right_points=[right_points;[tmp_x,tmp_y]];
    end
    
    if (pos_opponent(K(i),1)<-200 && pos_opponent(K(i+1),1)>=-200) ||...
       (pos_opponent(K(i),1)>=-200 && pos_opponent(K(i+1),1)<-200)
        d=(pos_opponent(K(i+1),2)-pos_opponent(K(i),2))/(pos_opponent(K(i+1),1)-pos_opponent(K(i),1));
        tmp_x=-200;
        tmp_y=pos_opponent(K(i),2)+d*(-200-pos_opponent(K(i),1));
        left_points=[left_points;[tmp_x,tmp_y]];
        middle_points=[middle_points;[tmp_x,tmp_y]];
    end
end

if ~isempty(left_points)
    probability(1)=polyarea(left_points(:,1),left_points(:,2));
else
    probability(1)=0;
end
if ~isempty(middle_points)
    probability(2)=polyarea(middle_points(:,1),middle_points(:,2));
else
    probability(2)=0;
end
if ~isempty(right_points)
    probability(3)=polyarea(right_points(:,1),right_points(:,2));
else
    probability(3)=0;
end

probability=probability/sum(probability);
end




