function wseed=interpolateseeds(data)
  
for i=1:length(data.pos(1,:))-1
    wdata.pos(:,i)=(data.pos(:,i)+data.pos(:,i+1))/2;
    wdata.vel(:,i)=(data.vel(:,i)+data.vel(:,i+1))/2;
    wdata.acc(:,i)=(data.acc(:,i)+data.acc(:,i+1))/2;
    wdata.t(:,i)=(data.t(:,i)+data.t(:,i+1))/2;
   % wdata.Fc1(:,i)=(data.Fc1(:,i)+data.Fc1(:,i+1))/2;

end


for i=2:2:2*(length(data.pos(1,:))-1)
    Newdata.pos(:,i-1)=data.pos(:,i/2);
    Newdata.pos(:,i)=wdata.pos(:,i/2);
    
    Newdata.vel(:,i-1)=data.vel(:,i/2);
    Newdata.vel(:,i)=wdata.vel(:,i/2);
    
    Newdata.acc(:,i-1)=data.acc(:,i/2);
    Newdata.acc(:,i)=wdata.acc(:,i/2);
    
    Newdata.t(:,i-1)=data.t(:,i/2);
    Newdata.t(:,i)=wdata.t(:,i/2);
    
%     Newdata.Fc1(:,i-1)=data.Fc1(:,i/2);
%     Newdata.Fc1(:,i)=wdata.Fc1(:,i/2);
end

    
i=2*(length(data.pos(1,:))-1)+1
    Newdata.pos(:,i)=data.pos(:,end);
    Newdata.vel(:,i)=data.vel(:,end);
    Newdata.acc(:,i)=data.acc(:,end);
    Newdata.t(:,i)=data.t(:,end);
    %Newdata.Fc1(:,i)=data.Fc1(:,end);
   
%for inputs and vertical GRF in double support     
nDounble=round(length(data.pos(1,:))/2);

for i=1:nDounble-1
    wdata.input_D(:,i)=(data.input_D(:,i)+data.input_D(:,i+1))/2;
  %  wdata.Fc2(:,i)=(data.Fc2(:,i)+data.Fc2(:,i+1))/2;
end

for i=2:2:2*(nDounble-1)
    Newdata.input_D(:,i-1)=data.input_D(:,i/2);
    Newdata.input_D(:,i)=wdata.input_D(:,i/2);
    
%     Newdata.Fc2(:,i-1)=data.Fc2(:,i/2);
%     Newdata.Fc2(:,i)=wdata.Fc2(:,i/2);
end

iend=2*(nDounble-1)+1;
Newdata.input_D(:,iend)=data.input_D(:,end);
%Newdata.Fc2(:,iend)=data.Fc2(:,end);

for i=1:length(data.pos(1,:))-nDounble-1
    wdata.input_S(:,i)=(data.input_S(:,i)+data.input_S(:,i+1))/2;
end

for i=2:2:2*(length(data.pos(1,:))-nDounble-1)

    Newdata.input_S(:,i-1)=data.input_S(:,i/2);
    Newdata.input_S(:,i)=wdata.input_S(:,i/2);
    
end

iend=2*(length(data.pos(1,:))-nDounble-1)+1;

if iend~=length(Newdata.t)-length(Newdata.input_D)
    for i=iend:length(Newdata.t)-length(Newdata.input_D)
        Newdata.input_S(:,i)=data.input_S(:,end);
    end
else
    Newdata.input_S(:,iend)=data.input_S(:,end);
end


wseed.q = Newdata.pos;
wseed.qd = Newdata.vel;
wseed.qdd = Newdata.acc;
wseed.t = Newdata.t;
% wseed.Fc_1 = Newdata.Fc1;
% wseed.Fc_2 = Newdata.Fc2;
wseed.aD = data.aD;
wseed.aS = data.aS;
wseed.uS = Newdata.input_S;
wseed.uD = Newdata.input_D;