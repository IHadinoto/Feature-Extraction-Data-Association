close all;

%% Part A

Data=load('Measurements_AAS01.mat');       %We load the data
Data=Data.A;
Map=load('MapMTRN4010.mat');
Map=Map.Map;
Mapx=(Map.x)';
Mapy=(Map.y)';

t=Data.t; % sample times, {t}.     
t=double(t)*0.0001; %scale time to seconds; original data is integer type, 1 count = 0.1 ms.

% Scale speed to m/s ; originally in mm/s, integer type.
% scale heading rate to degrees/s ; originally in integer type, [1 count = 1/100 degrees/sec].
w = double(Data.Z(2,:))/100;                    % GyroZ readings, "{w(t)}"
gb = mean(w(1:4256)); % Gyro Bias
wgb = zeros(length(w),1);

% Gyro bias
for i=1:length(w)
    wgb(i) = w(i)-gb;
end
v = double(Data.Z(1,:))/1000;                     % speed readings   "{v(t)}" 
    
% plot those measurements.
figure;
subplot(211) ; plot(t,v);  xlabel('time (seconds)'); ylabel('speed (m/s)');
subplot(212) ; plot(t,wgb); xlabel('time (seconds)'); ylabel('angular rate (degrees/s)');

% Starting Values for robot going to the right
X = [0;0;0] ;  % initial pose. You will need it.

for k = 1:length(v)-1
    dt = t(k+1)-t(k);
    dX = [v(k)*cos(X(3,k));v(k)*sin(X(3,k));deg2rad(wgb(k))];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
theta = X(3,:); % save the value of theta

% Starting Values for robot going straight
X = [0;0;pi/2] ;  % initial pose. You will need it.

for k = 1:length(v)-1
    dt = t(k+1)-t(k);
    dX = [v(k)*cos(X(3,k));v(k)*sin(X(3,k));deg2rad(wgb(k))];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end

figure;plot(X(1,:),X(2,:));
grid on
xlabel('x axis')
ylabel('y axis')
title('Path of Robot using Model')
legend('Path')

%% Parts B,C,D,E
% Plotting Local Frame
figure; subplot(1,2,1);hold on;grid on;axis([-15,15,-5,15]); 
hL=plot(0,0,'.');
hL2=plot(0,0,'.r');  % for showing brillant points, later.
hL3=plot(0,0,'or');  % for showing poles
hL4=plot([0 -0.5 0.5 0 0 -10 10 0],[0.5 -0.5 -0.5 0.5 0 0 0 0],'-g');  % for showing robot
legend({'Points','Brilliant Points','Poles','Robot'});
title('LiDAR scans Local Frame');
xlabel('X distance from Robot');ylabel('Y distance from Robot'); 

% Plotting Global Frame
subplot(1,2,2);hold on;grid on;axis([-15,15,-10,10]);
hG=plot(0,0,'.');
hG2=plot(0,0,'.r');  % for showing brillant points, later.
hG3=plot(0,0,'or');  % for showing poles
hG4=plot(0,0,'-g');  % for showing robot
hM=plot(Mapx,Mapy,'sr');  % for showing Map pole points
hE=plot(0,0,'-r'); % for showing EKF estimator 1
hE2=plot(0,0,'-r'); % for showing EKF estimator 2
hE3=plot(0,0,'-r'); % for showing EKF estimator 3
hE4=plot(0,0,'-r'); % for showing EKF estimator 4
hE5=plot(0,0,'-r'); % for showing EKF estimator 5
legend({'Points','Brilliant Points','Poles','Robot','Actual Pole Positions'});
title('LiDAR scans Global Frame');
xlabel('X distance');ylabel('Y distance'); 


L=Data.L;       %number of samples in this dataset.
i0=1; % you should start at i0=1 or close. (up to 52870)

for i=i0:L
    tic
    m = Data.Z(:,i);
    indexScan = m(3);

    % Now, if there is a LiDAR scan at this time?
    if (indexScan>1)

        %extract ranges and intensities, of the 361 "pixels"
        [r,I]=GetRangeAndIntensityFromRawScan(Data.scans(:,indexScan));

        [xcoor,ycoor,xseg,yseg,xrob,yrob,xgf,ygf,xgfseg,ygfseg,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessLidar_z5161724_Hadinoto_Ian(r,I,X,i,theta,Mapx,Mapy);
        %here you may process the LiDAR
        %data.  variable "X" is the current pose of the platform
        %(assuming you keep estimating it). If your procesing of the
        %LiDAR data does require it.

        % if we want to show Local Frame, ...
        set(hL,'xdata',xcoor,'ydata',ycoor);
        % which points do have intensity>0??
        ii=find(I>0); set(hL2,'xdata',xcoor(ii),'ydata',ycoor(ii));
        % Grouping brilliant points
        set(hL3,'xdata',xseg,'ydata',yseg);
        
        % if we want to show Global Frame, ...
        set(hG,'xdata',xgf,'ydata',ygf);
        % which points do have intensity>0??
        set(hG2,'xdata',xgf(ii),'ydata',ygf(ii));
        % Grouping brilliant points
        set(hG3,'xdata',xgfseg,'ydata',ygfseg);
        % Showing Robot moving
        set(hG4,'xdata',xrob,'ydata',yrob);
        % Showing EKF
        set(hE,'xdata',linx,'ydata',liny);
        set(hE2,'xdata',linx2,'ydata',liny2);
        set(hE3,'xdata',linx3,'ydata',liny3);
        set(hE4,'xdata',linx4,'ydata',liny4);
        set(hE5,'xdata',linx5,'ydata',liny5);
        pause(0.001);  % short pause, just to see some animation, more slowly.You may change this.
    end
    toc
end
%-------------------------------------------------------
