%% Turtlebot 1

clear all
close all
clc

global ActorMSG TBot1MSG TBot2MSG

%setenv('ROS_MASTER_URI','http://192.168.43.52:11311')
%setenv('ROS_IP','192.168.43.109')

setenv('ROS_MASTER_URI','http://localhost:11311')
setenv('ROS_IP','localhost')

% Create a new node named /NODE and connect it to the master.
%node = rosmatlab.node('matlab', [], [], 'rosIP', '10.35.12.62');
%node = rosmatlab.node('matlab', 'http://10.35.12.62:11311');
%node = rosmatlab.node('matlab', '192.168.43.52', 11311);
node = rosmatlab.node('matlab', 'http://localhost:11311');

% Publisher 
publisher = rosmatlab.publisher('InitRobPos','std_msgs/Float32MultiArray',node);

% Subscriber 
subscriberActor = node.addSubscriber('ActorMoCapData2','std_msgs/Float64MultiArray',1);
subscriberActor.setOnNewMessageListeners({@ActorGetData});

subscriberTBot1 = node.addSubscriber('TBot1MoCapData','std_msgs/Float64MultiArray',1);
subscriberTBot1.setOnNewMessageListeners({@TBot1GetData});

subscriberTBot2 = node.addSubscriber('TBot2MoCapData','std_msgs/Float64MultiArray',1);
subscriberTBot2.setOnNewMessageListeners({@TBot2GetData});

% Create a new message of type std_msgs/String.
msg = rosmatlab.message('std_msgs/Float32MultiArray',node);

% Update the data field of the message and then publish the message
% iteratively.

display('waiting');
% while (isempty(ActorMSG))
%     % Waiting that the data arrives
%     %disp(isempty(ActorMSG));
%     pause(1/150)
% end
disp('event');
load data2

fcutoff=0.4;
f=fcutoff * 2/150;

% Extract Actor Data about POS and VEL
ActorPx = filtrateTraj(ActorMSG(1:3:size(ActorMSG)), f, 1);
ActorPy = filtrateTraj(ActorMSG(2:3:size(ActorMSG)), f, 1);
ActorYaw = ActorMSG(3:3:size(ActorMSG));

% Computing the velocity of the last meter
ActorVx = derivate(ActorPx(find(ActorPy>ActorPy(end)-1.0)), 1/150);
ActorVy = derivate(ActorPy(find(ActorPy>ActorPy(end)-1.0)), 1/150);


figure(1)
hold on
plot(ActorPx,'r')
plot(ActorPy,'g');
hold off

figure(2)
hold on
plot(ActorVx,'r')
plot(ActorVy,'g');
hold off

% Position of Actor on X and Y at time 0 
% (only last meter considered to compute the mean value)

PAx0 = mean(ActorPx(find(ActorPy>ActorPy(end)-1.0)));
PAy0 = mean(ActorPy(find(ActorPy>ActorPy(end)-1.0)));

% Velocity of Actor on X and Y 
% (only last meter considered to compute the mean value)

load datafile
VAx = mean(VAxTrials);
VAy = mean(VAyTrials);

% Define cross point 
PAxf = PAx0;
PAyf = 0.9166;
PRy0 = PAyf;

% Compute the time Actor Init-Goal given the velocity
t = (PAyf - PAy0)/VAy;

% Velocity of the Robot
VRx = -0.71;
VRy = 0;
ARxmax = 0.4946;
ARxmax = 0;
tgammaMax = 1.2733;

% Compute the mpd
mpd = 0;

% Compute Init Robot Pos for mpd = 0
[PRx0mpd0, PRy0mpd0] = computeinitposrobot(VRx, VRy, 0, PAxf, PAyf, t, tgammaMax);

% Compute Init Robot Pos for mpd set
[PRx0mpdset, PRy0mpdset] = computeinitposrobot(VRx, VRy, mpd, PAxf, PAyf, t, tgammaMax);

if (mpd<0)
    PRx0 = PRx0mpd0 - abs(PRx0mpd0-PRx0mpdset);
else
    PRx0 = PRx0mpdset;
end

PRxy0 = [PRx0; PRy0; PAxf];

disp('publishing');
for i = 1:100*1000
    msg.setData(PRxy0);


   publisher.publish(msg);
    pause(0.030)
end



% Remove the subscriber from the node.
node.removeSubscriber(subscriberActor);
node.removeSubscriber(subscriberTBot1);
node.removeSubscriber(subscriberTBot2);
node.removePublisher(publisher);

% Delete the master.


%% Clean up workspace.

