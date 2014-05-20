function function3(message)
global ActorMSG;
%disp([sprintf('Message received: '), message.getData()]);
%disp(message.getData());
%disp('trigger');
ActorMSG = message.getData();

end



