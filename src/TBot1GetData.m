function TBot1GetData(message)
global TBot1MSG;
%disp([sprintf('Message received: '), message.getData()]);
%disp(message.getData());
%disp('trigger');
TBot1MSG = message.getData();

end