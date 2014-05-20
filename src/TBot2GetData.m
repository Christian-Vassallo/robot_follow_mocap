function TBot2GetData(message)
global TBot2MSG;
%disp([sprintf('Message received: '), message.getData()]);
%disp(message.getData());
%disp('trigger');
TBot2MSG = message.getData();

end