function res = derivate(data, step)

% 20070328
% Cette fonction derive un ensemble de donnees.
% Ces donnees doivent etre fournies avec une colonne
% pour chaque type de donnee et une ligne par instant
% Entrees :
%		- data : donnee a deriver (tableau)
%		- step : pas de temps entre deux points (float)

% Alloue meme taille de tableau
res=data;
n=size(data,1);
for i=2:n-1,
	res(i,:)=(data(i+1,:)-data(i-1,:))/(2*step);
end;
res(1,:)=(data(2,:)-data(1,:))/step;
res(n,:)=(data(n,:)-data(n-1,:))/step;
