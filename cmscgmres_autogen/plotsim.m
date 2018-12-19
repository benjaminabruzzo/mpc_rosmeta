% A Program to Plot a Simulation Result of RHC
%
%            T. Ohtsuka,  March 2000
%

% fname = 'agex00'; % Header of filenames
fname = 'agsad00'; % Header of filenames
xf=[];


%-------------------------------
eval(['load ',fname,'x.m;']); 
eval(['xs = ',fname,'x;']);
eval(['clear ',fname,'x;']);

eval(['load ',fname,'u.m;']); 
eval(['us = ',fname,'u;']);
eval(['clear ',fname,'u;']);

eval(['load ',fname,'e.m;']); 
eval(['es = ',fname,'e;']);
eval(['clear ',fname,'e;']);

eval([fname,'c;']); 

dt = ht*dstep;
ts = 0:dt:tsim;

ns = length(xs)
ts = ts(1:ns);
xs = xs(1:ns,:);
us = us(1:ns,:);
es = es(1:ns,:);
normes = sqrt(sum(es'));  % Is this OK for your Problem? 
maxerr=max(normes)
aveerr=mean(normes)

%-------------------------------
dimxs = size(xs,2);
dimus = size(us,2);
plotno = dimxs+dimus+1;
plotnox = floor(plotno/3);
plotnoy = ceil(plotno/plotnox);

clf;

for j=1:dimxs
 if isempty(xf)
   subplot(plotnoy,plotnox,j), plot(ts,xs(:,j)); 
 else
   subplot(plotnoy,plotnox,j), plot(ts,xs(:,j),[0 tsim],[xf(j) xf(j)],':'); 
 end
   xlabel('Time'); ylabel(['x',num2str(j)]); 
end
for j=1:dimus
   subplot(plotnoy,plotnox,dimxs+j), plot(ts,us(:,j));
   xlabel('Time'); ylabel(['u',num2str(j)]);
end
subplot(plotnoy,plotnox,plotno), plot(ts,normes);
xlabel('Time'); ylabel('Error');
