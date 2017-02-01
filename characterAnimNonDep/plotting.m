clear;


fileID = fopen('rhumerus_axis.txt','r');
formatSpec = '%f %f %f %f';
sizeA = [4 Inf];
A = fscanf(fileID,formatSpec, sizeA);
fclose(fileID);

figure ;
hold on ; 

plot(A(1,:),  A(2,:), 'b');

hold on ; 
plot(A(1,:),  A(3,:), 'g'); 

hold on ;
plot(A(1,:),  A(4,:), 'r');

hold off ; 
