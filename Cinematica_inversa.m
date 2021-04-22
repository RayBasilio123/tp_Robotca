%Calcula Cinematica Inversa para o ponto p0_1=[-19.75,12.2];
%%
p0_1=[-19.75,12.2];
a1=20;
a2=10;
Y=p0_1(1,2);
X=p0_1(1,1);
D = (((X^2)+(Y^2)-(a1^2)-(a2^2))/(2*a1*a2));
theta2_r = atand(-(sqrt(1-(D^2)))/(D)); 
Theta2= theta2_r +90;
A=rad2deg(atan2(Y,X));
T=acos(((X^2)+(Y^2)+(a1^2)-(a2^2))/(2*a1*(sqrt((X^2)+(Y^2)))));
B=rad2deg(T);

Theta1_1 = A+B%%
Theta1_2 = A-B%%

disp('theta 1   -> p1');
disp(Theta1_1);
disp('theta 2 + 90°-> p1 ');
disp(Theta2);




%%    
%Calcula Cinematica Inversa para o ponto p0_2=[20.36 ,17.3];
%%
p0_2=[20.36 ,17.3];

a1=20;
a2=10;


Y=p0_2(1,2);
X=p0_2(1,1);

D = (((X^2)+(Y^2)-(a1^2)-(a2^2))/(2*a1*a2));

theta2_r = atand(-(sqrt(1-(D^2)))/(D)); %%
Theta2= theta2_r+90;
A=rad2deg(atan2(Y,X));

T=acos(((X^2)+(Y^2)+(a1^2)-(a2^2))/(2*a1*(sqrt((X^2)+(Y^2)))));
B=rad2deg(T);

Theta1_1 = A+B;%%-> cotovelo abaixo
Theta1_2 = A-B;

disp('theta 1 -> p2 ');
disp(Theta1_1);

disp('theta 2 + 90° -> p2 ');
disp(Theta2);

