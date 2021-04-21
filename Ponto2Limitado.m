clc;
clear all;

%Rob� Planar RR
%%
%Limites Mec�nicos do Servo do Levantamento do Bra�o:
    % M�nimo: 0�.
    % M�ximo: +180�.
%Limites Mec�nicos da Servo do Cotovelo do Bra�o:
    % M�nimo: -50�.
    % M�ximo: +90�.
%%    
%Defini��o dos par�metros do Rob� RR
%%
a1=20;
a2=10;
alfa=-pi/2;% par�metro necess�rio para que a posi��o zero do rob� tenha a configura��o desejada na qual o eixo x2 (cotovelo)  est� a -90� em rela��o a x1 (levantamento do bra�o)                                                                                       
%% 

%Espa�o de Trabalho
%%
i=0;

theta1_limInf=0;%limite inferior de theta 1 em graus
theta1_limSup=180;%limite superior de theta 1 em graus
theta2_limInf=-50;%limite inferior de theta 2 em graus
theta2_limSup=90;%limite superior de theta 2 em graus

theta1_limInf=deg2rad(theta1_limInf);
theta1_limSup=deg2rad(theta1_limSup);
theta2_limInf=deg2rad(theta2_limInf);
theta2_limSup=deg2rad(theta2_limSup);


%for theta1=theta1_limInf:(theta1_limSup-theta1_limInf)/199:theta1_limSup,
 %   for theta2=theta2_limInf:(theta2_limSup-theta2_limInf)/199:theta2_limSup,
  %      i=i+1;
        
   %     x(i)=a1*cos(theta1)+a2*cos(theta1+theta2+alfa);
    %    y(i)=a1*sin(theta1)+a2*sin(theta1+theta2+alfa);
     %end
%end
%plot(x,y,'.b');
%hold on;
for theta1=theta1_limInf:(theta1_limSup-theta1_limInf)/199:theta1_limSup,
    for theta2=theta2_limInf:(theta2_limSup-theta2_limInf)/199:theta2_limSup,
        i=i+1;
        x(i)=a1*cos(theta1)+a2*cos(theta1+theta2+alfa);
        y(i)=a1*sin(theta1)+a2*sin(theta1+theta2+alfa);
        if (x(i)<=20.38 && x(i)>=20.32 && y(i)>=17.2 && y(i)<=17.3)
            disp(rad2deg(theta1))
            disp(rad2deg(theta2))
            disp('a')
        end
     end
end
plot(x,y,'.b');
hold on;

%%

%Defini��o da Trajet�ria Retil�nea

% Ponto 1  
%p0_r=[-19.75,12.2];
%p1_r=[18.37,19.07];

% Ponto 2
p0_r=[20.36,17.3];
p1_r=[-6.165,12.47];

Dx=p1_r(1)-p0_r(1);
Dy=p1_r(2)-p0_r(2);
A=Dy/Dx;
b=p0_r(1,2)-A*p0_r(1,1);
k=0;
n_pontos_r=80;
for i=p0_r(1,1):(p1_r(1,1)-p0_r(1,1))/(n_pontos_r-1):p1_r(1,1),
    k=k+1;
    tx_r(k)=i;
    ty_r(k)=A*tx_r(k)+b ;
end
%%

%Defini��o do Tempo da Trajet�ria Retil�nea e do Intervalo de Amostragem
%%
Tempo_r=30e-3; % Intervalo de amostragem igual a 40 milissegundos

T_r(1)=0;
for i=2:n_pontos_r,
    T_r(i)=T_r(i-1)+Tempo_r;
end

%%

%Defini��o dos Intervalos de Tempo da Trajet�ria Retil�nea
%%
dT_r=diff(T_r);
%%

%Controlador Proporcional para Trajet�ria Retil�nea
%%
K_r=[10,0;0,10];
%%

%Posi��o Inicial
%%
%  theta1_r=173.6823;%em graus
%  theta2_r=5.5812;%em graus

theta1_r=58.7955;%em graus
theta2_r=32.3133;%em graus

theta1_r=deg2rad(theta1_r);%em rad
theta2_r=deg2rad(theta2_r);%em rad

q_r(:,1)=[theta1_r;
        theta2_r];
%%  

%Defini��o das velocidade desejadas do efetuador para trajet�ria Retil�nea
%%    
dtx_r=diff(tx_r);
dty_r=diff(ty_r);
%%

%CONTROLE DE TRAJET�RIA RETIL�NEA
%%
for i=1:n_pontos_r,
    X_pot_r(:,i)=[a1*cos(q_r(1,i))+a2*cos(q_r(1,i)+q_r(2,i)+alfa);
                a1*sin(q_r(1,i))+a2*sin(q_r(1,i)+q_r(2,i)+alfa)];
    Xd_r(:,i)=[tx_r(i);
               ty_r(i)];
    e_r(:,i)=Xd_r(:,i)-X_pot_r(:,i);
    if(i<n_pontos_r)
        dXd_r(:,i)=[dtx_r(i)./dT_r(:,i);
                  dty_r(i)./dT_r(:,i)];
        u_r(:,i)=K_r*e_r(:,i)+dXd_r(:,i);
        dq_r(:,i)=1/(a1*a2*sin(q_r(2,i)+alfa))*([a2*cos(q_r(1,i)+q_r(2,i)+alfa), a2*sin(q_r(1,i)+q_r(2,i)+alfa); -a1*cos(q_r(1,i))-a2*cos(q_r(1,i)+q_r(2,i)+alfa), -a1*sin(q_r(1,i))-a2*sin(q_r(1,i)+q_r(2,i)+alfa)])*u_r(:,i);
        q_r(:,i+1)=q_r(:,i)+dq_r(:,i)*dT_r(:,i);
        if q_r(1,i+1)>theta1_limSup,%Limite Mec�nico Medido
           q_r(1,i+1)=theta1_limSup;
        end
        if q_r(1,i+1)<theta1_limInf, %Limite Mec�nico Medido
           q_r(1,i+1)=theta1_limInf;
        end
        if q_r(2,i+1)>theta2_limSup, %Limite Mec�nico Medido
           q_r(2,i+1)=theta2_limSup;
        end
        if q_r(2,i+1)<theta2_limInf, %Limite Mec�nico Medido
           q_r(2,i+1)=theta2_limInf;
        end
         if dq_r(1,i) >= (80*pi/180), %Velocidade m�xima limitada para junta 1 em 80�/s 
           dq_r(1,i) = 80*pi/180;
        end
        if dq_r(1,i) <= (-80*pi/180), %Velocidade m�xima limitada para junta 1 em -80�/s 
           dq_r(1,i) = -80*pi/180;
        end
        if dq_r(2,i) >= (80*pi/180), %Velocidade m�xima limitada para junta 2 em 80�/s 
           dq_r(2,i) = 80*pi/180;
        end
        if dq_r(2,i) <= (-80*pi/180), %Velocidade m�xima para junta em 80�/s (limite inferior)
             dq_r(2,i) = -80*pi/180;
        end
    end
end
%%

%Gr�fico da Trajet�ria Retil�nea juntamente com Espa�o de Trabalho
%%
plot(tx_r,ty_r,'r');
xlabel('X (em cm)'), ylabel('Y (em cm)'), title('Determina��o da Trajet�ria Retil�nea no Espa�o de Trabalho');
axis equal;
%%

%Defini��o da Trajet�ria Circular
%%
p0_c=[22,0]; %ponto do centro da circunfer�ncia
r=6;%raio 5 = bom% raio da circunfer�ncia em cm;
n_pontos_c=80;
k=0;

for i=0:2*pi/(n_pontos_c-1):2*pi,
    k=k+1;
    tx_c(k)=p0_c(1,1)+r*cos(i);
    ty_c(k)=p0_c(1,2)+r*sin(i);
end

%%

%Defini��o do Tempo da Trajet�ria e do Intervalo de Amostragem
%%
Tempo_c=30e-3; % Intervalo de amostragem igual a 70 milissegundos
%k=0;
%for i=0:Tempo_c/(n_pontos_c-1):Tempo_c,
%    k=k+1;
%    T_c(k)=i;
%end
T_c(1)=0;
for i=2:n_pontos_c,
    T_c(i)=T_c(i-1)+Tempo_c;
end

%%

%Defini��o dos Intervalos de Tempo da Trajet�ria Circular
%%
dT_c=diff(T_c);
%%

%Controlador Proporcional para Trajet�ria Circular
%%0
K_c=[10,0;0,10];
%%

%Posi��o Inicial das Juntas do Rob� para Trajet�ria Circular
%%
theta1_c=17;%em graus
theta2_c=38;%em graus
theta1_c=deg2rad(theta1_c);%em rad
theta2_c=deg2rad(theta2_c);%em rad

q_c(:,1)=[theta1_c;
        theta2_c];
%%

%Defini��o das velocidades desejadas do efetuador para Trajet�ria Circular
%%
dtx_c=diff(tx_c);
dty_c=diff(ty_c);
%%

%CONTROLE DE TRAJET�RIA CIRCULAR
%%
for i=1:n_pontos_c,
    X_pot_c(:,i)=[a1*cos(q_c(1,i))+a2*cos(q_c(1,i)+q_c(2,i)+alfa);
                a1*sin(q_c(1,i))+a2*sin(q_c(1,i)+q_c(2,i)+alfa)];
    Xd_c(:,i)=[tx_c(i);
               ty_c(i)];     
    e_c(:,i)=Xd_c(:,i)-X_pot_c(:,i);
    if(i<n_pontos_c)
        dXd_c(:,i)=[dtx_c(i)./dT_c(:,i);
                  dty_c(i)./dT_c(:,i)];
        u_c(:,i)=K_c*e_c(:,i)+dXd_c(:,i);
        dq_c(:,i)=1/(a1*a2*sin(q_c(2,i)+alfa))*([a2*cos(q_c(1,i)+q_c(2,i)+alfa), a2*sin(q_c(1,i)+q_c(2,i)+alfa); -a1*cos(q_c(1,i))-a2*cos(q_c(1,i)+q_c(2,i)+alfa), -a1*sin(q_c(1,i))-a2*sin(q_c(1,i)+q_c(2,i)+alfa)])*u_c(:,i);
        q_c(:,i+1)=q_c(:,i)+dq_c(:,i)*dT_c(:,i);
        if q_c(1,i+1)>theta1_limSup,%Limite Mec�nico Medido
           q_c(1,i+1)=theta1_limSup;
        end
        if q_c(1,i+1)<theta1_limInf, %Limite Mec�nico Medido
           q_c(1,i+1)=theta1_limInf;
        end
        if q_c(2,i+1)>theta2_limSup, %Limite Mec�nico Medido
           q_c(2,i+1)=theta2_limSup;
        end
        if q_c(2,i+1)<theta2_limInf, %Limite Mec�nico Medido
           q_c(2,i+1)=theta2_limInf;
        end
    end
end
%%

%Gr�fico da Trajet�ria Circular juntamente com a Trajet�ria Retil�nea e 
%o Espa�o de Trabalho
%%
%plot(tx_c,ty_c,'r');
%xlabel('X (em cm)'), ylabel('Y (em cm)'), title('Determina��o do Espa�o de Trabalho');
%axis equal;
%%

%Convers�o de Radianos para Graus
%%
q_r=rad2deg(q_r); %Vari�veis de Juntas para Trajet�ria Retil�nea
dq_r=rad2deg(dq_r); %Velocidades das Juntas para Trajet�ria Retil�nea
q_c=rad2deg(q_c); %Vari�veis de Juntas para Trajet�ria Circular
dq_c=rad2deg(dq_c); %Velocidades das Juntas para Trajet�ria Circular

%%

%Gr�ficos Pedidos

%********Trajet�ria Desejada X Trajet�ria Efetuada*************************
%%
%Trajet�ria Retil�nea
figure,
plot(tx_r,ty_r,'b',X_pot_r(1,:),X_pot_r(2,:),'k--');
ylabel('Y (em cm)'), xlabel('X (em cm)'), title('Controle de Trajet�ria Retil�nea do Efetuador');
legend('Trajet�ria Desejada','Trajet�ria Simulada');
axis equal

%Trajet�ria Circular
% figure,
% plot(tx_c,ty_c,'b',X_pot_c(1,:),X_pot_c(2,:),'k--');
% ylabel('Y (em cm)'), xlabel('X (em cm)'), title('Controle de Trajet�ria Circular do Efetuador');
% legend('Trajet�ria Desejada','Trajet�ria Simulada');
% axis equal;
%%

%********Posi��es Angulares das Juntas X Tempo*****************************
%%
%Trajet�ria Retil�nea
figure,
plot(T_r,q_r(1,:),'k--');
ylabel('Posi��o de Theta1 (em graus)'), xlabel('t (em s)'), title('Varia��o de Theta1 ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
%axis equal;
figure,
plot(T_r,q_r(2,:),'k--');
ylabel('Posi��o de Theta2 (em graus)'), xlabel('t (em s)'), title('Varia��o de Theta2 ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
%axis equal;

%Trajet�ria Circular
% figure,
% plot(T_c,q_c(1,:),'k--');
% ylabel('Posi��o de Theta1 (em graus)'), xlabel('t (em s)'), title('Varia��o de Theta1 ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
% %axis equal;
% figure,
% plot(T_c,q_c(2,:),'k--');
% ylabel('Posi��o de Theta2 (em graus)'), xlabel('t (em s)'), title('Varia��o de Theta2 ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
% %axis equal;
%%

%********Erro de Posi��o X Tempo*******************************************
%%
%Trajet�ria Retil�nea
figure,
plot(T_r,e_r(1,:),'k--');
ylabel('Erro em X (em cm)'), xlabel('t (em s)'), title('Varia��o do Erro em X ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
axis([0,2.5,-2,1]);
%axis equal;
figure,
plot(T_r,e_r(2,:),'k--');
ylabel('Erro em Y (em cm)'), xlabel('t (em s)'), title('Varia��o de Erro em Y ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
axis([0,2.5,-2,1]);
%axis equal;

%Trajet�ria Circular
% figure,
% plot(T_c,e_c(1,:),'k--');
% ylabel('Erro em X (em cm)'), xlabel('t (em s)'), title('Varia��o do Erro em X ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
% %axis equal;
% figure,
% plot(T_c,e_c(2,:),'k--');
% ylabel('Erro em Y (em cm)'), xlabel('t (em s)'), title('Varia��o do Erro em Y ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
%axis equal;
%%

%********Velocidade das Juntas X Tempo*******************************************
%%
%Trajet�ria Retil�nea
figure,
plot(T_r(1:n_pontos_r-1),dq_r(1,:),'k--');
ylabel('Velocidade de Theta1 (em graus/s)'), xlabel('t (em s)'), title('Varia��o da Velocidade da Junta 1 ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
axis([0,2.5,-80,80]);
%axis equal;
figure,
plot(T_r(1:n_pontos_r-1),dq_r(2,:),'k--');
ylabel('Velocidade de Theta2 (em graus/s)'), xlabel('t (em s)'), title('Varia��o da Velocidade da Junta 2 ao longo do tempo - Trajet�ria Retil�nea');
legend('Simula��o');
axis([0,2.5,-80,80]);
%axis equal;

%Trajet�ria Circular
% figure,
% plot(T_c(1:n_pontos_c-1),dq_c(1,:),'k--');
% ylabel('Velocidade de Theta1 (em graus/s)'), xlabel('t (em s)'), title('Varia��o da Velocidade da Junta 1 ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
% %axis equal;
% figure,
% plot(T_c(1:n_pontos_c-1),dq_c(2,:),'k--');
% ylabel('Velocidade de Theta2 (em graus/s)'), xlabel('t (em s)'), title('Varia��o da Velocidade da Junta 2 ao longo do tempo - Trajet�ria Circular');
% legend('Simula��o');
% axis equal;
%%
