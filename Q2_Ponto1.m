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
a3=5;
alfa=-pi/2;% par�metro necess�rio para que a posi��o zero do rob� tenha a configura��o desejada na qual o eixo x2 (cotovelo)  est� a -90� em rela��o a x1 (levantamento do bra�o)                                                                                       
%% 

%Espa�o de Trabalho
%%
i=0;

theta1_limInf=0;%limite inferior de theta 1 em graus
theta1_limSup=180;%limite superior de theta 1 em graus
theta2_limInf=-50;%limite inferior de theta 2 em graus
theta2_limSup=90;%limite superior de theta 2 em graus
theta3_limInf=-135;%limite inferior de theta 2 em graus
theta3_limSup=135;%limite superior de theta 2 em graus

theta1_limInf=deg2rad(theta1_limInf);
theta1_limSup=deg2rad(theta1_limSup);
theta2_limInf=deg2rad(theta2_limInf);
theta2_limSup=deg2rad(theta2_limSup);
theta3_limInf=deg2rad(theta3_limInf);
theta3_limSup=deg2rad(theta3_limSup);


for theta1=theta1_limInf:(theta1_limSup-theta1_limInf)/100:theta1_limSup,
    for theta2=theta2_limInf:(theta2_limSup-theta2_limInf)/100:theta2_limSup,
         for theta3=theta3_limInf:(theta3_limSup-theta3_limInf)/100:theta3_limSup,
        i=i+1;
        x(i)=a1*cos(theta1)+a2*cos(theta1+theta2+alfa)+a3*cos(theta1+theta2+theta3);
        y(i)=a1*sin(theta1)+a2*sin(theta1+theta2+alfa)+a3*sin(theta1+theta2+theta3);
         if(x(i)>= -19.8 && x(i)<= -19.7 && y(i)>=12.2 && y(i)<=12.3)
         
                disp(x(i));
                disp(y(i));
                disp('Valor de theta 1 para o ponto (x,y)=(2.6064,13.6463):')
                disp(rad2deg(theta1))
                disp('Valor de theta 2 para o ponto (x,y)=(2.6064,13.6463):')
                disp(rad2deg(theta2))
                disp('Valor de theta 3 para o ponto (x,y)=(2.6064,13.6463):')
                disp(rad2deg(theta3))
            end  
     end
end
end
plot(x,y,'.b');
hold on;

%%

%Defini��o da Trajet�ria Retil�nea
%%
p0_r=[-19.8,12.2];
p1_r=[18.37,19.07];

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
K_r=[20,0;0,20];
%%

%Posi��o Inicial
%%
theta1_r=158.4;%em graus
theta2_r=28.4;%em graus
theta3_r=83.7;%em graus
theta1_r=deg2rad(theta1_r);%em rad
theta2_r=deg2rad(theta2_r);%em rad
theta3_r=deg2rad(theta3_r);%em rad
%q_r(:,1)=[theta1_r;
 %      theta2_r];
%%  

q_r(:,1)=[theta1_r;
      theta2_r;
      theta3_r];
%Defini��o das velocidade desejadas do efetuador para trajet�ria Retil�nea
%%    
dtx_r=diff(tx_r);
dty_r=diff(ty_r);
%%

%CONTROLE DE TRAJET�RIA RETIL�NEA
%%
for i=1:n_pontos_r,
    X_pot_r(:,i)=[a1*cos(q_r(1,i))+a2*cos(q_r(1,i)+q_r(2,i)+alfa)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i));
                a1*sin(q_r(1,i))+a2*sin(q_r(1,i)+q_r(2,i)+alfa)+ a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i))];
    Xd_r(:,i)=[tx_r(i);
               ty_r(i)];
    e_r(:,i)=Xd_r(:,i)-X_pot_r(:,i);
    if(i<n_pontos_r)
        dXd_r(:,i)=[dtx_r(i)./dT_r(:,i);
                  dty_r(i)./dT_r(:,i)];
        u_r(:,i)=K_r*e_r(:,i)+dXd_r(:,i);
        J = [-a1*sin(q_r(1,i))-a2*sin(q_r(1,i)+q_r(2,i)+alfa)-a3*sin(q_r(1,i)+q_r(2,i)+alfa+q_r(3,i)), -a2*sin(q_r(1,i)+q_r(2,i)+alfa)-a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i)), -a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i)); a1*cos(q_r(1,i))+a2*cos(q_r(1,i)+q_r(2,i)+alfa)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i)), a2*cos(q_r(1,i)+q_r(2,i)+alfa)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i)), a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i))];
        J_inv = J'*inv((J*J'));
        dq_r(:,i)=(J_inv*u_r(:,i));
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
    end
end
%%

%Gr�fico da Trajet�ria Retil�nea juntamente com Espa�o de Trabalho
%%
plot(tx_r,ty_r,'r');
xlabel('X (em cm)'), ylabel('Y (em cm)'), title('Determina��o da Trajet�ria Retil�nea no Espa�o de Trabalho');
axis equal;


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
