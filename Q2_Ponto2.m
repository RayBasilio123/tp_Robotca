clc;
clear all;

%Robô Planar RR
%%
%Limites Mecânicos do Servo do Levantamento do Braço:
    % Mínimo: 0°.
    % Máximo: +180°.
%Limites Mecânicos da Servo do Cotovelo do Braço:
    % Mínimo: -50°.
    % Máximo: +90°.
%%    
%Definição dos parâmetros do Robô RR
%%
a1=20;
a2=10;
a3=5;
alfa=-pi/2;% parâmetro necessário para que a posição zero do robô tenha a configuração desejada na qual o eixo x2 (cotovelo)  está a -90º em relação a x1 (levantamento do braço)                                                                                       
%% 

%Espaço de Trabalho
%%
i=0;

theta1_limInf=0;%limite inferior de theta 1 em graus
theta1_limSup=180;%limite superior de theta 1 em graus
theta2_limInf=-50;%limite inferior de theta 2 em graus
theta2_limSup=90;%limite superior de theta 2 em graus
theta3_limInf=-135;%limite inferior de theta 3 em graus
theta3_limSup=135;%limite superior de theta 3 em graus

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
         if(x(i)>= 20.30 && x(i)<= 20.36 && y(i)>=17.2 && y(i)<=17.3)
         
                disp(x(i));
                disp(y(i));
                disp('Valor de theta 1 para o ponto (x,y):')
                disp(rad2deg(theta1))
                disp('Valor de theta 2 para o ponto (x,y):')
                disp(rad2deg(theta2))
                disp('Valor de theta 3 para o ponto (x,y):')
                disp(rad2deg(theta3))
            end  
     end
end
end
plot(x,y,'.b');
hold on;

%%

%Definição da Trajetória Retilínea
%%
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

%Definição do Tempo da Trajetória Retilínea e do Intervalo de Amostragem
%%
Tempo_r=30e-3; % Intervalo de amostragem igual a 40 milissegundos

T_r(1)=0;
for i=2:n_pontos_r,
    T_r(i)=T_r(i-1)+Tempo_r;
end

%%

%Definição dos Intervalos de Tempo da Trajetória Retilínea
%%
dT_r=diff(T_r);
%%

%Controlador Proporcional para Trajetória Retilínea
%%
K_r=[30,0;0,30];
%%

%Posição Inicial
%%
theta1_r=41.4;%em graus
theta2_r=59.2;%em graus
theta3_r=54.5;%em graus
theta1_r=deg2rad(theta1_r);%em rad
theta2_r=deg2rad(theta2_r);%em rad
theta3_r=deg2rad(theta3_r);%em rad
%q_r(:,1)=[theta1_r;
 %      theta2_r];
%%  

q_r(:,1)=[theta1_r;
      theta2_r;
      theta3_r];
%Definição das velocidade desejadas do efetuador para trajetória Retilínea
%%    
dtx_r=diff(tx_r);
dty_r=diff(ty_r);
%%

%CONTROLE DE TRAJETÓRIA RETILÍNEA
%%
a5=-pi/2;
for i=1:n_pontos_r,
    X_pot_r(:,i)=[a1*cos(q_r(1,i))+a2*cos(q_r(1,i)+q_r(2,i)+a5)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i));
                a1*sin(q_r(1,i))+a2*sin(q_r(1,i)+q_r(2,i)+a5)+a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i))];
    Xd_r(:,i)=[tx_r(i);
               ty_r(i)];
    e_r(:,i)=Xd_r(:,i)-X_pot_r(:,i);
    if(i<n_pontos_r)
        dXd_r(:,i)=[dtx_r(i)./dT_r(:,i);
                  dty_r(i)./dT_r(:,i)];
        u_r(:,i)=K_r*e_r(:,i)+dXd_r(:,i);
        %dq_r(:,i)=1/(a1*a2*sin(q_r(2,i)+a5))*([a2*cos(q_r(1,i)+q_r(2,i)+a5), a2*sin(q_r(1,i)+q_r(2,i)+a5); -a1*cos(q_r(1,i))-a2*cos(q_r(1,i)+q_r(2,i)+a5), -a1*sin(q_r(1,i))-a2*sin(q_r(1,i)+q_r(2,i)+a5)])*u_r(:,i);
        J = [-a1*sin(q_r(1,i))-a2*sin(q_r(1,i)+q_r(2,i)+a5)-a3*sin(q_r(1,i)+q_r(2,i)+a5+q_r(3,i)), -a2*sin(q_r(1,i)+q_r(2,i)+a5)-a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i)), -a3*sin(q_r(1,i)+q_r(2,i)+q_r(3,i)); a1*cos(q_r(1,i))+a2*cos(q_r(1,i)+q_r(2,i)+a5)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i)), a2*cos(q_r(1,i)+q_r(2,i)+a5)+a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i)), a3*cos(q_r(1,i)+q_r(2,i)+q_r(3,i))];
        J_inv = J'*inv((J*J'));
        dq_r(:,i)=J_inv*u_r(:,i);
        q_r(:,i+1)=q_r(:,i)+dq_r(:,i)*dT_r(:,i);
        
        
        if q_r(1,i+1)>theta1_limSup,%Limite Mecânico Medido
           q_r(1,i+1)=theta1_limSup;
        end
        if q_r(1,i+1)<theta1_limInf, %Limite Mecânico Medido
           q_r(1,i+1)=theta1_limInf;
        end
        if q_r(2,i+1)>theta2_limSup, %Limite Mecânico Medido
           q_r(2,i+1)=theta2_limSup;
        end
        if q_r(2,i+1)<theta2_limInf, %Limite Mecânico Medido
           q_r(2,i+1)=theta2_limInf;
        end
    end
end
%%

%Gráfico da Trajetória Retilínea juntamente com Espaço de Trabalho
%%
plot(tx_r,ty_r,'r');
xlabel('X (em cm)'), ylabel('Y (em cm)'), title('Determinação da Trajetória Retilínea no Espaço de Trabalho');
axis equal;
%%

%Definição da Trajetória Circular
%%
p0_c=[22,0]; %ponto do centro da circunferência
r=6;%raio 5 = bom% raio da circunferência em cm;
n_pontos_c=80;
k=0;

for i=0:2*pi/(n_pontos_c-1):2*pi,
    k=k+1;
    tx_c(k)=p0_c(1,1)+r*cos(i);
    ty_c(k)=p0_c(1,2)+r*sin(i);
end

%%

%Definição do Tempo da Trajetória e do Intervalo de Amostragem
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

%Definição dos Intervalos de Tempo da Trajetória Circular
%%
dT_c=diff(T_c);
%%

%Controlador Proporcional para Trajetória Circular
%%0
K_c=[10,0;0,10];
%%

%Posição Inicial das Juntas do Robô para Trajetória Circular
%%
theta1_c=17;%em graus
theta2_c=38;%em graus
theta1_c=deg2rad(theta1_c);%em rad
theta2_c=deg2rad(theta2_c);%em rad

q_c(:,1)=[theta1_c;
        theta2_c];
%%

%Definição das velocidades desejadas do efetuador para Trajetória Circular
%%
dtx_c=diff(tx_c);
dty_c=diff(ty_c);
%%

%CONTROLE DE TRAJETÓRIA CIRCULAR
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
        if q_c(1,i+1)>theta1_limSup,%Limite Mecânico Medido
           q_c(1,i+1)=theta1_limSup;
        end
        if q_c(1,i+1)<theta1_limInf, %Limite Mecânico Medido
           q_c(1,i+1)=theta1_limInf;
        end
        if q_c(2,i+1)>theta2_limSup, %Limite Mecânico Medido
           q_c(2,i+1)=theta2_limSup;
        end
        if q_c(2,i+1)<theta2_limInf, %Limite Mecânico Medido
           q_c(2,i+1)=theta2_limInf;
        end
    end
end
%%

%Gráfico da Trajetória Circular juntamente com a Trajetória Retilínea e 
%o Espaço de Trabalho
%%
%plot(tx_c,ty_c,'r');
%xlabel('X (em cm)'), ylabel('Y (em cm)'), title('Determinação do Espaço de Trabalho');
%axis equal;
%%

%Conversão de Radianos para Graus
%%
q_r=rad2deg(q_r); %Variáveis de Juntas para Trajetória Retilínea
dq_r=rad2deg(dq_r); %Velocidades das Juntas para Trajetória Retilínea
q_c=rad2deg(q_c); %Variáveis de Juntas para Trajetória Circular
dq_c=rad2deg(dq_c); %Velocidades das Juntas para Trajetória Circular

%%

%Gráficos Pedidos

%********Trajetória Desejada X Trajetória Efetuada*************************
%%
%Trajetória Retilínea
figure,
plot(tx_r,ty_r,'b',X_pot_r(1,:),X_pot_r(2,:),'k--');
ylabel('Y (em cm)'), xlabel('X (em cm)'), title('Controle de Trajetória Retilínea do Efetuador');
legend('Trajetória Desejada','Trajetória Simulada');
axis equal

%Trajetória Circular
% figure,
% plot(tx_c,ty_c,'b',X_pot_c(1,:),X_pot_c(2,:),'k--');
% ylabel('Y (em cm)'), xlabel('X (em cm)'), title('Controle de Trajetória Circular do Efetuador');
% legend('Trajetória Desejada','Trajetória Simulada');
% axis equal;
%%

%********Posições Angulares das Juntas X Tempo*****************************
%%
%Trajetória Retilínea
figure,
plot(T_r,q_r(1,:),'k--');
ylabel('Posição de Theta1 (em graus)'), xlabel('t (em s)'), title('Variação de Theta1 ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
%axis equal;
figure,
plot(T_r,q_r(2,:),'k--');
ylabel('Posição de Theta2 (em graus)'), xlabel('t (em s)'), title('Variação de Theta2 ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
%axis equal;

%Trajetória Circular
% figure,
% plot(T_c,q_c(1,:),'k--');
% ylabel('Posição de Theta1 (em graus)'), xlabel('t (em s)'), title('Variação de Theta1 ao longo do tempo - Trajetória Circular');
% legend('Simulação');
% %axis equal;
% figure,
% plot(T_c,q_c(2,:),'k--');
% ylabel('Posição de Theta2 (em graus)'), xlabel('t (em s)'), title('Variação de Theta2 ao longo do tempo - Trajetória Circular');
% legend('Simulação');
% %axis equal;
%%

%********Erro de Posição X Tempo*******************************************
%%
%Trajetória Retilínea
figure,
plot(T_r,e_r(1,:),'k--');
ylabel('Erro em X (em cm)'), xlabel('t (em s)'), title('Variação do Erro em X ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
axis([0,2.5,-2,1]);
%axis equal;
figure,
plot(T_r,e_r(2,:),'k--');
ylabel('Erro em Y (em cm)'), xlabel('t (em s)'), title('Variação de Erro em Y ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
axis([0,2.5,-2,1]);
%axis equal;

%Trajetória Circular
% figure,
% plot(T_c,e_c(1,:),'k--');
% ylabel('Erro em X (em cm)'), xlabel('t (em s)'), title('Variação do Erro em X ao longo do tempo - Trajetória Circular');
% legend('Simulação');
% %axis equal;
% figure,
% plot(T_c,e_c(2,:),'k--');
% ylabel('Erro em Y (em cm)'), xlabel('t (em s)'), title('Variação do Erro em Y ao longo do tempo - Trajetória Circular');
% legend('Simulação');
%axis equal;
%%

%********Velocidade das Juntas X Tempo*******************************************
%%
%Trajetória Retilínea
figure,
plot(T_r(1:n_pontos_r-1),dq_r(1,:),'k--');
ylabel('Velocidade de Theta1 (em graus/s)'), xlabel('t (em s)'), title('Variação da Velocidade da Junta 1 ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
axis([0,2.5,-80,80]);
%axis equal;
figure,
plot(T_r(1:n_pontos_r-1),dq_r(2,:),'k--');
ylabel('Velocidade de Theta2 (em graus/s)'), xlabel('t (em s)'), title('Variação da Velocidade da Junta 2 ao longo do tempo - Trajetória Retilínea');
legend('Simulação');
axis([0,2.5,-80,80]);
%axis equal;

%Trajetória Circular
% figure,
% plot(T_c(1:n_pontos_c-1),dq_c(1,:),'k--');
% ylabel('Velocidade de Theta1 (em graus/s)'), xlabel('t (em s)'), title('Variação da Velocidade da Junta 1 ao longo do tempo - Trajetória Circular');
% legend('Simulação');
% %axis equal;
% figure,
% plot(T_c(1:n_pontos_c-1),dq_c(2,:),'k--');
% ylabel('Velocidade de Theta2 (em graus/s)'), xlabel('t (em s)'), title('Variação da Velocidade da Junta 2 ao longo do tempo - Trajetória Circular');
% legend('Simulação');
% axis equal;
%%

