%% Grafica de sistema en coordenadas absolutas
close all
clear all
sigmaV=10; %Definición de incertidumbre de velocidad
k=1; %ganancia de ley de control
% velocidades de desplazamiento de los vehiculos 
u1=[0,50]
u2=[0,65]
u3=[0,50]
u4=[0,50]
u=[u1,u2,u3,u4]'
% posiciones iniciales de los agentes 
q1=[0,0]
q2=[600,0]
q3=[-100,300]
q4=[200,300]
% definición de formación a seguir
pdeseada=[-200,-200;200,-200;200,200;-200,200]';
%posiciones relativas deseadas con la formación
z1d=[pdeseada(:,1),pdeseada(:,1),pdeseada(:,1)]-pdeseada(:,2:end); %respecto a agente a_1
z2d=[pdeseada(:,2),pdeseada(:,2),pdeseada(:,2)]-[pdeseada(:,1),pdeseada(:,3:end)]; %respecto a agente a_2
z3d=[pdeseada(:,3),pdeseada(:,3),pdeseada(:,3)]-[pdeseada(:,1:2),pdeseada(:,4)]; %respecto a agente a_3
z4d=[pdeseada(:,4),pdeseada(:,4),pdeseada(:,4)]-[pdeseada(:,1:3)]; %respecto a agente a_4

%Estados
q=[q1,q2,q3,q4]'
qini=q
%matrices F y G para coordenadas absolutas
F=eye(8)
dt=0.1
G=F*dt
%Matrices de coordenadas relativas 
Frel=eye(6);
P=eye(6)*0; %se asume incertidumbre inicial nula
Q=eye(6)*(sqrt(2)*sigmaV)^2; %matriz de incertidumbre de entrada (velocidades relativas)
Grel=eye(6)*dt;
qrel=[q(3)-q(1),q(4)-q(2),q(5)-q(1),q(6)-q(2),q(7)-q(1),q(8)-q(2)]'; %posiciones relativas iniciales
%tiempo de simulacion
t=20;
%tiempo entre mediciones tmed
tmed=5 %s


% generacion de trayectoria real 
qreal=q;
% trayectoria estimada
qlog=q;
i=2
figure;
% pause();
for j=dt:dt:t
    %velocidad en el instante k 
    ureal=[normrnd(u(1),sigmaV),normrnd(u(2),sigmaV),normrnd(u(3),sigmaV),normrnd(u(4),sigmaV)...
        normrnd(u(5),sigmaV),normrnd(u(6),sigmaV),normrnd(u(7),sigmaV),normrnd(u(8),sigmaV)]';
    %posicion en el instante k+1
    qreal(:,i)=qreal(:,i-1)+G*ureal;
    %posiciones relativas reales
    qx21=qreal(3,:)-qreal(1,:);
    qy21=qreal(4,:)-qreal(2,:);
    qx31=qreal(5,:)-qreal(1,:);
    qy31=qreal(6,:)-qreal(2,:);
    qx41=qreal(7,:)-qreal(1,:);
    qy41=qreal(8,:)-qreal(2,:);
    %velocidades relativas
    urel=[u(3)-u(1),u(4)-u(2),u(5)-u(1),u(6)-u(2),u(7)-u(1),u(8)-u(2)]';
    
     if(mod(i,round(tmed/dt))==0) %se corrige periodicamente, simulo la llegada de distancias
       
         
        %distancias reales medidas 
       yhatm=[sqrt(qx21(i-1)^2+qy21(i-1)^2); %d21
               sqrt(qx31(i-1)^2+qy31(i-1)^2); %d31
               sqrt((qx31(i-1)-qx21(i-1))^2+(qy31(i-1)-qy21(i-1))^2); %d32
               sqrt(qx41(i-1)^2+qy41(i-1)^2); %d41
                sqrt((qx41(i-1)-qx21(i-1))^2+(qy41(i-1)-qy21(i-1))^2); %d42
                sqrt((qx41(i-1)-qx31(i-1))^2+(qy41(i-1)-qy31(i-1))^2); %d43
                0]; 
        %matrices H para cada distancia
        H1=[qrel(1,i-1)/sqrt(qrel(1,i-1)^2+qrel(2,i-1)^2),qrel(2,i-1)/sqrt(qrel(1,i-1)^2+qrel(2,i-1)^2),0,0,0,0];
        H2=[0,0,qrel(3,i-1)/sqrt(qrel(3,i-1)^2+qrel(4,i-1)^2),qrel(4,i-1)/sqrt(qrel(3,i-1)^2+qrel(4,i-1)^2),0,0];
        aux=sqrt((qrel(3,i-1)-qrel(1,i-1))^2+(qrel(4,i-1)-qrel(2,i-1))^2);
        H3=[(qrel(1,i-1)-qrel(3,i-1))/aux,((qrel(2,i-1)-qrel(4,i-1)))/aux,...
            (-(qrel(1,i-1)-qrel(3,i-1))/aux),(-(qrel(2,i-1)-qrel(4,i-1)))/aux,0,0];
        H4=[0,0,0,0,qrel(5,i-1)/sqrt(qrel(5,i-1)^2+qrel(6,i-1)^2),qrel(6,i-1)/sqrt(qrel(5,i-1)^2+qrel(6,i-1)^2)];
        aux=sqrt((qrel(5,i-1)-qrel(1,i-1))^2+(qrel(6,i-1)-qrel(2,i-1))^2);
        H5=[(qrel(1,i-1)-qrel(5,i-1))/aux,((qrel(2,i-1)-qrel(6,i-1)))/aux,0,0,...
            (-(qrel(1,i-1)-qrel(5,i-1))/aux),(-(qrel(2,i-1)-qrel(6,i-1)))/aux];
        aux=sqrt((qrel(5,i-1)-qrel(3,i-1))^2+(qrel(6,i-1)-qrel(4,i-1))^2);
        H6=[0,0,(qrel(3,i-1)-qrel(5,i-1))/aux,((qrel(4,i-1)-qrel(6,i-1)))/aux...
            (-(qrel(3,i-1)-qrel(5,i-1))/aux),(-(qrel(4,i-1)-qrel(6,i-1)))/aux];
        H7=[0,1,0,0,0,0];
        H=[H1;H2;H3;H4;H5;H6;H7];

        
        
        Pmedida_distancia=eye(7)*10^2; %sigma=10cm-> sigma^2=100cm^2
%       
        Pmedida_distancia(7,7)=100;
        Py=H*P*H'; %covarianza de la distancia estimada a partir de las posiciiones estimadas
        S=Pmedida_distancia+Py;
            
        K=P*H'*S^-1;
        H*qrel(:,i-1)-yhatm;
        P=P-K*H*P;
        qrel(:,i-1)=qrel(:,i-1)+K*(yhatm-H*qrel(:,i-1));

       
     end
    
    %posicion en el instante k+1
    qrel(:,i)=qrel(:,i-1)+Grel*urel;
    P=Frel*P*Frel'+Grel*Q*Grel';
    

    %Aplico ley de control
    po=[0,0];
    pr=[qrel(1,i),qrel(2,i)];
    p1=[qrel(3,i),qrel(4,i)];
    p2=[qrel(5,i),qrel(6,i)];
    
   z1=[po;po;po]'-[pr;p1;p2]';
   z2=[pr;pr;pr]'-[po;p1;p2]';
   z3=[p1;p1;p1]'-[po;pr;p2]';
   z4=[p2;p2;p2]'-[po;pr;p1]';
   
   L1=-k*sum((z1-z1d)');
   u(1:2)=L1+u1;
   L2=-k*sum((z2-z2d)');
   u(3:4)=L2+u2;
   L3=-k*sum((z3-z3d)');
   u(5:6)=L3+u3;
   L4=-k*sum((z4-z4d)');
   u(7:8)=L4+u4;
   
subplot(2,1,1)

%posición real
  plot(qreal(1,i),qreal(2,i),'kx','LineWidth',5)
hold on
plot(qreal(3,i),qreal(4,i),'rx','LineWidth',5)
hold on
plot(qreal(5,i),qreal(6,i),'gx','LineWidth',5)
hold on
plot(qreal(7,i),qreal(8,i),'bx','LineWidth',5)
hold off
title('Coordenadas globales')
   xlabel('x(cm)')
   ylabel('y(cm)')
   legend('$a_1$','$a_2$','$a_3$','$a_4$','Interpreter','Latex')
   set(gca,'FontSize',16)
   axis([-40,800,-100 1000])
   axis('equal')
subplot(2,1,2)
plot(0,0,'k*','LineWidth',5)
   hold on
 plot(qrel(1,i),qrel(2,i),'r*','LineWidth',5)
hold on
plot(qrel(3,i),qrel(4,i),'g*','LineWidth',5)
hold on
plot(qrel(5,i),qrel(6,i),'b*','LineWidth',5)
   % Coordendas relativas reales 
 hold on
plot(qreal(3,i)-qreal(1,i),qreal(4,i)-qreal(2,i),'rd','LineWidth',3)
hold on
plot(qreal(5,i)-qreal(1,i),qreal(6,i)-qreal(2,i),'gd','LineWidth',3)
hold on
plot(qreal(7,i)-qreal(1,i),qreal(8,i)-qreal(2,i),'bd','LineWidth',3)
hold on
   %puntos deseados

   hold on
   ellipse(20,20,0,-z1d(1,1),-z1d(2,1),'r')
  
   ellipse(20,20,0,-z1d(1,2),-z1d(2,2),'g')
   
   ellipse(20,20,0,-z1d(1,3),-z1d(2,3),'b')
   
   axis('equal')
   
   xlabel('x(cm)')
   ylabel('y(cm)')
   legend('$a_1$','$a_2$','$a_3$','$a_4$','Interpreter','Latex')
   set(gca,'FontSize',16)
hold off
prreal=[qreal(3,i)-qreal(1,i),qreal(4,i)-qreal(2,i)];
p1real=[qreal(5,i)-qreal(1,i),qreal(6,i)-qreal(2,i)];
p2real=[qreal(7,i)-qreal(1,i),qreal(8,i)-qreal(2,i)];
aux=z2d(:,1)-prreal';
   dformro(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z3d(:,1)-p1real';
   dform1o(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z4d(:,1)-p2real';
   dform2o(i)=sqrt(aux(1)^2+aux(2)^2);


auto(i)=max(eig(P));

pause(0.01)
i=i+1;
end 
%% Grafica de distancias
close all
figure;
taux=dt:dt:t
plot(taux(1:end),dformro(2:end),'r')
hold on
plot(taux(1:end),dform1o(2:end),'g')
hold on
plot(taux(1:end),dform2o(2:end),'b')
hold on
yline(20,'-','LineWidth',3);
xlabel('t(s)')
ylabel('d_{i1}^{obj}(cm)')
set(gca, 'XScale', 'log')
% set(gca, 'YScale', 'log')
legend('$a_2$','$a_3$','$a_4$','Límite de desviación','Interpreter','Latex')
set(gca,'FontSize',18)

%Grafica de autovalor Maximo
figure;

ttotal=0:dt:t

plot(ttotal,auto,'LineWidth',1)
hold on

xlabel('t(s)')
ylabel('\lambda (cm^{2})')
legend('$\lambda_{max}$','Límite ','Interpreter','Latex')
set(gca,'FontSize',18)
