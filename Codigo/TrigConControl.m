%% Inicialización del algoritmo 

close all
clear all
%parametros de la simulación 
%definicion de incertidumbre de medicion del sensor
sigma=10 %cm


% factor de ganancia de la ley de control
k=0.5;
%distancia temporal entre las mediciones
tmed=0.05 %s
deltaT=tmed  %s
dt=deltaT
%tiempo de simulaición 
ttotal=20;
%vector de tiempo
t=0:deltaT:ttotal;

%posiciones deseadas con la formación
pdeseada=[-200,-200;200,-200;200,200;-200,200]';

% Posciones relativas deseadas entre agentes 
z1d=[pdeseada(:,1),pdeseada(:,1),pdeseada(:,1)]-pdeseada(:,2:end); %respecto a agente a_o
z2d=[pdeseada(:,2),pdeseada(:,2),pdeseada(:,2)]-[pdeseada(:,1),pdeseada(:,3:end)]; %respecto a agente a_r
z3d=[pdeseada(:,3),pdeseada(:,3),pdeseada(:,3)]-[pdeseada(:,1:2),pdeseada(:,4)]; %respecto a agente a_1
z4d=[pdeseada(:,4),pdeseada(:,4),pdeseada(:,4)]-[pdeseada(:,1:3)]; %respecto a agente a_2


%Posiciones iniciales de los agentes 
ao=[0,0] 
ar=[400,200] 
a1=[-100,300]
a2=[200, 300]
ai=[a1;a2]
%definición de recta para clasificación del resto de los agentes 
m=(ar(2)-ao(2))/(ar(1)-ao(1))
c=ar(2)-m*ar(1)
% definición de agentes ai

signo=[zeros(length(ai(:,2)),1)]
if ar(1)>ao(1)
        theta=atan((ar(2)-ao(2))/(ar(1)-ao(1)))
        for i=1:length(ai(:,2))
           if(ai(i,2)>m*ai(i,1))
                signo(i)=1;
           else
               signo(i)=-1;
           end
        end
else
    theta=atan((ar(2)-ao(2))/(ar(1)-ao(1)))-pi
    for i=1:length(ai(:,2))
           if(ai(i,2)>m*ai(i,1))
                signo(i)=-1;
           else
               signo(i)=1;
           end
        end
end

A=[ao;ar;a1;a2];
R=[cos(theta) sin(theta);-sin(theta) cos(theta)];
arN=(R*(ar-ao)')';
a1N=(R*(a1-ao)')';
a2N=(R*(a2-ao)')'
AN=[ao-ao;arN;a1N;a2N];

% Ya en este punto estan inicializadas las posiciones inicales 

%se definen velocidades vdes de los agentes 
uo=[0,50]; %cm/s
ur=[0,65]; %cm/s
u1=[0,50]; %cm/s
u2=[0,50]; %cm/s




%posciones iniciales del algoritmo
po=[0,0];
pr=arN;
p1=a1N;
p2=a2N;
%estados iniciales y velocidades iniciales
x=[ao,ar,a1,a2]'  %
u=[uo,ur,u1,u2]'

F=eye(8) 
G=eye(8)*deltaT


figure;
i=1;
set(gcf, 'Position',  [0,0, 1500, 1500])
subplot(2,2,1)
%Coordenadas globales
 h1=plot(ao(1),ao(2),'kx','LineWidth',5)
hold on
h2=plot(ar(1),ar(2),'rx','LineWidth',5);
h3=plot(a1(1),a1(2),'gx','LineWidth',5);
h4=plot(a2(1),a2(2),'bx','LineWidth',5);
hglobal=[h1,h2,h3,h4];
title('Coordenadas globales')
   xlabel('x(cm)')
   ylabel('y(cm)')
   legend('$a_1$','$a_2$','$a_3$','$a_4$','Location','bestoutside','Interpreter','Latex')
   set(gca,'FontSize',14)
   axis([-40,800,-100 2000])
   axis('equal')
subplot(2,2,3)
%Coordenadas relativas
plot(0,0,'k*','LineWidth',5)
 hold on
 h5=plot(pr(1),pr(2),'r*','LineWidth',5);
h6=plot(p1(1),p1(2),'g*','LineWidth',5);
h7=plot(p2(1),p2(2),'b*','LineWidth',5);
hrel=[h5,h6,h7];
   % Coordendas relativas reales 
h8=plot(pr(1),pr(2),'rd','LineWidth',3)
h9=plot(p1(1),p1(2),'gd','LineWidth',3)
h10=plot(p2(1),p2(2),'bd','LineWidth',3)
   %puntos deseados
   hold on
   ellipse(20,20,0,-z1d(1,1),-z1d(2,1),'r') 
   ellipse(20,20,0,-z1d(1,2),-z1d(2,2),'g') 
   ellipse(20,20,0,-z1d(1,3),-z1d(2,3),'b') 
%    axis([-200,600,-100 500])
   axis('equal')
   title('Coordenadas relativas')
   xlabel('x(cm)')
   ylabel('y(cm)')
   legend('$a_1$','$a_2$','$a_3$','$a_4$','Location','bestoutside','Interpreter','Latex')
   set(gca,'FontSize',14)

% Distancia al objetivo

prreal=[pr(1),pr(2)];
p1real=[p1(1),p1(2)];
p2real=[p2(1),p2(2)];
aux=z2d(:,1)-prreal';
   dformro(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z3d(:,1)-p1real';
   dform1o(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z4d(:,1)-p2real';
   dform2o(i)=sqrt(aux(1)^2+aux(2)^2);
taux=0:dt:ttotal
subplot(2,2,[2,4])
h11=plot(taux(1:i),dformro(1:i),'r')
hold on
h12=plot(taux(1:i),dform1o(1:i),'g')
h13=plot(taux(1:i),dform2o(1:i),'b')
yline(20,'-','LineWidth',3);
xlabel('t(s)')
ylabel('d_{i1}^{obj}(cm)')
xlim([0 ttotal])
% set(gca, 'XScale', 'log')
set(gca, 'YScale', 'log')
legend('$a_2$','$a_3$','$a_4$','L\''imite','Location','best','Interpreter','Latex')
set(gca,'FontSize',18)



pause();
for i=2:length(t)
   %evolución del sistema real
   x=real(F*x+G*u);
   %distancias con incertidumbre
    hatdro=sqrt((x(1)-x(3))^2+(x(2)-x(4))^2);
   hatd1o=sqrt((x(1)-x(5))^2+(x(2)-x(6))^2); 
   hatdr1=sqrt((x(5)-x(3))^2+(x(6)-x(4))^2);
   hatd2o=sqrt((x(1)-x(7))^2+(x(2)-x(8))^2);
   hatdr2=sqrt((x(7)-x(3))^2+(x(8)-x(4))^2);
   dro=sigma*randn+hatdro;
   d1o=sigma*randn+hatd1o;
   dr1=sigma*randn+hatdr1;
   d2o=sigma*randn+hatd2o;
   dr2=sigma*randn+hatdr2;
   po=[0,0];
   pr=[dro,0];
   alpha1=acosd((+d1o^2+dro^2-dr1^2)/(2*dro*d1o));
   alpha2=acosd((+d2o^2+dro^2-dr2^2)/(2*dro*d2o));
   p1=[cosd(alpha1)*d1o,signo(1)*sind(alpha1)*d1o];
   p2=[cosd(alpha2)*d2o,signo(2)*sind(alpha2)*d2o];
   prreal=[hatdro,0];
   alpha1=acosd((+hatd1o^2+hatdro^2-hatdr1^2)/(2*hatdro*hatd1o));
   alpha2=acosd((+hatd2o^2+hatdro^2-hatdr2^2)/(2*hatdro*hatd2o));
   p1real=[cosd(alpha1)*hatd1o,signo(1)*sind(alpha1)*hatd1o];
   p2real=[cosd(alpha2)*hatd2o,signo(2)*sind(alpha2)*hatd2o];
 
   
   %  Coordenadas globales 
   set(h1,'Xdata',real(x(1)),'Ydata',real(x(2)))
    set(h2,'Xdata',real(x(3)),'Ydata',real(x(4)))
    set(h3,'Xdata',real(x(5)),'Ydata',real(x(6)))
    set(h4,'Xdata',real(x(7)),'Ydata',real(x(8)))
  
  
   
   %coordenadas relativas estimadas
set(h5,'Xdata',real(pr(1)),'Ydata',real(pr(2)))
set(h6,'Xdata',real(p1(1)),'Ydata',real(p1(2)))
set(h7,'Xdata',real(p2(1)),'Ydata',real(p2(2)))
%coordenadas relativas reales 
set(h8,'Xdata',real(prreal(1)),'Ydata',real(prreal(2)))
set(h9,'Xdata',real(p1real(1)),'Ydata',real(p1real(2)))
set(h10,'Xdata',real(p2real(1)),'Ydata',real(p2real(2)))
   
   %Aplico ley de control
   z1=[po;po;po]'-[pr;p1;p2]';
   z2=[pr;pr;pr]'-[po;p1;p2]';
   z3=[p1;p1;p1]'-[po;pr;p2]';
   z4=[p2;p2;p2]'-[po;pr;p1]';
   
   L1=-k*sum((z1-z1d)');
   u(1:2)=L1+uo;
   L2=-k*sum((z2-z2d)');
   u(3:4)=L2+ur;
   L3=-k*sum((z3-z3d)');
   u(5:6)=L3+u1;
   L4=-k*sum((z4-z4d)');
   u(7:8)=L4+u2;
   %calculo de distancias objetivo
   aux=z2d(:,1)-prreal';
   dformro(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z3d(:,1)-p1real';
   dform1o(i)=sqrt(aux(1)^2+aux(2)^2);
   aux=z4d(:,1)-p2real';
   dform2o(i)=sqrt(aux(1)^2+aux(2)^2);
   
   set(h11,'Xdata',taux(1:i),'Ydata',real(dformro(1:i)))
   set(h12,'Xdata',taux(1:i),'Ydata',real(dform1o(1:i)))
   set(h13,'Xdata',taux(1:i),'Ydata',real(dform2o(1:i)))
   
   pause(0.01)
   
end
%% Grafiaca de evolucion de distancias
close all
figure;

plot(t(2:end),dformro(2:end),'r')
hold on
plot(t(2:end),dform1o(2:end),'g')
hold on
plot(t(2:end),dform2o(2:end),'b')
hold on
yline(20,'-','LineWidth',3);
xlabel('t(s)')
ylabel('d_{i1}^{obj}(cm)')
set(gca, 'YScale', 'log')
legend('$a_2$','$a_3$','$a_4$','Limite de desviación','Interpreter','Latex')
set(gca,'FontSize',18)
