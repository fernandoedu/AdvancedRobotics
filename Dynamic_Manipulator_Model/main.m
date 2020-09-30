
clear
clc
close all;

%config = 1 -> R manipulator
%config = 2 -> P manipulator
%config = 3 -> RR manipulator
%config = 4 -> RP manipulator

config = 3;

%time sample
tspan=linspace(0, 1, 1000);  %0 to 1 second, sampled for 1000 pts

if( config < 3 )
  x0=[0;0;0];
else
  x0=[0;0;0;0;0;0];
endif

figure(1)
title('Joint Displacment')
xlabel('time'), ylabel('displacement'), hold on
figure(2)
title('Joint Velocity')
xlabel('time'), ylabel('velocity'), hold on
figure(3)
title('Joint Acceleration')
xlabel('time'), ylabel('acceleration'), hold on

x = lsode('xdot', x0, tspan);

if(config < 3)
  %Plot displacement
  figure(1)
  hold on
  plot(tspan, x(:,1))
  grid on
  
  %Plot velocity
  figure(2)
  hold on
  plot(tspan, x(:,2))
  grid on
  
  %Plot acceleration
  figure(3)
  hold on
  plot(tspan, x(:,3))
  grid on
  
else
  %Plot displacement
  figure(1)
  hold on
  plot(tspan, x(:,1), 'b', tspan, x(:,2), 'r')
  grid on
  legend("Joint 1", "Joint 2")
  
  %Plot velocity
  figure(2)
  hold on
  plot(tspan, x(:,3), 'b', tspan, x(:,4), 'r')
  grid on
  legend("Joint 1", "Joint 2")
  
  %Plot acceleration
  figure(3)
  hold on
  plot(tspan, x(:,5), 'b', tspan, x(:,6), 'r')
  grid on
  legend("Joint 1", "Joint 2")
  
  
endif
