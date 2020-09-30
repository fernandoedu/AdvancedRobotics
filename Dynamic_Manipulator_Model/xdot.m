
%config = 1 -> R manipulator
%config = 2 -> P manipulator
%config = 3 -> RR manipulator
%config = 4 -> RP manipulator
 
%For 2-link manipulator
%X(1) = q1
%X(2) = q2
%X(3) = q1'
%X(4) = q2'
%X(5) = q1''
%X(6) = q2''
 
function xdot = xdot(X, t)
  
  config = 3;
  g=9.81;
  
if (config < 3)
  l = 10;  #1 link
  m = 2;  #1 link
  
  f = 10; #1 link

else
  l1 = 10;   #2 link
  l2 = 5;   #2 link
  m1 = 2;   #2 link
  m2 = 0.5;   #2 link
  
  f = [10;10]; #2 link
endif


if(config == 1)
  %Inertia Matrix  
  D = (1/3)*m*(l^2);
  
  %Coriolis, Centrigual and Gravity Matrix
  V = 0;
  G = -(1/2)*m*l*g*cos(X(1));

  Dinv = D^-1;
  Q = Dinv*(f-V-G);
  qdd = Q;
  
  xdot = [X(2);
          qdd;
          (qdd-X(3))/0.001;
          ];
          
elseif(config==2)
  %Inertia Matrix  
  D = m;
  
  %Coriolis, Centrigual and Gravity Matrix
  V = 0;
  G = m*g*sin(X(1));

  Dinv = D^-1;
  Q = Dinv*(f-V-G);
  qdd = Q;
  
  xdot = [X(2);
          qdd;
          (qdd-X(3))/0.001;
          ];
          
elseif(config == 3)
  %Inertia Matrix  
  D11 = ((1/3)*m1+m2)*(l1^2)+(1/3)*m2*(l2^2)+ m2*l1*l2*cos(X(2));
  D12 = (1/3)*m2*(l2^2)+(1/2)*m2*l1*l2*cos(X(2));
  D21 = D12;
  D22 = (1/3)*m2*(l2^2);
  
  %Coriolis, Centrigual and Gravity Matrix
  D111 = 0;
  D112 = -(1/2)*m2*l1*l2*sin(X(2));
  D121 = D112;
  D122 = -(1/2)*m2*l1*l2*sin(X(2));
  D211 = (1/2)*m2*l1*l2*sin(X(2));
  D212 = 0;
  D221 = D212;
  D222 = (1/2)*m2*l1*l2*sin(X(2));
  G1 = (1/2)*m1*g*l1*cos(X(1)) + m2*g*l1*cos(X(1)) + (1/2)*m2*g*l2*cos(X(1)+X(2));
  G2 = (1/2)*m2*g*l2*cos(X(1)+X(2));
  
  D = [D11 D12;D21 D22];
  V = [(D111*(X(3)^2))+(D112*X(3)*X(4))+(D121*X(3)*X(4))+(D122*(X(4)^2));
       (D211*(X(3)^2))+(D212*X(3)*X(4))+(D221*X(3)*X(4))+(D222*(X(4)^2)) 
      ];
  G = [ G1;
        G2
      ];

  Dinv = inv(D);
  Q = Dinv*(f-V-G);
  q1dd = Q(1);
  q2dd = Q(2);
  
  xdot = [X(3);
          X(4);
          q1dd;
          q2dd;
          (q1dd-X(5))/0.001;
          (q2dd-X(6))/0.001;
          ];
  
else
  %Inertia Matrix  
  D11 = (4/3)*m2*(l2^2) + m2*(l1^2) + 2*m2*l1*l2 + (1/3)*m1*(l1^2);
  D12 = 0;
  D21 = D12;
  D22 = m2;
  
  %Coriolis, Centrigual and Gravity Matrix
  D111 = 0;
  D112 = m2*l1+m2*l2;
  D121 = D112;
  D122 = 0;
  D211 = -m2*l1-m2*l2;
  D212 = 0;
  D221 = D212;
  D222 = 0;
  G1 = (1/2)*m1*g*l1*cos(X(1)) + m2*g*cos(X(1))*((l1/2)+l2);
  G2 = m2*g*sin(X(1));
  
  D = [D11 D12;D21 D22];
  V = [(D111*(X(3)^2))+(D112*X(3)*X(4))+(D121*X(3)*X(4))+(D122*(X(4)^2));
       (D211*(X(3)^2))+(D212*X(3)*X(4))+(D221*X(3)*X(4))+(D222*(X(4)^2)) 
      ];
  G = [ G1;
        G2
      ];

  Dinv = inv(D);
  Q = Dinv*(f-V-G);
  q1dd = Q(1);
  q2dd = Q(2);
  
  xdot = [X(3);
          X(4);
          q1dd;
          q2dd;
          (q1dd-X(5))/0.001;
          (q2dd-X(6))/0.001;
          ];

endif

endfunction
