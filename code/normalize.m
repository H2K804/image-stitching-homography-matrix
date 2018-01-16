function [matches, TL, TR] = normalize(matches)

uL = mean(matches(:, 1));
vL = mean(matches(:, 2));
uR = mean(matches(:, 3));
vR = mean(matches(:, 4));
matches(:,1) = matches(:,1) - uL;
matches(:,2) = matches(:,2) - vL;
matches(:,3) = matches(:,3) - uR;
matches(:,4) = matches(:,4) - vR;

distL = sqrt(matches(:,1).^2 + matches(:,2).^2);
distR = sqrt(matches(:,3).^2 + matches(:,4).^2);

scaleL = sqrt(2)/mean(distL);
scaleR = sqrt(2)/mean(distR);

matches(:, [1,2]) = matches(:,[1,2])*scaleL;
matches(:, [3,4]) = matches(:,[3,4])*scaleR;

TL = [scaleL    0     -scaleL*uL
       0    scaleL   -scaleL*vL
       0      0         1   ]; 
   

TR = [scaleR    0     -scaleR*uL
       0    scaleR   -scaleR*vL
       0      0         1   ];

end