function [R U V] = PolDecompFunc(F)

[m n] = size(F);
if m ~= n
    error('Matrix must be square.');
end
C = F'*F;
[Q0 lambdasquare] = eig(C);
lambda = sqrt(diag((lambdasquare)));  
Uinv = repmat(1./lambda',size(F,1),1).*Q0*Q0';
R = F*Uinv;
U = R'*F;
V = F*R';