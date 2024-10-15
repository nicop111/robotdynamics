function pinvA = pseudoInverseMat(A, lambda)

[m,n] = size(A);

if (m>n)
  pinvA = (A'*A + lambda*lambda*eye(n,n))\A';
elseif (m<=n)
  pinvA = A'/(A*A' + lambda*lambda*eye(m,m));
end
end
