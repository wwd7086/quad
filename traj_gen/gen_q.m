function Q = gen_q(p, r, t)
%p is the number of polynomial coefficient
%r is the order of derivative
%Q is a P*P matrix

%Q is the hessian of polynomial integration(0->1)
%pQp is a quadratic approximation of the original function
%it looks like a ball

Q = zeros(p);

for i = r:p
	for l = r:p
		q=1;
		
		for m=0:(r-1) 
			q = q*((i-m)*(l-m));
		end

		q = 2*q*(t^(i+l-2*r+1)/(i+l-2*r+1));

		Q(i,l) = q;

	end
end

end