function ret = v_op(R)
	ret = zeros(3,1);
	ret(1) = -R(2,3);
	ret(2) = R(1,3);
	ret(3) = -R(1,2);
end