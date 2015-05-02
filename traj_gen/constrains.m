function ret = constrains(input, min, max)

	if min>=max
		warning('constrains: min equal or bigger than max');
	end

	input(input < min)=min;
	input(input > max)=max;

	ret = input;

end