% generate trajetory in a single direction based on points and time
% ployOrder: the order of the trajectory polynomial
% derOrder: the order of derivative which integration will be optimized
% contOrder: the order of derivatie which satisfise the continuity constraints
function traj=gen_traj_dp(polyOrder,derOrder,points,times,contOrder)
	%generate traj by dubin curves
	%poly is specified in this order
	%p0 + P1*t + p2*t*t + ........

	numOfTraj = size(points,2)-1;
	if size(times,1) ~= numOfTraj
		warning('time and point not consistent');
	end

	%% generate Q
	QQ = zeros(polyOrder*numOfTraj);
	for i=1:numOfTraj
		Q = gen_q(polyOrder,derOrder,times(i));
		sInd = polyOrder*(i-1)+1;
		eInd = polyOrder*i;
		QQ(sInd:eInd,sInd:eInd) = Q;	
	end
	
	%% generate constraints
	constrainOrder = size(points, 1);
	AA = zeros(constrainOrder*numOfTraj*2,polyOrder*numOfTraj);
	BB = zeros(constrainOrder*numOfTraj*2,1);
    
    %% generate Ai
	Ai = zeros(constrainOrder,polyOrder);
	Ai(1,1) = 1;
	for r=1:constrainOrder-1
		for n=0:polyOrder-1
			if n==r
				v = 1;
				for m=0:r-1
					v = v*(r-m);
				end
				Ai(r+1,n+1) = v;
			end
		end
    end
        
	for i=1:numOfTraj
		vsInd = polyOrder*(i-1)+1;
		veInd = polyOrder*i;
        
		AA(constrainOrder*(2*i-2)+1:constrainOrder*(2*i-1),vsInd:veInd) = Ai;
        BB(constrainOrder*(2*i-2)+1:constrainOrder*(2*i-1),1) = points(:,i);

		%% generate Af
		Af = zeros(constrainOrder,polyOrder);
		Af(1,:) = 1;
		for r=1:constrainOrder-1
			for n=0:polyOrder-1
				if n>=r
					v = 1;
					for m=0:r-1
						v = v*(n-m);
					end
					Af(r+1,n+1) = v*(times(i).^(n-r));
				end
			end
		end
		AA(constrainOrder*(2*i-1)+1:constrainOrder*2*i,vsInd:veInd) = Af;
        BB(constrainOrder*(2*i-1)+1:constrainOrder*2*i,1) = points(:,i+1);
		
	end

	%% generate concatenate constraints
	if constrainOrder<contOrder
		extraOrder = contOrder - constrainOrder;
		AAS = zeros(extraOrder*(numOfTraj+1),polyOrder*numOfTraj);
		BBS = zeros(extraOrder*(numOfTraj+1),1);
    
        %% generate ASi
		ASi = zeros(extraOrder,polyOrder);
		for r=constrainOrder:contOrder-1
			for n=0:polyOrder-1
				if n==r
					v = 1;
					for m=0:r-1
						v = v*(r-m);
					end
					ASi(r+1-constrainOrder,n+1) = v;
				end
			end
        end
        alt = 1;
		for i = 1:numOfTraj
			vsInd = polyOrder*(i-1)+1;
			veInd = polyOrder*i;

			AAS(extraOrder*(i-1)+1:extraOrder*i,vsInd:veInd) = ASi*alt;

			%% generate ASf
			ASf = zeros(extraOrder,polyOrder);
			for r=constrainOrder:contOrder-1
				for n=0:polyOrder-1
					if n>=r
						v = 1;
						for m=0:r-1
							v = v*(n-m);
						end
						ASf(r+1-constrainOrder,n+1) = v*(times(i).^(n-r));
					end
				end
			end
			AAS(extraOrder*i+1:extraOrder*(i+1),vsInd:veInd) = ASf*alt;

			alt = -alt;
        end
        
        AA = [AA;AAS];
        BB = [BB;BBS];
    end

	%% run quadratic programming
	
	% traj should be numTraj*oerderPoly , 1
	% be carful with the order of poly!
	traj = quadprog(QQ, zeros(polyOrder*numOfTraj,1),[],[],AA,BB);

end