function qref = motionplan(q0, qf, t1, t2, myrobot, obs, tol)
   alpha_i = 0.01;
   q = q0;
   
   % while not in threshold, compute new tau's to update q0
   % not considering obstacles yet
   while norm(q(end,1:5) - qf(end,1:5)) > tol
       q_cur = q(end, 1:6);
       attract = att(q_cur,qf,myrobot);
       
       repulse = 0;
       % disp(length(obs))
       if length(obs) >= 1
           for i = 1:length(obs)
               repulse = repulse + rep(q_cur,myrobot,obs{i});
           end
       end
       q_next = q_cur + (alpha_i * (attract + repulse)).'; % extra param of 0.8 works best on repulse
       q(end+1,1:6) = q_next;
   end
   t = linspace(t1,t2,size(q,1));
   qref = spline(t,q'); % defines a spline object with interpolation
                        % times in t and interpolation values the columns of q
end