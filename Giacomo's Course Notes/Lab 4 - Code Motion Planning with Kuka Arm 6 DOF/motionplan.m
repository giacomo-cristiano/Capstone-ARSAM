function qref = motionplan(q0, qf, t1, t2, myrobot, obs, tol, att_alpha, rep_alpha)
   alpha_i = 1;
   q = q0;

   max_itr = 5000; % max iterations we'll allow it to run for
   itr = 0;
   
   while itr < max_itr
       q_cur = q(end, 1:6);
       attract = att(q_cur,qf,myrobot); % calculate attractive potential
       
       repulse = 0;
       if length(obs) >= 1 % calculate repulsive potentials for all objects
           for i = 1:length(obs)
               repulse = repulse + rep(q_cur,myrobot,obs{i});
           end
       end

       % update q_next
       q_next = q_cur + (alpha_i * (att_alpha * attract + rep_alpha * repulse)).';
       q(end+1,1:6) = q_next; % append q_next into q array

       itr = itr + 1;
       if norm(q(end,1:5) - qf(end,1:5)) <= tol % when in threshold, stop
           break;
       end
   end
   q(:,6) = linspace(q0(6), qf(6), itr+1)'; % update q6 as tau has no effect
   disp(itr)

   % disp(q(end,1:6))
   
   t = linspace(t1,t2,size(q,1));
   qref = spline(t,q'); % defines a spline object with interpolation
                        % times in t and interpolation values the columns of q
end