function mysegment()
    % setup kuka
    delta = fminunc(@deltajoint, [0,0]); %   -2.2207  -10.6655
    kuka = mykuka_search(delta);
    
    % setup path
    X_workspace = zeros(3,100);
    X_workspace(1,:) = 620;
    X_workspace(2,:) = linspace(-100,100,100);
    X_workspace(3,:) = -1;

    plot(X_workspace(1,:),X_workspace(2,:));

    R = [0 0  1;
         0 -1 0;
         1 0  0];
    for i = 1:100 % run through path
        X_baseframe = FrameTransformation(X_workspace(:,i));
        H = [R X_baseframe; 0,0,0,1];
        q = inverse_kuka(H,kuka);
        setAngles(q,0.04);
    end
end