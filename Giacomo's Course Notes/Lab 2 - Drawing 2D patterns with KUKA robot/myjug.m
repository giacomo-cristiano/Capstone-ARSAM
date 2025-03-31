function myjug()
    % setup kuka
    delta = fminunc(@deltajoint, [0,0]); %   -2.2207  -10.6655
    kuka = mykuka_search(delta);
    
    % setup path
    data=xlsread('jug.xlsx');
    xdata=550 + 10*data(:,1);
    ydata=10*data(:,2);
    zdata=-ones(length(data),1);

    X_workspace = zeros(3,length(data));
    X_workspace(1,:) = xdata;
    X_workspace(2,:) = ydata;
    X_workspace(3,:) = zdata;

    figure
    plot(xdata,ydata);
    
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