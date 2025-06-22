function [dL, dR] = deriveDistance(Ldir, Rdir, inf_dir, a)
    % DERIVEDISTANCE    Compute the distances of 3D world points from the
    % camera center exploiting the "theorem of cosines" and the
    % corresponding backprojection rays.
    %
    % Ldir: backprojection ray of the first (left) point
    % Rdir: backprojection ray of the second (right) point
    % inf_dir: backprojection ray of the point at infinity
    % a: length of the 3D world segment connecting the two points
    
    % Compute angles inside the triangle
    %cos_alpha = Ldir'*Rdir;
    cos_beta = Ldir'*inf_dir;
    cos_gamma = -Rdir'*inf_dir;
    % to account for errors derive the third angle from the previous two
    
    syms b c    % distance of camera from right and left feature point respectively
    syms cos_alpha 
    
    % define equations of system
    eqns = [c^2 == a^2 + b^2 - 2*a*b*cos_gamma, ...
            a^2 == b^2 + c^2 - 2*b*c*cos_alpha, ...
            b^2 == a^2 + c^2 - 2*a*c*cos_beta];
    [solb, solc, ~] = solve(eqns);
    if solb(1)<0 || solc(1)<0
        dR = double(solb(2));
        dL = double(solc(2));
    else
        dR = double(solb(1));
        dL = double(solc(1));
    end
end