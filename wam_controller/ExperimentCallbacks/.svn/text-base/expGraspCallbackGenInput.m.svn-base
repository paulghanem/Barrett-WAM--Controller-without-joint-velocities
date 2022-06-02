function [inputParams,offsets] = expGraspCallbackGenInput(palmDist, rad, ncfgs)
    dec = 2*pi/ncfgs;
    th = 0:dec:(2*pi-1e-10);
    
    offsets = rad*[cos(th); sin(th)]';
    inputParams = struct('cylExpPos', [.458 .5968], 'cylTol', [.001 .001], 'palmDist', palmDist);
end
