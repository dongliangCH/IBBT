% MAKESTATEMATRIX Make a block matrix for the initial state term
%
%   For a discrete linear system system
%
%       x_{k + 1} = A_k x_k + M_k u_k,      k = 0, 1, ..., N - 1
%
%   If we define X = [x_0', ..., x_N']' and U = [u_0', ..., u_{N - 1}']', then
%   this function takes in an array of A_k and outputs the matrix ScriptA such
%   that:
% 
%       X = ScriptA * x0 + ScriptM * U
% 
%   See function makeInputMatrix to make the matrix ScriptA.
% 
% Inputs:
%   AA          State matrix:         	size (nx, nx, N),   A_k = AA(:, :, k)
% 
% Outputs:
%   ScriptA     Block matrix:           size ((N + 1) * nx, N * nm)
% 
% Author: Jack Ridderhof

function ScriptA = makeStateMatrix(AA, N)

nx = size(AA, 1);
ScriptA = zeros((N + 1) * nx, nx);

for i = 0:N
    ndx = (i * nx) + (1:nx);
    ScriptA(ndx, :) = A_k1_k0(AA, i - 1, 0);
end

end


% From report, A_{k1, k0} function
function Ak1k0 = A_k1_k0(AA, k1, k0)

    nx = size(AA, 1);
    
    if k1 < k0
        Ak1k0 = eye(nx);
    else
        
        Phi = eye(nx);
        for i = k0:k1
            Phi = AA(:, :, i + 1) * Phi;
        end
        Ak1k0 = Phi;
    end

end