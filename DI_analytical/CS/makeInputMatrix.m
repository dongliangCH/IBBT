% MAKEINPUTMATRIX Make block matrix for input term
%
%   For a discrete linear system system
%
%       x_{k + 1} = A_k x_k + M_k u_k,      k = 0, 1, ..., N - 1
%
%   If we define X = [x_0', ..., x_N']' and U = [u_0', ..., u_{N - 1}']', then
%   this function takes in an array of A_k and M_k and outputs the matrix
%   ScriptB such that:
% 
%       X = ScriptA * x0 + ScriptM * U
% 
%   See function makeStateMatrix to make the matrix ScriptA.
% 
% Inputs:
%   AA          State matrix:         	size (nx, nx, N),   A_k = AA(:, :, k)
%   MM          Input coeff. matrix:    size (nx, nm, N), 	M_k = MM(:, :, k)
% 
% Outputs:
%   ScriptM     Block matrix:           size ((N + 1) * nx, N * nm)
% 
% Author: Jack Ridderhof

function ScriptM = makeInputMatrix(AA, MM, N)

nx = size(AA, 1);
nm = size(MM, 2);

ScriptM = zeros((N + 1) * nx, N * nm);

for i = 0:N
    ndx = (i * nx) + (1:nx);

    % For notation: see cdc 2019 paper by Ridderhof, Okamoto, Tsiotras
    Mk_bar = zeros(nx, nm * i);

    for j = 0:(i - 1)
        Mk_bar(:, (j * nm) + (1:nm)) = M_k1_k0(AA, MM, i - 1, j);
    end
    ScriptM(ndx, 1:(i * nm)) = Mk_bar;

end

end


% From report, B_{k1, k0} and G_{k1, k0} follow same construction
function Mk1k0 = M_k1_k0(AA, MM, k1, k0)

    if k1 == k0
        Mk1k0 = MM(:, :, k1 + 1);
    else
        Mk1k0 = A_k1_k0(AA, k1, k0 + 1) * MM(:, :, k0 + 1);
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