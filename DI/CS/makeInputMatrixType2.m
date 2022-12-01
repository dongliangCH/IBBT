% MAKEINPUTMATRIXTYPE2 Make block matrix for inputs, type 2 (see descr.)
% 
%   For a discrete linear system system (note M_{k + 1}, not M_k)
%
%       x_{k + 1} = A_k x_k + M_{k + 1} u_{k + 1},      k = 0, 1, ..., N
%
%   If we define X = [x_0', ..., x_N']' and U = [u_0', ..., u_N']', then
%   this function takes in an array of A_k and M_k and outputs the matrix
%   ScriptB such that:
% 
%       X = ScriptA * x0 + ScriptM * U
% 
%   See function makeStateMatrix to make the matrix ScriptA.
% 
% Inputs:
%   AA          State matrix:         	size (nx, nx, N),     A_k = AA(:, :, k)
%   MM          Input coeff. matrix:    size (nx, nm, N + 1), M_k = MM(:, :, k)
% 
% Outputs:
%   ScriptM     Block matrix:           size ((N + 1) * nx, (N + 1) * nm)
% 
% Author: Jack Ridderhof

function ScriptM = makeInputMatrixType2(AA, MM, N)

nx = size(AA, 1);
nm = size(MM, 2);

ScriptM = zeros((N + 1) * nx, (N + 1) * nm);

for i = 0:N
    ndx = (i * nx) + (1:nx);

    % Recursive definition:
    %   M0_bar = M_0
    %   Mi_bar = [ A_{i - 1} * M{i - 1}bar, M_i ]
    % Block matrix has Mi_bar has first columns on ith row.
    
    if i == 0
        
        ScriptM(ndx, 1:nm) = MM(:, :, 1);
        Mim1_bar = MM(:, :, 1);
        
    else
        
        Mi_bar = [AA(:, :, i) * Mim1_bar, MM(:, :, i + 1)];
        ScriptM(ndx, 1:((i + 1) * nm)) = Mi_bar;
        Mim1_bar = Mi_bar;
        
    end
    
end

end
