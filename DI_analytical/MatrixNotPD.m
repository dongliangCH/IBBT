function p = MatrixNotPD(M)
    % If p = 0, then the input matrix is symmetric positive definite.
    [~,p] = chol(M);

end