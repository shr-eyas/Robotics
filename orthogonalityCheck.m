function orthogonalityCheck = orthogonalityCheck(r)
inv_r = inv(r);
tolerance = 1e-10;  % Adjust the tolerance value as per your requirements

% Check if r is orthogonal
is_orthogonal = isequal(r * r', eye(size(r)), 'AbsTol', tolerance);

% Check if r has determinant 1
has_det_one = abs(det(r) - 1) < tolerance;

% Check if inv_r is orthogonal
is_inv_orthogonal = isequal(inv_r * inv_r', eye(size(inv_r)), 'AbsTol', tolerance);

if is_orthogonal && has_det_one && is_inv_orthogonal
    disp("It is orthogonal");
else
    disp("Not orthogonal")
end
end