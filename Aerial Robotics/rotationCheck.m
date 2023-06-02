function rotationCheck = rotationCheck(r)
    
inv_r = inv(r);

% Check if r is orthogonal
is_orthogonal = isequal(r * r', eye(size(r)));

if is_orthogonal && (det(r)=1) 
    disp("It is a rotation matrix");
else
    disp("It is not a rotation matrix")
end

end
