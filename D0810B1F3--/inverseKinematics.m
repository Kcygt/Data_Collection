function qp = inverseKinematics(q, xp)
    J = jacobian(q);  % Assuming you already have the jacobian function implemented
   
    Jdagger = pinv(J);  % Pseudo-inverse of J using the pinv function
   
    qp = Jdagger * xp;
end