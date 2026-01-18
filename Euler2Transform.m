% Convert euler angles to a rotation matrix
function rot = Euler2Transform(euler)
    psi = euler(3); theta = euler(2); phi = euler(1);
    rot = [cos(phi)*cos(psi)+sin(phi)*cos(theta)*sin(psi) -cos(phi)*sin(psi)-sin(phi)*cos(theta)*cos(psi) sin(phi)*sin(theta)
           sin(phi)*cos(psi)-cos(phi)*cos(theta)*sin(psi) -sin(phi)*sin(psi)+cos(phi)*cos(theta)*cos(psi) -cos(phi)*sin(theta)
           sin(theta)*sin(psi) sin(theta)*cos(psi) cos(theta)];
end
