function next = simulate(x, u, v, dist, dt)
  if dist == 0
      dist = [0,0]';
  end
  dx = zeros(length(x),1);
  dx(1) = v * cos(x(3)) + dist(1);
  dx(2) = v * sin(x(3)) + dist(2);
  dx(3) = u;
  next = x + dt.*dx;
end