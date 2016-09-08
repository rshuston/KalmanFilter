function [x , P] = kalman(N, t, F, G, Q, z, H, R, x0, P0)
  x = zeros(2, 1, N);
  P = zeros(2, 2, N);
  x_ = x0;
  P_ = P0;
  x(:,:,1) = x_;
  P(:,:,1) = P_;
  for k = [2 : N]
    dt = t(k) - t(k-1);
    F(1,2) = dt;
    G = [ 0.5 * dt * dt ; dt ];
    [x_, P_] = kalman_1(x_, P_, F, G, Q, z(k), H, R);
    x(:,:,k) = x_;
    P(:,:,k) = P_;
  endfor
endfunction
