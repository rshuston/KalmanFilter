function [x , P] = kalman(N, t, F, G, Q, z, H, R, x0, P0)
  x = zeros(2, 1, N);
  P = zeros(2, 2, N);
  x_ = x0;
  P_ = P0;
  for k = [1 : N]
    [x_, P_] = kalman_1(x_, P_, F, G, Q, z(k), H, R);
    x(:,:,k) = x_;
    P(:,:,k) = P_;
  endfor
endfunction