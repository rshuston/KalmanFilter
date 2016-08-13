function [ x, P ] = kalman_1(x, P, F, G, Q, z, H, R)
  # Prediction:
  x_ = F * x;
  P_ = F * P * F' + G * Q * G';

  # Correction:
  dz = z - H * x_;
  S = H * P_ * H' + R;
  K = P_ * H' / S;
  x = x_ + K * dz;
  P = (eye(2) - K * H) * P_;
endfunction
