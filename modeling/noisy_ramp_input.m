#
# Filter Design:
#
# x = F x + G w
# Q = E{w w'}
#
# z = H x + v
# R = E{v v'}
#
# x = [ x , v ]'
# x0 = [0 ; 0]
# F = [ 1 , T,
#       0 , 1 ]
# G = [ 0.5*T*T , T ]'
# Q = var_a = 0.0001
#
# P0 = [1 , 0 ; 0 , 0.5 ]
#
# z = [ z ]'
# H = [ 1 , 0 ]
# R = var_z = 1
#

pkg load statistics

N = 128;
dt = 1;
v = 0.5;
t = dt * [0 : N-1];
t_corner = 10;
x = (v * (t - t_corner)) .* (t >= t_corner);
n = random("normal", 0, 1, [1, N]);
z = x + n;

F = [ 1, dt ; 0 , 1 ];
G = [ 0.5 * dt * dt ; dt ];
Q = [ 0.0001 ];

H = [ 1, 0 ];
R = [ 1 ];

x0 = [0 ; 0];
P0 = [1 , 0 ; 0 , 0.5 ];

[xf, P] = kalman(N, t, F, G, Q, z, H, R, x0, P0);

xf_1 = xf(1,:);
xf_2 = xf(2,:);

P_11 = zeros(1, N);
P_12 = zeros(1, N);
P_22 = zeros(1, N);
for k = [1 : N]
  P_11(k) = P(1, 1, k);
  P_12(k) = P(1, 2, k);
  P_22(k) = P(2, 2, k);
endfor

figure(1);
plot(t, z, t, x, t, xf_1);

figure(2);
plot(t, xf_2);

figure(3);
plot(t, P_11, t, P_12, t, P_22);

filename = "noisy_ramp_input.csv";
fid = fopen (filename, "w");
fprintf(fid, "t, x, z, xf, vf, P11, P12, P22\n");
for k = [1 : N]
    fprintf(fid, "%f, %f, %f, %f, %f, %f, %f, %f\n", t(k), x(k), z(k), xf_1(k), xf_2(k), P_11(k), P_12(k), P_22(k));
endfor
fclose (fid);
