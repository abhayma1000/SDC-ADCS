function R_IB = triad(v1b, v2b, v1i, v2i)
eps_val = 1e-9;

v1b = v1b(:); v2b = v2b(:); v1i = v1i(:); v2i = v2i(:);
assert(all([numel(v1b),numel(v2b),numel(v1i),numel(v2i)]==3), ...
    'All inputs must be 3-element vectors.');

n1b = norm(v1b); n1i = norm(v1i);
if n1b < eps_val || n1i < eps_val
    error('First input vector has zero length.');
end
q_b = v1b / n1b;
q_i = v1i / n1i;

c_b = cross(v1b, v2b); nc_b = norm(c_b);
c_i = cross(v1i, v2i); nc_i = norm(c_i);
if nc_b < eps_val || nc_i < eps_val
    error('Input vectors are (nearly) collinear; TRIAD is ill-conditioned.');
end
r_b = c_b / nc_b;
r_i = c_i / nc_i;

s_b = cross(q_b, r_b);
s_i = cross(q_i, r_i);

M_b = [q_b, r_b, s_b];
M_i = [q_i, r_i, s_i];

R_IB = M_i * M_b';
end
