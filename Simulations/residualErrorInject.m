function [state, P] = residualErrorInject(oldState, oldP, sens, H, h, R)
    state = oldState;

    residual = sens - h;

    S = H*oldP*H' + R;

    K = (oldP * H') / S;
    posterioriErrorState = K * residual;
    P = (eye(3) - K * H) * oldP * (eye(3) - K*H)' + K*R*K';

    rot_vec = 1.0 * posterioriErrorState(1:3);
    dq = [1, 0.5 * rot_vec'];
    q = quatmultiply(oldState(1:4)', dq);
    q = q / norm(q);
    state(1:4) = q';
    
end
