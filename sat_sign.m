function y = sat_sign(s, delta)
% Saturated sign: linear in |s|<delta, sign(s) outside.

    y = zeros(size(s));
    for i = 1:numel(s)
        if abs(s(i)) < delta
            y(i) = s(i) / delta;
        else
            y(i) = sign(s(i));
        end
    end
end
