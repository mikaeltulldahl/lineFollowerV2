function R = rot(angle)
c = cos(angle);
s = sin(angle);
R = [c, -s;
     s,  c];
end