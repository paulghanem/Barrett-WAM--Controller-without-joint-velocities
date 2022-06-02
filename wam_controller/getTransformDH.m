function T = getTransformDH(a,d,alpha,q)
    ct = cos(q);
    st = sin(q);
    cb = cos(alpha);
    sb = sin(alpha);
    
    T = [ ct, -st*cb,  st*sb, a*ct;
          st,  ct*cb, -ct*sb, a*st;
          0, sb, cb, d;
          0, 0, 0, 1 ];
end
