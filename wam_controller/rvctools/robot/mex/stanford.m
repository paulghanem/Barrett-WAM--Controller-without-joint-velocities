L{1}=link([-pi/2 0 0 0 0 1  0  1  0 1 1 1 0 0 0 291e-6  -62.6111]);
L{2}=link([ pi/2 0 0 1 0 1  0 -1  0 1 1 1 0 0 0 409e-6  107.815]);
L{3}=link([    0 0 0 0 1 1  0  0 -1 1 1 1 0 0 0 299e-6  -53.7063]);
L{4}=link([-pi/2 0 0 0 0 1  0  1  0 7 0.5 11 9 3 2 291e-6       -62.6111]);
L{5}=link([ pi/2 0 0 0 0 1  0  1  0 3 2 4 5 -3 -6 409e-6        107.815]);
L{6}=link([             0 0 0 1 0 1  0  0 -1 1 1 1 0 0 0 299e-6 -53.7063]);
stan=robot(L, 'Stanford');
clear L