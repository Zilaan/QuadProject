function writeC(K, Kr)

% Daniel
system('rm -rf ~/CrazyFlieStuff/crazy//modules/src/matrix.c');
system('touch ~/CrazyFlieStuff/crazy//modules/src/matrix.c');
f = fopen('~/CrazyFlieStuff/crazy//modules/src/matrix.c', 'w');
% cd ~/CrazyFlieStuff/crazy/ % Daniel


% Raman
% system('rm -rf ~/Documents/Programmering/Chalmers/Embedded/Project/crazyflie-firmware/modules/src/matrix.c');
% system('touch ~/Documents/Programmering/Chalmers/Embedded/Project/crazyflie-firmware/modules/src/matrix.c');
% f = fopen('~/Documents/Programmering/Chalmers/Embedded/Project/crazyflie-firmware/modules/src/matrix.c', 'w');


fprintf(f, '#include "matrix.h"\n\n');
fprintf(f, '%s\n', print1DCMatrix(K, 'kMatrix'));
fprintf(f, '%s\n', print1DCMatrix(Kr, 'krMatrix'));
fclose(f);
end