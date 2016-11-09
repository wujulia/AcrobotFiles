% Load controller
load swingUpStuff.mat

% Controller
uc = sosControlExp(tv,xtape);

disp('Sending commands now...')
runLCM(uc,[],[]);
disp('Done');



