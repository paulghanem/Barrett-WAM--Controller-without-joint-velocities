
%
% readWAMData.m
% 
% Gets the files that were written to the xPC Target machine during execution of the
% wamjc7 model.  These files are the current joint angles (Q), the changes in desired
% joint angle during a cartesian move (Qddes), and the error vector durint a cartesian
% move
%
% Author: Hans Vorsteveld
%

function readWAMData()
    % Create file system object
    fsys = xpctarget.fs;

    % Open file on the target file system
    FileQFT = fsys.fopen('QFT001.dat');
    FileQdes = fsys.fopen('QDES001.dat');
    FileError = fsys.fopen('ERROR001.dat');

    % Read the data from the target file into a MATLAB variable. Note that 
    % this data will still be represented in xPC Target file 
    % format (i.e. not bytes) 
    dataQFT = fsys.fread(FileQFT);
    dataQdes = fsys.fread(FileQdes);
    dataError = fsys.fread(FileError);

    % Close file on file system
    fsys.fclose(FileQFT);
    fsys.fclose(FileQdes);
    fsys.fclose(FileError);
    
    % Call READXPCFILE to convert the data from xPC Target 
    % file format to bytes for use in MATLAB
    MatDataQFT = readxpcfile(dataQFT);
    MatDataQdes = readxpcfile(dataQdes);
    MatDataError = readxpcfile(dataError);
    
    t = MatDataQFT.data(:,57);
    Q = MatDataQFT.data(:,1:7);
    Qd = MatDataQFT.data(:,8:14);
    Qdd = MatDataQFT.data(:,15:21);
    Force = MatDataQFT.data(:,22:24);
    Torque = MatDataQFT.data(:,25:27);
    ForceOriginalFilter = MatDataQFT.data(:,28:30);
    TorqueOriginalFilter = MatDataQFT.data(:,31:33);
    ForceNewFilter = MatDataQFT.data(:,34:36);
    TorqueNewFilter = MatDataQFT.data(:,37:39);
    GripperForce = MatDataQFT.data(:,40);
    GripperPosition = MatDataQFT.data(:,41);
    GripperGraspSuccess = MatDataQFT.data(:,42);
    GripperAxisBlocked = MatDataQFT.data(:,43);
    SensorExpFT = MatDataQFT.data(:,44:49);
    Tau = MatDataQFT.data(:,50:56);


%     t = MatDataQFT.data(:,78);
%     Q = MatDataQFT.data(:,1:7);
%     Qd = MatDataQFT.data(:,8:14);
%     Qdd = MatDataQFT.data(:,15:21);
%     Force = MatDataQFT.data(:,22:24);
%     Torque = MatDataQFT.data(:,25:27);
%     ForceOriginalFilter = MatDataQFT.data(:,28:30);
%     TorqueOriginalFilter = MatDataQFT.data(:,31:33);
%     ForceNewFilter = MatDataQFT.data(:,34:36);
%     TorqueNewFilter = MatDataQFT.data(:,37:39);
%     GripperForce = MatDataQFT.data(:,40);
%     GripperPosition = MatDataQFT.data(:,41);
%     GripperGraspSuccess = MatDataQFT.data(:,42);
%     GripperAxisBlocked = MatDataQFT.data(:,43);
%     SensorExpFT = MatDataQFT.data(:,44:49);
%     Tau = MatDataQFT.data(:,71:77);


    tdes = MatDataQdes.data(:,22);
    Qdes = MatDataQdes.data(:,1:7);
    Qddes = MatDataQdes.data(:,8:14);
    Qdddes = MatDataQdes.data(:,15:21);
    
    terr = MatDataError.data(:,29);
    Error = MatDataError.data(:,1:6);
    Type = MatDataError.data(:,7);
    Xdes = MatDataError.data(:,8:10);
    Xddes = MatDataError.data(:,11:13);
    FTError = MatDataError.data(:,14);
    ContactOrNot = MatDataError.data(:,15);
    ContactScenario = MatDataError.data(:,16);
    Done = MatDataError.data(:,17);
    ImpErr = MatDataError.data(:,18);
    FE = MatDataError.data(:,19:24);
    Cout1 = MatDataError.data(:,25);
    Cout2 = MatDataError.data(:,26);
    ErrorCounts = MatDataError.data(:,27);
    ContactCounts = MatDataError.data(:,28);

    %fname = input('Enter file name to save data as: ','s');
    %save(sprintf('%s_%s.mat',fname,datestr(now,'mm-dd-yyyy_HHMM')), 't', 'Q', 'Qd', 'Qdes', 'Qddes', 'Error','Type','Xdes','Xddes','Force','Torque');
    desc = input('Enter a filename: ','s');
    
    fprintf('   0 : invalid contact case\n');
    fprintf('   1 : left_fingertip (side with cables)\n');
    fprintf('   2 : right_fingertip\n');
    fprintf('   3 : both_fingertip_touch\n');
    fprintf('   4 : palm\n');
    fprintf('   5 : left_fingerback (side with cables)\n');
    fprintf('   6 : right_fingerback\n');
    fprintf('   7 : right_finger\n');
    fprintf('   8 : left_finger (side with cables)\n');
    
    actualContactMode = input('Enter the initial actual contact mode: ');
    online_contact_mode = input('Enter the online contact mode: ');
    timeStamp = datestr(now,'mm-dd-yyyy_HHMM');
    fmin_search_data = 1;
    
    save(desc, 't','tdes', 'Q', 'Qd','Qdd',...
        'Qdes', 'Qddes','Qdddes', 'Error','Type','Xdes','Xddes','Force','Torque',...
        'ForceOriginalFilter','TorqueOriginalFilter','ForceNewFilter','TorqueNewFilter','desc',...
        'GripperForce','GripperPosition','GripperGraspSuccess','GripperAxisBlocked','SensorExpFT',...
        'timeStamp','ContactOrNot','ContactScenario','FTError','terr','actualContactMode',...
        'online_contact_mode','fmin_search_data','Tau','Done','ImpErr','FE','Cout1','Cout2','ErrorCounts','ContactCounts');
end
