function readGravData( )
%READGRAVDATA Read data from xPC target written by the gravcomp model
%  Gets the files that were written to the xPC Target machine during execution of the
%  gravcomp model.  The read data is the current joint angles, velocities and accelerations,
%  joint torques, and readings from the 6-axis F/T sensor at the wrist
%

    % Create file system object
    fsys = xpctarget.fs;

    % Open file on the target file system
    FileGrav = fsys.fopen('GRAV001.dat');

    % Read the data from the target file into a MATLAB variable. Note that 
    % this data will still be represented in xPC Target file 
    % format (i.e. not bytes) 
    dataGrav = fsys.fread(FileGrav);

    % Close file on file system
    fsys.fclose(FileGrav);
    
    % Call READXPCFILE to convert the data from xPC Target 
    % file format to bytes for use in MATLAB
    MatDataGrav = readxpcfile(dataGrav);

    t = MatDataGrav.data(:,35);
    Q = MatDataGrav.data(:,1:7);
    Qd = MatDataGrav.data(:,8:14);
    Qdd = MatDataGrav.data(:,15:21);
    JointTorque = MatDataGrav.data(:,22:28);
    ForceTorque = MatDataGrav.data(:,29:34);
    
    desc = input('Enter a filename to save experiment: ','s');
    timeStamp = datestr(now,'mm-dd-yyyy_HHMM');
    save(desc, 't', 'Q', 'Qd','Qdd',...
        'JointTorque','ForceTorque','timeStamp');

end

