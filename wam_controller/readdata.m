% used to read data of the xPC target FAT partition
% you can put an xPC target scope in the simulink model that writes
% data to the FAT partition for debugging purposes
function output = readdata(filename)
    % Create file system object
    fsys = xpctarget.fs;

    % Open file on the target file system
    h = fsys.fopen(filename);

    % Read the data from the target file into a MATLAB variable. Note that 
    % this data will still be represented in xPC Target file 
    % format (i.e. not bytes) 
    data = fsys.fread(h);

    % Close file on file system
    fsys.fclose(h);

    % Call READXPCFILE to convert the data from xPC Target 
    % file format to bytes for use in MATLAB
    new_data = readxpcfile(data);
    
    output = new_data.data;
    %plot(velocity);
end