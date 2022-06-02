% Send a mat file through a tcp connection
% con: handle to the connection socket
% tmpfile: complete path to the mat file
function pnet_putfile(con, tmpfile)
pnet(con,'printf','--matfile--\n');
filedata=dir(tmpfile);
pnet(con,'Write',uint32(filedata.bytes));
pnet(con,'WriteFromFile',tmpfile);
end
