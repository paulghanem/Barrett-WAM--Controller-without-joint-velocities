% Receive a matfile from the tcp connection
% conn: handle to the tcp connection that has been set up
% tmpfile: the complete file path where the received file want to kept
function pnet_getfile(conn, tmpfile)

dataclass = pnet(conn,'readline',1024);
if strcmp(dataclass, '--matfile--')
    bytes = double(pnet(conn,'Read',[1 1],'uint32'));
    pnet(conn,'ReadToFile',tmpfile, bytes);
end

end
