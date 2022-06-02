% uses LOAD DATA LOCAL INFILE to do inserts really quickly
% http://dev.mysql.com/doc/refman/5.1/en/load-data.html
function dbBulkInsert(dbConn,table,columns,data)
    % form the format strings
    fieldCount = length(columns);
    formatStr = [repmat('%.8f\t',1,fieldCount-1) '%.8f\t\n'];
    columnStr = [sprintf('%s,',columns{1:end-1}) columns{end}];
    
    % print all of the output data into the bulk variable, replacing NaN with \N
    bulk = strrep(sprintf(formatStr, data'),'NaN','\N');
    
    % now write the file
    fid = fopen('bulk.sql','wt');
    fprintf(fid, '%s', bulk);
    fclose(fid);
    
    % execute the query
    % if this tells you something about closeSqlStatement, it means the query failed, check e.message
    query = sprintf('LOAD DATA LOCAL INFILE ''bulk.sql'' INTO TABLE %s FIELDS TERMINATED BY ''\\t'' ESCAPED BY ''\\\\'' LINES TERMINATED BY ''\\n'' (%s)',table,columnStr);
    e = exec(dbConn,query);
    close(e);
end
