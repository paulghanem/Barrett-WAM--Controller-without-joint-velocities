% USE THIS FUNCTION WITH CAUTION!!!!!!!!
% IF THERE IS MEANINGFUL DATA IN THE DATABASE, DO NOT USE THIS!!!!!!!!
function dbPurgeDatabase()
    % check that the user really wants to do this
    fprintf('\n***About to delete ALL data in the grasp database***\n');
    yn = upper(input('Are you sure you really want to do this? Y/N [N]: ', 's'));
    if (length(yn) ~= 1 || (yn ~= 'Y' && yn ~= 'N'))
        fprintf('\nInvalid input. Answer must be Y or N (caps insensitve). Exiting now.\n\n');
        return;
    end
    
    % back out if necessary
    if(yn == 'N')
        fprintf('\nData not deleted. Exiting now.\n\n');
        return;
    end
    
    fprintf('\n');
    
    % connect using JDBC (faster than ODBC :-D)
    % a matlab bug causes all global variables to be cleared
    % from the workspace when calling javaaddpath
    javaaddpath('mysql-connector-java-5.1.13-bin.jar');
    dbConn = database('Experiments', 'robotics', 'sensornet','com.mysql.jdbc.Driver','jdbc:mysql://grasp.robotics:3306/Experiments');
    
    if(~isconnection(dbConn))
        error('Connection Error\n%s', dbConn.Message);
    end
    
    % if something screws up, shut the connection and rollback if necessary
    c = onCleanup(@()dbCleanupGracefully(dbConn));
    
    % set AutoCommit to off, so that no bad data goes in the database if
    % the function fails for whatever reason (like user actually realizes
    % what they're doing and Ctrl-C's)
    set(dbConn,'AutoCommit','off');
    
    tableNames = {'calibrations'; 'experiments'; 'frames'; 'trackables'; 'trackableMarkers'; ...
        'trackableFrames'; 'trackableMarkerFrames'; 'rawMarkers'; 'wamData'};
    for i=1:length(tableNames)
        fprintf('Deleting from %s...\n', tableNames{i});
        exec(dbConn, sprintf('DELETE FROM %s', tableNames{i}));
    end

    % ALTER TABLE statements do an implicit commit so we should do those
    % separately from the DELETE statements
    % let's commit here and be  about it, since it is going to happen on the
    % first statement in the loop anyway
    fprintf('\nCommitting the deletion... ');
    commit(dbConn);
    fprintf('done.\n');
    fprintf('Updating auto increments... ');
    for i=1:length(tableNames)
        exec(dbConn, sprintf('ALTER TABLE %s AUTO_INCREMENT=1', tableNames{i}));
    end
    fprintf('done.\n\n');
end
