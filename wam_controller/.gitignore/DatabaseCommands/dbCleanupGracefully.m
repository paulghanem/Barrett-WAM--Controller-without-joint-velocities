function dbCleanupGracefully(dbConn)
    % rollback any changes
    % this is safe to call even if
    %   a) there's nothing to rollback, or
    %   b) we were successful and already committed changes
    try
        rollback(dbConn);
    catch innerException %#ok<NASGU>
    end

    % close dbConn, so we don't leave connections hanging around
    try
        close(dbConn);
    catch innerException %#ok<NASGU>
    end
end
