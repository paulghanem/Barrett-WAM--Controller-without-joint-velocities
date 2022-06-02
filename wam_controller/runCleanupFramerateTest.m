function runCleanupFramerateTest()
    try
        if(libisloaded('NPTrackingTools'))
            calllib('NPTrackingTools', 'TT_Shutdown');
        end
    catch
    end
end
