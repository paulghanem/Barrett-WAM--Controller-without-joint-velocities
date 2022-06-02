REM Set the search directories, %~dp0 is the directory where our dll and .bat files are there
path=%path%;%windir%\Microsoft.NET\Framework\v2.0.50727\;%~dp0;

REM we first Unregister the .NET dll (if previous version exists)
regasm.exe /u VirtualRobotXML.dll /tlb

@pause