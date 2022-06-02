REM Set the search directories, %~dp0 is the directory where our dll and .bat files are there
path=%path%;%windir%\Microsoft.NET\Framework64\v2.0.50727\;%~dp0;

REM we then register the .NET dll. A tlb file is created and entries in registry are made
regasm.exe /codebase VirtualRobotXML.dll /tlb

@pause