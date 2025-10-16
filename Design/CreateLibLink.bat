REM Create a directory junction from %1 to %2
REM Usage: CreateLibLink.bat <Path to Directory for Link/Link Name> <Directory Path 2 Link To>
REM Requires admin privileges

mlink /J %1 %2
