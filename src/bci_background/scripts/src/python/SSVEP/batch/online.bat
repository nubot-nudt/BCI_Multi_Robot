start server.bat
cd ..\prog
start operat.exe
ping -n 1 127.1 >nul	
start RDAClient.exe  			127.0.0.1 
ping -n 1 127.1 >nul			
start MatlabSignalProcessing.exe      	--MatlabWD="./Matlab_SsvepMov/SsvepMov_Sim01_5" 127.0.0.1
ping -n 1 127.1 >nul	
start PythonApplication.exe		--PythonAppClassFile=Client.py 127.0.0.1
cd ..
