cd ..\python
start Server.py 127.0.0.1 

cd ..\prog
start operat.exe

start RDAClient.exe  			127.0.0.1 
			
start MatlabSignalProcessing.exe      	--MatlabWD="./Matlab_SsvepMov/SsvepMov_Sim01_5" 127.0.0.1
	
start PythonApplication.exe		--PythonAppClassFile=Client.py 127.0.0.1
cd ..
