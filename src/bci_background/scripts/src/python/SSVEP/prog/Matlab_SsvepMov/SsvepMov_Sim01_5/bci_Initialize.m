function bci_Initialize( in_signal_dims, out_signal_dims )

% Filter initialize demo
% 
% Perform configuration for the bci_Process script.

% BCI2000 filter interface for Matlab
% juergen.mellinger@uni-tuebingen.de, 2005
% (C) 2000-2009, BCI2000 Project
% http://www.bci2000.org

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% matlab process used in Simulated Moving Objects Recognition Project with SSVEP
% ## SsvepMov_Sim_01 ##
% ## matLAB_Initialize ##
% version: 0.1
% Created on 2014
% Updated on 2015.6.26
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% % Parameters and states are global variables.
global bci_Parameters bci_States;

global signal out_sig
signal = [];
out_sig = zeros(out_signal_dims)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %get from source
global samplingrate blocksize
samplingrate = 200;
samplingrate = str2double(bci_Parameters.SamplingRate);
blocksize = str2double(bci_Parameters.SampleBlockSize);

% global sourcechgain sourcechoffset;
% sourcechgain = str2double(bci_Parameters.SourceChGain);
% sourcechoffset = str2double(bci_Parameters.SourceChOffset);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% % get from storage
global  currenttrial subject_name subject_session subject_run
subject_name = char(bci_Parameters.SubjectName)%num2str
subject_session = char(bci_Parameters.SubjectSession)%num2str
subject_run = char(bci_Parameters.SubjectRun)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %get from matlabsig
global blockindex;
blockindex = 0;
global channel_ssvep ;%chnum_ssvep ;
channel_ssvep = str2double(bci_Parameters.SSVEPChannel);
% chnum_ssvep = length(channel_ssvep);

global BPfilterSSVEP;
temp = load('BP_4_35_n96.mat');
BPfilterSSVEP = temp.Num;
global BP_4_35_N96;
BP_4_35_N96 = fir1(96,[4 35].*2./200);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %get from app
global Mode f_ssvep numssvep_Hz %TargetList
Mode = str2double(bci_Parameters.Mode);
f_ssvep = str2double(bci_Parameters.HzList);
numssvep_Hz = length(f_ssvep);
% TargetList=str2double(bci_Parameters.TargetList);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% % new savefiles saved by .mat format
global savepath
p = mfilename('fullpath')
currpath = fileparts(p)
pos_v = strfind(currpath,'prog')
savepath = fullfile(currpath(1:pos_v-1),'data\',strcat(subject_name,subject_session),'\')

bci_savename = strcat(subject_name,'S',subject_session,'R')
len = length(bci_savename)
x = dir(savepath);

dir_list = zeros()
subject_run_num = zeros()

for i = 1:length(x)
	dir_list(i) = x(i).isdir;
end
if min(dir_list) == 1
	subject_run = char('01')
else
	dlt=find(dir_list==0)
	for i=1:length(dlt)
		try
			if strcmp(x(dlt(i)).name(1:len),bci_savename)
				subject_run_num(i)=str2double(x(dlt(i)).name(len+1:len+2))
			else subject_run_num(i)=0
			end
		catch
			subject_run_num(i)=0
		end
	end
	subject_run=max(subject_run_num)+1
	if subject_run < 10
		subject_run = strcat('0',num2str(subject_run))
	else 
		subject_run = num2str(subject_run)
	end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% % get Threshold from .dat using exect function 
global threshold
Threshold_Create = strcat(subject_name,'S',subject_session,'R00.dat')
if mod(Mode(1),2)==1
	if exist(fullfile(savepath,Threshold_Create))
		threshold=mean(Ssvep_Mov_Threshold(fullfile(savepath,Threshold_Create)))
	else
		threshold=0
	end
	threshold=0
end
threshold=0