%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% matlab process used in Simulated Moving Objects Recognition Project with SSVEP
% ## SsvepMov_Sim_01 ##
% ## mATLAB_Process ##
% version: 0.1
% Created on 2014
% Updated on 2015.6.25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out_signal = bci_Process( in_signal )

% bci_Parameters and states are global variables.
global bci_Parameters bci_States;

% global variables from function bci_Initalize.
global samplingrate;
global blocksize;

	%get from matlabsig
global blockindex;
global channel_ssvep ;
global signal;
global num_SourceCh;
num_SourceCh = int8(str2double(bci_Parameters.SourceCh));

	%get from app
global f_ssvep score_CCA ;
% f_ssvep = str2double(bci_Parameters.HzList);

global BPfilterSSVEP;
global sigpssvep_filter
sigpssvep_filter=[]

global ssvep_begin ssvep_end out_sig
global pre_phaseinsequence
phaseinsequence = bci_States.PhaseInSequence;

global  currenttrial subject_name subject_session subject_run subject_save savepath
currenttrial = bci_States.CurrentTrial
subject_save = strcat(subject_name,'S',subject_session,'R',subject_run)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if phaseinsequence >= 1

	updata_signal(in_signal);       											%get data
	% signal_ssvep = signal(:,channel_ssvep)										%get ssvep channels signal
	sigpssvep_filter = filter(BPfilterSSVEP, 1, signal(:,channel_ssvep));
	
		%ssvep begin and end index
		if (phaseinsequence == 2) && (pre_phaseinsequence == 1)
			ssvep_begin = blockindex - blocksize + 1;

		elseif (phaseinsequence == 3) && (pre_phaseinsequence == 2)				%include 1 point in phaseinsequence == 3
			ssvep_end = blockindex - blocksize + 1;
			
		end
		pre_phaseinsequence = phaseinsequence;

		if ssvep_end > ssvep_begin
			sigssvep_slice = sigpssvep_filter(ssvep_begin:ssvep_end-1,:);		%get ssvep channels signal
			ssvep_len = length(sigssvep_slice);									%the length of ssvep pieces
			num_SSVEP = length(f_ssvep);										%the number of ssvep frequences

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%	% CCA for ssvep
			tt = [1:ssvep_len]' * 1/samplingrate;
			score_CCA = zeros()
			cca_channel = zeros(num_SourceCh-1,num_SSVEP);
			
			for qq = 1:num_SSVEP
				YY = [sin(2*pi*f_ssvep(qq)*tt),cos(2*pi*f_ssvep(qq)*tt),sin(4*pi*f_ssvep(qq)*tt),...
					cos(4*pi*f_ssvep(qq)*tt),sin(6*pi*f_ssvep(qq)*tt),cos(6*pi*f_ssvep(qq)*tt)];
				[A,B,r] = canoncorr(sigssvep_slice,YY);
				score_CCA(qq) = max(r);
			end
			
			Test_maxCCA = max(score_CCA);
			index_CCA = find(score_CCA == Test_maxCCA);
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%	% FFT1 for ssvep
			
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%	% SNR for ssvep
			
			
			out_signal = [index_CCA,0,0] 
			
			% out_sig(currenttrial,:) = out_signal;
			
			% save(fullfile(savepath,subject_save),'out_sig')
			
			ssvep_begin = 0;
			ssvep_end = 0;  

		else
			out_signal = [0,0,0];         
		end

else
	out_signal = [0,0,0];
	blockindex = 0;
	signal = [];
	sigpssvep_filter=[];
end

end