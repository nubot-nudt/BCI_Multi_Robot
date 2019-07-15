%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% matlab process used in Simulated Moving Objects Recognition Project with SSVEP
% ## SsvepMov_Sim_01 ##
% ## mATLAB_Process ##
% Version: with 0.5.2
% Updated on 2015.9.14
% Descripption: outsig = max(outsig under sliding window per dare_duration)
% Advise: need much rationalize outsig
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out_signal = bci_Process( in_signal )

% bci_Parameters and states are global variables.
global bci_Parameters bci_States;

% global variables from function bci_Initalize.
global samplingrate;
samplingrate = 200;
global blocksize;

	%get from matlabsig
global blockindex;
global channel_ssvep ;
global signal;
global num_SourceCh;
num_SourceCh = int8(str2double(bci_Parameters.SourceCh));

	%get from app
global f_ssvep score_CCA score_CCA1 Mode;

global BPfilterSSVEP;
global sigpssvep_filter
sigpssvep_filter=[]

global ssvep_begin ssvep_end out_sig outsg
global ssvep_begin1 ssvep_end1
global pre_phaseinsequence
phaseinsequence = bci_States.PhaseInSequence;

global  currenttrial subject_name subject_session subject_run subject_save savepath Threhold_Create
currenttrial = bci_States.CurrentTrial
subject_save = strcat(subject_name,'S',subject_session,'R',subject_run)

global epoch threshold out_result
epoch=2
out_result=0

global global_k currt_k raw_outsg raw_outsg_ilen result_win
% save(fullfile(savepath,subject_save),'threshold','Threshold_Create')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if phaseinsequence >= 1

	updata_signal(in_signal);       		%get data
	% signal_ssvep = signal(:,channel_ssvep)					%get ssvep channels signal
	sigpssvep_filter = filter(BPfilterSSVEP, 1, signal(:,channel_ssvep));
	
	out_signal = [0,0,0];
	% out_signal=[length(signal(:,1)),0,0]
	
% 	if mod(Mode(1),2)==0
        	
		if (phaseinsequence == 2) && (pre_phaseinsequence == 1)
			ssvep_begin = blockindex - blocksize + 1;
			ssvep_end = ssvep_begin+blocksize*(samplingrate/blocksize*epoch);
			raw_outsg=zeros()
			currt_k=0

			ssvep_begin1 = blockindex - blocksize + 1;
			
		elseif (phaseinsequence == 3) && (pre_phaseinsequence == 2)	%include 1 point in phaseinsequence == 3
			ssvep_begin = 0;
			ssvep_end = 0;
			raw_outsg_ilen =zeros();
			for i = 1:max(raw_outsg)
				raw_outsg_ilen(i)=length(find(raw_outsg==i));
			end
			% out_signal = [max(raw_outsg),0,0];
			
			% out_sig(currenttrial,:) = out_signal;
			% outsg(currenttrial,:) = raw_outsg;
			% save(fullfile(savepath,subject_save),'out_sig')
			
			ssvep_end1 = blockindex - blocksize + 1;
		end
		pre_phaseinsequence = phaseinsequence;
		
		if (length(sigpssvep_filter(:,1))>=ssvep_end)&(ssvep_end>ssvep_begin)
			sigssvep_slice = sigpssvep_filter(ssvep_begin:ssvep_end-1,:);		%get ssvep channels signal
			ssvep_len = length(sigssvep_slice);									%the length of ssvep pieces
			num_SSVEP = length(f_ssvep);										%the number of ssvep frequences

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%	% CCA for ssvep
			tt = [1:ssvep_len]' * 1/samplingrate;
			score_CCA = zeros()
			currt_k = currt_k+1
			
			for qq = 1:num_SSVEP
				YY = [sin(2*pi*f_ssvep(qq)*tt),cos(2*pi*f_ssvep(qq)*tt),sin(4*pi*f_ssvep(qq)*tt),...
					cos(4*pi*f_ssvep(qq)*tt),sin(6*pi*f_ssvep(qq)*tt),cos(6*pi*f_ssvep(qq)*tt)];
				[A,B,r] = canoncorr(sigssvep_slice,YY);
				score_CCA(qq) = max(r);
			end
			
			Test_maxCCA = max(score_CCA);
			index_CCA = find(score_CCA == Test_maxCCA);
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			ssvep_begin = ssvep_begin+blocksize;
			ssvep_end = ssvep_begin+blocksize*(samplingrate/blocksize*epoch) 
			
			raw_outsg(currt_k)=index_CCA  
			% out_signal = [index_CCA,0,length(raw_outsg)];
		end
		
		if ssvep_end1 > ssvep_begin1
			sigssvep_slice1 = sigpssvep_filter(ssvep_begin1:ssvep_end1-1,:);		%get ssvep channels signal
			ssvep_len = length(sigssvep_slice1);									%the length of ssvep pieces
			num_SSVEP = length(f_ssvep);										%the number of ssvep frequences

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%	% CCA for ssvep
			tt = [1:ssvep_len]' * 1/samplingrate;
			score_CCA1 = zeros()
			for qq = 1:num_SSVEP
				YY = [sin(2*pi*f_ssvep(qq)*tt),cos(2*pi*f_ssvep(qq)*tt),sin(4*pi*f_ssvep(qq)*tt),...
					cos(4*pi*f_ssvep(qq)*tt),sin(6*pi*f_ssvep(qq)*tt),cos(6*pi*f_ssvep(qq)*tt)];
				[A,B,r] = canoncorr(sigssvep_slice1,YY);
				score_CCA1(qq) = max(r);
			end
			
			Test_maxCCA1 = max(score_CCA1);
			index_CCA1 = find(score_CCA1 == Test_maxCCA1);
			ssvep_begin1=0;
			ssvep_end1=0;
			
			% out_sig(currenttrial) = std(score_CCA1,0,2);
			% save(fullfile(savepath,subject_save),'out_sig')
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			if max(raw_outsg_ilen)>=.55*sum(raw_outsg_ilen)
				out_result=min(find(raw_outsg_ilen==max(raw_outsg_ilen)))
			else
				out_result=5
			end
			% out_signal = [max(find(raw_outsg_ilen==max(raw_outsg_ilen))),index_CCA1,length(raw_outsg)];
			out_signal = [min(find(raw_outsg_ilen==max(raw_outsg_ilen))),out_result,index_CCA1];%length(raw_outsg)];%max(raw_outsg_ilen)];,
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		end
		
% 	end

% 	if mod(Mode(1),2)==1
%         save mod1 Mode
% 		if (phaseinsequence == 2) && (pre_phaseinsequence == 1)
% 			ssvep_begin = blockindex - blocksize + 1;
% 			ssvep_end = ssvep_begin+blocksize*(samplingrate/blocksize*epoch);
% 			raw_outsg=zeros()
% 			currt_k=0
% 			result_win=zeros(1,5)
% 
% 		elseif (phaseinsequence == 3) && (pre_phaseinsequence == 2)				%include 1 point in phaseinsequence == 3
% 			ssvep_begin = 0;
% 			ssvep_end = 0;
% 		end
% 		pre_phaseinsequence = phaseinsequence;
% 
% 		if (length(sigpssvep_filter(:,1))>=ssvep_end)&(ssvep_end>ssvep_begin)
% 			sigssvep_slice = sigpssvep_filter(ssvep_begin:ssvep_end-1,:);		%get ssvep channels signal
% 			ssvep_len = length(sigssvep_slice);									%the length of ssvep pieces
% 			num_SSVEP = length(f_ssvep);										%the number of ssvep frequences
% 			
% 			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 			%%	% CCA for ssvep
% 			tt = [1:ssvep_len]' * 1/samplingrate;
% 			score_CCA = zeros()
% 			cca_channel = zeros(num_SourceCh-1,num_SSVEP);
% 			currt_k = currt_k+1
% 			
% 			for qq = 1:num_SSVEP
% 				YY = [sin(2*pi*f_ssvep(qq)*tt),cos(2*pi*f_ssvep(qq)*tt),sin(4*pi*f_ssvep(qq)*tt),...
% 					cos(4*pi*f_ssvep(qq)*tt),sin(6*pi*f_ssvep(qq)*tt),cos(6*pi*f_ssvep(qq)*tt)];
% 				[A,B,r] = canoncorr(sigssvep_slice,YY);
% 				score_CCA(qq) = max(r);
% 			end
% 			
% 			Test_maxCCA = max(score_CCA);
% 			
% 			if Test_maxCCA>=threshold
% 				index_CCA = find(score_CCA == Test_maxCCA);
% 			end
% 			
% 			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 			ssvep_begin = ssvep_begin+blocksize;
% 			ssvep_end = ssvep_begin+blocksize*(samplingrate/blocksize*epoch) 
% 			
% 			raw_outsg(currt_k)=index_CCA
% 			% out_signal=[index_CCA,0,currenttrial]
% 			if currt_k<=length(result_win)
% 				result_win(currt_k)=index_CCA
% 				out_signal = [0,0,0]
% 			else
% 				result_win(length(result_win)+1) = index_CCA   % push
% 				result_win(1) = []  % pop
% 				result=unique(result_win)
% 				if length(result)>1
% 					out_signal = [0,0,0]
% 				elseif length(result)==1
% 					out_signal=[result(1),0,currenttrial]
% 				elseif length(result)==2
% 					out_signal=[max(length(find(result_win==result(1))),length(find(result_win==result(2)))),currenttrial]
% 				end
% 			end
% 		end
% 	end

else
	out_signal = [0,0,0];
	blockindex = 0;
	signal = [];
	sigpssvep_filter=[];
end

end