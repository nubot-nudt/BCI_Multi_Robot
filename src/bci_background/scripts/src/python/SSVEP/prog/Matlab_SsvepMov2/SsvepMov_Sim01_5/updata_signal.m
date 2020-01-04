function updata_signal(in_signal)
%
% Description
%   updata_signal(in_signal, mode) updata the global variable 'signal' with 
%   the in_signal
%
% INPUT
%   in_signal: the signal block.
%   
% OUTPUT
%   updata the global variable 'signal'.

global signal blockindex blocksize;

%     signal = [signal, in_signal];

%source ->DataIOFilter already have SourceChGain and SourceChOffset 
%     Gains   = repmat((sourcechgain), [1 blocksize])';
%     Offsets = repmat((sourcechoffset), [1 blocksize])';
%     Gains = single(Gains);
%     Offsets = single(Offsets);
%     in_signal = single(in_signal);
%     in_signal = Gains .* (in_signal-Offsets);

    signal = cat(1,signal,in_signal');         %  in_signal size is channels*blocksize, must change to blocksize*channels
    blockindex = blockindex + blocksize;         

end