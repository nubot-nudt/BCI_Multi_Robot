function [ out_signal_dim ] = bci_Preflight( in_signal_dim )

% Filter preflight demo
% 
% Check whether parameters and states are accessible, and whether
% parameters have values that allow for safe processing by the 
% bci_Process function.
% Report any errors as demonstrated below.
% Also, report output signal dimensions in the 'out_signal_dim' argument.

% BCI2000 filter interface for Matlab
% juergen.mellinger@uni-tuebingen.de, 2005
% (C) 2000-2009, BCI2000 Project
% http://www.bci2000.org

% Parameters and states are global variables.
global bci_Parameters bci_States;

out_signal_dim = [1,3];

end

