clear all
%�ļ���ȡ
[datfiles, datdir] = ...
    uigetfile('*.dat','Select the BCI2000 SSVEP (.dat) data file(s)','multiselect','on','..\data\');
if datdir == 0, return; end
if ~iscell(datfiles)
    datfiles = {datfiles};
end
datfiles = sort(datfiles);

%��Ϣ��ȡ
[signal, state, parms] = getInfo(datfiles, datdir);
save dwall5 signal state parms


