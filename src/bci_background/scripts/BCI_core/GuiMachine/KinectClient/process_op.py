#-*- coding:utf-8 -*-
import win32com.client,os

def check_exsit(process_name): 
    WMI = win32com.client.GetObject('winmgmts:') 
    processCodeCov = WMI.ExecQuery('select * from Win32_Process where Name="%s"' % process_name) 
    if len(processCodeCov) > 0:return 1
    else:return 0

def kill_process(process_name):
    if os.system('taskkill /F /IM ' + process_name)==0:return 1
    else:return 0