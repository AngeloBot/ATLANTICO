# -*- coding: utf-8 -*-
"""
Created on Tue Sep 29 10:31:51 2020

@author: Windows
"""

import os	
from datetime import datetime

print(os.getcwd())

def main():
    '''
    Abre o arquivo log
    '''
    nome_arquivo = str(input("Nome do arquivo a ser traduzido: "))
    
    f=open(nome_arquivo+'.txt')
    
    identificador_output=str(input("Nome do arquivo de output: "))
    
    f_output=open("log_"+identificador_output+".txt",mode="w+", newline="\n")
    f_output_t=open("t_"+identificador_output+".txt",mode="w+", newline="\n")
    f_output_x=open("x_"+identificador_output+".txt",mode="w+", newline="\n")
    
    line_track=0
    
    f_list=f.readlines()
    f.close()

    #print(f_list)
    len_f_list=len(f_list)
    
    delta=0
    time=0
    message_end=0;
    first_flag=1;
    
    for line_track in range(len_f_list-1):
        
        if "====" in f_list[line_track]:
            
            if first_flag==1:
                first_flag=0;
            
            
                previous_time= datetime.strptime(list(f_list[line_track-1])[0:13], "%H:%M:%S.%f")
            
            while(message_end==0):
            
                line_track+=1
                if "====" not in f_list[line_track]:
                
                    line=f_list[line_track]
                    print("working line ",line)
                    #print(type(line))
                    #print("next line ", next_line)
            
                        
                    timestamp="".join(list(line)[0:13])
            
                    this_time= datetime.strptime(timestamp, "%H:%M:%S.%f")
                    
                    #this_time= datetime.strptime(timestamp, "%m-%d-%Y\t%H:%M\t%S.%f")
                    
                    print("this time ", this_time)
                    diff=this_time-previous_time
                    #print("diff ",diff)
                    
                    line="".join(list(line)[13:])
                    print("line: ",line)
                    line=list(line)
                    line.pop()
                    #print("in")
                    
    
                        
                    print(time)
                    f_output.write(str(timestamp+"\t"+str(time)+"\t"+info_i))
                    f_output_t.write(str(time))
                    f_output_x.write(info_i)
                    f_output.flush()
                    f_output_t.flush()
                    f_output_x.flush()
                    time+=delta
        
    f_output.close()
    f_output_t.close()
    f_output_x.close()

main()