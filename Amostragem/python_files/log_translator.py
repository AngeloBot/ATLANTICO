import os	
from datetime import datetime

print(os.getcwd())

def main():
    '''
    Abre o arquivo log
    '''
    nome_arquivo = str(input("Nome do arquivo a ser traduzido: "))
    
    f=open(nome_arquivo+'.log')
    
    identificador_output=str(input("Nome do arquivo de output: "))
    
    f_output=open("log_"+identificador_output+".txt",mode="w+", newline="\n")
    f_output_t=open("t_"+identificador_output+".txt",mode="w+", newline="\n")
    f_output_x=open("x_"+identificador_output+".txt",mode="w+", newline="\n")
    
    line_track=0
        
    info=[]
    
    f_list=f.readlines()
    f.close()

    #print(f_list)
    len_f_list=len(f_list)
    
    delta=0
    time=0
    for line_track in range(len_f_list-1):
        
        line=f_list[line_track]
        next_line=f_list[line_track+1]
        print("working line ",line)
        #print(type(line))
        #print("next line ", next_line)
        
        if line_track >= 2:

                
            timestamp="".join(list(line)[0:23])
            next_timestamp="".join(list(next_line)[0:23])

            this_time= datetime.strptime(timestamp, "%m-%d-%Y %H:%M:%S.%f")
            next_time= datetime.strptime(next_timestamp, "%m-%d-%Y %H:%M:%S.%f")
            #this_time= datetime.strptime(timestamp, "%m-%d-%Y\t%H:%M\t%S.%f")
            
            #print("this time ", this_time)
            #print("next time ", next_time)
            diff=next_time-this_time
            #print("diff ",diff)
            
            line="".join(list(line)[34:])
            print("line: ",line)
            line=list(line)
            line.pop()
            #print("in")
           
            f_output_list=[]
            info_count=0
            
            for char in line:
                
                #print("info before ",info)
                if "0D0A" in "".join(info):
                    #print("im in")
                    #len_info=len(info)
                    #print(len_info)
                    #print(str(info))
                
                    if len(info)==15 or len(info)==13 or len(info)==11 or len(info)==9 or len(info)==7:
                        info.pop(0)
            
                    elif len(info) > 16:
                        #print("im DAT in")
                        #print("".join(info))
                        while len(info) > 16:
                            print(info)
                    
                            info.pop(0)
                        
                        f_output_list.append(info[:])
                        info=[]
                        info_count+=1
                        
               
                    elif len(info) == 16 or len(info) == 14 or len(info) == 12 or len(info)==10 or len(info)==8 or len(info) == 6:
                        #print("im SUPER in")
                        f_output_list.append(info[:])
                        info=[]
                        info_count+=1
                    
                    else:
                        info=[]
                    
                    #print("f_output_list ",f_output_list)

                info.append(char)
                #print("info after ", info)
            
            print("f_output_list ",f_output_list)
            info_translate=[]
            for info_i in f_output_list:
                try:
                    info_translate.append(bytes.fromhex("".join(info_i)).decode("ascii"))    
                except UnicodeDecodeError:
                    info_count-=1
                    print(str(timestamp+"\t"+"DATA UNREADABLE"))
                    
            try:
                delta=(diff.seconds+diff.microseconds*0.000001)/info_count     
            except ZeroDivisionError:
                print("Empty Line")
            print(info_translate)
            for info_i in info_translate:
                
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