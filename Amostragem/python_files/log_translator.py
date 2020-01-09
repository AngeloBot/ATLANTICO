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
    
    line_track=0
        
    info=[]
    

    for line in f:
        print("working line ",line)
        print(type(line))
        
        
        if line_track >= 2:
            
            timestamp=line[0:23]
            '''
            timestamp=list(timestamp)
            timestamp[10]='\t'
            timestamp[16]='\t'
            timestamp="".join(timestamp)
            '''
            this_time= datetime.strptime(timestamp, "%m-%d-%Y %H:%M:%S.%f")
            #this_time= datetime.strptime(timestamp, "%m-%d-%Y\t%H:%M\t%S.%f")
            
            if line_track == 2:
                delta="0"
                last_time=this_time
                
            else:
                delta=float(delta)
                diff=this_time-last_time
                delta+=diff.seconds+diff.microseconds*0.000006
                delta=str(delta)
                last_time=this_time
                
            #this_time= datetime.strptime(timestamp, "%m-%d-%Y\t%H:%M\t%S.%f")
            
            
            line=line[33:]
            print("line: ",line)
            line=list(line)
            line.pop()
            
            print("in")
            
            for char in line:
                
                print("info before ",info)
                if "0D0A" in "".join(info):
                    print("im in")
                    #len_info=len(info)
                    #print(len_info)
                    #print(str(info))
                    
                    if len(info)==15 or len(info)==13:
                        info.pop(0)
                    
                    elif len(info) > 16:
                        print("im DAT in")
                        print("".join(info))
                        while len(info) > 16:
                            print(info)
                            
                            info.pop(0)
                        try:
                            f_output.write(str(timestamp+"\t"+delta+"\t"+bytes.fromhex("".join(info)).decode("ascii")))
                            f_output.flush()
                            info=[]
                        except UnicodeDecodeError:
                            info=[]
                            print(str(timestamp+"\t"+"DATA UNREADABLE"))
                       
                    elif len(info) == 16 or len(info) == 14 or len(info) == 12:
                        print("im SUPER in")
                        try:
                            f_output.write(str(timestamp+"\t"+delta+"\t"+bytes.fromhex("".join(info)).decode("ascii")))
                            f_output.flush()
                            info=[]
                        except UnicodeDecodeError:
                            info=[]
                            print(str(timestamp+"\t"+"DATA UNREADABLE"))
                    
                    else:
                        info=[]
    
                info.append(char)

                    
        line_track+=1
        
    f.close()
    f_output.close()

main()