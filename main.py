import os
import time
import EPICS_Initial as E
import gclib
import numpy as np
import matplotlib.pyplot as plt
from epics import caget


def Declare_Variable():
    global Device
    Device = "FE-DI-XBPM-051"

    #Declare Variable for Motor Control
    global g
    g = gclib.py()

    global Galil_IP
    Galil_IP = "172.18.171.5"

    global SP,PR
    SP = 20000
    PR = 12500

    global DelayTime
    DelayTime=0.8

    global Hori_flag, Vert_flag
    Hori_flag = 1 
    Vert_flag = 0 #down

    #Declare Array Size
    global ArraySize_Y,ArraySize_X
    ArraySize_X = 40
    ArraySize_Y = 40

    #Declare Scan Status Variable and Array
    global count_X, count_Y
    global Signal_Status
    count_X = 0
    count_Y = 0
    Signal_Status = np.zeros((ArraySize_Y,ArraySize_X))

    #Declare EPICS Data Array
    global Signal_A, Signal_B, Signal_C, Signal_D
    global Signal_X, Signal_Y
    Signal_A = np.zeros((ArraySize_Y,ArraySize_X))
    Signal_B = np.zeros((ArraySize_Y,ArraySize_X))
    Signal_C = np.zeros((ArraySize_Y,ArraySize_X))
    Signal_D = np.zeros((ArraySize_Y,ArraySize_X))
    Signal_X = np.zeros((0))
    Signal_Y = np.zeros((0))

#################### GALIL #######################
class Galil_function:
    def Link_Initial(Galil_IP):
        print("gclib version:", g.GVersion()) #print gclib version
        g.GOpen('%s --direct' %Galil_IP) #Connect to Galil Device
        print("Galil Info:")
        print(g.GInfo()) #print device and connect info
        print()

    def Close():
        g.GClose()
        
    def Move_Status(Axis):
        g.GCommand("SC%c" %Axis)

def Hori_Moto_Dir(Hori,SP,PR):
    #Horizotal move function
    #Hori_flag == 1 => Right
    #Hori_flag != 1 => Left
    if Hori == 1: #Right move
        print:("Right")
        g.GCommand('SH')
        g.GCommand('SPA=%d' %SP)
        g.GCommand('PRA=%d' %PR)
        BG_Respone = g.GCommand('BGA')
        return BG_Respone

    else: #Left move
        print:("Right")
        g.GCommand('SH')
        g.GCommand('SPA=%d' %SP)
        g.GCommand('PRA=-%d' %PR)
        BG_Respone = g.GCommand('BGA')
        return BG_Respone

def Vert_Moto_Dir(Vert,SP,PR):
    #Vertical move function
    #Vert_flag == 1 => Up
    #Vert_flag != 1 => Down
    if Vert == 1: #Up Move
        print:("Up")
        g.GCommand('SH')
        g.GCommand('SPB=%d' %SP)
        g.GCommand('PRB=%d' %PR)
        BG_Respone = g.GCommand('BGB')
        return BG_Respone

    else: #Down Move
        print:("Down")
        g.GCommand('SH')
        g.GCommand('SPB=%d' %SP)
        g.GCommand('PRB=-%d' %PR)
        BG_Respone = g.GCommand('BGB')
        return BG_Respone
def first_line_scan():
    global Hori_flag,count_X,count_Y
    print("Poositon is: %d,%d" %(count_X+1,count_Y+1))
    Signal_Saving_Status()
    EPICS.Current_log()
    print("EPICS Data Save to Array is done")
    print()
    time.sleep(DelayTime)

    for i in range(ArraySize_Y-1):
        print("in First loop")
        print("Right: %d" %(i+1))
        count_X = (count_X +1)
        print("Position is: %d,%d" %(count_X+1,count_Y+1))
        Hori_Moto_Dir(Hori_flag,SP,PR)
        Signal_Saving_Status()
        EPICS.Current_log()
        print("EPICS Data Save to Array is done")
        print()
        time.sleep(DelayTime)
        print()
    Hori_flag = not Hori_flag

################ Intensity Graphy ################
def Show_intensity():
    plt.imshow(Signal_Status, cmap='bone', interpolation='spline36')
    plt.colorbar()
    plt.show()

############# Scan Status and counter ############
def Signal_Saving_Status():
    global count_Y,count_X,Signal_Status
    Signal_Status[count_Y,count_X]=1
    print('\x1b[5;30;42m' + "Signal writing status:" + '\x1b[0m')
    print(Signal_Status)
    print()
    print("===============================")
    print()

def Position_Counter_X(flag,for_loop_conter):
    global count_X, count_Y
    if flag == 1:
        print("Right: %d" %(for_loop_conter+1))
        count_X = (count_X + 1)
        print("Poositon is: %d,%d" %(count_X+1,count_Y+1))
    else:
        print("Left: %d" %(for_loop_conter+1))
        count_X = (count_X - 1)
        print("Poositon is: %d,%d" %(count_X+1,count_Y+1))
    
def Position_Counter_Y():
    global count_X,count_Y
    count_Y = (count_Y + 1)
    print("Poositon is: %d,%d" %(count_X+1,count_Y+1))

################### EPICS ########################
class EPICS:
    def Current_log():
        global Signal_A, Signal_B, Signal_C, Signal_D, Device
        currentA = caget('%s:signals:sa.A' %Device)
        currentB = caget('%s:signals:sa.B' %Device)
        currentC = caget('%s:signals:sa.C' %Device)
        currentD = caget('%s:signals:sa.D' %Device)
        print("Current Value:")
        print("A => %s, B => %s, C => %s, D => %s" %(currentA,currentB,currentC,currentD))
        Signal_A[count_Y,count_X]= currentA    
        Signal_B[count_Y,count_X]= currentB
        Signal_C[count_Y,count_X]= currentC    
        Signal_D[count_Y,count_X]= currentD
        print('\x1b[6;31;47m' + "Current Value Writing Done!" + '\x1b[0m')
        print()

    def Position_log():
        Position_X =PV('FE-DI-XBPM-091:SA:SA_X_MONITOR')
        Position_Y =PV('FE-DI-XBPM-091:SA:SA_Y_MONITOR')
        Signal_X = np.append(Signal_X,Position_X)
        Signal_Y = np.append(Signal_X,Porstion_Y)

    def Data_saving(Signal_Array,File_Name):
        np.savetxt('%s.txt' %File_Name, Signal_Array, fmt='%d' ,delimiter=',')

def Scan_Integration_Y():
        
####################Main####################
def main():
    Declare_Variable() #Declare Variable
    global count_X,count_Y
    global Hori_flag
    E.Initial()
    Galil_function.Link_Initial(Galil_IP)

    init_flag=0
    
    while init_flag==0:
        print('\x1b[5;30;41m' + '======Start======' + '\x1b[0m')
        init_flag=1
        first_line_scan()

        #Before Second line scan
        for i in range(ArraySize_Y-1):
            Position_Counter_Y()
            Vert_Moto_Dir(Vert_flag,SP,PR*2)
            Signal_Saving_Status()
            EPICS.Current_log()
            print("EPICS Data Save to Array is done")
            print()
            time.sleep(DelayTime)

            if Hori_flag == 1:
                for i in range(ArraySize_X-1):
                    Position_Counter_X(Hori_flag,i)
                    Hori_Moto_Dir(Hori_flag,SP,PR)
                    Signal_Saving_Status()
                    EPICS.Current_log()
                    print("EPICS Data Save to Array is done")
                    print()
                    time.sleep(DelayTime)
                Hori_flag= not Hori_flag #Change Direction 

            else:
                for i in range(ArraySize_X-1):
                    Position_Counter_X(Hori_flag,i)
                    Hori_Moto_Dir(Hori_flag,SP,PR)
                    Signal_Saving_Status()
                    EPICS.Current_log()
                    print("EPICS Data Save to Array is done")
                    print()
                    time.sleep(DelayTime)
                Hori_flag= not Hori_flag #Change Direction


    print(Signal_D)
    print()
    
    #Data saving
    EPICS.Data_saving(Signal_A,"Signal_A")
    print("Saving Signal_A array Done")
    EPICS.Data_saving(Signal_B,"Signal_B")
    print("Saving Signal_B array Done")
    EPICS.Data_saving(Signal_C,"Signal_C")
    print("Saving Signal_C array Done")
    EPICS.Data_saving(Signal_D,"Signal_D")
    print("Saving Signal_D array Done")
    print()

    #Data plot
    plt.imshow(Signal_A, cmap='jet', interpolation='spline36')
    plt.colorbar()
    plt.title("%s Signal A" %Device)
    plt.show()
    
    plt.imshow(Signal_B, cmap='jet', interpolation='spline36')
    plt.colorbar()
    plt.title("%s Signal B" %Device)
    plt.show()

    plt.imshow(Signal_C, cmap='jet', interpolation='spline36')
    plt.colorbar()
    plt.title("%s Signal C" %Device)
    plt.show()

    plt.imshow(Signal_D, cmap='jet', interpolation='spline36')
    plt.colorbar()
    plt.title("%s Signal D" %Device)
    plt.show()
    
    print("Done")   

if __name__== "__main__":
    main()