import serial
import io
import win32gui
import win32api
import win32con

import sys
import time
from math import atan2,cos
def LeftClick():
    pos = get_curpos()
    handle = get_win_handle(pos)
    client_pos = win32gui.ScreenToClient(handle, pos)
    tmp = win32api.MAKELONG(client_pos[0], client_pos[1])
    win32gui.SendMessage(handle, win32con.WM_ACTIVATE, win32con.WA_ACTIVE, 0)
    win32api.SendMessage(handle, win32con.WM_LBUTTONDOWN, win32con.MK_LBUTTON, tmp)
    win32api.SendMessage(handle, win32con.WM_LBUTTONUP, win32con.MK_LBUTTON, tmp)

def RightClick():
    pos = get_curpos()
    handle = get_win_handle(pos)
    client_pos = win32gui.ScreenToClient(handle, pos)
    tmp = win32api.MAKELONG(client_pos[0], client_pos[1])
    win32gui.SendMessage(handle, win32con.WM_ACTIVATE, win32con.WA_ACTIVE, 0)
    win32api.SendMessage(handle, win32con.WM_RBUTTONDOWN, win32con.MK_RBUTTON, tmp)
    win32api.SendMessage(handle, win32con.WM_RBUTTONUP, win32con.MK_RBUTTON, tmp)

def scroll(i):
    x,y = get_curpos()
    win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, x, y, i, 0)

def MoveRelative(x,y):
    pos = get_curpos()
    win32api.SetCursorPos((pos[0]+x,pos[1]-y))

def MoveRelativeXAbsY(x,y):
    pos = get_curpos()
    win32api.SetCursorPos((pos[0]+x,y))

def MoveRelativeYAbsX(x,y):
    pos = get_curpos()
    win32api.SetCursorPos((x,pos[1]))

def get_curpos():
    return win32gui.GetCursorPos()

def get_win_handle(pos):
    return win32gui.WindowFromPoint(pos)

if __name__ == '__main__':
    left = False
    right = False
    Gyro_gain = 0.061
    DT = 0.001
    rate_gyr_x = 0
    rate_gyr_y = 0
    rate_gyr_z = 0
    gxAngle = 0
    gyAngle = 0
    gzAngle = 0
    CFaX = 0
    CFaY = 0
    CFaZ = 0
    ser = serial.Serial("COM6", 115200,timeout=1)
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    print "Config Complete"
    prevy = 0
    prevz = 0
    xOff=0
    yOff=0
    zOff=0
    accx = 0
    accy = 0
    accz = 0
    pCFaX = 0
    pCFaZ = 0
    pX = 0
    avgZ = 0
    avgX = 0
    avgY = 0
    buffer_string = ""
    
    while(1):
        if win32api.GetAsyncKeyState(ord('Q')) :
            break
        try:
            buffer_string = buffer_string + ser.read(ser.inWaiting())
            if '\n' in buffer_string:
            	# Every new line Decode input 
                lines = buffer_string.split('\n') 
                last_received = lines[-2]
                buffer_string = lines[-1]
                #print last_received
                """
                line = sio.readline()
                line.strip()
                print line
                """
                inputs = map(int,last_received.split(","))
                p1 = inputs[0]
                p2 = inputs[1]
                rate_gyr_x = inputs[2] * Gyro_gain
                rate_gyr_y = inputs[3] * Gyro_gain
                rate_gyr_z = inputs[4] * Gyro_gain
                avgZ = 0.3 * rate_gyr_z + avgZ * 0.7
                avgX = 0.3 * rate_gyr_x + avgX * 0.7
                avgY = 0.3 * rate_gyr_y + avgY * 0.7
                if(1<abs(avgY)<10):
                    scroll(int(avgY))
                # play with the values
                #if((1<abs(rate_gyr_x + 1.6)<10)and(1<abs(rate_gyr_z)<10)):
                
                if((1<abs(avgX + 1.6)<10)and(1<abs(avgZ)<10)):
                    MoveRelative(-int(avgZ*1.1),-int((avgX+1.6)*1.1))
                elif(1<abs(avgX + 1.1)<10):
                    MoveRelative(0, -int((avgX+1.5)*1.6))
                elif(1<abs(avgZ)<10):
                    MoveRelative(-int(avgZ*1.1), 0)
                # working code
                # if((0.5<abs(rate_gyr_x + 1.5)<10)and(0.5<abs(rate_gyr_z)<10)):
                #     MoveRelative(-int(avgZ*1.6),-int((avgX+1.5)*1.6))
                # elif(0.5<abs(avgX + 1.5)<10):
                #     MoveRelative(0, -int((avgX+1.5)*1.6))
                # elif(0.5<abs(avgZ)<10):
                #     MoveRelative(-int(avgZ*1.6), 0)

                #print rate_gyr_x
                #if((1.5<abs(rate_gyr_x + 1.5)<10)and(1.5<abs(rate_gyr_z)<10)):
                #    MoveRelative(-int(rate_gyr_z*1.6),-int((rate_gyr_x + 1.5)*1.6))
                #elif(3<abs(rate_gyr_x + 1.5)<10):
                #    MoveRelative(0,-int((rate_gyr_x + 1.5)*1.6))
                #elif(3<abs(rate_gyr_z)<10):
                #    MoveRelative(-int(rate_gyr_z*1.6), 0)

                if(p1 == 1):
                    left = True
                else:
                    if(left):
                        left = False
                        LeftClick()
                
                if(p2 == 1):
                    right = True

                else:
                   if(right):
                        right = False
                        RightClick()
                        #time.sleep(0.005)

#		except IndexError:
#            pass
        except ValueError:
        	  pass
        except:
            print "Unexpected error:", sys.exc_info()[0]
            raise
    ser.close()
	            # INPUT[5] Z
            # INPUT[7] X
            #print rate_gyr_z

            #print avgZ

            #if(1<abs(avgZ)<10):
            ##    MoveRelative(-int(avgZ*0.5),0)
