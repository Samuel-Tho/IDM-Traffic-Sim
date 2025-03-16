import numpy as np
import random


#########################################################################
#      1. UDP_POS_VEL
#########################################################################
def upd_pos_vel(Ncar, pos, vel, acc, headway, dv, posnew, velnew, params):
    'Updates the position, velocity, and acceleration of cars based on their '
    'current state and parameters such as desired speed, time gap, and acceleration '
    'limits.'

    des_speed_inv = params[0] #target speed inverse inverse
    acc_exp       = params[1] #acc exponent
    time_gap      = params[2] #bumper to bumper min time gap
    comf_decel    = params[3] #decelleration constant
    min_gap       = params[4] #minmum spacial gap
    acc_max       = params[5] #max acc val
    del_t         = params[6] #time step

                

    sstar = np.zeros(Ncar)
    for i in range(Ncar):
        sstar[i] = min_gap+np.max((0,(vel[i]*time_gap)+((vel[i]*dv[i])/(2*np.sqrt(acc_max*comf_decel)))))



    for i in range(Ncar):
        if headway[i]<0:
            print('ACTIVATION')
            acc[i] = 0
            vel[i] = 0
        else:
            acc[i] = acc_max*(1-((vel[i]*des_speed_inv)**acc_exp)-(sstar[i]/(headway[i]))**2)



    for i in range(Ncar):

        posnew[i] = pos[i] + (vel[i]*del_t) + (0.5*acc[i]*(del_t)**2)
        velnew[i] = vel[i] + (acc[i]*del_t)

        if velnew[i] < 0:
            posnew[i] = pos[i] + (vel[i]*(-vel[i]/acc[i])) + (0.5*acc[i]*(-vel[i]/acc[i])**2)
            velnew[i] = 0

    return posnew, velnew, acc




#########################################################################
#      2. DETECT_LOOP
#########################################################################
def detect_loop(Ncar,pos,vel,acc,posnew,velnew, det_point, time_pass):
    '''Detects when cars pass a specific detection point and records 
    their time and velocity at that point.'''

    detect_time = []
    detect_vel = []
    for i in range(Ncar):
        if pos[i] < det_point and posnew[i] >= det_point:
            detect_time.append(time_pass)
            detect_vel.append(vel[i])

    return detect_time, detect_vel




#########################################################################
#      3. UPDATE_CARS
#########################################################################
def update_cars(Ncar, pos, vel, posnew, velnew, headway, dv, params):
   '''Updates the positions, velocities, headways, and relative velocities 
   (dv) of cars after a time step to set up calculation for the next timestep.'''



   length  = params[7]
   L       = params[8]
   min_gap = params[4]

    
   for i in range(Ncar):
    vel[i] = velnew[i]
    if posnew[i]>L:
        pos[i] = (posnew[i]-L)
    else:
        pos[i] = posnew[i]

   for i in range(Ncar):
        DisGap = 0
        if i+1 != Ncar:
            if pos[i]>(pos[i+1]-length):
                DisGap = (pos[i+1]-length-pos[i]+L)
            elif pos[i]<(pos[i+1]-length):
                DisGap = (pos[i+1]-length-pos[i])
            dv[i] = vel[i]-vel[i+1]
        else:
            if pos[i]>(pos[0]-length):
                DisGap = (pos[0]-length-pos[i]+L)
            else:
                DisGap = (pos[0]-length-pos[i])
            dv[i] = vel[i]-vel[0]

        headway[i] = DisGap


# We have updated all information needed for the time step.        
   return pos, vel, headway, dv




#########################################################################
#     4. FLOW_GLOBAL
#########################################################################

def flow_global(Ncar,vel,road_l):
    '''Calculates the traffic density (vehicles per km) 
    and flow rate (vehicles per hour) on the road.'''

    dens = Ncar/road_l
    flowcount = 0
    for i in range(Ncar):
        flowcount += vel[i]*3.6
    
    flow = flowcount/(road_l/1000)
    # dens in vehicles/km ,,, flow in vehicles/hr
    return dens,flow





#########################################################################
#     5. FLIP
#########################################################################
def flip(x):
    return random.random()<x





#########################################################################
#     6. INSERT CAR
#########################################################################
def insert_car(pos,vel,acc,headway,dv,N,posnew,velnew,L,length,loc,start_speed,que,insert):
    '''Inserts a new car into the traffic flow at a specified location,
      updating all relevant arrays (position, velocity, acceleration, etc.).'''
    pos = np.insert(pos,insert+1,loc)
    vel = np.insert(vel,insert+1,start_speed)
    acc = np.insert(acc,insert+1,0)
    Disgap = 0.01
    dvin = 0
    if insert+1 != N:
        if loc>(pos[insert+2]-length):
            DisGap = (pos[insert+2]-length-loc+L)
        else: 
            DisGap = (pos[insert+2]-length-loc)
        dvin = start_speed-vel[insert+2]
    else:
        if loc>(pos[0]-length):
            DisGap = (pos[0]-length-loc+L)
        else:
            DisGap = (pos[0]-length-loc)
        dvin = start_speed-vel[0]
    headway = np.insert(headway, (insert+1), Disgap)
    dv = np.insert(dv, (insert+1), dvin)
    que.pop(0)
    N = N+1
    posnew = np.insert(posnew,0,0)
    velnew = np.insert(velnew,0,0)
    print('CAR ADDED')
    return pos,vel,acc,headway,dv,N,posnew,velnew





def insert_car_empty(pos,vel,acc,headway,dv,N,posnew,velnew,loc, start_speed, que, length, L):
    '''Inserts a car into traffic that does not move'''
    pos = np.append(pos,loc)
    vel = np.append(vel,start_speed)
    acc = np.append(acc,0)
    N += 1
    headway = np.append(headway,(L-length))
    dv = np.append(dv,(start_speed))
    posnew = np.append(posnew,0)
    velnew = np.append(velnew,0)
    que.pop(0)
    print('CAR ADDED')
    return pos,vel,acc,headway,dv,N,posnew,velnew




#########################################################################
#     7. REMOVE CARS
#########################################################################
def remove_car(pos, vel, acc, headway, dv, N, posnew, velnew, antique, remove):
    if type(remove)==str:
        remove = -1
    pos = np.delete(pos,remove+1)
    vel = np.delete(vel,remove+1)
    acc = np.delete(acc,remove+1)
    headway = np.delete(headway, remove+1)
    dv = np.delete(dv,remove+1)
    N = N-1
    posnew = np.delete(posnew,remove+1)
    velnew = np.delete(velnew,remove+1)
    antique.pop(0)
    print('CAR REMOVED')
    return pos,vel, acc, headway, dv, N, posnew, velnew




#############################################################################
#        8. TRAFFIC LIGHT
#############################################################################
'''Traffic light works by inserting an empty car (that cannot move) at the 
location of the traffic light, this prevents cars from moving past the traffic
light when the light is red.'''
class trafficlight:
    def __init__(self, loc, steps_on, length_on, braking_dist, stop_index): #int,list,int,int,list
        self.loc = loc
        self.steps_on = steps_on
        self.length_on = length_on
        self.braking_dist = braking_dist
        self.stop_index = stop_index
    
    def orange_light(self, pos, vel):
        passers = []
        for ii in range(len(pos)):
            if pos[ii]>=(self.loc-self.braking_dist) and pos[ii]<=self.loc:
                passers.append(ii)
        if len(passers)>0:
            self.stop_index.append((passers[0]-1))
        else:
            for j in range(self.loc, -1, -1):
                for k in range(len(pos)):
                    if j == np.ceil(pos[k]):
                        passers.append(k)
                        break
                else:
                    continue
                break
            if len(passers) == 0:
                for jj in range(L, self.loc-1, -1):
                    for kk in range(len(pos)):
                        if jj == np.floor(pos[kk]):
                            passers.append(kk)
                            break
                    else:
                        continue
                    break
            if len(passers)>0:
                self.stop_index.append((passers[0]-1))
            else:
                self.stop_index.append('Empty')
            
    def red_light(self, pos, vel, acc, headway, dv, N, posnew, velnew, redlight):
        if type(self.stop_index[0])==str:
            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car_empty(pos,vel,acc,headway,dv,N,posnew,velnew,self.loc, 0, [1], length, L)
            redlight = True
        else:
            insert = self.stop_index[0]
            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos,vel,acc,headway,dv,N,posnew,velnew,L,length,self.loc,0,[1],insert)
            redlight = True
        return pos,vel,acc,headway,dv,N,posnew,velnew,redlight
        
    def retain_state(self, posnew, velnew, acc):
        if redlight and type(self.stop_index[0])==str:
            posnew[0] = self.loc
            velnew[0] = 0
            acc[0] = 0
        elif redlight:
            posnew[self.stop_index[0]+1] = self.loc
            velnew[self.stop_index[0]+1] = 0
            acc[self.stop_index[0]+1] = 0
        return posnew,velnew,acc
    
    def green_light(self, pos, vel, acc, headway, dv, N, posnew, velnew, redlight, index):
        green = False
        for ll in self.steps_on:
            if ll+self.length_on == index:
                green = True
        if green:
            pos,vel, acc, headway, dv, N, posnew, velnew = remove_car(pos, vel, acc, headway, dv, N, posnew, velnew, self.stop_index, self.stop_index[0])
            redlight = False
        return pos,vel, acc, headway, dv, N, posnew, velnew, redlight
        



#############################################################################
#        9. Junction
#############################################################################
'''By default, allows cars to enter road road when there is space. If there is 
a traffic light at the same location, the junction will only allow cars to enter 
after a red light is passed to the traffic on the main road. The amount of cars 
trying to join the main road from the junction is controlled by prob_on. Cars can 
exit the road with probability prob_off when they reach the junctions location.'''


class junction:
    def __init__(self, loc, prob_on, prob_off, que, antique, leaving_dist, start_speed):
        self.loc = loc
        self.prob_on = prob_on
        self.prob_off = prob_off
        self.que = que
        self.antique = antique
        self.leaving_dist = leaving_dist
        self.start_speed = start_speed
        
    def car_on(self, pos, vel, acc, dv, headway, L, length, N, posnew, velnew):
        if flip(self.prob_on):
            self.que.append(1)
        if len(self.que)>0:
            upcoming = []
            for j in range(self.loc, -1, -1):
                for k in range(len(pos)):
                    if j == np.ceil(pos[k]):
                        upcoming .append(k)
                        break
                else:
                    continue
                break
            if len(upcoming) == 0:
                for jj in range(L, self.loc-1, -1):
                    for kk in range(len(pos)):
                        if jj == np.floor(pos[kk]):
                            upcoming.append(kk)
                            break
                    else:
                        continue
                    break
            if len(upcoming)==0:
                pos,vel,acc,headway,dv,N,posnew,velnew = insert_car_empty(pos,vel,acc,headway,dv,N,posnew,velnew,self.loc, self.start_speed, self.que,length,L)
            else:
                insert = upcoming[0]
                if pos[insert] < self.loc:
                    if insert+1 != N:
                        if self.loc-pos[insert]>self.leaving_dist and pos[insert+1]>(5*length)+self.loc:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)
                    else:
                        if self.loc-pos[insert]>self.leaving_dist and pos[0]>(5*length)+self.loc:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)
                else:
                    if insert+1 != N:
                        if self.loc + (L-pos[insert]) > self.leaving_dist and pos[insert+1]>(5*length)+self.loc:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)
                    else:
                        if self.loc + (L-pos[insert]) > self.leaving_dist and pos[0]>(5*length)+self.loc:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)

        return pos,vel,acc,dv,headway,N,posnew,velnew
    
    def car_on_lights(self, pos, vel, acc, headway, dv, N, posnew, velnew, L, length, lightpoint, braking_distance):
        if flip(self.prob_on):
            self.que.append(1)
        if len(self.que)>0:
            if redlight and lightpoint+1 == self.loc:
                tlfinder = []
                for p in range(len(pos)):
                    if pos[p] == self.loc-1:
                        tlfinder.append(p)
                insert = tlfinder[0]
                if len(pos)>1:
                    if insert+1!=N:
                        if pos[insert+1]>self.loc+3 or pos[insert+1]<self.loc-braking_distance:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)
                    else:
                        if pos[0]>self.loc+3 or pos[0]<self.loc-braking_distance:
                            pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert)
    
                else:
                    pos,vel,acc,headway,dv,N,posnew,velnew = insert_car(pos, vel, acc, headway, dv, N, posnew, velnew, L, length, self.loc, self.start_speed, self.que, insert) 
        return pos,vel,acc,headway,dv,N,posnew,velnew

    def car_off(self, pos, vel, acc, dv, headway, L, length, N, posnew, velnew):
        if flip(self.prob_off):
            self.antique.append(1)
        if len(self.antique)>0:
            upcoming = []
            for j in range(self.loc, -1, -1):
                for k in range(len(pos)):
                    if j == np.ceil(pos[k]):
                        upcoming .append(k)
                        break
                else:
                    continue
                break
            if len(upcoming) == 0:
                for jj in range(L, self.loc-1, -1):
                    for kk in range(len(pos)):
                        if jj == np.floor(pos[kk]):
                            upcoming.append(kk)
                            break
                    else:
                        continue
                    break
            remove = upcoming[0]
            if pos[remove]<=self.loc and posnew[remove]>=self.loc:
                pos,vel, acc, headway, dv, N, posnew, velnew = remove_car(pos, vel, acc, headway, dv, N, posnew, velnew, self.antique, remove-1)
        
        return pos,vel,acc,dv,headway,N,posnew,velnew

