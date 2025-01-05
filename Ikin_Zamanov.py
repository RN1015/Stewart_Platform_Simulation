import pypot.dynamixel
import numpy as np
import itertools
import time
import math

def main():

    # Initialization of the Dynamixel Motors
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    port = ports[0]
    print("connecting on the first available port:  ports[0]")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    print("dxl_io:"), dxl_io
    ids = dxl_io.scan([1, 2, 3, 5, 6])
    print("Found ids:"), ids
    print("Present motor position"), dxl_io.get_present_position(ids)
    #setting speed and position to be inital  values
    #speed = dict(zip(ids, itertools.repeat(50)))
    #dxl_io.set_moving_speed(speed)
    #dxl_io.set_goal_position({2: (pos[0][0]), 6: (pos[1][0]), 5: (pos[2][0]), 3: (pos[3][0]), 1: (pos[4][0])})
    print("Setting up")
    #time.sleep(2)
    zero_corr = [-28.35+24.75, 16.22-24.75, -39.6+24.75, 26.77-24.75, -33.1+24.75, 14.64-24.75]
    firstflag = 0
    #Define 6RUS Zamanov design parameter
    base_length = 0.142
    top_length = 0.067
    d_b = 0.23
    d_t = 0.25
    l1 = 0.075
    l2 = 0.23

    inter_value = ((d_b/2 + base_length/4)*2)/math.sqrt(3);
    r_b = math.sqrt(inter_value*inter_value + (base_length/2)*(base_length/2));
    rem_angle = 2*math.asin(base_length/(2*r_b));
    half_angle = (2*math.pi/3) - rem_angle;

    inter_value2 = ((d_t/2 + top_length/4)*2)/math.sqrt(3);
    r_p = math.sqrt(inter_value2*inter_value2 + (top_length/2)*(top_length/2));
    rem_angle_top = 2*math.asin(top_length/(2*r_p));
    theta_p = (2*math.pi/3) - rem_angle_top;
    
    #Define Base Platform
    #Vertex 1 of base platform
    b1 = np.array([[float(0)],[-float(base_length)/(2*math.tan(rem_angle/2))],[-float(base_length)/2]])
    #Vertex 2 of base platform
    rotangle = rem_angle
    Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
    b2 =  Rx_clock.dot(b1)
    #Defining the rest vertices of base platform
    rotangle = rem_angle+half_angle
    Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
    b3 = Rx_clock.dot(b1)
    b4 = Rx_clock.dot(b2)
    b5 = Rx_clock.dot(b3)
    b6 = Rx_clock.dot(b4)
    
    #b1 to b6 are column vectors
    Base_matrix = np.concatenate((b1,b2,b3,b4,b5,b6), axis=1)
    #print Base_matrix
    #t0 is the epoch of the time
    #t1 is the next instant.So, the program runs till t-t0 becomes greater than time specified
    t0 = time.time()
    tr = 0
    ex = 0.20
    increment = 0.000025
    sign = 1
    print("t0:", t0)
    while tr - t0 <= 30:
        tr = time.time()
        #Define Trajectory
        dt = int(tr-t0)
        print("TIme : ",dt)
        #sign = -2*((dt/10)%2)+1
        print("Sign : ", sign)
        ex = ex + increment
        print(ex)
        ey = float(0.02)*math.cos(tr-t0)
        ez = float(0.02)*math.sin(tr-t0)
        roll = 0 #float(0.05)*math.cos(tr-t0)
        pitch =0 #float(0.05)*math.sin(tr-t0)
        yaw = 0
        translate = np.array([[ex],[ey],[ez]])
        R_roll = np.array([[math.cos(roll), -math.sin(roll), 0],[math.sin(roll), math.cos(roll), 0],[0, 0, 1]])
        R_yaw = np.array([[1, 0, 0],[0, math.cos(yaw),-math.sin(yaw)],[0, math.sin(yaw), math.cos(yaw)]])
        R_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)],[0, 1, 0],[-math.sin(pitch), 0, math.cos(pitch)]])

        #Defining the top platform 
        t1 = np.array([[0],[-r_p*math.cos(((2*math.pi/3) - theta_p)/2)],[-r_p*math.sin(((2*math.pi/3) - theta_p)/2)]])
        #Vertex 2 of top platform
        rotangle = (2*math.pi/3) - theta_p
        Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
        t2 =  Rx_clock.dot(t1)
        #Defining the rest vertices of top platform
        rotangle = 2*math.pi/3
        Rx_clock = np.array([[1, 0, 0],[0, math.cos(rotangle), math.sin(rotangle)],[0, -math.sin(rotangle), math.cos(rotangle)]])
        t3 = Rx_clock.dot(t1)
        t4 = Rx_clock.dot(t2)
        t5 = Rx_clock.dot(t3)
        t6 = Rx_clock.dot(t4)
        t1f = R_roll.dot(R_pitch).dot(R_yaw).dot(t1) + translate 
        t2f = R_roll.dot(R_pitch).dot(R_yaw).dot(t2) + translate
        t3f = R_roll.dot(R_pitch).dot(R_yaw).dot(t3) + translate
        t4f = R_roll.dot(R_pitch).dot(R_yaw).dot(t4) + translate
        t5f = R_roll.dot(R_pitch).dot(R_yaw).dot(t5) + translate
        t6f = R_roll.dot(R_pitch).dot(R_yaw).dot(t6) + translate

        Top_matrix = np.concatenate((t1f,t2f,t3f,t4f,t5f,t6f), axis=1)
        rangle_vectclock = np.array([5*math.pi/6, -5*math.pi/6, math.pi/6, math.pi/2, -math.pi/2, -math.pi/6])
        #print("Rangle_val: ",rangle_vectclock)
        th = []
        for i in range(0,6):
            Rx_clock = np.array([[1, 0, 0],[0, math.cos(rangle_vectclock[i]), math.sin(rangle_vectclock[i])],[0, -math.sin(rangle_vectclock[i]), math.cos(rangle_vectclock[i])]])
            T = Rx_clock.dot(Top_matrix[:,i]-Base_matrix[:,i])
            th3 = -math.asin(T[2]/l2)
            pl2 = l2*math.cos(th3)
            th2 = math.acos((T[0]*T[0] + T[1]*T[1] - l1*l1 - pl2*pl2)/(2*l1*pl2))
            r = math.sqrt(l1*l1 + pl2*pl2 + 2*l1*pl2*math.cos(th2))
            phi = math.atan((l1 + pl2*math.cos(th2))/(pl2*math.sin(th2)))
            th1 = math.asin(T[1]/r)+phi
            th.append((th1*180/math.pi))
        print(th)
        speed = 15
        #print(speed)
        dxl_io.set_moving_speed({1: speed, 2: speed, 3: speed, 4: speed, 5: speed, 6: speed})
        dxl_io.set_goal_position({1: (th[0]+zero_corr[0]), 2: (-th[1]+zero_corr[1]), 3: (th[2]+zero_corr[2]), 4: (-th[3]+zero_corr[3]), 5: (th[4]+zero_corr[4]), 6: (-th[5]+zero_corr[5])})
        if(firstflag == 0):
            time.sleep(7)
            firstflag = 1
        time.sleep(0.01)


if __name__ == "__main__":
    main()
