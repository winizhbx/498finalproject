#!/usr/bin/python3
 
import sys, termios, tty, os, time
 
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def demo_getch():
    button_delay = 0.2
    while True:
        char = getch()

        if (char == "p"):
            print("Stop!")
            exit(0)

        if (char == "a"):
            print("Left pressed")
            time.sleep(button_delay)

        elif (char == "d"):
            print("Right pressed")
            time.sleep(button_delay)
 
        elif (char == "w"):
            print("Up pressed")
            time.sleep(button_delay)

        elif (char == "s"):
            print("Down pressed")
            time.sleep(button_delay)

        elif (char == "1"):
            print("Number 1 pressed")
            time.sleep(button_delay)

def display_manu(stage=0):

    if stage == 0:
    
        print "Dear GSI and professor, Welcome to localization playground."
        time.sleep(1)
        print "Here is some options for you:"
        time.sleep(1)

        print "1. Toy demo for Kalman Filter in simple 2D env."
        print "2. Integrated demo for Particle filter and Kalman filter in OpenRave env."
        print "please press 1 or 2 on your keyboard."
        
        while True:
            char = getch()
            if char == '1':
                print("Catch option 1.")
                print("")
                break

            elif char == '2':
                print("Catch option 2.")
                print("")
                break

        return char

    elif stage == 1:
        print "Welcome back, I guess you can't wait to play some tricks on a real robot."
        time.sleep(1)
        print "Y: Begin OpenRave demo."
        print "N: Wrong guess. Quit the whole demo program."

        while True:
            char = getch()
            if char == 'y':
                print("Catch option y.")
                time.sleep(0.5)
                print("After this demo ends, please FIRST close the OpenRave window and THEN press enter to continue!!!")
                time.sleep(0.5)
                break

            elif char == 'n':
                print("Catch option n. Now quit.")
                break

        return char

    
    elif stage == 2:

        print "Now you choose OpenRave env for integrated test."
        print "Please choose an environment for your robot."
        time.sleep(2)

        print "a. Empty room."
        print "b. A large room with just a few doors and tables."
        print "c. A large symmetric room with 6 doors and a lot of tables"
        print "please press a, b or c on your keyboard."

        while True:
            char = getch()
            if char == 'a':
                print("Catch option a.")
                char = 0
                break

            elif char == 'b':
                print("Catch option b.")
                char = 1
                break

            elif char == 'c':
                print("Catch option c.")
                char = 2
                break

        return char

    elif stage == 3:

        print "Wait. What kind of sensor do you allow your robot to have?"
        time.sleep(2)
        print "a. Noisy GPS, which gives rough measurement on postion (x,y)."
        print "b. Noisy distance measurement sensor, which gives rough horizontal distance to underground beacon located on (0, 0, -10)"
        print "please press a or b on your keyboard."

        while True:
            char = getch()
            if char == 'a':
                print("Catch option a.")
                print("As for KF demo, you choose GPS:")
                print("Blue dot shows observed position;")
                print("Purple dot shows regular KF estimated position.")
                print("")
                time.sleep(3)
                char = 0
                break

            elif char == 'b':
                print("Catch option b.")
                print("As for KF demo, you choose distance measurement:")
                print("red line shows noisy horizontal distance between observed position and fixed beacon on (0,0,-10)")
                print("Purple dot shows extended KF estimated position.")
                time.sleep(3)
                print("")
                char = 1
                break

        print("And for particle filter demo, robot always using information about whether it hits the wall and noisy sonar measurements")
        print("Green lines shows the noisy sonar measurements.")
        time.sleep(3)
        print("After this demo ends, please FIRST close the OpenRave window and THEN press enter to continue!!!")
        time.sleep(0.5)

        return char

    elif stage == 4:

        print "Welcome back."
        print "Do you want to watch integrated demo again with other options? [Y/N]"
        time.sleep(0.5)
        print "Notice: Press N will quit the whole demo program."

        while True:
            char = getch()
            if char == 'y':
                print("Robot: Poor me.")
                time.sleep(2)
                break

            elif char == 'n':
                print("Robot: Aha! Thank you! I can relax a while.")
                time.sleep(2)
                print("Now quit the whole demo program.")
                char = 'q'
                break

        return char


