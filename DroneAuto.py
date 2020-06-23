# A program simulating manual and automatic operation of a drone
# in manual mode use:
#   UP/DOWN keys fopr forward/backward
#   RIGHT/LEFT to rotate CW/CCW
#   Close window to quit
#   Panning sideways in not supported
# Configuration:
#   Some parameters are configurable - see params.txt configuration file
#   configuration file must exist in same directory as program
#   configuration file format must comply to Microsoft INI file format
# Maze image must be black and white. black = wall, white = coridor
# All calculations are based on "real world" cgs units which are rounded to nearest whole cm
# and only those related directly to the maze bitmap and drawing are pixel based
import pygame
import math
import configparser
import random

#define some colors
black = (0,0,0)
white = (255,255,255)
gray = (200, 200, 200)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)


def drone(screen, long,lat, heading, drone_size, pix):
    '''
    draw the drone with a small heading line
    :param screen: the display object
    :param long, lat, heading: position coordinates and heading of the drone
    :param drone_size: drone size, actually its safe perimeter, assumed round
    :param pix: pixel size in cm
    :return: none
    '''
    #drone body (circle)
    x_start = int(long/pix)
    y_start = int(lat/pix)
    radius = int(drone_size/(2*pix))
    pygame.draw.circle(screen, red, (x_start,y_start), radius, 0)
    # heading pointer - extends 50% radius out of body
    # this line can be eliminated if drawing range lines and there is a forward range sensor
    x_end = x_start + int(2*radius*math.cos(heading))
    y_end = y_start - int(2*radius*math.sin(heading))
    pygame.draw.line(screen, blue, (x_start,y_start), (x_end,y_end), 2)

# check for colission with a wall
# rectangular check size is 1 pixel larger than the drone safe perimeter
def check_collision(background,long,lat, drone_size, pix):
    '''
    check for a collision with a wall
    checking is done on a rectangle 1 pixel larger than the drone safe perimeter
    :param background: the maze picture
    :param long, lat: position coordinates of the drone
    :param drone_size: drone size, assumed round
    :param pix: pixel size in cm
    :return: collision - true/false
    '''
    collide = False
    x = int(long/pix)
    y = int(lat/pix)
    radius = int(drone_size/(2*pix))
    check_rect = pygame.Rect(x,y,radius,radius)
    if background.get_at(check_rect.topleft)==(0,0,0,255) \
    or background.get_at(check_rect.topright)==(0,0,0,255) \
    or background.get_at(check_rect.bottomleft)==(0,0,0,255) \
    or background.get_at(check_rect.bottomleft)==(0,0,0,255):
        collide = True
    return collide


def distance(background,long,lat, direction, drone_size, pix, range_noise):
    '''
    check the distance of a wall from the drone safe perimeter
    measurment is randomly moisy according to the noise parameter
    :param background: the maze picture
    :param long, lat, direction: position coordinates and heading of the range sensor
    :param drone_size: drone size, actually its safe perimeter, assumed round
    :param pix: pixel size in cm
    :param range_noise: measurment maximum inaccuracy 
    :return: range on cm
    '''
    x = int(long/pix)
    y = int(lat/pix)
    radius = int(drone_size/(2*pix))
    noise = random.randint(-range_noise, range_noise)
    for i in range (radius, 9999):
        test_pixel = int(x + i * math.cos(direction)), int(y - i * math.sin(direction))
        try:
            if background.get_at(test_pixel) == (0,0,0,255):
                return i*pix + noise
        except IndexError:
            return (i-1)*pix + noise

def draw_ranges(screen, long, lat, ranges, pix):
    '''
    draw the range lines from the drone
    :param screen: the display object
    :param long, lat: position coordinates and heading of the range senan array of the emasured ranges
    :param pix: pixel size in cm
    :return: none
    '''
    x_start = int(long/pix)
    y_start = int(lat/pix)
    for i in ranges:
        x_end=int(i[0]/pix)
        y_end=int(i[1]/pix)
        pygame.draw.line(screen,gray,(x_start,y_start),(x_end,y_end))    


def turn(range_f, range_r, range_l, side_sense_dir, min_range):
    '''
    calculate a turning instruction
    0 - straight, do not rotate
    1 - turn left
    -1 turn right
    The algorithm basically wishes to move towards the most open direction by averaging the sensors readings 
    If the decision is not a clear cut or when facing a heads on collission with a wal than some randomality is introduced to the turn
    The randomality ensures that the drone will not repeats tracks even if it passes again in a previous location at same orientation 
    IMPROVMENT - change so that it will be able to handle more than 3 sensors at various angles
    :param range_f, range_r, range_ln: the ranges of the snesorsthe directionof the side sensors relative to drone front heading
    :param min_range: minimum range allowed from the walls
    :return: turning instruction
    '''
    instruction = 0
    alpha = side_sense_dir / 360 * math.pi * 2  #side sensors direction
    #break the range vectors to to cartesian coordinates
    f_x = range_f
    f_y = 0
    r_x = range_r * math.cos(alpha)
    r_y = -range_r * math.sin(alpha)
    l_x = range_l * math.cos(alpha)
    l_y = range_l * math.sin(alpha)
    #calculate the average
    x_aver = (f_x + r_x + l_x) / 3
    y_aver = (f_y + r_y + l_y) / 3
    head_aver = math.atan(y_aver/x_aver)
    #The forward arc between the right and left sensor is divided to 3 equal sections
    #to decide where to turn based on the comupetd averag heading
    if head_aver > 2*alpha/3:   #certainly left
        instruction = 1
    elif head_aver > alpha/3:   #slightly left - randomly turn left or continue straight 
        if random.randint(1,100) <50:
            instruction = 1
        else:
            instruction = 0
    elif head_aver < -2*alpha/3:    #crtainly right
        instruction = -1
    elif head_aver < -alpha/3:      #slightly right - randomly turn right or continue straight
        if random.randint(1,100) <50:
            instruction = -1
        else:
            instruction = 0
    else:                           #certainly straight
        instruction = 0
    #getting too close to walls heads on - check also the sides and turn accordingly
    #if not clear cut which side more dangerous then turn randomly
    if range_f < min_range:
        if  range_r > range_l:
            instruction = -1
        elif range_l > range_r:
            instruction = 1
        else:
            if random.randint(1,100) <50:
                instruction = 1
            else:
                instruction = -1
    #getting too close to walls on the side 
    if range_r < min_range:
            instruction = 1
    if range_l < min_range:
            instruction = -1
    #print(x_aver, y_aver, alpha, head_aver, instruction)     
    return instruction         


def desired_speed(range_f, max_speed, acc):
    '''
    calculate the desired speed
    Optimally work at maximum speed,
    however reduce speed when approaching an obstacle based on decceleration capabilities
    use the formula 2*acc*distance = final_velocity^2 - initial_velocity^2
    where in this case the final velocity should be zero
    No safety margin is taken. This will be handled inthe turning algorithm
    :param range_f: range forward
    :param max_speed: maximum speed
    :param acc: acceleration
    :return: desired speed
    '''
    speed = math.sqrt(2*acc*range_f)
    if speed>max_speed:
        speed = max_speed
    return speed

    
def actual_speed(des_speed, cur_speed, acc, period):
    '''
    calculate actual speed for a desired speed based on current speed and integration of acceleration
    :param des_speed: desired speed
    :param cur_speed: current speed
    :param acc: acceleratoin 
    :param period: integration period
    :return: actual speed
    '''
    if des_speed > cur_speed:
        speed = cur_speed + acc*period
    elif des_speed < cur_speed:
        speed = cur_speed - acc*period
    else:
        speed = cur_speed
    return speed

# draw points to the screen form a provided pixel coordinates list
def draw_points(screen, coord_list, color, pix):
    '''
    draw points to the screen form a provided coordinates list
    :param screen: the display object
    :param coord_list: coordinates list (in cm)
    :param color: color of the pixels to be drawn
    :param pix: pixel size (in cm)
    :return: none
    '''    
    for i in coord_list:
        pygame.draw.circle(screen, color, (int(i[0]/pix), int(i[1]/pix)), 1, 0)
        
 
def text_objects(text, font):
    '''
    text support for message_display
    :param text: text to be displayed
    :param font: fony
    :return: surface for text display
    '''
    textSurface = font.render(text, True, red)
    return textSurface, textSurface.get_rect()


def message_display(screen, width, height, text):
    '''
    display a message to the screen
    :param screen: the display object
    :param width, height: width and height in pixels
    :param text: text to be displayed
    :return: none
    '''
    largeText = pygame.font.Font('freesansbold.ttf',50)
    TextSurf, TextRect = text_objects(text, largeText)
    TextRect.center = ((int(width/2)),(int(height/2)))
    screen.blit(TextSurf, TextRect)
    pygame.display.update()


def main():
    '''
    main function
    :param: none
    :return: none
    '''
    #read operational parameters from a configuration file
    config = configparser.ConfigParser()
    config.read('params.txt')
    auto = bool(int(config['ALL']['auto']))
    pix = float(config['ALL']['pix'])                                       #pixels size in cm
    long = int(config['ALL']['lat'])                                        #drone longitude coordinate (cm)
    lat = int(config['ALL']['long'])                                        #drone latitude coordinate (cm)
    heading = int(config['ALL']['heading'])/360 * 2*math.pi                 #drone heading (radians - East/Right = 0)
    drone_size = float(config['ALL']['drone_size'])                         #drone safe boundary size (cm)
    maze_image = config['ALL']['maze_image']                                #maze image
    time_limit = int(config['ALL']['time_limit'])*60*1000                   #time limit for simulation (miliseconds)
    d_rotate = float(config['ALL']['d_rotate'])/360 * 2*math.pi             #rate of rotation per key press or control loop (radians)
    d_linear = float(config['ALL']['d_linear'])                             #rate of linear advance per key press or per control loop (cm)
    side_sense_dir = int(config['ALL']['side_sense_dir'])                   #view direction of lateral range sensors from center line (degrees)
    max_range = int(config['ALL']['max_range'])* 100                        #maximum range measurement (cm)
    max_speed = int(config['AUTO']['max_speed'])* 100                       #maximum speed (cm/s)
    acc = int(config['AUTO']['acc'])* 100                                   #acceleration (cm/sec^2)
    samp_rate = int(config['AUTO']['samp_rate'])                            #sampling rate of the sensors (samples/sec)
    sens_num = int(config['AUTO']['sens_num'])                              #number of saensors
    loop_delay = float(sens_num/samp_rate)                                  #contorl loop period - allow all sensors to complete sensing (seconds)
    range_noise = int(config['AUTO']['range_noise'])                        #range sensors inaccuracy
    no_range = int(config['AUTO']['no_range'])                              #range sensors rate of no reading
    margin = int(config['AUTO']['margin'])                                  #safety margin in terms of linear distance covered per control oop
    gyro_drift = int(config['AUTO']['gyro_drift'])                          #gyro drift in degrees per minute
    #print(drone_size)
    
    #linear advance and rotation variables
    linear = 0  #linear advance
    rotate=0    #rotattion incrementt
    trn=0       #turn command
    
    # the conditions that stop the simulation - crash, timeout or operator command
    crashed = False
    timeout = False
    finish = False
    
    #arrays for recording information
    track = []
    walls = []

    #desired and actual speed
    speed_desired=0
    speed_actual=0
    
    #start the clock
    clock = pygame.time.Clock()

    #open the window and load the maze
    pygame.init()
    pygame.display.set_caption('Autonomous Drone')
    background = pygame.image.load(maze_image)
    screen = pygame.display.set_mode(background.get_rect().size)
    height = background.get_height()
    width = background.get_width()
    screen.blit(background, (0,0))


    #all calculaitons a "real world" units
    #so convert bitmap size to "true" size
    long_max = width*pix
    lat_max =  height*pix

    #variables for ranges initialized here to non zero small number
    #because randomly they may not be read on first loop
    front_range=1
    right_range=1
    left_range=1
    

    
    #infinite control loop as long as no reason to stop
    while not crashed and not timeout and not finish:

        #get user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                finish = True

            # get movement keys (used only inmanual mode)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    trn = 1
                elif event.key == pygame.K_RIGHT:
                    trn = -1
                elif event.key == pygame.K_UP:
                    linear = d_linear
                elif event.key == pygame.K_DOWN:
                    linear = -d_linear
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    trn = 0
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    linear = 0

        #check distances to walls
        #noise is implemeted in the distance function
        #arbitrarily at a certain rate (based on no_range parameter) there is no reading at all, in which case previous readings are used            
        if random.randint(0,no_range) > 1:                    
            front_range = distance(background,long,lat, heading, drone_size, pix, range_noise)
            right_range = distance(background,long,lat, heading-side_sense_dir, drone_size, pix, range_noise)
            left_range = distance(background,long,lat, heading+side_sense_dir, drone_size, pix, range_noise)
            
        #calculate ranges
        #and update walls information if within max range
        #ranges are rounded to nearest cm
        #front range
        ranges = []
        range_long = int(long + front_range * math.cos(heading)+0.5)
        range_lat = int(lat - front_range * math.sin(heading)+0.5)
        ranges.append((range_long,range_lat))
        if front_range < max_range:
            walls.append((range_long,range_lat))
        #right side range    
        range_long = int(long + right_range * math.cos(heading-side_sense_dir)+0.5)
        range_lat = int(lat - right_range * math.sin(heading-side_sense_dir)+0.5)
        ranges.append((range_long,range_lat))                                      
        if right_range < max_range:
            walls.append((range_long,range_lat))
        #left side range            
        range_long = int(long + left_range * math.cos(heading+side_sense_dir)+0.5)
        range_lat = int(lat - left_range * math.sin(heading+side_sense_dir)+0.5)
        ranges.append((range_long,range_lat))                                      
        if left_range < max_range:
            walls.append((range_long,range_lat))

        #calculate desired speed and derive from it the actual speed
        speed_desired = desired_speed(front_range, max_speed, acc)
        speed_actual = actual_speed(speed_desired, speed_actual, acc, loop_delay)
        
        #move and rotate to new position and heading
        #in auto mode calculate incremental linear movement 
        if auto == True:
            linear = speed_actual*(loop_delay)
        #print(d_linear)
        #in auto mode calculate incremental rotation and new heading
        #minimum range is according to our current linear advance with a safety margin of 1 drone size + linear movement
        if auto == True:            
             trn = turn(front_range, right_range, left_range, side_sense_dir, drone_size+margin*linear)
        heading += trn*d_rotate
        if heading >= math.pi *2:
            heading=heading - math.pi *2
        if  heading < 0:
            heading = math.pi *2 - heading
        #print(heading*360/(math.pi*2))    
        #calulate new position rounded to closest cm
        long+=int(linear*math.cos(heading)+0.5)
        lat-=int(linear*math.sin(heading)+0.5)
        #print(heading*360/(math.pi*2),x,y,x_int,x_mod,y_int,y_mod)            

        #check for out of bound condition (take into account drone radius)
        #in this case simply do not move
        if long < drone_size/2:
            long = drone_size/2
        if long > long_max - drone_size/2:
            long = long_max -drone_size/2
        if lat < drone_size/2:
            lat = drone_size/2
        if lat > lat_max- drone_size/2:
            lat = lat_max - drone_size/2
                        
        
        #check for collision with walls or reaching boundary
        if check_collision(background,long,lat,drone_size, pix) == True:
            crashed = True

        #print(long, lat, heading, linear, front_range, right_range, left_range, trn)
            
        #update the track information
        track.append((long,lat))
       
        #update the scene
        screen.blit(background, (0,0))
        drone(screen,long,lat, heading, drone_size, pix)
        draw_ranges(screen,long,lat,ranges,pix)
        draw_points(screen, walls, green,pix)
        draw_points(screen,track, blue,pix)
        pygame.display.flip()


        #delay according to the loop delay that was computed due to sampling rate and number of sensors
        if auto == True:
            pygame.time.delay(int(loop_delay*1000))
        else:
            clock.tick(10)

        #check for time out
        if pygame.time.get_ticks() >= time_limit:
            timeout = True


    #out of the main loop - show the reason fro exit        
    if crashed == True:
        message_display(screen,width, height,'Crashed!')
    if timeout == True:
        message_display(screen,width, height,'Timed Out!')    
    if finish == True:
        message_display(screen,width, height,'Operator Quit!')

    #pygame.time.wait(3000)
    #quit()



if __name__ == '__main__':
    main()    
