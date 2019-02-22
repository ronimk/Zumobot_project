/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This
    library has basic methods of various sensors and communications so that you can make what you want with them. <br>
    <br><br>
    </p>

    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>

    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
/**
 * @file    main.c
 * @brief
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

/*
    This repo contains my solutions to the introduction course to Smart Systems and IoT
    offered by Metropolia UAS to all first year students.
    There were three main tasks to be completed in teams of three student in the course - all tasks involving Zumobot
    (https://www.pololu.com/category/169/zumo-robot-for-arduino)

    In task 1 we had to program a track-driving robot that starts by pressing a remote control button.
    In task 2 we had to program a sumo wrestling roobot.
    In task 3 we had to program a maze solver.

    I programmed tasks 1 and 3 myself and one of my teammates programmed task 2 (which for that reason is not shown here).
    We did have problems with managing time since only two in our team were capable of working the required final tasks.
    Therefore we decided to divide the work this way instead of both of us being partly involved in all of the final tasks.
    Since tasks 1 and 2 could easily reuse the same line following logic I had already implemented before and since
    we didn't have much time left to work on the project just the two of us, I volunteered to do the two "line following" tasks.

    The scheduling problems shine through in the thirs (final) task since I never got to implement the breadth-first-search pathfinding
    algorithm I had designed. I hardly had time to implement linked-list datastructure to support the pathfinding algorithm, much less
    test it for anything more than basic functionality. Therefore I am not completely sure if it works without memory leakages.

    Of course the linked list DS is not used in the program in any way, so this is just a theoretical issue now.

    Another problem was that we weren't able to completely familiarize ourselves with the Zumobot during the course, and this shows in some
    parts of the code.

    We also could have structured the code better and separate some of the code in this file into their own library/libraries...

    Still, overall the course taught me a lot and I am very satisfied with my simple yet elegant solution for deciding where to drive next
    along a line (turn, straight or backwards).

    The main program contains the final solution unaltered that the Zumobot was used to demonstrate our coursework.
    Thus it has much room for improvement!

    To make the code work, insert the proper MQTT info in the zumo_config.h file.
*/

#if 1
/* Final Assignment 1 - Trackdriving */
#include <stdlib.h>

#define MIN_SPEED   0
#define MAX_SPEED   255
#define START_SPEED 75
#define TRACK_SPEED MAX_SPEED

#define BLACK 1
#define WHITE 0


/* Change the value of NUM_OF_CHECKPOINTS to the appropriate value
   according to the track being driven through. The start and end
   lines are considered special checkpoints where the robot must act
    in a special way. The robot ignores all other checkpoints. */
#define NUM_OF_CHECKPOINTS  3


/*  literals for identifying corrections to the movement:
    Based on the differentce between the robot's left sensor
    weight and right sensor weight, the robot does different
    correction moves. It either moves straight forward, turns
    a bit to the right/left, backs off if it doesn't see a line
    anymore, or deals with a checkpoint that it has reached. */
enum {STRAIGHT_LINE, BACK_OFF, CURVE_RIGHT, CURVE_LEFT, CHECKPOINT, CLEAR_CHECKPOINT};


/* literals for adjusting corrections to the movement:
   We didn't have much time left to tweak around and find
   optimal delays and adjusts so we ended up not using this
   functionality in the program.. On the hindsight, it probably
   wasn't wise to turn off the speed of other motor completely
   when making curve corrections (the #define CURVE_ADJUST 0
   results exactly in this...) */
#define CURVE_ADJUST      0
#define BACK_OFF_ADJUST   1
#define FORWARD_DELAY     0
#define BACK_OFF_DELAY    0
#define CURVE_DELAY       0

/* MQTT topics: */
#define START_TOPIC     "Zumo021/start"
#define SIDETRACK_TOPIC "Zumo021/sidetracked"
#define END_TOPIC       "Zumo021/end"
#define RUNTIME_TOPIC   "Zumo021/total-runtime"

TickType_t start_time;   /* timestamp for when the robot starts driving the track */
TickType_t end_time;     /* timestamp for when the robot stops driving the track */

void init(void);
void wait_for_IR(void);
void wait_for_Button(void);
uint8 check_correction_type(void);
void  make_correction(uint8 correction_type);
void end_program(void);

void zmain(void *p)
{
    (void) p; // we don't use this parameter

    init();
    wait_for_Button();
    print_mqtt(START_TOPIC, "Ready...");

    for(;;)
    {
        make_correction(check_correction_type());
    }
}

void init(void)
/* init is responsible for initializing the robot ready to perform.
   it basically sets up sensors, turns on motors etc. */
{
    print_mqtt(START_TOPIC, "Starting engines...");
    /* Start the motor: */
    motor_start();
    motor_forward(0, 0);
    /* Start reflectance sensors: */
    reflectance_start();
    /* set sensor tresholds for the reflectance sensors: */
    reflectance_set_threshold(11000, 11000, 11000, 11000, 11000, 11000);
    IR_Start();
}

void wait_for_IR(void)
/* can be used to "listen to" if a button has been pressed on IR-controller */
{
    IR_flush();
    IR_wait();
    vTaskDelay(180); /* this works as an "ad-hock" check for making sure that the button doesn't stay pressed */
                     /* It is ugly, but at least it works. This function was added very late to the program and
                        we didn't have time to figure out the proper way to handle the button "unpress". */
}

void wait_for_Button(void)
/* can be used to "listen to" if the micro-controller's SW1-button has been pressed. */
{
    while (SW1_Read() == 1);
}

/* macros for checking the reflection sensors' data. */
#define OUTER_LEFT    (ref.l3)
#define MIDDLE_LEFT   (ref.l2)
#define INNER_LEFT    (ref.l1)
#define OUTER_RIGHT   (ref.r3)
#define MIDDLE_RIGHT  (ref.r2)
#define INNER_RIGHT   (ref.r1)

uint8 check_correction_type(void)
/* check_corrections checks which kind of a correction has to be performed next
   and returns the approppriate correction. */
{
    static struct sensors_ ref;
    reflectance_digital(&ref);

    /* The following algorithm weighs both sides of the robot by adding together sensor readings for each side
       It uses a distorted weight balance to more precisely solve "tight tracks".
       The outer sensors' values are undistorted (1 for black, 0 for white),
       The middle sensors' values are distorted by a multiplier of 2.
       The inner sensors' values are distorted by a multiplier of 4.
       Thus, even if one side only reads in the inner sensor's data (a value of 4)
       And the other side reads in both middle and outer-sensors' data but not the inner
       sensor's data (a value of 3 when summed together), the inner sensor weighs more and
       forces the robot to correct its move to the heavier side, making sure that the robot
       stays in the right track segment, even if two different track segments are very close
       together.

        Thus our robot can survive very tight tracks with a simple and elegant design: */
    int left_weight  = OUTER_LEFT + (MIDDLE_LEFT << 1) + (INNER_LEFT << 2);
    int right_weight = OUTER_RIGHT + (MIDDLE_RIGHT << 1) + (INNER_RIGHT << 2);
    int balance      = left_weight - right_weight;

    if (balance > 0)
        return CURVE_LEFT;
    else if (balance < 0)
        return CURVE_RIGHT;
    else
    {
        if (left_weight == 0)
            return BACK_OFF;
        else if (left_weight == 7)
            return CHECKPOINT;

        return STRAIGHT_LINE;
    }
}

void make_correction(uint8 correction_type)
/* handles making the actual correction.

   to make the checkpoint portion as simple and uniform as possible,
   we use gotos to "tilt" the drive logic to handle the encountered
   checkpoint without the fear of counting the same checkpoint multiple
   times.

   It is my view that there is nothing inherently wrong in gotos
   themselves (in a caricaturic way, Scheme's Call with Current Continuation
   is just an advanced form of a goto statement, and it can be extremely useful
   in certain situations). However goto statements have certainly been abused
   in the past by poor knowledge. */
{
    static uint8 move_mem=STRAIGHT_LINE, checkp_num=0, motor_speed=START_SPEED;
    /* move_mem could be utilized by performing smarter move corrections. Right now it's
       mainly used to send MQTT data and help in solving a "mystery motor malfunction" problem
       motors when necessary.*/
retry:
    switch (correction_type)
    {
        case STRAIGHT_LINE: /* for straight line corrections, keep moving forward */
            motor_forward(motor_speed, FORWARD_DELAY);
            break;

        case CURVE_LEFT: /* for left curve corrections, turn a little bit to the left */
            motor_turn(motor_speed*CURVE_ADJUST, motor_speed, CURVE_DELAY);
            break;

        case CURVE_RIGHT: /* for left curve corrections, turn a little bit to the right */
            motor_turn(motor_speed, motor_speed*CURVE_ADJUST, CURVE_DELAY);
            break;


        case BACK_OFF: /* for back off corrections, print a SIDETRACKED-message, and then
                          back off a little bit */
            if (move_mem != correction_type && checkp_num > 0)
            {
                print_mqtt(SIDETRACK_TOPIC, "%d", xTaskGetTickCount());
            }
            motor_backward(motor_speed*BACK_OFF_ADJUST, BACK_OFF_DELAY);
            break;

        case CHECKPOINT: /* the logic for encountering and handling checkpoint corrections: */
            checkp_num++;
            if (checkp_num == 1) {
                /* for the first checkpoint, wait */
                /* for IR-control start command   */
                /* ------------------------------ */
                /* First, stop the engines:       */
                motor_forward(0, 0);
                motor_speed = TRACK_SPEED;
                /* Then, set up IR-reciever and wait for an action: */
                wait_for_IR();
                /* when an IR-control button has been pressed,
                   set the start-tick-timestamp and print a message: */
                start_time = xTaskGetTickCount();
                print_mqtt(START_TOPIC, "%d", start_time);
            } else if (checkp_num == NUM_OF_CHECKPOINTS)
            {   /* for the last checkpoint, call end_program to handle all the
                   necessary shutdown preparations: */
                end_program();
            }
            /* finally, clear the checkpoint for all other than the last checkpoint: */
            correction_type = CLEAR_CHECKPOINT;
            move_mem = CHECKPOINT;
            motor_forward(0, 0);
            goto retry;

        case CLEAR_CHECKPOINT:
            /* Clear the checkpoint by driving through it: */
            while ((correction_type=check_correction_type()) == CHECKPOINT)
                motor_forward(motor_speed, FORWARD_DELAY);
            /* the checkpoint has been cleared and a new correction type is in the buffer.
               handle the new correction type: */
            motor_forward(0, 0);
            goto retry;
    }

    /* finally decide if motor speeds have to be turned off: */
    if (move_mem != correction_type)
            motor_forward(0, 0);
    move_mem = correction_type;

    /* To be honest, we didn't have enough time to figure out when exactly the motors' speeds
       have to be turned off to not cause unexpected problems when driving the Zumobot.
       Thus, in the code, we probably call motor_forward(0, 0); too many times and in unnecessary situations -
       which in turn hinders the robot's movement...

       When, at first the robot's motors were always running, strange things kept happening to the robot and
       this all got fixed when we started turning the motors off - at first after every single move, then using
       motor_mem to basically only turn the motors off whenever the current movement differed from the last.

       In hindsight, it's probably only necessary to turn a motor off before changing it's rotation direction,
       but we didn't really have time to focus on finding the root cause of the problem and fixing it. */
}

void end_program(void)
{
/* end_program is used to handle all the necessary shut down procedures. */
    motor_stop();
    end_time = xTaskGetTickCount();
    print_mqtt(END_TOPIC, "%d", end_time);
    print_mqtt(RUNTIME_TOPIC, "%d", end_time-start_time);
    vTaskDelay(10000);
    exit(0);
}

#endif


#if 0
/****************************************
* Final assignment 3 - Labyrinth solver *
* The driving logic is almost the same  *
* as in assignment 1                    *
*****************************************/
#include <stdlib.h>

#define DEBUG_MODE  0
#define MIN_SPEED   0
#define MAX_SPEED   255
#define TRACK_SPEED 125
#define START_SPEED 75

#define BLACK 1
#define WHITE 0


/*  literals for identifying corrections to the movement: */
typedef enum {STRAIGHT_LINE, BACK_OFF, CURVE_RIGHT, CURVE_LEFT, NODE, INIT_PHASE, END_PHASE} correction_type;

/* literals for adjusting corrections to the movement: */
#define CURVE_ADJUST      0
#define BACK_OFF_ADJUST   1
#define FORWARD_DELAY     0
#define BACK_OFF_DELAY    0
#define CURVE_DELAY       0
#define NODE_DELAY        125
#define TANK_TURN_DELAY   130
/* definitions for tank turning */
#define TO_LEFT           0
#define TO_RIGHT          1

/* MQTT topics: */
#define STARTUP_TOPIC   "Zumo021/startup"
#define TIMER_TOPIC     "Zumo021/Clock"
#define ERROR_TOPIC     "Zumo021/ERROR"
#define MAP_TOPIC       "Zumo021/Map"
#define DEBUG_TOPIC     "Zumo021/DEBUG"
#define END_TOPIC       "Zumo021/Stop"

struct sensors_ REF;
TickType_t START_TIME = 0;
TickType_t STOP_TIME = 0;
uint8 MAZE_FINISHED = true;

typedef enum direction{NORTH, EAST, SOUTH, WEST} direction;
/* Struct to hold the x/y coordinates of the robot */
/* The y coordinate range is 1...7
   The x coordinate range is 0..14 */
struct coordinates {
    uint8 x;
    uint8 y;
};

struct nav {
    struct coordinates pos;
    direction dir;
} robot_nav;

/* The maze consists of a network of 7 x 15 nodes, that are all at first unknown.
   A node itself can be one of the following:
   - E(1/2)  ('-' or '|')  for a node that the robot has passed once - depending on the direction the robot took.
   - UKN     ('?') for an unknown node.
   - WALL    ('W') for an unpassable node.
   - LIM     (' ') for a boundary of the maze.
   - USLS    ('+') for a node that the robot has passed twice or more. */
#define ENS      '-'   /* NORTH-SOUTH direction */
#define EEW      '|'   /* EAST-WEST direction   */
#define UKN      '?'
#define WALL     'W'
#define USLS     '+'
#define LIM      ' '

#define MAZE_X_SIZE 9
#define MAZE_Y_SIZE 17
#define END_X       4
#define END_Y       14
/* The initial maze map:*/
char maze_map[MAZE_X_SIZE][MAZE_Y_SIZE] =
                        { {LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, LIM, LIM, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, LIM, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, '\0'},
                          {ENS, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, ENS, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, LIM, '\0'},
                          {LIM, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, UKN, LIM, LIM, LIM, '\0'},
                          {LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, LIM, '\0'} };
/* Even though the actual maze was only 7*14, a 9*17-two dimensional array is used here for two reasons:
   1) when the robot is on an edge, its x/y-indexes can never be 0 or MAZE_[X/Y]_SIZE. This helps a lot
      when checking for neighbouring nodes. For example: we don't have to adjust the checking logic for maze_map
      boundaries. The correlation of this is that LIM-nodes in the maze_map -array work exactly like WALL-nodes,
      but they are printed as spaces in the final output of the maze, so they don't show in the final print of the maze.
   2) the arrays can be made true strings with a string delimiter character '\0' by adding an additional
      array-cell in the end of each 1-dimensional inner array. This way it is very easy to send and print the discovered
      maze to the MQTT server. */

void init(void);
void wait_for_IR(void);
void wait_for_Button(void);
int sample_Ultra(void);
void update_maze(void);
void mark_map(uint8 xpos, uint8 ypos, char mark);
uint8 check_wall_in_front(void);
direction choose_next_node(void);
uint8 passable_node(struct coordinates *node);
uint8 passable_coodrs(uint8 posx, uint8 posy);
uint8 passable_mark(char mark);
void update_coords(void);
correction_type check_correction_type(void);
correction_type make_correction(correction_type ct);
void node_turn(direction dir);
void tank_turn(uint8 dir);
void end_program(void);


/* We also had planned for a breadth-first search alorithm for pathfinding but never had the time to test
   and implement it properly. Here are all the data structures and functions created for the BFS: */
typedef struct path_node {
    struct coordinates pos;
    struct path_node *next;
} path;

typedef struct path_search_node {
    path *p;
    struct path_search_node *next;
} path_search_list;

uint8 path_length(const path *p);
path *copy_path(const path *orig);
uint8 contains_node(const struct coordinates *node, const path *path); /* checks whether a path list already contains a certain node */
uint8 coord_equal(const struct coordinates *pos1, const struct coordinates *pos2); /* To test if two coords have the same x- and y-positions */
void add_to_path(const struct coordinates *pos, path **head);    /* appends a path with a node */
void add_to_path_end(const struct coordinates *pos, path **head); /* appends a path with a node to the end of the path */
void append_to_psl(path *append_p, path_search_list **psl); /*appends a path to the end of a path_search_list */
path* pop(path_search_list **psl); /* extracts a pointer to the head from a pat_search_list and returns it. */
uint8 empty_search_list(path_search_list **psl);
void free_path(path *path);
void free_psl(path_search_list *psl);

void zmain(void *p)
{
    (void) p; // we don't use this parameter

    init();

    wait_for_Button();
    make_correction(INIT_PHASE);
    /* start the main loop: */
    for(;;)
    {

        /* Now the robot is on a node */
        /* For debug purposes, print_mqtt the current coordinates: */
        #if DEBUG_MODE
            print_mqtt(DEBUG_TOPIC, "(x, y): (%d, %d)", robot_nav.pos.x, robot_nav.pos.y);
        #endif
        /* 1: Check to see if we are on the final node: */
        if (robot_nav.pos.x == END_X && robot_nav.pos.y == END_Y)
        {
            update_maze();
            make_correction(END_PHASE);
        }

        /* 2: update the current maze position of the robot: */
        update_maze();

        /* 3&4: check (and mark) all the walls around the robot */
        /* and also, choose a suitable path to follow: */
        uint8 blocked = true;
        do {
            node_turn(choose_next_node());
            blocked = check_wall_in_front();
        } while (blocked);

        /* 5: move to the chosen path node: */
        while (make_correction(check_correction_type()) != NODE) ;

        /* 6: Finally, update the robot's position: */
        update_coords();
    }
}

void init(void)
{
    /* Start the motor: */
    motor_start();
    motor_forward(0,0);
    /* Start ultrasonic sensors: */
    Ultra_Start();
    /* set sensor tresholds for the reflectance sensors: */
    reflectance_start();
    reflectance_set_threshold(11000, 11000, 11000, 11000, 11000, 11000);
    IR_Start();

    /* Set the starting position of the robot:
       (the node number of the starting node is 1 */
    robot_nav.dir   = NORTH;
    robot_nav.pos.x = 4;
    robot_nav.pos.y = 1;

    print_mqtt(STARTUP_TOPIC, "Starting up the program\n");
}

void wait_for_IR(void)
{
    IR_flush();
    IR_wait();
    vTaskDelay(180);
}

void wait_for_Button(void)
{
    while (SW1_Read() == 1);
}

int sample_Ultra(void)
/* returns true if there is an obstacle within react_dist length
   false, if the path forward is clear */
{
    #define react_dist 13
    #define sample_amount 5
    int values[sample_amount];

    for (int i=0; i<sample_amount; i++) {
        values[i] = Ultra_GetDistance();
    }

    /* sort the array:
       (for such a small sample set, bubble sort suits just fine) */
    for (int i=0; i<sample_amount-1; i++)
    {
        for (int j=0; j<sample_amount-i-1; j++)
        {
            if (values[j] > values[j+1])
            {
                int t = values[j+1];
                values[j+1] = values[j];
                values[j] = t;
            }
        }
    }

    if (values[sample_amount/2] < react_dist)
    {
        #if DEBUG_MODE
            Beep(100, 100);
        #endif
        return true;
    }


    return false;
}

void update_maze(void)
/* update_maze updates the robot's position in the maze with the appropriate mark. */
{
    uint8 posx = robot_nav.pos.x, posy = robot_nav.pos.y;
    char old_mark = maze_map[posx][posy];

    if (old_mark == UKN)
    {
        if (robot_nav.dir == NORTH || robot_nav.dir == SOUTH)
            mark_map(posx, posy, ENS);
        else
            mark_map(posx, posy, EEW);
    }
    else if (old_mark == ENS || old_mark == EEW)
        mark_map(posx, posy, USLS);
}

void mark_map(uint8 posx, uint8 posy, char mark)
/* marks a position in a map with a certain mark. */
{
    maze_map[posx][posy] = mark;
}

uint8 check_wall_in_front(void)
/* check_wall_in_front checks (and marks if necessary) for a wall right in front of the robot.
   Either I don't understand how the Ultrasensors work, or then there was a problem in the
   hardware/RTOS of the robot, because I couldn't get this to work at all.
   Sometimes it saw unexisting walls, and sometimes it didn't see walls at all. */
{
    uint8 posx=robot_nav.pos.x, posy=robot_nav.pos.y;

    if (sample_Ultra())
    {
        switch (robot_nav.dir)
        {
            case NORTH: posy++;
                        break;
            case EAST:  posx++;
                        break;
            case SOUTH: posy--;
                        break;
            case WEST: posx--;
                        break;
        }
        mark_map(posx, posy, WALL);
        return true;
    }

    return false;
}

direction choose_next_node(void)
/* choose_next_node returns the best direction to move to next based on the robot's current knowledge of the maze. */
{
    uint8 centerx = robot_nav.pos.x;
    uint8 centery = robot_nav.pos.y;

    char n, s, e, w;
    /* init north node mark: */
    n = maze_map[centerx][centery+1];
    /* init east node mark: */
    e = maze_map[centerx+1][centery];
    /* init south node mark: */
    s = maze_map[centerx][centery-1];
    /* init west node mark: */
    w = maze_map[centerx-1][centery];

    /* prefer unknown nodes: */
    if (n == UKN)
        return NORTH;
    else if (robot_nav.pos.x<4 && e == UKN)
        return EAST;
    else if (robot_nav.pos.x>=4 && w == UKN)
        return WEST;
    else if (e == UKN)
        return EAST;
    else if (w == UKN)
        return WEST;
    /* however never prefer SOUTH... */
    /* At this point, there are no unknown nodes (other than possibly in south), so check for known nodes first: */
    else if (passable_mark(n))
        return NORTH;
    else if (robot_nav.pos.x<4 && passable_mark(e))
        return EAST;
    else if (robot_nav.pos.x>=4 && passable_mark(w))
        return WEST;
    else if (passable_mark(e))
        return EAST;
    else if (passable_mark(w))
        return WEST;
    else if (passable_mark(s) || s == USLS)
        return SOUTH; /* if nothing else, go SOUTH */
    else
        end_program(); /* the robot is stuck and cannot move anymore... */
}

uint8 passable_node(struct coordinates *node)
/* passable_node checks whether the given node in the maze contain a passable node or not. */
{
    return passable_mark(maze_map[node->x][node->y]);

}

uint8 passable_coodrs(uint8 posx, uint8 posy)
/* passable_coords checks whether the given x- and y-coordinates contain a passable node in the maze or not. */
{
    return passable_mark(maze_map[posx][posy]);

}

uint8 passable_mark(char mark)
/* passable_mark checks whether a given mark is passable or not */
{
    return (mark == UKN || mark == ENS || mark == EEW);
}

void update_coords(void)
/* update_coords updates the robots position in the maze, based on the direction it last moved to. */
{
    switch (robot_nav.dir)
    {
        case NORTH: robot_nav.pos.y++;
                    break;
        case EAST:  robot_nav.pos.x++;
                    break;
        case SOUTH: robot_nav.pos.y--;
                    break;
        case WEST:  robot_nav.pos.x--;
                    break;
    }
}

/* macros for checking the reflection sensors' data */
#define OUTER_LEFT    (REF.l3)
#define MIDDLE_LEFT   (REF.l2)
#define INNER_LEFT    (REF.l1)
#define OUTER_RIGHT   (REF.r3)
#define MIDDLE_RIGHT  (REF.r2)
#define INNER_RIGHT   (REF.r1)

correction_type check_correction_type(void)
{
    reflectance_digital(&REF);

    /* The following algorithm weighs both sides of the robot by adding together sensor readings for each side
       It uses a distorted weight balance to more precisely identify "tight tracks". */

    int left_weight  = OUTER_LEFT + (MIDDLE_LEFT << 1) + (INNER_LEFT << 2);
    int right_weight = OUTER_RIGHT + (MIDDLE_RIGHT << 1) + (INNER_RIGHT << 2);
    int balance      = left_weight - right_weight;

	if (left_weight == 7 || right_weight == 7)
			return NODE;
    else if (balance > 0)
        return CURVE_LEFT;
    else if (balance < 0)
        return CURVE_RIGHT;
    else if (left_weight == 0) /* balance == 0 */
            return BACK_OFF;

    return STRAIGHT_LINE;
}

correction_type make_correction(correction_type ct)
/* make_correction is very similar to the final assignment 1's make_correction with few exceptions.
   Since the beginning and end of the maze solving task are more complicated than those of the
   line following/track driving task, they have been separated into their own correction logic. */
{
    static uint8 move_mem=STRAIGHT_LINE, motor_speed=TRACK_SPEED;
    /* Cuirrently, move_mem is only used for checking whether the motors have to be "reset"
       It could be interesting to try to make the robot change its behavior by comparing
       the current and the previous move, but there was no time to implement any of that */

    switch (ct)
    {
        case STRAIGHT_LINE:
            motor_forward(motor_speed, FORWARD_DELAY);
            break;

        case CURVE_LEFT:
            motor_turn(motor_speed*CURVE_ADJUST, motor_speed, CURVE_DELAY);
            break;

        case CURVE_RIGHT:
            motor_turn(motor_speed, motor_speed*CURVE_ADJUST, CURVE_DELAY);
            break;


        case BACK_OFF:
            motor_backward(motor_speed*BACK_OFF_ADJUST, BACK_OFF_DELAY);
            break;

        case INIT_PHASE:
            do /* drive to the beginning line: */
            {
                motor_forward(START_SPEED, FORWARD_DELAY);
            } while (check_correction_type() != NODE);
            motor_forward(TRACK_SPEED, NODE_DELAY);
            motor_forward(0,0);
            /* Then, set up IR-reciever and wait for an action: */
            wait_for_IR();
            /* When a button in IR-controller has been pressed, drive to the first node of the maze: */
            START_TIME = xTaskGetTickCount();
            print_mqtt(TIMER_TOPIC, "Start Time: %d", START_TIME);
            do
            {
                motor_forward(motor_speed, FORWARD_DELAY);
            } while (check_correction_type() != NODE);
            motor_forward(0,0);
            make_correction(NODE);
            break;

        case NODE:
            motor_forward(TRACK_SPEED, NODE_DELAY);
            motor_forward(0,0);
            break;

        case END_PHASE:
            /* turn NORTH and drive along the line until it stops. Then end the program. */
            node_turn(NORTH);
            while (ct != BACK_OFF)
            {
                ct = check_correction_type();
                make_correction(ct);
            }
            motor_forward(0,0);
            end_program();
            break;
    }

    if (move_mem != ct)
            motor_forward(0, 0);
    move_mem = ct;
            return ct;
}

void node_turn(direction dir)
/* node_turn handles making the robot turn to the given direction based on its current direction. */
{
    if (dir == robot_nav.dir)
        return;

    switch (robot_nav.dir)
    {
        case NORTH: if (dir == EAST) tank_turn(TO_RIGHT);
                    else if (dir == WEST) tank_turn(TO_LEFT);
                    else {
                        tank_turn(TO_LEFT);
                        tank_turn(TO_LEFT);
                    }
                    break;

        case EAST:  if (dir == SOUTH) tank_turn(TO_RIGHT);
                    else if (dir == NORTH) tank_turn(TO_LEFT);
                    else {
                        tank_turn(TO_LEFT);
                        tank_turn(TO_LEFT);
                    }
                    break;

        case SOUTH: if (dir == WEST) tank_turn(TO_RIGHT);
                    else if (dir == EAST) tank_turn(TO_LEFT);
                    else {
                        tank_turn(TO_LEFT);
                        tank_turn(TO_LEFT);
                    }
                    break;

        case WEST:  if (dir == NORTH) tank_turn(TO_RIGHT);
                    else if (dir == SOUTH) tank_turn(TO_LEFT);
                    else {
                        tank_turn(TO_LEFT);
                        tank_turn(TO_LEFT);
                    }
                    break;
    }
    robot_nav.dir = dir;
}

void tank_turn(uint8 dir)
/* tank_turn is used to make a proper, delay-independent tank turn. */
{
    #define corr_delay 7
    MotorDirLeft_Write((dir == TO_LEFT)? 1 : 0);      // set LeftMotor direction
    MotorDirRight_Write((dir == TO_LEFT)? 0 : 1);     // set RightMotor direction
    PWM_WriteCompare1(TRACK_SPEED);
    PWM_WriteCompare2(TRACK_SPEED);
    vTaskDelay(TANK_TURN_DELAY);
    reflectance_digital(&REF);
    while (!(REF.l1==BLACK && REF.r1==BLACK))
    { /* keep turning */
    reflectance_digital(&REF);
    }
    vTaskDelay(corr_delay); /* a small corrective move to get the robot more to the center */
    motor_forward(0, 0);

}

void end_program(void)
/* end_program is used to handle all the actions necessary for shutting down the program. */
{
    motor_stop();

    STOP_TIME = xTaskGetTickCount();
    if (!MAZE_FINISHED)
        print_mqtt(ERROR_TOPIC, "Maze was never finished");
    else
        print_mqtt(END_TOPIC, "Maze finished!");
    print_mqtt(TIMER_TOPIC, "Finish Time: %d", STOP_TIME);
    print_mqtt(TIMER_TOPIC, "Total driving time: %d", (STOP_TIME-START_TIME));
    print_mqtt(MAP_TOPIC, "Below is the discovered map of the maze:");
    for (int i=0; i<MAZE_X_SIZE; i++)
    {
        print_mqtt(MAP_TOPIC, "%s", maze_map[i]);
    }
    print_mqtt(MAP_TOPIC, "(W: a wall; ?: unknown; -: NORTH-SOUTH pass; |: EAST-WEST pass;");
    print_mqtt(MAP_TOPIC, " +: node passed through 2 times)");
    vTaskDelay(10000);
    exit(0);
}

/* The never-finished interfaces to use the PATH-list and PATH-SEARCH-list data structures:
   it hasn't been properly tested if these hold memory leaks or not but with or without
   possible memory leaks, they at least work... */
uint8 path_length(const path *p)
{
    int l = 0;

    while (p != NULL)
    {
        l++;
        p = p->next;
    }

    return l;
}

path *copy_path(const path* orig)
{
    if (orig == NULL)
        return NULL;

    path *cp = malloc(sizeof(path));
    path *tmp = cp;
    if (cp == NULL)
        exit(-1);

    while (orig != NULL)
    {
        tmp->pos.x = orig->pos.x;
        tmp->pos.y = orig->pos.y;
        tmp->next = NULL;

        if (orig->next != NULL)
            tmp->next = malloc(sizeof(path));

        orig = orig->next;
        tmp = tmp->next;
    }

    return cp;
}

uint8 contains_node(const struct coordinates *node, const path *path)
/* contains_node is used to test whether a given path already contains a given coordinate. */
{
    while (path != NULL)
    {
        if (coord_equal(&(path->pos), node)) /* coord_equal was never implemented */
            return true;

        path = path->next;
    }

    return false;
}

uint8 coord_equal(const struct coordinates *pos1, const struct coordinates *pos2)
/* coord_equal is used to test if two coords have the same x- and y-positions */
{
    if (pos1->x != pos2->x)
        return false;

    if (pos1->y != pos2->y)
        return false;

    return true;
}

/* interfaces for the pathfinding datastructures: */
void add_to_path(const struct coordinates *pos, path **head)
/* appends a path_node. Returns the new head. */
{
    path *new_node = malloc(sizeof(path));

    if (new_node==NULL)
        exit(-1);

    new_node->pos.x = pos->x;
    new_node->pos.y = pos->y;

    if (*head == NULL)
        new_node->next = NULL;
    else
        new_node->next = *head;

    *head = new_node;
}

void add_to_path_end(const struct coordinates *pos, path **head)
/* appends a path_node at the end of the path list. Returns the new head. */
{
    path *new_node = malloc(sizeof(path));

    if (new_node==NULL)
        exit(-1);

    new_node->pos.x = pos->x;
    new_node->pos.y = pos->y;
    new_node->next = NULL;

    if (*head == NULL)
        *head = new_node;
    else
    {
        path *n = *head;
        while (n->next != NULL) n = n->next;
        n->next = new_node;
    }
}

void append_to_psl(path *append_p, path_search_list **psl)
/*appends a path to the end of a path_search_list. Returns a pointer to the head of psl */
{
    path_search_list *new_tail = malloc(sizeof(path_search_list));

    if (new_tail==NULL)
        exit(-1);

    new_tail->p = append_p;
    new_tail->next = NULL;

    if (*psl == NULL)
    {
        *psl = new_tail;
        return;
    }

    path_search_list *tmp = *psl;

    while (tmp->next != NULL) tmp = tmp->next;
    tmp->next = new_tail;
}

path *pop(path_search_list **psl)
/* extracts the head from a pat_search_list and returns it. */
{
    if (*psl == NULL)
        return NULL;
    path *tmp = (*psl)->p;
    path_search_list *next = (*psl)->next;
    (*psl)->next = NULL;
    free(*psl);
    *psl = next;
    return tmp;
}

uint8 empty_search_list(path_search_list **psl)
{
    return (*psl == NULL)? false: true;
}

void free_path(path *p)
/* frees all memory of a path list */
{
    if (p == NULL)
        return;

    path *next = p->next;

    while (next != NULL)
    {
        free(p);
        p = next;
        next = p->next;
    }
    free(p);
}

void free_psl(path_search_list *psl)
/* frees all memory of a path search list */
{
    if (psl == NULL)
        return;

    path_search_list *next = psl->next;

    while (next != NULL)
    {
        free_path(psl->p);
        free(psl);
        psl = next;
        next = psl->next;
    }

    free_path(psl->p);
    free(psl);
}

#endif
