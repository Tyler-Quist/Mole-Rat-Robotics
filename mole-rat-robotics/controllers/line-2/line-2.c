/*
 * File:          line.c
 * Date:          30-March-2020
 * Description:   Stays between two black lines
 * Author:        Karl R. Wurst
 * Modifications: Modified from e-puck line by Jean-Christophe Zufferey
 * Instead of staying on top of a line and following it, the robot 
 * uses a modified ground sensor array to stay between two lines.
 */

/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/***************************************************************************

  e-puck_line -- Base code for a practical assignment on behavior-based
  robotics. When completed, the behavior-based controller should allow
  the e-puck robot to follow the black line, avoid obstacles and
  recover its path afterwards.
  Copyright (C) 2006 Laboratory of Intelligent Systems (LIS), EPFL
  Authors: Jean-Christophe Zufferey
  Email: jean-christophe.zufferey@epfl.ch
  Web: http://lis.epfl.ch

  This program is free software; any publications presenting results
  obtained with this program must mention it and its origin. You
  can redistribute it and/or modify it under the terms of the GNU
  General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
  USA.

***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define LEFT 0
#define RIGHT 1
#define TIME_STEP 32  // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 550
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;
#define NORMAL_SPEED 300

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main() {
  int i, speed[2];

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];

  /* set up ground sensors */ 
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  
  /* set up motors */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  //left or right randomization
  srand(time(NULL));
  int rand_num = rand() % 2;
  
  //time variables
  time_t start, end;
  double cpu_time_used = 0;
  
    start = time(NULL);
    for (;;) {  // Main loop

      // Run one simulation step
      wb_robot_step(TIME_STEP);
    
      // Read ground sensors values
      for (i = 0; i < NB_GROUND_SENS; i++)
        gs_value[i] = wb_distance_sensor_get_value(gs[i]);
       
        //ending condition    
        if (gs_value[GS_LEFT] < 350 && gs_value[GS_RIGHT] < 350){
           speed[LEFT] = 0;
          speed[RIGHT] = 0;
          end = time(NULL);
          wb_motor_set_velocity(left_motor, 0);
          wb_motor_set_velocity(right_motor, 0);
          wb_robot_cleanup();
          break;
        }
        
        //continue maze 
        else{
        
        //right maze traverse
        if(rand_num == 0){
        
        //turn condition
        if (gs_value[GS_CENTER] < 350) {
          if(gs_value[GS_RIGHT] > 350 || gs_value[GS_CENTER] < 350){
            speed[LEFT] = 0;
            speed[RIGHT] = NORMAL_SPEED;
            wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
            wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
           }
       } else if (gs_value[GS_RIGHT] > 350) {
          speed[LEFT] = NORMAL_SPEED;
          speed[RIGHT] = - NORMAL_SPEED;
          wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
          wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      } else if (gs_value[GS_RIGHT] < 350) {
        speed[LEFT] = NORMAL_SPEED;
        speed[RIGHT] = NORMAL_SPEED + 100;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      }
    }
    
    //left maze traversal
    else if (rand_num == 1){
   
        //turn condition
        if (gs_value[GS_CENTER] < 350) {
          if(gs_value[GS_LEFT] > 350 || gs_value[GS_CENTER] < 350){
            speed[LEFT] = NORMAL_SPEED;
            speed[RIGHT] = 0;
            wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
            wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
           }
           
       } else if (gs_value[GS_LEFT] > 350) {
          speed[RIGHT] = NORMAL_SPEED;
          speed[LEFT] = - NORMAL_SPEED;
          wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
          wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      } 
      else if (gs_value[GS_LEFT] < 350) {
        speed[RIGHT] = NORMAL_SPEED;
        speed[LEFT] = NORMAL_SPEED + 100;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      }
    }
    }
      // Debugging printing
      printf("Sensors Left %d Middle %d Right %d  Speed Left %d Right %d  \n", 
        gs_value[GS_LEFT], gs_value[GS_CENTER], gs_value[GS_RIGHT],
        speed[LEFT], speed[RIGHT]);
      fflush(stdout);
    }
   cpu_time_used = ((double) (end - start));
  printf("robot sucessfully completed the maze\n");
  printf("random number was %d\n", rand_num);
  printf("program took %0.2lf seconds to complete the maze\n", cpu_time_used);
  return 0;
}
