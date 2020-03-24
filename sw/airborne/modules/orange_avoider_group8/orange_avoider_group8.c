/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider_group8/orange_avoider_group8.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/lib/vision/image.h"

#include <time.h>
#include <stdio.h>
#include <inttypes.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseIncrementAvoidance(void);

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS
};

// define settings

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
uint32_t green_color_count = 0;               // green color count from color filter for obstacle detection
uint32_t orange_color_count = 0;               // orange color count from color filter for obstacle detection

int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
int16_t left_free_confidence = 0;
int16_t right_free_confidence = 0;
float heading_increment = 5.f;          // heading angle increment [deg]
float heading_increment_flight = 5.f;          // heading angle flight increment [deg]
float maxDistance = 1.25;//2.25;               // max waypoint displacement [m]

//uint8_t first = 0;
struct image_t prev_img;
struct image_t next_img;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID2
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID2 ABI_BROADCAST
#endif

static abi_event green_color_detection_ev;
static abi_event orange_color_detection_ev;

static void green_color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    green_color_count = quality;

}

static void orange_color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                     int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                                     int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                                     int32_t quality, int16_t __attribute__((unused)) extra)
{
    orange_color_count = quality;

}


/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
    // Initialise random values
    srand(time(NULL));
    chooseIncrementAvoidance();

    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &green_color_detection_ev, green_color_detection_cb);
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID2, &orange_color_detection_ev, orange_color_detection_cb);

}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }

    // compute current color thresholds
    int32_t green_color_count_threshold  = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
    int32_t orange_color_count_threshold = orange_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

    // update our safe confidence using color thresholds
    if(green_color_count > green_color_count_threshold && orange_color_count < orange_color_count_threshold){
        obstacle_free_confidence++;
    } else {
        obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    }

    // Checks the pixels on the central part, left and right of the image, adds a CCW or CW rotation if for more than 3 frames, left or right have more open areas
    int32_t left_score  = left_green_count  - left_orange_count;
    int32_t right_score = right_green_count - right_orange_count;
    int32_t central_score = 0.3*(green_color_count - orange_color_count);

    // TODO: Make sliders for confidences; make slider for 0.3,

    // printf("Left score %" PRId32 ", Right score %" PRId32 "Central score %" PRId32 "\n", left_score, right_score, central_score);
    if (left_score > central_score) {
        heading_increment_flight = -3.f;
        left_free_confidence++;
    }
    if (right_score > central_score) {
        heading_increment_flight = 3.f;
        right_free_confidence++;
    }
    if (left_score < central_score) {
        left_free_confidence = 0;
    }
    if (right_score < central_score) {
        right_free_confidence = 0;
    }


    // bound obstacle_free_confidence
    // If obstacle_free_confidence < 0, set to 0, if obstacle_free_confidence > max, set to max
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

    switch (navigation_state){
        case SAFE:
            printf("SAFE: %d\n", obstacle_free_confidence);
            printf("right free confidence %d", right_free_confidence);
            printf("left free confidence %d",left_free_confidence);

            // induces a CCW or CW rotation if, for more than 3 frames, left or right have more open areas
            if (right_free_confidence > 3 || left_free_confidence > 3){
                increase_nav_heading(heading_increment_flight);
            }

            // Move waypoint forward
            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = OUT_OF_BOUNDS;
            }
            else if (obstacle_free_confidence == 0){
                navigation_state = OBSTACLE_FOUND;
            }
            else {
                moveWaypointForward(WP_GOAL, moveDistance);
            }

            break;
        case OBSTACLE_FOUND:
            printf("OBSTACLE FOUND\n");
            // select new search direction depending on amount of green pixels in bottom corners and orange in corners
            chooseIncrementAvoidance();

            navigation_state = SEARCH_FOR_SAFE_HEADING;

            break;
        case SEARCH_FOR_SAFE_HEADING:

            printf("SEARCHING FOR SAFE HEADING\n");
            increase_nav_heading(heading_increment);

            // TODO: MAKE DRONE FLY FORWARD WHILE TURNING

            // make sure we have a couple of good readings before declaring the way safe
            if (obstacle_free_confidence >= 2){
                navigation_state = SAFE;
            }
            break;
        case OUT_OF_BOUNDS:
            printf("OUT OF BOUNDS\n");
            navigation_state = SEARCH_FOR_SAFE_HEADING;

            break;

        default:
            printf("DEFAULT\n");
            break;
    }
    return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
    float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

    // normalize heading to [-pi, pi]
    FLOAT_ANGLE_NORMALIZE(new_heading);

    // set heading
    nav_heading = ANGLE_BFP_OF_REAL(new_heading);

    //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
    return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
    float heading  = stateGetNedToBodyEulers_f()->psi;

    // Now determine where to place the waypoint you want to go to
    new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
    new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
    //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
    //              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
    //              stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
    return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
    //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                  //POS_FLOAT_OF_BFP(new_coor->y));
    waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
    return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
    struct EnuCoor_i new_coor;
    calculateForwards(&new_coor, distanceMeters);
    moveWaypoint(waypoint, &new_coor);
    return false;
}

/*
 * Sets the variable 'heading_increment' positive/negative based on the amount of pixels to the left/right
 */
uint8_t chooseIncrementAvoidance(void)
{
    // choose CW or CCW avoiding direction
    int32_t left_score  = left_green_count  - left_orange_count;
    int32_t right_score = right_green_count - right_orange_count;
    printf("Left: %" PRIu32 " - %" PRIu32 " = %" PRId32 " \nRight: %" PRIu32 " - %" PRIu32 " = %" PRId32 "\n", left_green_count, left_orange_count, left_score, right_green_count, right_orange_count, right_score);
    if (left_score < right_score) {
        heading_increment = 5.f;
        printf("%" PRId32 " < %" PRId32 ":\tTurning Right\n", left_score, right_score);
        //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
    } else {
        heading_increment = -5.f;
        printf("%" PRId32 " > %" PRId32 ":\tTurning Left\n", left_score, right_score);
        //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
    }
    return false;
}
