/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

#include <stdint.h>


// settings
extern float green_color_count_frac;        // Green colour fraction threshold
extern float orange_color_count_frac;       // Orange colour fraction threshold

//extern int16_t obstacle_free_confidence;
//extern int16_t left_free_confidence;
//extern int16_t right_free_confidence;
extern float central_coefficient;           // Weight determining how much importance should be give to the central part of the image
extern int16_t n_trajectory_confidence;     // Forward trajectory confidence threshold
extern int16_t n_turning_confidence;        // Turning trajectory confidence threshold

extern uint32_t left_green_count;           // Green pixels in bottom left
extern uint32_t right_green_count;          // Green pixels in bottom right
extern uint32_t left_orange_count;          // Orange pixels in bottom left
extern uint32_t right_orange_count;         // Orange pixels in bottom right

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

