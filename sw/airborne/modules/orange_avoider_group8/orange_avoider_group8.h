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
extern float green_color_count_frac;
extern float orange_color_count_frac;

//extern int16_t obstacle_free_confidence;
//extern int16_t left_free_confidence;
//extern int16_t right_free_confidence;
extern float central_coefficient;
extern int16_t n_trajectory_confidence;
extern int16_t n_turning_confidence;

extern uint32_t left_green_count;
extern uint32_t right_green_count;
extern uint32_t left_orange_count;
extern uint32_t right_orange_count;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

