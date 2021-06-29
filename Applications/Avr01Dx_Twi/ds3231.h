#pragma once
//======================================================================
//  ds3231.h
//======================================================================
#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
//#include <util/delay.h>

bool ds3231_seconds      (uint8_t* seconds); //return true = success
