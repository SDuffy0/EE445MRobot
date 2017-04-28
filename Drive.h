// Drive.h
// Runs on TM4C123
#ifndef __DRIVE_H
#define __DRIVE_H  1

#define MAXSPEED 127

void Drive_Init(void);
void Drive_WheelDirection(int8_t dir);
void Drive_Speed(int8_t speed);
void Drive(int8_t speed, int8_t dir);
void Drive_PIControlDirection(int32_t E);
void Drive_Stop(void);
#endif
