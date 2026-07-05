#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions ------------------------------------------------------*/
void Robot_Init();
void Robot_Start();
void Robot_Execute();
void Robot_Stop();

#ifdef __cplusplus
}
#endif

#endif
