#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* C-linkage entry points for main.c. app.cpp routes them to the application
   selected by FIRMWARE_MODE (configuration.h): the Robot in NORMAL mode, or
   a single self-contained debug environment in the DEBUG_* modes. */
void App_Init(void);
void App_Start(void);
void App_Execute(void);
void App_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
