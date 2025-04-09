#include "FreeRTOS.h"
#include "task.h"

// #include "led.h"

// void appMain() {
//     while (1) {
//         ledSet(LED_RED_L, true);
//         vTaskDelay(pdMS_TO_TICKS(100)); 
//         ledSet(LED_RED_L, false);
//         vTaskDelay(pdMS_TO_TICKS(100)); 
//     }
// }

#include "debug.h"

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
  
    while(1) {
      vTaskDelay(M2T(2000));
      DEBUG_PRINT("Hello World!\n");
    }
  }