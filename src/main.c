#include "FreeRTOS.h"
#include "task.h"

#include "led.h"

#include "log.h"

static float meu_valor = 0.0f;

static void ledRedTask(void *param)
{
  while (1)
  {
    ledSet(LED_RED_L, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    ledSet(LED_RED_L, false);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void ledGreenTask(void *param)
{
  while (1)
  {
    ledSet(LED_GREEN_L, true);
    vTaskDelay(pdMS_TO_TICKS(200));
    ledSet(LED_GREEN_L, false);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

#include "debug.h"

static void printTask(void *param)
{
  while (1)
  {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Hello World!\n");
    meu_valor += 0.1f;
  }
}

void appMain()
{
  xTaskCreate(ledRedTask, "RedLED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(ledGreenTask, "GreenLED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(printTask, "PrintTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  LOG_GROUP_START(meulog)
  LOG_ADD(LOG_FLOAT, valor, &meu_valor)
  LOG_GROUP_STOP(meulog)
}