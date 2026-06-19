/*
 * keg_dispense.c
 *
 *  Created on: Nov 4, 2025
 *      Author: eostroff_enyuare
 */

#include "keg_dispense.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim15;  // for timer IRQs

static keg_dispense_ctx_t g_keg = {0};

void keg_dispense_on_timer_irq(void) {
  if (!g_keg.active) return;
  if ((HAL_GetTick() - g_keg.t0_ms) >= g_keg.duration_ms) {
    HAL_GPIO_WritePin(g_keg.fet_port, g_keg.fet_pin, GPIO_PIN_RESET); // or SET if active-low
    g_keg.active = 0;
    HAL_TIM_Base_Stop_IT(&htim15);  // disable tim17 updates
    (void)async_evt_push_isr(g_keg.slot, ASYNC_RES_OK);
  }
}

bool keg_dispense_start(GPIO_TypeDef *fet_port, uint16_t fet_pin,
                        uint32_t duration_ms, uint8_t async_slot)
{
  if (!fet_port || duration_ms == 0 || g_keg.active) return false;
  g_keg.active = 1;
  g_keg.fet_port = fet_port;
  g_keg.fet_pin  = fet_pin;
  g_keg.duration_ms = duration_ms;
  g_keg.t0_ms = HAL_GetTick();
  g_keg.slot  = async_slot;

  // enable tim17 updates
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_GPIO_WritePin(fet_port, fet_pin, GPIO_PIN_SET);   // ON (flip if active-low)
  return true;
}

void keg_dispense_abort(void) {
  if (!g_keg.active) return;
  HAL_GPIO_WritePin(g_keg.fet_port, g_keg.fet_pin, GPIO_PIN_RESET);
  g_keg.active = 0;
  (void)async_evt_push_isr(g_keg.slot, ASYNC_RES_ABORTED);
}


bool keg_dispense_busy(void) {
  return g_keg.active != 0;
}
