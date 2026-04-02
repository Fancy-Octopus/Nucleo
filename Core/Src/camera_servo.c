/*
 * camera_servo.c
 *
 *  Created on: May 4, 2025
 *      Author: gruetzmacherg
 */

#include <main.h>
#include "camera_servo.h"
#include "rover_controller.h"

#define CAM_IDLE_PULSEWIDTH   480
#define CAM_READY_PULSEWIDTH   1415


void InitCameraServo(void) {
  // 1. Enable GPIOA and TIM3 clocks (RCC)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  // Enable GPIOA clock
  RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;  // Enable TIM3 clock
  //RCC-> |= RCC_APB1ENR_TIM3EN;   // Enable TIM3 clock

  // 2. Configure PA6 as TIM3_CH1 (AF2)
  GPIOA->MODER &= ~GPIO_MODER_MODE6;          // Clear mode bits
  GPIOA->MODER |= (2 << GPIO_MODER_MODE6_Pos); // Alternate function mode
  GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL6_Pos); // AF2 for TIM3_CH1 (PA6)

  // 3. Configure TIM3 for PWM (50Hz, 1ms pulse)
  TIM3->PSC = 63;      // Prescaler: 200MHz / (199 + 1) = 1 MHz timer clock
  TIM3->ARR = 19999;    // Auto-reload: 1MHz / 20000 = 50Hz (20ms period)
  TIM3->CCR1 = CAM_IDLE_PULSEWIDTH;    // Pulse width: 1ms (1000 ticks @ 1MHz)

  // 4. Configure PWM mode (Channel 1)
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 (OC1M=110)
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for CCR1
  TIM3->CCER |= TIM_CCER_CC1E;    // Enable output for channel 1
  TIM3->CCER &= ~TIM_CCER_CC1P;   // Active high polarity

  // 5. Enable timer and auto-reload preload
  TIM3->CR1 |= TIM_CR1_ARPE;      // Auto-reload preload enabled
  TIM3->CR1 |= TIM_CR1_CEN;       // Start timer
}

void CameraServoPoll(void) {
  switch(CurrentRoverState()) {
    case ROVER_IDLE:
      TIM3->CCR1 = CAM_IDLE_PULSEWIDTH;
      break;
    case ROVER_READY:
      TIM3->CCR1 = CAM_READY_PULSEWIDTH;
    break;
  }
}


