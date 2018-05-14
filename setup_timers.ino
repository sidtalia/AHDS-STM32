void setup_esc_control()
{  
  pinMode(PA8, PWM);
  pinMode(PA11, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
  
  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = 0;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = CLOCK_SPEED;
  TIMER4_BASE->ARR = 2500;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;

  TIMER1_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER1_BASE->CR2 = 0;
  TIMER1_BASE->SMCR = 0;
  TIMER1_BASE->DIER = 0;
  TIMER1_BASE->EGR = 0;
  TIMER1_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE;
  TIMER1_BASE->CCMR2 = (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER1_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC4E;
  TIMER1_BASE->PSC = CLOCK_SPEED;
  TIMER1_BASE->ARR = 2500;
  TIMER1_BASE->DCR = 0;
  TIMER1_BASE->CCR1 = 1000;
  TIMER1_BASE->CCR4 = 1000;
}

void setup_receiver_channels()
{
  Timer2.attachCompare1Interrupt(handler_channel_1);//the timer 2 has it's first pin connected to PA0
  Timer2.attachCompare2Interrupt(handler_channel_2);//second pin to PA1
  Timer2.attachCompare3Interrupt(handler_channel_3);//and so on
  Timer2.attachCompare4Interrupt(handler_channel_4);//note that timer 2 has only 4 pre-defined interrupt channels.
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = CLOCK_SPEED;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;
}

