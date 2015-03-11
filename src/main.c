/**
 * \file main.c
 *
 * \date 11 mars 2015
 * \author ldo
 */

/**
 * PORTA : ANA input
 * PORTB : JTAG + ANA inputs
 * PORTC : Communication
 * 	PC0 (SDA) : i2c communication
 * 	PC1 (SCL) : i2c communication
 * 	PC2 (RX) :
 * 	PC3 (TX) :
 * 	PC6 (RX1) : Lidar communication
 * 	PC7 (TX1) :
 * PORTD : Communication
 * 	PD2 (RX2) : MotherBoard communication
 * 	PD3 (TX2) :
 * 	PD4 : DIR1
 * 	PD5 : DIR2
 * 	PD6 : DIR3
 * PORTE : Timer + PWM
 * 	PE0 (OC0A) : PWM1
 * 	PE1 (OC0B) : PWM2
 * 	PE2 (OC0C) : PWM3
 * 	PE3 (OC0D) :
 * 	PE4 : encoder A1
 * 	PE5 : encoder B1
 * PORTF : Timer decoder quadrature
 * 	PF0 : encoder A2
 * 	PF1 : encoder B2
 * 	PF4 : encoder A3
 * 	PF5 : encoder B3
 * PORTH : Numeric port
 * PORTJ :
 * PORTK :
 * PORTQ : TOSC
 * PORTR : PDI + XTAL
 *
 * use TCF0 to decode quadrature counter 1
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <xmega_timer.h>

/**
 * \fn void clock_setup (void)
 * \brief setup system clock
 *
 */
void
clock_setup (void)
{
  /* Configuration change protection : Protected IO register
   * disable automatically all interrupts for the next four CPU instruction cycles */
  CCP = CCP_IOREG_gc;

  /* Oscillator : 32MHz Internal Oscillator Enable */
  OSC.CTRL = OSC_RC32MEN_bm;

  /* Wait for the internal 32 MHz RC oscillator to stabilize */
  while (!(OSC.STATUS & OSC_RC32MRDY_bm))
    ;

  /* Configuration change protection : Protected IO register */
  CCP = CCP_IOREG_gc;

  /* Clock : 32MHz Internal Oscillator */
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc; //CLK.CTRL = CLK_SCLKSEL_RC2M_gc;
}

/**
 * \fn void interrupt_setup (void)
 */
void
interrupt_setup (void)
{
  /* Programmable Multilevel Interrupt Controller */
  PMIC.CTRL |= PMIC_LOLVLEN_bm; /* Low-level Interrupt Enable */

  /* global interrupt enable */
  sei ();
}

/**
 * \fn void pin_setup (void)
 */
void
pin_setup (void)
{

  /* Configure PE0, PE1 and PE2 as output pin */
  PORTE.DIRSET = PIN0_bm;
  PORTE.DIRSET = PIN1_bm;
  PORTE.DIRSET = PIN2_bm;

}

/**
 *
 */
void
setup (void)
{
  clock_setup ();
  pin_setup ();
  interrupt_setup ();

  /* setup frequency waveform generation (PWM) */
  xmega_timer_0_pwm_mode_setup (&TCE0, 200, TC_CLKSEL_DIV8_gc); // 20KHz
  xmega_timer_0_pwm_enable (&TCE0, 0); // PE0
  xmega_timer_0_pwm_enable (&TCE0, 1); // PE1
  xmega_timer_0_pwm_enable (&TCE0, 2); // PE2
}

int
main (void)
{
  setup ();

  while (1)
    {
      xmega_timer_0_pwm_duty_cycle (&TCE0, 0, 100);
      xmega_timer_0_pwm_duty_cycle (&TCE0, 1, 50);
      xmega_timer_0_pwm_duty_cycle (&TCE0, 2, 25);
    }
}
