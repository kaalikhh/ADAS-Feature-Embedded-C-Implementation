#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include<math.h>
#include<stdbool.h>
#include<stdlib.h>
#include"allhfile.h"
#define SET_BIT(PORT,PIN)  PORT |=  (1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT &= ~(1<<PIN)
#define READ_BIT1(PORT,BIT) !(PORT & (1<<BIT))// traffic_sign_rajan
#define READ_BIT2(PORT,PIN) PORT & (1<<PIN)//ABS
#define INTERRUPT_INIT  SREG |= (1<<7)
#define ENGINE_PIN PD2
#define modeSwitch_PIN PD4
#define HDC_Display PB0
#define Potentio_PIN PC0
#define PWM_OUTPUT_Switch PD6
#define ENGINE_FLAG(PIN) ((PIND & (1<<PIN)))
#define MODE_SELECT_FLAG(PIN) ((PIND & (1<<PIN)))
#define ENGINE_FLAG(PIN) ((PIND & (1<<PIN)))
#define AIR_BAG(PIN) ((PIND & (1<<PIN)))
#define ACC_FLAG(PIN) ((PIND & (1<<PIN)))
#define ENGINE_PIN1  PD3
#define AIR_BAG_PIN PD2
#define ACC_SW PD4
#define PWM_PIN PD6
#define BRAKE_PIN PB2

void lkas_system(void);
void CA_SYSTEM(void);
void traffic_sign(void);
void ABS_system(void);
void hdc_control(void);
void acc_system(void);
void drowsiness(void);

typedef uint16_t ut16;
typedef uint8_t ut8;
ut16 Timer_Counter=0;
//static ut16 Final=0;
ut16 cur_speed=15;

typedef enum {
 cruise_ctrl_mode,
 ACC_off_mode
}MODE_SELECT;
MODE_SELECT mode=ACC_off_mode;

typedef enum {
engineOFF,
autoMode,
}MODE_SELECT1;
MODE_SELECT1 mode1=engineOFF;

typedef enum{
	RED_STATE,
  	YELLOWSTATE,    GREEN_STATE,
}STATE;
    int noInterupt=0;
	int interupt1=1;
  	int interupt2=2;
STATE state = RED_STATE;
STATE next_state;
uint16_t Timer_overflow_count=0;
volatile uint8_t FLAG=0;
float velocity     = 0;
float w_actual     = 30;
float w_required   = 0;
float wheel_radius = 0.3;
float slip         = 0;
int Input_ADC_value= 0;


int main(){

CLR_BIT(DDRD,PD2);
SET_BIT(PORTD,PD2);

CLR_BIT(DDRD,PD0);
SET_BIT(PORTD,PD0);

CLR_BIT(DDRD,PD1);
SET_BIT(PORTD,PD1);

CLR_BIT(DDRD,PD3);
SET_BIT(PORTD,PD3);

CLR_BIT(DDRD,PD4);
SET_BIT(PORTD,PD4);

CLR_BIT(DDRD,PD5);
SET_BIT(PORTD,PD5);

CLR_BIT(DDRD,PD7);
SET_BIT(PORTD,PD7);

SET_BIT(DDRB,PB2);
SET_BIT(DDRB,PB0);
SET_BIT(DDRB,PB6);
SET_BIT(DDRD,PD6);

CLR_BIT(DDRC,PC0);
CLR_BIT(DDRC,PC1);
CLR_BIT(DDRC,PC2);
CLR_BIT(DDRC,PC3);



CLR_BIT(DDRB,PB3);
SET_BIT(PORTB,PB3);
CLR_BIT(DDRB,PB4);
SET_BIT(PORTB,PB4);
CLR_BIT(DDRB,PB5);
SET_BIT(PORTB,PB5);

while(1)
{

if(!(PINB&(1<<PB3))&&!(PINB&(1<<PB4))&&!(PINB&(1<<PB5))){
    lkas_system();
}
else if(!(PINB&(1<<PB3))&&!(PINB&(1<<PB4))&&(PINB&(1<<PB5))){
   CA_SYSTEM();
}
else if(!(PINB&(1<<PB3))&&(PINB&(1<<PB4))&&!(PINB&(1<<PB5))){
    traffic_sign();
}
else if(!(PINB&(1<<PB3))&&(PINB&(1<<PB4))&&(PINB&(1<<PB5))){
    ABS_system();
}
else if((PINB&(1<<PB3))&&!(PINB&(1<<PB4))&&!(PINB&(1<<PB5))){
    hdc_control();
}
else if((PINB&(1<<PB3))&&!(PINB&(1<<PB4))&&(PINB&(1<<PB5))){
    acc_system();
}
else if((PINB&(1<<PB3))&&(PINB&(1<<PB4))&&!(PINB&(1<<PB5))){
    drowsiness();
}



}
    return 0;
}
/* ------------------- TIMER OVERFLOW INTERUPT ----------------------*/
ISR(TIMER0_OVF_vect)
{
    cli();
    Timer_overflow_count++;          // Incrementing the timer count
    if(Timer_overflow_count>=977)    // 977 for 1 sec delay
    {
        Timer_overflow_count=0;
        FLAG=1;
    }
    SREG |= (1<<7);
}
/**LKAS SYSTEM START*/

void lkas_system(void)
{
float cm1,cm2,speed;
float yaw_angle;
servo_init();
InitADC();


    while(1)
    {

        if(!(PIND&(1<<PD2))){


         SET_BIT(PORTB,PB0);
         yaw_angle=(45.0*ReadADC(2))/256.0 - 90;
         cm1=(ReadADC(0)*80.0)/1023.0;
         cm2=(ReadADC(1)*80.0)/1023.0;
         speed = (ReadADC(3)*180.0/1023.0);


  if(cm1>=40 && yaw_angle!=0 && speed>=60){
    if(cm2<40){
           if(yaw_angle>0){
          correction(yaw_angle);
          }

          if(yaw_angle<0){
           servo(90);
           _delay_ms(500);
           servo(90);
           _delay_ms(500);
          }
    }
    if(cm2>=40) {
            CLR_BIT(PORTB,PB2);
            CLR_BIT(PORTB,PB0);
            servo(90);
           _delay_ms(500);
            servo(90);
            _delay_ms(500);
    }
  }

  if(cm2>=40 && yaw_angle!=0 && speed>=60){
    if(cm1<40){
             if(yaw_angle<0){
           correction(yaw_angle);
           }
           if(yaw_angle>0){
               servo(90);
               _delay_ms(500);
                servo(90);
                _delay_ms(500);
           }
    }
    if(cm1>=40) {
            CLR_BIT(PORTB,PB2);
            CLR_BIT(PORTB,PB0);
           servo(90);
           _delay_ms(500);
            servo(90);
            _delay_ms(500);
    }
        }else{

        CLR_BIT(PORTB,PB2);
        servo(90);
        _delay_ms(500);
        servo(90);
        _delay_ms(500);

        }
        }
        else {
        servo(90);
        _delay_ms(500);
        servo(90);
        _delay_ms(500);
        CLR_BIT(PORTB,PB2);
        CLR_BIT(PORTB,PB0);
        }

    }

}


/**COLLISION AVOIDANCE SYSTEM START - SAI KRISHNA */

void CA_SYSTEM(void)
{
    uint16_t distance=0;
    uint16_t speed=0;
    uint16_t Front_speed=0;


    adc_cas_init();

    while(1)
    {
        if(!(PIND&(1<<PD2))) // Input as engine on
        {
          speed = READ_adc_cas(3);
          Front_speed = READ_adc_cas(2);

          if(speed > Front_speed)
          {
             distance = READ_adc_cas(0);

             if(speed>=25.575 && speed<=255.75) // 25.575--5KMPH = 0.375v    and   255.75--50KMPH= 1.25v
             {
                 if(distance<=51.15)  //  51.15-- 5METERS =  0.25v
                 {
                    SET_BIT(PORTB,PB0);
                    SET_BIT(PORTB,PB2);
                 }
                 else if(distance<=102.3) // 102.3-- 10METERS = 0.5v
                 {
                    SET_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
                 else
                 {
                    CLR_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
             }
             else if(speed>=260.865 && speed<=511.5) // 260.865--51KMPH = 1.275v  and   511.5--100KMPH = 2.5v
             {
                 if(distance<=102.3)     //  102.3-- 10METERS = 0.5v
                 {
                    SET_BIT(PORTB,PB0);
                    SET_BIT(PORTB,PB2);
                 }
                 else if(distance<=204.6) //  204.6-- 20METERS = 1v
                 {
                    SET_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
                 else
                 {
                    CLR_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
             }
             else if(speed>=516.615 && speed<=767.25) // 516.615--101KMPH = 2.525v  and  767.25--150KMPH = 3.75v
             {
                 if(distance<=204.6)     // 204.6-- 20METERS = 1v
                 {
                     SET_BIT(PORTB,PB0);
                     SET_BIT(PORTB,PB2);
                 }
                 else if(distance <= 306.9) // 306.9-- 30METERS = 1.5v
                 {
                     SET_BIT(PORTB,PB0);
                     CLR_BIT(PORTB,PB2);
                 }
                 else
                 {
                    CLR_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
             }
             else if(speed>=772.365 && speed<=1023) // 772.365--151KMPH = 3.775v  and 1023--200KMPH = 5v
             {
                 if(distance<=306.9)  // 306.9-- 30METERS = 1.5v
                 {
                     SET_BIT(PORTB,PB0);
                     SET_BIT(PORTB,PB2);
                 }
                 else if(distance<=409.2) // 409.2-- 40METERS = 2v
                 {
                     SET_BIT(PORTB,PB0);
                     CLR_BIT(PORTB,PB2);
                 }
                 else
                 {
                    CLR_BIT(PORTB,PB0);
                    CLR_BIT(PORTB,PB2);
                 }
             }
             else
             {
                 CLR_BIT(PORTB,PB0);
                 CLR_BIT(PORTB,PB2);
             }
          }
          else
          {
              CLR_BIT(PORTB,PB0);
              CLR_BIT(PORTB,PB2);
          }
        }
        else
        {
        CLR_BIT(PORTB,PB0); // WARNING INDICATION AS LED OFF
        CLR_BIT(PORTB,PB2); // AUTO BRAKING AS LED OFF
        }

    }


}

/**TRAFFIC SIGNNAL - RAJAN */

void traffic_sign()
{
    CLR_BIT(DDRD,PD0);// switch1
  	SET_BIT(PORTD,PD0);//switch 1
  	SET_BIT(PORTD,PD1);// switch 2
  	CLR_BIT(DDRD,PD1); // switch 2
  	SET_BIT(PORTD,PD4);// camera switch
  	CLR_BIT(DDRD,PD4); // camera switch

  	SET_BIT(DDRB,PB0);//led1
  	SET_BIT(DDRB,PB2);//led2
  	SET_BIT(DDRB,PB6);// led3

while(1){
        if(READ_BIT1(PIND,PD4)) // camera switch
        {
            SET_BIT(PORTB,PB2);//led2
            CLR_BIT(PORTB,PB6);//led3

        }
        else
        {
            CLR_BIT(PORTB,PB2);//led2
            SET_BIT(PORTB,PB6);//led3
        }

   int event=noInterupt;
if(READ_BIT1(PIND,PD0)) // switch 1
    {
      event = interupt1;


}
if(READ_BIT1(PIND,PD1)) // switch2
{
      event = interupt2;

}

    if(state == RED_STATE && event == noInterupt){
    	next_state = RED_STATE;
      redState_handler();
    }
    if(state == RED_STATE && event == interupt1){

      	next_state = YELLOWSTATE;
        yellowStatehandler();

    }
     else if(state == YELLOWSTATE && event == interupt2){

      	next_state = GREEN_STATE;
        greenState_handler();


    }
   else if(state == GREEN_STATE && event == interupt1)
   {
      	next_state = RED_STATE;
        yellowStatehandler();
      	event = noInterupt;

    }
    state = next_state;




}

  }
/**drowsiness */
void drowsiness()
        {
            ut16 (*ADCFnPtr)() = READ_ADC_Drowsiness;  // Function Pointer
        static ut16 Final=0;
        SREG |= (1<<7);
    while(1)         // infinite loop
    {
        if(!(PIND & 1<<PD2))  // when engine is on
        {

            if(!(PIND & 1<<PD3))
            {
                    Final = ADCFnPtr();

                if(Final > 750)
                    OCR0A = 63;
                else if(Final >= 300 && Final < 750)
                    OCR0A = 127;
                //else if(Final < 300)
                   else  OCR0A = 191;
                //Interrupt_State = 0;
                   // }
            }
            else
            {
                if(!(PIND & 1<<PD4))
                    { //High beam
                    OCR0A = 63;
                    }
                else if(!(PIND & 1<<PD5))
                    {
                    OCR0A = 127; //Low Beam
                    }
                else if(!(PIND & 1<<PD7))
                    {
                    OCR0A = 191; //Low Beam
                    }
                else
                {
                    //TIMSK0 &= ~(1<<TOIE0);
                    OCR0A = 0;
                }

            }
        }
        else
        {
            //TIMSK0 &= ~(1<<TOIE0);
            OCR0A = 0;
        }

      }

}

/** ARCHIT **/

void hdc_control(void)
{
uint16_t read_p=0;
ADC_Setup_Func1();
pwmSetup_Func1();
main_Setup_Func1();
HDCmode_Func();
while(1){

if(ENGINE_FLAG(ENGINE_PIN)==0x00)
  {
          ADCSRA|=(1<<ADSC);
          while(ADCSRA & (1<<ADSC));
          read_p=ADC/4;

      if(MODE_SELECT_FLAG(modeSwitch_PIN)==0x00)
        {
         if(read_p>100){
            OCR0A = 30;
         } else {
             OCR0A = read_p;
             }
         SET_BIT(PORTB,HDC_Display);
        }
      else
         {
       OCR0A = read_p;
         CLR_BIT(PORTB,HDC_Display);
         }
    }
else{
     OCR0A=0;
      CLR_BIT(PORTB,HDC_Display);}


}

}

/** ACC - Pradipto  */

void acc_system(void)
{
    // uint8_t itr=0;
    pwmSetup_Func();
    main_Setup_Func();
    ADC_Setup_Func();

    while(1){
     if(ENGINE_FLAG(ENGINE_PIN1)==0x00 && AIR_BAG(AIR_BAG_PIN)==0x00 && ACC_FLAG(ACC_SW)==0x00){
        dis=read_Dis();//pc1 ego_Spd
        ego_Spd=read_Ego_Spd();
        fv_Spd=read_Fv_Spd();

        cruise_Ctrl();
        mode=cruise_ctrl_mode;
     }
     else{
        mode=ACC_off_mode;
     }
    }
}

/** ABS -- NIKITA */

void ABS_system(void)
{
    CLR_BIT(DDRD,PD2);      // making PD2 as Input - ENGINE SWITCH
    SET_BIT(PORTD,PD2);     //PULL UP Enabled
    CLR_BIT(DDRD,PD3);      // making PD3 as Input - AUTO/MANUAL
    SET_BIT(PORTD,PD3);     //PULL UP Enabled
    SET_BIT(DDRB,PB1);      // making PB1 as Input - HIGH BEAM
    //CLR_BIT(PORTB,PB1);     //PULL UP Enabled
    SET_BIT(DDRB,PB2);      // making PB2 as Input - LOW BEAM
    //SET_BIT(PORTB,PB2);     //PULL UP Enabled
    SET_BIT(DDRB,PB3);
    ADC_initialize();
    ENABLE_ADC;

    TIMER0_initialize();

    PWM_initialize();

    INTERRUPT_INIT;

    while(1)
    {
        if(!(READ_BIT2(PIND,PD2)))
        {
/*--------------------------- AUTO MODE ---------------------------------------*/
            if(!(READ_BIT2(PIND,PD3)))
            {
                START_TIMER;                            // ENABLE THE TIMER INTERRUPT
                if(FLAG==1)                             // AFTER COMPLETING 1 SEC
                {
                    Input_ADC_value = READ_ADC(0);       // Read ADC value
                    velocity = 67*Input_ADC_value/1024;
                    Input_ADC_value = READ_ADC(1);
                    w_actual = 150*Input_ADC_value/1024;
                    w_required = velocity/wheel_radius;
                    slip = 1-(w_actual/w_required);

                    if(slip<=0.3)                     // HIGH LIGHT   - 75% INTENSITY
                        {
                        SET_PWM_VALUE(64);
                        CLR_BIT(PORTB, PB2);
                        CLR_BIT(PORTB, PB3);
                        SET_BIT(PORTB, PB1);
                        _delay_ms(500);
                        }
                    else if(slip>0.3 && slip<0.7)   // MEDIUM LIGHT - 65% INTENSITY
                        {
                        SET_PWM_VALUE(128);
                        CLR_BIT(PORTB, PB1);
                        CLR_BIT(PORTB, PB3);
                        SET_BIT(PORTB, PB2);
                        _delay_ms(500);
                        }
                    else                            // LOW LIGHT     - 50% INTENSITY
                        {
                        SET_PWM_VALUE(192);
                        CLR_BIT(PORTB, PB1);
                        CLR_BIT(PORTB, PB2);
                        SET_BIT(PORTB, PB3);
                        _delay_ms(500);
                        }

                    FLAG=0;
                }
            }

        else
        {
            STOP_TIMER;
            SET_PWM_VALUE(0);
        }
      }

}
}









