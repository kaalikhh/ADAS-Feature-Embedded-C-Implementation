#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#define SET_BIT(PORT,PIN)  PORT |=  (1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT &= ~(1<<PIN)
#include"allhfile.h"
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

/**LKAS */
void servo(int degrees){

int temp = ((7.8*degrees)/90);
OCR1A= temp + 15;

}

void correction(float yaw_angle){
    SET_BIT(PORTB,PB2);
    servo(90-yaw_angle);
    _delay_ms(500);
    servo(90-yaw_angle);
     _delay_ms(500);
    }

void InitADC()
{
 // Select Vref=AVcc
 ADMUX |= (1<<REFS0);
 //set prescaller to 128 and enable ADC
 ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}


uint16_t ReadADC(uint8_t ADCchannel)
{
 //select ADC channel with safety mask
 ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
 //single conversion mode
 ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );
 return ADC;
}

void servo_init(){
TCCR1A|=(1<<WGM10);
TCCR1A&=~(1<<WGM11);
TCCR1B|=(1<<WGM12);
TCCR1B&=~(1<<WGM13);
TCCR1B|=((1<<CS12)|(1<<CS10));
TCCR1B&=~(1<<CS11);
TCNT1H=0x00;
TCNT1L=0x00;
TCCR1A|=(1<<COM1A1);
TCCR1A&=~(1<<COM1A0);
DDRB|=(1<<PB1);


}

/** Collision avoidance*/

void adc_cas_init()
{
    ADMUX |= (1<<REFS0);
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

int READ_adc_cas(uint16_t ADCchannel)
{
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    ADCSRA |= (1<<ADSC);
    while( ADCSRA & (1<<ADSC) );
    return ADC;
}

/** RAJAN **/

void redState_handler()
{

    SET_BIT(PORTB,PB0); // led1

_delay_ms(100);
   CLR_BIT(PORTB,PB0);// led1
    _delay_ms(1000);

}
void yellowStatehandler()
{
    CLR_BIT(PORTB,PB0);//led1
       _delay_ms(1000);
    SET_BIT(PORTB,PB0);//led1
       _delay_ms(1000);

    CLR_BIT(PORTB,PB0);//led1
}
void greenState_handler()
{
     SET_BIT(PORTB,PB0); // led1
}

/** ABS */

void ADC_initialize()
{
    // Select Vref=AVcc
    ADMUX |= (1<<REFS0);
    //set prescaller to 128 and enable ADC
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}
int16_t READ_ADC(int8_t ADCchannel)
{
    //select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    //single conversion mode
    ADCSRA |= (1<<ADSC);
    // wait until ADC conversion is complete
    while( ADCSRA & (1<<ADSC) );
    return ADC;
}

void PWM_initialize()
{
    SET_BIT(DDRD,PD6);          // PD6 (OC0A)output
    TCNT0 =0x00;                // give access to timer and counter
    TCCR0A |= ((1<<WGM00)|(1<<WGM01)|(1<<COM0A1)); // enabled the mode and effect
    TCCR0B |= ((1<<CS00)|((1<<CS01))); //clock and WGM02
}
void TIMER0_initialize()
{
    TCNT0 =0x00;                // enables timer 0
    TCCR0A=0x00; //
    TCCR0B |= ((1<<CS00)|((1<<CS01)));
}

/***  ARCHIT */

void ADC_Setup_Func1(){
ADMUX&=0x00;
ADMUX|= (1<<REFS0);
ADCSRA|= (1<< ADEN);
}

void HDCmode_Func()
{
uint16_t read_PotenValue=0;
ADCSRA|=(1<<ADSC);
while(ADCSRA & (1<<ADSC));
read_PotenValue=ADC;

if(read_PotenValue>70)
    {

      OCR0A=30;
    }

}
void main_Setup_Func1(){

CLR_BIT(DDRD,Potentio_PIN);
CLR_BIT(DDRD,ENGINE_PIN);
SET_BIT( PORTD,ENGINE_PIN);
CLR_BIT(DDRD,modeSwitch_PIN);
SET_BIT(PORTD,modeSwitch_PIN);
SET_BIT(DDRD,PWM_OUTPUT_Switch);
SET_BIT(DDRB,HDC_Display);

}

void pwmSetup_Func1(){
SET_BIT(DDRD,PWM_OUTPUT_Switch);
TCCR0A|= ((1<<WGM01) | (1<<WGM00));
TCCR0A|=1<<COM0A1;
TCCR0A&=~(1<<COM0A0);
TCNT0=0x00;
OCR0A=0x00;
TCCR0B|=((1<<CS00)|(1<<CS02));
TCCR0B&=~(1<<CS01);
}

/** ACC- pradipto ***/

void ADC_Setup_Func(){
     ADMUX|=(1<<REFS0);
     ADMUX&=~(1<<REFS1);
     ADCSRA|= (1<< ADEN);
}

void cruise_Ctrl(){
    double temp1=0;
    double temp2=0;
    double temp=0;
    int8_t acclr=0;

    if(dis<set_Dis){
        if(fv_Spd>ego_Spd){
           temp1=(fv_Spd-ego_Spd);
           temp2=(set_Dis-dis);
           temp=floor(temp1*temp1/(2*temp2));
           acclr=(int8_t)(temp);
        }
        else if(fv_Spd<=ego_Spd && fv_Spd!=0){
            temp1=set_Dis/fv_Spd;
            temp2=ego_Spd*temp1;
            temp=-abs(ceil((temp2-dis)*2/(temp1*temp1)));
            acclr=(int8_t)(temp);

        }
        else{
            temp=-ceil((ego_Spd*ego_Spd)/(2*dis));
            acclr=(int8_t)(temp);
            if(acclr<=decc_Limit){
                SET_BIT(PORTB,PB1);  //collision detected
                SET_BIT(PORTB,PB3);
            }

        }

    }
    else{
      if(fv_Spd>=ego_Spd){
             temp1=set_Dis/fv_Spd;
            temp2=ego_Spd*temp1;
            temp=abs(ceil((temp2-dis)*2/(temp1*temp1)));
            acclr=(int8_t)(temp);
        }
        else if(fv_Spd<ego_Spd){
             temp1=(fv_Spd-ego_Spd);
             temp2=(set_Dis-dis);
             temp=floor(temp1*temp1/(2*temp2));
             acclr=(int8_t)(temp);
        }

    }

    if(acclr>max_acclr)
        acclr=max_acclr;
    if(acclr<0)
      OCR0A=floor((1+(acclr/8))*127);
    else{
      OCR0A=floor((1+(acclr/6))*127);
    }

}

void main_Setup_Func(){

  CLR_BIT(DDRD,ENGINE_PIN);
  CLR_BIT(DDRD,ACC_SW);
  CLR_BIT(DDRD,AIR_BAG_PIN);
  CLR_BIT(DDRC,PC0);
  CLR_BIT(DDRD,PC3);
  CLR_BIT(DDRD,PC2);

  SET_BIT(DDRD,PWM_PIN);
  SET_BIT(DDRB,BRAKE_PIN);


  SET_BIT(PORTD,ENGINE_PIN);
  SET_BIT(PORTD,ACC_SW);
  SET_BIT(PORTD,AIR_BAG_PIN);

  fv_Spd=0;
  ego_Spd=0;
  max_acclr=6; //6m/s^2
  decc_Limit=-8;
  set_Dis=40; //in 25m
  dis=0;
}
void pwmSetup_Func(){
        // PD6 (OC0A pin ) as output
       TCCR0A|= ((1<<WGM01) | (1<<WGM00));
       TCCR0A|=1<<COM0A1;
       TCCR0A&=~(1<<COM0A0);
       TCNT0=0x00;
       OCR0A=0x00;
       TCCR0B|=((1<<CS00)|(1<<CS02));// Clock selection 101
       TCCR0B&=~(1<<CS01); //101
}
uint8_t read_Dis(){
  uint16_t read_PotenValue=0;
  uint8_t dis=0;
  double temp=0;

  ADMUX&=0x00;
  ADMUX|= (1<<REFS0);
  ADCSRA|=(1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  read_PotenValue=ADC;
  temp=ceil(ADC*100/1023);
  dis=(uint8_t)(temp);

  return dis;
}
uint8_t read_Ego_Spd(){
    uint8_t spd=0;
    double temp=0;
    ADMUX&=0x00;
    ADMUX|= (1<<REFS0);
    ADMUX|=(1<<MUX0);
    ADCSRA|=(1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    temp=ceil(ADC*55/1023);
    spd=(uint8_t)(temp);
    return spd;
}
uint8_t read_Fv_Spd(){
    uint8_t spd=0;
    double temp=0;
    ADMUX&=0x00;
    ADMUX|= (1<<REFS0);
    ADMUX|=(1<<MUX1);
    ADCSRA|=(1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    temp=ceil(ADC*55/1023);
    spd=(uint8_t)(temp);
    return spd;
}

/****  Saurabh_drowsiness   **/
uint16_t READ_ADC_Drowsiness()
{
    CLR_BIT(DDRC,PC0);
    ADMUX  |= (1<<REFS0);
    ADCSRA |= (1 << ADSC);
    ADCSRA |= ((1<<ADPS1)|(1<<ADPS2)); // 64 pre scaler used
    while (ADCSRA & (1 << ADSC));
return ADC;
}

void PWM_Initialize()
{
    SET_BIT(DDRD,PD6); // PD6 (OC0A) output
    ADCSRA |= (1<< ADEN);
    TCNT0 =0x00;  // give access to timer and counter
    TCCR0A |= ((1<<WGM00)|(1<<WGM01)|(1<<COM0A1)); // enabled the mode and effect
    TCCR0B |= ((1<<CS00)|((1<<CS01))); //clock and WGM02
}

void TIMER_Initialize()
{
    TCNT0 =0x00; // enable timer 0
    TCCR0A=0x00; //
    TCCR0B |= ((1<<CS00)|((1<<CS01)));
}
