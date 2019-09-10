#define START_ADC  ADCSRA |= (1<<ADSC)
#define ENABLE_ADC ADCSRA |= (1<<ADEN)
#define START_TIMER TIMSK0|=(1<<TOIE0)
#define STOP_TIMER  TIMSK0 &= ~(1<<TOIE0)
#define SET_PWM_VALUE(value)  OCR0A = value
void main_Setup_Func(void);
volatile uint8_t max_Spd,fv_Spd,ego_Spd,max_acclr,set_Dis,dis,NOR_MAX_PWM;
volatile int8_t acclr,decc_Limit;
void pwmSetup_Func(void);
uint8_t read_Dis();
uint8_t read_Ego_Spd(void);
uint8_t read_Fv_Spd();
extern void correction(float);
extern void InitADC();
extern uint16_t ReadADC(uint8_t ADCchannel);
extern void servo_init();
extern void servo(int);
extern void adc_cas_init();
extern int READ_adc_cas();
extern void redState_handler();
extern void yellowStatehandler();
extern void greenState_handler();
void ADC_initialize();
int16_t READ_ADC(int8_t ADCchannel);
void PWM_initialize();
void TIMER0_initialize();
void ADC_Setup_Func1(void);
void HDCmode_Func();
void pwmSetup_Func1();
void main_Setup_Func1(void);
void ADC_Setup_Func(void);
void cruise_Ctrl(void );
uint16_t READ_ADC_Drowsiness(void);
void PWM_Initialize();
void TIMER_Initialize();
