// CC-by-www.Electrosmash.com
// Based on OpenMusicLabs previous works.
// clean_pedalshield_uno.ino reads the ADC and plays it back into the PWM output
// simplifying for noobs

#define COM_AB 0b1010   // up match: clear, down match: set
#define WGM_PFC 0b1000  // phase & freq. correct mode
#define WGM_F 0b1110    // fast mode
#define T_CS 0b001      // timer clk. source: internal, prescaler: 1
#define TOP 0xff        // timer top value, pwm freq = f_clk(16MHz)/(prescaler * TOP)
// halved for phase & freq. correct mode
#define UP 0x1
#define DN 0x2
#define LED 0x3

#define REFS 0b01       // ADC Vref: AVcc
#define MUX 0b0000      // select ADC0
#define ADPS 0b100      // ADC clk. prescaler: 16
#define ADTS_CE 0b111   // ADC trigger: capture event
#define ADTS_OV 0b110   // ADC trigger: overflow

#define BT_MAX 0x400
#define FLAG_MAX 0x10

int input;
byte ADC_low, ADC_high;
byte bt_stat;
boolean up_flag = 1;
unsigned int bt_count = BT_MAX;
byte flag_count = FLAG_MAX;
int up_thresh = 2047;
int dn_thresh = -4096;

void setup() {
  // put your setup code here, to run once:
  // setup ADC
  byte ADTS = ADTS_CE;
  ADMUX = ((REFS << 6) | (1 << ADLAR) | MUX);
  ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | ADPS);
  ADCSRB = ADTS;
  DIDR0 = (1 << MUX);

  // setup TIMER
  byte WGM = ((0) | (WGM_PFC));
  TCCR1A = ((COM_AB << 4) | (WGM & 0b0011));
  TCCR1B = (((WGM & 0b1100) << 1) | (T_CS));
  TIMSK1 = 1 << ICIE1;    // Interrupt on Timer reaching TOP(capture event)
  //TIMSK1 = 1 << TOIE1;  // Interrupt on Timer reaching BOTTOM(overflow)
  ICR1H = (TOP >> 8);
  ICR1L = (TOP & 0xff);
  DDRB |= 0b00000110;

  // setup buttons & LED
  DDRC &= ~((1 << UP) | (1 << DN));
  DDRC |= (1 << LED);
  PORTC |= (1 << UP) | (1 << DN) | (up_flag << LED);
  sei();

}

void loop() {
  // put your main code here, to run repeatedly:
  
}

ISR(TIMER1_CAPT_vect)
{
  // get ADC data
  ADC_low = ADCL; // you need to fetch the low byte first
  ADC_high = ADCH;
  // construct the input sumple summing the ADC low and high byte.
  input = ((ADC_high << 8) | ADC_low) + 0x8000; // make a signed 16b value
  bt_count--;
  if (!bt_count) {
    bt_stat = PINC;

    if (!(bt_stat & (1 << UP)) && (!(bt_stat & (1 << DN)))) {
      flag_count--;
      if (!flag_count) {
        up_flag = !up_flag;
        up_flag ? PORTC |= (1 << LED) : PORTC &= ~(1 << LED);
        flag_count = FLAG_MAX;
      }
    }
    else if (!(bt_stat & (1 << UP))) {
      up_flag ? (up_thresh > 32766 ? up_thresh = 32767 : up_thresh += 32) : (dn_thresh > -1 ? dn_thresh = 0 : dn_thresh += 32);
    }
    else if (!(bt_stat & (1 << DN))) {
      up_flag ? (up_thresh < 1 ? up_thresh = 0 : up_thresh -= 32) : (dn_thresh < -32767 ? dn_thresh = -32768 : dn_thresh -= 32);
    }
    bt_count = BT_MAX;
  }
  input = input > 0 ? (input > up_thresh ? 32767 : input > up_thresh/2 ? 16383 : input > up_thresh/4 ? 8191 : input > up_thresh/8 ? 4095 : input) : 
                      (input < dn_thresh ? -32768 : input < dn_thresh/2 ? -16384 : input < dn_thresh/4 ? -8192 : input < dn_thresh/8 ? -4096 : input);

  //write the PWM signal
  OCR1AL = ((input + 0x8000) >> 8); // convert to unsigned, send out high byte
  OCR1BL = input; // send out low byte

}
