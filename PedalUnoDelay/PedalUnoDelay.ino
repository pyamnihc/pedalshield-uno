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

#define REFS 0b01       // ADC Vref: AVcc
#define MUX 0b0000      // select ADC0
#define ADPS 0b100      // ADC clk. prescaler: 16
#define ADTS_CE 0b111   // ADC trigger: capture event
#define ADTS_OV 0b110   // ADC trigger: overflow

#define BT_MAX 0x400

#define DLEN 2000

int input;
byte ADC_low, ADC_high;
byte bt_stat;
unsigned int bt_count = BT_MAX;
byte prev_input[DLEN] = {0};
unsigned int dcount_max = DLEN;
unsigned int dcount = 0;

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
  PORTC |= (1 << UP) | (1 << DN);
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

  bt_count--;
  if (!bt_count) {
    bt_stat = PINC;

    if (!(bt_stat & (1 << UP))) {
      dcount_max >= DLEN ? dcount_max = DLEN : dcount_max += 20;
    }
    if (!(bt_stat & (1 << DN))) {
      dcount_max <= 0 ? dcount_max = 0 : dcount_max -= 20;
    }
    bt_count = BT_MAX;
  }
  
  prev_input[dcount] = ADC_high;
  dcount++;
  if (dcount >= dcount_max) dcount = 0;
  
  // construct the input sumple summing the ADC low and high byte.
  input = (((prev_input[dcount] << 8) | ADC_low) + 0x8000) + (((ADC_high << 8) | ADC_low) + 0x8000); // make a signed 16b value
  
  //write the PWM signal
  OCR1AL = ((input + 0x8000) >> 8); // convert to unsigned, send out high byte
  OCR1BL = 0; // send out low byte

}
