#include <avr/interrupt.h>

//  pb2----.-----.-----.-----.
//         |     |     |     |
//  pb1----|-----|-----|-----|
//         |     |     |     |
//  pb0----|-----|-----|-----|
//         |     |     |     |
//  pd7----|-----|-----|-----|
//         |     |     |     |
//        pb6   pb7   pb5   pd6

#define DELAY_REFRESH 512
//#define DELAY_REFRESH 1024
//#define DELAY_REFRESH 2048
//#define DELAY_REFRESH 4096

#define N_FRAME 37

char t;
volatile int16_t counter=0;
volatile int16_t counter1=0;
volatile char eo=0;

volatile uint16_t light_counter_lo;
volatile uint16_t light_counter_hi;

int rowpin[4] = { PB2, PB1, PB0, PD7 };
int colpin[4] = { PB6, PB7, PD5, PD6 };


// -------------
//
// SPI functions
//
// -------------


void spi_follower_init(void)
{

  // miso output, all others input
  //
  t = DDRB;
  t |= (1<<PB4);
  DDRB = t;

  // enable spi (slave?)
  //
  //SPCR = (1<<SPE) | (1<<SPIE);

}

// SPI slave can't initiate a message, it
// has to respond to a poll request.
//
// The master (leader) will send an SPI
// byte which will call this interrupt.
// The leader command message will
// be a simple one byte value and 
// the follower (this code) will
// respond with the appropriate message.
//
ISR(SPI_STC_vect) {
  unsigned char b;

  b = SPDR;

  if (b==0xaa) {
    SPDR = 0x55;
  }

  return;

  if      (b==1) {
    b = (unsigned char)(light_counter_lo&0xff);
    SPDR = b;
  }
  else if (b==2) {
    b = (unsigned char)((light_counter_lo>>8)&0xff);
    SPDR = b;
  }
  else if (b==3) {
    b = (unsigned char)(light_counter_hi&0xff);
    SPDR = b;
  }
  else if (b==4) {
    b = (unsigned char)((light_counter_hi>>8)&0xff);
    SPDR = b;
  }

}

/*
unsigned char spi_follower_rx_byte(void)
{
  while (!SPSR & (1<<SPIF));
  return SPDR;
}

unsigned char spi_follower_tx_byte(void)
{
  while (!SPSR & (1<<SPIF));
  return SPDR;
}
*/

// -------------
//
// SPI functions
//
// -------------



void reset_state(void) {
  PORTB = (1<<PB6) | (1<<PB7);
  PORTD = (1<<PD5) | (1<<PD6);
}

void activate_led(char r, char c) {

  // sink all row pins
  t = PORTB;
  t &= ~( (1<<PB2) | (1<<PB1) | (1<<PB0) );
  PORTB = t;
  t = PORTD;
  t &= ~( (1<<PD7) );
  PORTD = t;

  // sink the column pin
  //
  if (c<2) { t = PORTB; }
  else     { t = PORTD; }
  t &= ~( (1<<colpin[(int)c]) );
  if (c<2) { PORTB = t; }
  else     { PORTD = t; }


  // Source the pin in question
  if (r!=3) { t = PORTB; }
  else      { t = PORTD; }
  t |= (1<<rowpin[(int)r]);
  if (r!=3) { PORTB = t; }
  else      { PORTD = t; }

}

volatile char g_row = 0;
volatile char g_col = 0;
volatile char frame_copied = 0;

unsigned char on[2][2] = { {0b11111111, 0b11111111}, // @
                           {0b10000000, 0b00000001}, // '.
                         };


unsigned char edge_thing[2][2] = {
  {0b00100010, 0b00000100},
  {0b00110011, 0b00001100},
};

unsigned char edge_thing2[2][2] = {
  {0b11011101, 0b11111011},
  {0b00100010, 0b00000100},
};

unsigned char pulse_heart[3][2] = {
  {0b00000000, 0b01010010},
  {0b00000101, 0b01110010},
  {0b10011111, 0b11110110},
};


unsigned char ohs[7][2] = { {0b11111101, 0b10111111}, // O
                                {0b10011011, 0b11111001}, // H
                                {0b11111100, 0b00111111}, // S

                                // right justified
                                {0b01110001, 0b01100111}, // 2
                                {0b00110101, 0b01010111}, // 0
                                {0b00110111, 0b00110011}, // 1
                                {0b01110100, 0b00110111} // 5
                              };


char buf[4][4] = { { 0, 1, 0, 1 },
                   { 1, 0, 1, 0 },
                   { 0, 1, 0, 1 },
                   { 1, 0, 1, 0 } };

char back_buf[4][4] = { { 1, 0, 1, 0 },
                    { 0, 1, 0, 1 },
                    { 1, 0, 1, 0 },
                    { 0, 1, 0, 1 } };

// checkerboard
unsigned char anim0[2][2] = { {0b01011010, 0b01011010}, {0b10100101, 0b10100101} };

// glider
unsigned char anim1[5][2] = { {0b01000010, 0b11100000},
                              {0b00001010, 0b01100100},
                              {0b00000010, 0b10100110},
                              {0b00000100, 0b00110110},
                              {0b00000010, 0b00010111},
};

unsigned char beatanim[5][2] = {
  { 0b00000010, 0b01000000 },
  { 0b00000110, 0b01100000 },
  { 0b11111111, 0b11111111 },
  { 0b00000110, 0b01100000 },
  { 0b00000100, 0b00100000 },
};

unsigned char alphanum[38][2] = {
  { 0b11111011, 0b11111001 }, // A
  { 0b11101011, 0b11011111 }, // B
  { 0b11111100, 0b11001111 }, // C
  { 0b11101011, 0b10111110 }, // D
  { 0b11111100, 0b11101111 }, // E
  { 0b11111100, 0b11101100 }, // F
  { 0b11111000, 0b10111111 }, // G

  { 0b10011011, 0b11111001 }, // H
  { 0b11110110, 0b01101111 }, // I
  { 0b11110110, 0b01101110 }, // J
  { 0b10011110, 0b11101001 }, // K
  { 0b11001100, 0b11111111 }, // L

  { 0b11111111, 0b10111001 }, // M
  { 0b11011101, 0b10111011 }, // N
  { 0b11111101, 0b10111111 }, // O
  { 0b11111011, 0b11111000 }, // P
  { 0b11111101, 0b10111110 }, // Q
  { 0b11111011, 0b11101001 }, // R

  { 0b11111100, 0b00111111 }, // S
  { 0b11111111, 0b01100110 }, // T

  { 0b10111011, 0b11111111 }, // U
  { 0b10011011, 0b10110100 }, // V
  { 0b10011011, 0b11111111 }, // W
  { 0b10110111, 0b11101101 }, // X
  { 0b10111111, 0b01100110 }, // Y
  { 0b11110011, 0b11001111 }, // Z
  { 0b00000000, 0b00000000 }, // ' '
  { 0b00110011, 0b00000011 }, // !

  { 0b00110101, 0b01010111 }, // 0
  { 0b00110111, 0b00110011 }, // 1
  { 0b01110001, 0b01100111 }, // 2
  { 0b01110001, 0b00110111 }, // 3
  { 0b01000101, 0b01110011 }, // 4
  { 0b01110100, 0b00110111 }, // 5
  { 0b00110110, 0b01110111 }, // 6
  { 0b01110111, 0b00110011 }, // 7
  { 0b00110111, 0b01110111 }, // 8
  { 0b01110101, 0b00110011 }, // 9
};

unsigned char alpha[27][2] = { {0b11101010, 0b11101010},  // A
                               {0b10001110, 0b10101110},  // b
                               {0b00001110, 0b10001110},  // c
                               {0b00101110, 0b10101110},  // d
                               {0b11101010, 0b11001110},  // e
                               {0b01100100, 0b11100100},  // f
                               {0b11001100, 0b01001100},  // g
                               {0b10101010, 0b11101010},  // H
                               {0b01000000, 0b01000100},  // i
                               {0b01000000, 0b01001100},  // j
                               {0b10001010, 0b11001010},  // k
                               {0b01000100, 0b01000100},  // l
                               {0b00001000, 0b11111011},  // m
                               {0b00001000, 0b11101010},  // n
                               {0b11101010, 0b10101110},  // O
                               {0b11001100, 0b10001000},  // p
                               {0b11001100, 0b01000100},  // q
                               {0b00000100, 0b01100100},  // r
                               {0b11101000, 0b01001110},  // S
                               {0b01001110, 0b01000100},  // t
                               {0b00001010, 0b10101110},  // u
                               {0b00001010, 0b10100100},  // v
                               {0b00000000, 0b11011111},  // w
                               {0b00001010, 0b01001010},  // x
                               {0b10101110, 0b00101110},  // y
                               {0b11100100, 0b10001110},  // z
                               {0b00000000, 0b00000000},  // ' '
};


void load_back_buf(void) {
  buf[0][0] = back_buf[0][0];
  buf[0][1] = back_buf[0][1];
  buf[0][2] = back_buf[0][2];
  buf[0][3] = back_buf[0][3];

  buf[1][0] = back_buf[1][0];
  buf[1][1] = back_buf[1][1];
  buf[1][2] = back_buf[1][2];
  buf[1][3] = back_buf[1][3];

  buf[2][0] = back_buf[2][0];
  buf[2][1] = back_buf[2][1];
  buf[2][2] = back_buf[2][2];
  buf[2][3] = back_buf[2][3];

  buf[3][0] = back_buf[3][0];
  buf[3][1] = back_buf[3][1];
  buf[3][2] = back_buf[3][2];
  buf[3][3] = back_buf[3][3];
}

ISR(TIMER0_OVF_vect) {

  cli();

  counter1++;
  if (counter1 > DELAY_REFRESH) {
    frame_copied = 1;
    eo=1-eo;
    counter1=0;

    load_back_buf();
  }


  counter++;
  if (counter > 4)
  {

    reset_state();
    g_col++;
    if (g_col==4) { g_col=0; g_row++; }
    if (g_row==4) { g_row=0; }

    if (buf[(int)g_row][(int)g_col] == 1)  { activate_led(g_row, g_col); }

    counter=0;
  }

  sei();

}

void clear_back_buf(void) {
  back_buf[0][0] = 0; back_buf[0][1] = 0; back_buf[0][2] = 0; back_buf[0][3] = 0;
  back_buf[1][0] = 0; back_buf[1][1] = 0; back_buf[1][2] = 0; back_buf[1][3] = 0;
  back_buf[2][0] = 0; back_buf[2][1] = 0; back_buf[2][2] = 0; back_buf[2][3] = 0;
  back_buf[3][0] = 0; back_buf[3][1] = 0; back_buf[3][2] = 0; back_buf[3][3] = 0;
}

void copy_anim_to_back_buf(unsigned char *anim) {
  unsigned char r=0, c=0, i;

  for (i=0; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  for (i=0; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }
  

  for (i=0; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  for (i=0; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

}

void copy_anim_to_back_buf_shiftr(unsigned char *anim, unsigned char shiftr) {
  unsigned char r=0, c=0, i;

  c=shiftr; r=0;
  for (i=0; i<(4-shiftr); i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=shiftr; r=1;
  for (i=0; i<(4-shiftr); i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=shiftr; r=2;
  for (i=0; i<(4-shiftr); i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=shiftr; r=3;
  for (i=0; i<(4-shiftr); i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

}

void copy_anim_to_back_buf_shiftl(unsigned char *anim, unsigned char shiftl) {
  unsigned char r=0, c=0, i;

  c=0; r=0;
  for (i=shiftl; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=0; r=1;
  for (i=shiftl; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[0] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=0; r=2;
  for (i=shiftl; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(7-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

  c=0; r=3;
  for (i=shiftl; i<4; i++) {
    back_buf[(int)r][(int)c] = ( (anim[1] & (1<<(3-i))) ? 1 : 0 );
    c++; if (c==4) { c=0; r++; if (r==4) r=0; }
  }

}

//unsigned char msg[14] = "hello ohs2015";
//unsigned char msg[] = " hello rama";
unsigned char msg[] = "ohs2015";

unsigned char slen(unsigned char *m) {
  unsigned char x=0;
  for (x=0; x<127; x++) {
    if (m[x]==0) return x;
  }
  return x;
}

// Convert ASCII string to positions in the alphanum array
//
void cnvrt_msg(unsigned char *m) {
  int i;
  for (i=0; m[i]; i++) {
    if ((m[i] >= ' ') && (m[i]<='!')) m[i] = 26 + (m[i]-' ');
    else if ((m[i]>='0') && (m[i]<='9')) {
      m[i] -= '0';
      m[i] += 28;
    }
    else { m[i]-='a'; }
  }
}

#define STATE_MSG 0
#define STATE_BEAT_ANIM 1
unsigned char state=0;

void init(void) {
  // set 
  DDRB = 0b11000111;
  DDRD = 0b11100000;


  // prescale timer to every 1024 clock ticks
  // that is, decrement counter every 1024 clock ticks
  //TCCR0B |= (1<<CS02) | (0<<CS01) | (1<<CS00);

  // no prescaler
  TCCR0B |= (0<<CS02) | (0<<CS01) | (1<<CS00);

  PORTB = (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB7) | (1<<PB6);
  PORTD = (1<<PD5) | (1<<PD6) | (1<<PD7);

  // Turn off rows pb0, pb1 and pd7
  t = PORTB;
  t &= ~( (1<<PB1) | (1<<PB0) );
  PORTB = t;
  t = PORTD;
  t &= ~( (1<<PD7) );
  PORTD = t;

  // sink column pb6
  t = PORTB;
  t &= ~( (1<<PB6) );
  PORTB = t;

  // Finally turn on pb2
  t = PORTB;
  t |= (1<<PB2);
  PORTB = t;

  // enable timer overflow interrupt
  TIMSK0 |=1<<TOIE0;
  sei();

  spi_follower_init();

}

int main(void) {
  unsigned char frame=1;
  unsigned char n_frame=N_FRAME;

  unsigned char ticker_count=0;
  unsigned char next_frame=0;
  unsigned char ticker_focus_delay=2;
  unsigned char ticker_focus=0;

  n_frame = slen(msg);
  cnvrt_msg(msg);

  //copy_anim_to_back_buf(&(anim0[0][0]));
  //load_back_buf();

  //copy_anim_to_back_buf(&(ohs[0][0]));
  //load_back_buf();

  frame=0;  next_frame=1;  ticker_count=1;
  ticker_focus=0;
  copy_anim_to_back_buf(&(alphanum[msg[frame]][0]));
  load_back_buf();


  //copy_anim_to_back_buf(&(on[0][0]));
  //load_back_buf();


  /*
  // set 
  DDRB = 0b11000111;
  DDRD = 0b11100000;


  // prescale timer to every 1024 clock ticks
  // that is, decrement counter every 1024 clock ticks
  //TCCR0B |= (1<<CS02) | (0<<CS01) | (1<<CS00);

  // no prescaler
  TCCR0B |= (0<<CS02) | (0<<CS01) | (1<<CS00);

  PORTB = (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB7) | (1<<PB6);
  PORTD = (1<<PD5) | (1<<PD6) | (1<<PD7);

  // Turn off rows pb0, pb1 and pd7
  t = PORTB;
  t &= ~( (1<<PB1) | (1<<PB0) );
  PORTB = t;
  t = PORTD;
  t &= ~( (1<<PD7) );
  PORTD = t;

  // sink column pb6
  t = PORTB;
  t &= ~( (1<<PB6) );
  PORTB = t;

  // Finally turn on pb2
  t = PORTB;
  t |= (1<<PB2);
  PORTB = t;

  // enable timer overflow interrupt
  TIMSK0 |=1<<TOIE0;
  sei();
  */
  init();

  /*
  t = PORTB;
  t &= ~(1<<PB2);
  PORTB = t;
  */

  while(1) {
    if (frame_copied) {
      cli();

      frame_copied=0;
      //r=c=0;

      //copy_anim_to_back_buf(&(on[frame][0]));
      //frame = (frame+1)%2;

      //copy_anim_to_back_buf(&(anim0[frame][0]));
      //frame = (frame+1)%n_frame;

      //copy_anim_to_back_buf(&(anim1[frame][0]));
      //frame = (frame+1)%5;

      //copy_anim_to_back_buf(&(alpha[frame][0]));
      //frame = (frame+1)%26;

      //copy_anim_to_back_buf(&(alpha[msg[frame]][0]));
      //frame = (frame+1)%n_frame;

      //copy_anim_to_back_buf(&(ohs[frame][0]));
      //frame = (frame+1)%n_frame;

      //copy_anim_to_back_buf(&(pulse_heart[frame][0]));
      //frame = (frame+1)%N_FRAME;

      //copy_anim_to_back_buf(&(edge_thing[frame][0]));
      //frame = (frame+1)%N_FRAME;

      //copy_anim_to_back_buf(&(edge_thing2[frame][0]));
      //frame = (frame+1)%N_FRAME;

      //copy_anim_to_back_buf(&(alphanum[frame][0]));
      //frame = (frame+1)%n_frame;

      //copy_anim_to_back_buf(&(alphanum[msg[frame]][0]));
      //frame = (frame+1)%n_frame;


      if (state == STATE_MSG) {
        if (ticker_focus<ticker_focus_delay) {
          ticker_focus++;
        } else {

          if (ticker_count==5) {
            ticker_count=0;
            frame = next_frame;
            copy_anim_to_back_buf(&(alphanum[msg[frame]][0]));
            next_frame = (next_frame+1)%n_frame;

            ticker_focus=0;
          } else {
            clear_back_buf();
            copy_anim_to_back_buf_shiftl(&(alphanum[msg[frame]][0]), ticker_count);
            if (ticker_count>=2) {
              copy_anim_to_back_buf_shiftr(&(alphanum[msg[next_frame]][0]), 5-ticker_count);
            }
          }
          ticker_count++;

        }
      } else if (state == STATE_BEAT_ANIM) {
        copy_anim_to_back_buf(&(beatanim[frame][0]));
        frame = (frame+1)%5;
      }

      sei();
    }
  }

}

