

#define RANGE_SIZE 96
#define RANGE_STEP 5
#define BUFFER_SIZE 480

#define LINEAR_AMP  4.0f

// You can use any (4 or) 5 pins 
#define sclk 15
#define mosi 16
#define rst  5
#define dc   6
#define cs   7


// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#define GREY            0x7AEF

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>

// Option 1: use any pins but a little slower
//Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);  
// Option 2: must use the hardware SPI pins 
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);



#define PIN_TX1 9
#define PIN_TX2 10

#define PIN_TRIG A3
#define PIN_ECHO A2

#define ADC_TIME_104  ADCSRA=(ADCSRA&0xF80)|0x07   
#define ADC_TIME_52   ADCSRA=(ADCSRA&0xF80)|0x06   
#define ADC_TIME_26   ADCSRA=(ADCSRA&0xF80)|0x05   
#define ADC_TIME_13   ADCSRA=(ADCSRA&0xF80)|0x04   

void timer_init() {
  // setup timer4 
  TCCR4B = (0<<CS43) | (0<<CS42) | (0<<CS41) | (1<<CS40);  // Freq(PLL) / 1
  TCCR4E = (1<<ENHC4); 
  TCCR4D = (0<<WGM41) | (1<<WGM40); //set it to phase and frequency correct mode    
  // TCCR4A = (1<<PWM4B) | (0<<COM4B1) | (1<<COM4B0); // pin 9, 10 set to !OC9B, OC9B 
  TIFR4 = (0<<OCF4D) | (0<<OCF4A) | (0<<OCF4B) | (1<<TOV4); // interrupt on overflow
  TIMSK4 |= (1<<TOIE4); // enable interrupt
} 

volatile byte timer_loops = 0; 

void timer_start(int freq, byte loops) {
  OCR4C = freq; // timer max
  OCR4B = freq; // 50% duty cycle (huh? whatever.)
  timer_loops = loops; // 8 loops
  TCNT4 = 0; // clear counter
  TCCR4A = (1<<PWM4B) | (0<<COM4B1) | (1<<COM4B0); // pin 9, 10 set to !OC9B, OC9B 
  TCCR4B = (0<<CS43) | (0<<CS42) | (1<<CS41) | (0<<CS40);  // Freq(PLL) / 1
}

inline void timer_stop() {
  TCCR4B = (0<<CS43) | (0<<CS42) | (0<<CS41) | (0<<CS40);  // clock stopped
  TCCR4A = (1<<PWM4B) | (0<<COM4B1) | (0<<COM4B0); // outputs back to normal
}

// when timer4 overflows
ISR(TIMER4_OVF_vect) {
  if(--timer_loops == 0) timer_stop();
}


void ping() {
  // turn on power to the noisy MAX232A
  digitalWrite(PIN_TRIG,LOW); 
  // wait for it to warm up
  delay(5);
  // send pulses out at 40khz
  timer_start(612,12);
  // wait for it to be over
  delayMicroseconds(300);
  timer_stop();
  // show how many were left?
  // Serial.println(timer_loops);
  // pull both low again
  digitalWrite(PIN_TX1,LOW); digitalWrite(PIN_TX2,LOW);
  // shutdown HV again
  digitalWrite(PIN_TRIG,HIGH);
  // all done
}


int buffer_index;
int buffer_remain;
int buffer_samples[BUFFER_SIZE];

float range_samples[RANGE_SIZE];


void sample() {
  // set the sampling speed
  // ADC_TIME_26;
  ADC_TIME_52;
  // reset the buffer index
  buffer_index = 0;
  buffer_remain = BUFFER_SIZE;
  // start sampling as fast as we can
  while(buffer_remain-- > 0) {
   buffer_samples[ buffer_index++] = analogRead(PIN_ECHO);
  }
  // done
}

int sample_average() {
  long r = 0;
  for(int i=0; i<BUFFER_SIZE; i++) {
    r += buffer_samples[i];
  }
  return r / BUFFER_SIZE;
}

void buffer_graph(int w, int h, int vmin, int vrange, char vbar, char hbar, char corner, char dot) {
  // iterate through each line
  int last_i = 1024;
  for(int y=h; y>0; ) {
    y--;
    int next_i = (long)y * (long)vrange / (long)h;
    // vertical bar to start
    Serial.print(vbar);
    // for each x position
    int last_j = 0;
    for(int x=0; x<w; ) {
      x++; // increment first
      // where should we be up to in the buffer?
      int next_j = (long)x * (long)BUFFER_SIZE / (long)w;
      // count how occupied we are
      int occ = 0;
      // go through the X range
      for(int j=last_j; j<next_j; j++) {
        int v = buffer_samples[j] - vmin;
        // is this sample on this line?
        if( (v<last_i) && (v>=next_i) ) occ++;
      }
      // if we had occupancy, draw the dot
      Serial.print(occ==0 ? ' ' : dot);
      last_j = next_j;
    }
    // end line
    Serial.println();
    last_i = next_i;
  }
  // last line
  Serial.print(corner);
  // for each x position
  for(int x=0; x<w; x++) Serial.print(hbar);
  Serial.println();
}

void range_graph(int w, int h, float vmin, float vmax, char vbar, char hbar, char corner, char dot) {
  // iterate through each line
  float last_i = vmax;
  float vrange = vmax - vmin;
  for(int y=h; y>0; ) {
    y--;
    float next_i = vmin + (float)y * vrange / (float)h;
    // vertical bar to start
    Serial.print(vbar);
    // for each x position
    int last_j = 0;
    for(int x=0; x<w; ) {
      x++; // increment first
      // where should we be up to in the buffer?
      int next_j = (long)x * (long)RANGE_SIZE / (long)w;
      // count how occupied we are
      int occ = 0;
      // go through the X range
      for(int j=last_j; j<next_j; j++) {
        float v = range_samples[j] - vmin;
        // is this sample on this line?
        // if( (v<last_i) && (v>=next_i) ) occ++;
        if( v >= next_i ) occ++;
      }
      // if we had occupancy, draw the dot
      Serial.print(occ==0 ? ' ' : dot);
      last_j = next_j;
    }
    // end line
    Serial.println();
    last_i = next_i;
  }
  // last line
  Serial.print(corner);
  // for each x position
  for(int x=0; x<w; x++) Serial.print(hbar);
  Serial.println();
}

int drawn_y[96];

void sonar_graph(int w, int h, float vmin, float vmax) {
  // display.fillScreen(BLACK);
  int h2 = h / 2;
  float vstep = (vmax - vmin) / (float)h2;
  // for each x position
  int last_j = 0;
  for(int x=0; x<w; x++) {
    // where should we be up to in the buffer?
    int next_j = (long)(x+1) * (long)RANGE_SIZE / (long)w;
    // go through the X range, find the maximum
    float v = 0;
    for(int j=last_j; j<next_j; j++) {
      float s = range_samples[j] - vmin;
      v = max(v, s );
    }
    // vertical proportion
    int y = (v - vmin) / vstep;
    // y = h - y;
    if(y>=h2) { y = h2-1; }
    if(y<0) { y = 0; }
    
    // remove old pixels before drawing new ones
    int py = (int)drawn_y[x];
    display.drawPixel(x,32 + py, BLACK);
    display.drawPixel(x,32 - py, BLACK);
    // turn pixel height into yellow/blueness
    unsigned int iy = (32 - y);
    unsigned int color = ( (y<<12) &RED) | ( (min(y<<2,63)<<5) &GREEN) | ( (iy>>1) &BLUE);
    display.drawPixel(x,32 + y, color);
    display.drawPixel(x,32 - y, color);
    drawn_y[x] = (byte)y;
  
    last_j = next_j;
  }
}

void redraw_sonar_graph(int w, int h) {
  // display.fillScreen(BLACK);
  int h2 = h / 2;
  // for each x position
  for(int x=0; x<w; x++) {
    // redraw old pixels
    int y = (int)drawn_y[x];
    // turn pixel height into yellow/blueness
    unsigned int iy = (32 - y);
    unsigned int color = ( (y<<12) &RED) | ( (min(y<<2,63)<<5) &GREEN) | ( (iy>>1) &BLUE);
    display.drawPixel(x,32 + y, color);
    display.drawPixel(x,32 - y, color);
  }
}
 
void draw_ping_circle(int counter, unsigned int color) {
  display.drawCircle(0, 32, 8 + counter * 16, color);
}

void setup() {
  // pull TRIG/HVOFF high
  pinMode(PIN_TRIG,OUTPUT); digitalWrite(PIN_TRIG,LOW); 
  // ECHO pin input
  pinMode(PIN_ECHO,INPUT);
  // give the module power?
  // digitalWrite(A1,LOW); pinMode(A1,OUTPUT);
  // make sure the TX lines start low
  digitalWrite(PIN_TX1,LOW); pinMode(PIN_TX1,OUTPUT);
  digitalWrite(PIN_TX2,LOW); pinMode(PIN_TX2,OUTPUT);
  // start the screen
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  display.begin();
  display.fillScreen(BLACK);
  // wait a moment for everything to stabilize
  delayMicroseconds(1000);
  timer_init();
  for(int i = 0; i<RANGE_SIZE; i++) range_samples[i] = 0;
}

int control_loop = 0;
float vmin,vmax;

void loop() {
  int i,j,k;
  float v;
  // send a ping
  ping();
  // sample the reply
  sample();
  // get the average
  int sa = sample_average();
  // draw the buffer graph around the average
  // buffer_graph(128,32, sa - 128, 256, '|', '-', '+', '#');
  // Serial.print("avg "); Serial.print(sa); Serial.print(" "); 
  // turn sample power into range increments
  k = 0;
  for(i = 0; i<RANGE_SIZE; i++) {
    v = range_samples[i];
    // Serial.print(v); Serial.print(',');
    for(j = 0; j<RANGE_STEP; j++) {
      int d = buffer_samples[k++] - sa;
      v += abs(d);
    }
    range_samples[i] = v;
  }
  // Serial.println();
  // erase last circle
  draw_ping_circle(control_loop, BLACK);
  // dump stats?
  if(++control_loop == 8) {
    // find extents
    for(i = 0; i<RANGE_SIZE; i++) {
      v = range_samples[i];
      v = v * (1.0 + (float)i/(float)RANGE_SIZE * LINEAR_AMP);
      if(i==0) {
        // first
        vmin = v; vmax = v;
      } else {
        // stats
        vmin = min(vmin,v); vmax = max(vmax,v); 
      }
      range_samples[i] = v;
    }
    // widen range?
    vmax = max(vmax, vmin + 4.0f);
    // Serial.print("vmin "); Serial.print(vmin); Serial.print("vmax "); Serial.print(vmax); Serial.println();
    // range_graph(128,16, 0.0f, 2000.0f, '|', '-', '+', '#');
    // range_graph(128,16, vmin,vmax, '|', '-', '+', '#');
    sonar_graph(96,64, vmin,vmax);
    control_loop = 0;
    // clear the samples
    for(i = 0; i<RANGE_SIZE; i++) range_samples[i] = 0.0f;
    // delay(100);
  } else {
    // draw the expanding circle
    // delay(10);
    //draw_ping_circle(control_loop, (RED>>2)&RED );
    draw_ping_circle(control_loop, GREY);
    redraw_sonar_graph(96,64);
  }
}
