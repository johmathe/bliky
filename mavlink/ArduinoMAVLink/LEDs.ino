#include <OctoWS2811.h>


#define SATURATION 255
#define VALUE 180
const int NUM_LEDS = 300;
const int ledsPerStrip = NUM_LEDS;
DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

#define RED    0x440000
#define GREEN  0x004400
#define BLUE   0x000044
#define YELLOW 0x448600
#define PINK   0x400010
#define ORANGE 0x440800
#define WHITE  0x202020 //1010

void initLeds() {
  leds.begin();
  leds.show();
}

void wipeAll(int hue) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds.setPixel(i, hue);
  }
  leds.show();
}

int pwmToHue(int pwm) {
  float ratio = - ((float(pwm) - 1000.0) / 1000.0);
  return int(120 * ratio);
}

const byte dim_curve[] = {
    0,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
    3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,
    4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,
    6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
    8,   8,   9,   9,   9,   9,   9,   9,   10,  10,  10,  10,  10,  11,  11,  11,
    11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  14,  15,
    15,  15,  16,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,
    20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  25,  26,  26,
    27,  27,  28,  28,  29,  29,  30,  30,  31,  32,  32,  33,  33,  34,  35,  35,
    36,  36,  37,  38,  38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,
    48,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,
    63,  64,  65,  66,  68,  69,  70,  71,  73,  74,  75,  76,  78,  79,  81,  82,
    83,  85,  86,  88,  90,  91,  93,  94,  96,  98,  99,  101, 103, 105, 107, 109,
    110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
    146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
    193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

void hsv_to_rgb(int hue, int sat, int val, int colors[3]) { 
  val = dim_curve[val];
  sat = 255-dim_curve[255-sat];
 
  int r = 0;
  int g = 0;
  int b = 0;
  int base = 0;
 
  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    colors[0]=val;
    colors[1]=val;
    colors[2]=val;  
  } else  { 
 
    base = ((255 - sat) * val)>>8;
 
    switch(hue/60) {
    case 0:
        r = val;
        g = (((val-base)*hue)/60)+base;
        b = base;
    break;
 
    case 1:
        r = (((val-base)*(60-(hue%60)))/60)+base;
        g = val;
        b = base;
    break;
 
    case 2:
        r = base;
        g = val;
        b = (((val-base)*(hue%60))/60)+base;
    break;
 
    case 3:
        r = base;
        g = (((val-base)*(60-(hue%60)))/60)+base;
        b = val;
    break;
 
    case 4:
        r = (((val-base)*(hue%60))/60)+base;
        g = base;
        b = val;
    break;
 
    case 5:
        r = val;
        g = base;
        b = (((val-base)*(60-(hue%60)))/60)+base;
    break;
    }
 
    colors[0]=r;
    colors[1]=g;
    colors[2]=b; 
  }   
}

void wipeHue(int hue) {
   int rgb[3];
   hsv_to_rgb(hue, SATURATION, VALUE, rgb);
   int total_rgb = (rgb[1] << 16) | (rgb[0] << 8) | (rgb[2]);
   wipeAll(total_rgb);
}

void wipePwm(int pwm) {
   int rgb[3];
   int hue = pwmToHue(pwm);
   hsv_to_rgb(hue, SATURATION, VALUE, rgb);
   int total_rgb = (rgb[1] << 16) | (rgb[0] << 8) | (rgb[2]);
   wipeAll(total_rgb);
}

//Byte val 2PI Cosine Wave, offset by 1 PI 
//supports fast trig calcs and smooth LED fading/pulsing.
uint8_t const cos_wave[256] PROGMEM =  
{0,0,0,0,1,1,1,2,2,3,4,5,6,6,8,9,10,11,12,14,15,17,18,20,22,23,25,27,29,31,33,35,38,40,42,
45,47,49,52,54,57,60,62,65,68,71,73,76,79,82,85,88,91,94,97,100,103,106,109,113,116,119,
122,125,128,131,135,138,141,144,147,150,153,156,159,162,165,168,171,174,177,180,183,186,
189,191,194,197,199,202,204,207,209,212,214,216,218,221,223,225,227,229,231,232,234,236,
238,239,241,242,243,245,246,247,248,249,250,251,252,252,253,253,254,254,255,255,255,255,
255,255,255,255,254,254,253,253,252,252,251,250,249,248,247,246,245,243,242,241,239,238,
236,234,232,231,229,227,225,223,221,218,216,214,212,209,207,204,202,199,197,194,191,189,
186,183,180,177,174,171,168,165,162,159,156,153,150,147,144,141,138,135,131,128,125,122,
119,116,113,109,106,103,100,97,94,91,88,85,82,79,76,73,71,68,65,62,60,57,54,52,49,47,45,
42,40,38,35,33,31,29,27,25,23,22,20,18,17,15,14,12,11,10,9,8,6,6,5,4,3,2,2,1,1,1,0,0,0,0
};


//Gamma Correction Curve
uint8_t const exp_gamma[256] PROGMEM =
{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,
4,4,4,4,4,5,5,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,9,10,10,10,11,11,12,12,12,13,13,14,14,14,15,15,
16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,
34,35,35,36,37,38,39,39,40,41,42,43,44,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
61,62,63,64,65,66,67,68,70,71,72,73,74,75,77,78,79,80,82,83,84,85,87,89,91,92,93,95,96,98,
99,100,101,102,105,106,108,109,111,112,114,115,117,118,120,121,123,125,126,128,130,131,133,
135,136,138,140,142,143,145,147,149,151,152,154,156,158,160,162,164,165,167,169,171,173,175,
177,179,181,183,185,187,190,192,194,196,198,200,202,204,207,209,211,213,216,218,220,222,225,
227,229,232,234,236,239,241,244,246,249,251,253,254,255
};

#define COLS_LEDs 64
#define ROWS_LEDs 64


void swipeColor(unsigned int c) {
  static int color = 0;
  wipeHue(color++);
  if (color >= 360) {
    color = 0;
  }
  leds.show();
}


void swipeLoop(unsigned int c) {
  static int px = 0;
  for (int i = 0; i < px + 40; i+=2) {
    leds.setPixel(i, c);
    leds.setPixel(i+1, c);
  }
  for (int i = 0; i < px; i+=2) {
    leds.setPixel(i, 0);
    leds.setPixel(i+1, 0);
  }
  px++;
  if (px == 300) {
    px = 0;
  }
  leds.show();
}

void plasmaLoop()
{
  static unsigned long frameCount=25500;  // arbitrary seed to calculate the three time displacement variables t,t2,t3
  
    frameCount++ ; 
    uint16_t t = fastCosineCalc((42 * frameCount)/100);  //time displacement - fiddle with these til it looks good...
    uint16_t t2 = fastCosineCalc((35 * frameCount)/100); 
    uint16_t t3 = fastCosineCalc((38 * frameCount)/100);

    for (uint8_t y = 0; y < ROWS_LEDs; y++) {
      int left2Right, pixelIndex;
      if (((y % (ROWS_LEDs/8)) & 1) == 0) {
        left2Right = 1;
        pixelIndex = y * COLS_LEDs;
      } else {
        left2Right = -1;
        pixelIndex = (y + 1) * COLS_LEDs - 1;
      }
      for (uint8_t x = 0; x < COLS_LEDs ; x++) {
        //Calculate 3 seperate plasma waves, one for each color channel
        uint8_t r = fastCosineCalc(((x << 1) + (t >> 1) + fastCosineCalc((t2 + (y << 1)))));
        uint8_t g = fastCosineCalc(((y << 1) + t + fastCosineCalc(((t3 >> 2) + (x << 1)))));
        uint8_t b = fastCosineCalc(((y << 1) + t2 + fastCosineCalc((t + x + (g >> 2)))));
        //uncomment the following to enable gamma correction
        leds.setPixel(pixelIndex, ((r << 16) | (g << 8) | b));
        pixelIndex += left2Right;
      }
    }
    
    leds.show();  // not sure if this function is needed  to update each frame
   
}

inline uint8_t fastCosineCalc( uint16_t preWrapVal)
{
  uint8_t wrapVal = (preWrapVal % 255);
  if (wrapVal<0) wrapVal=255+wrapVal;
  return (pgm_read_byte_near(cos_wave+wrapVal)); 
}
