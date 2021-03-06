
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>


#include "clk.h"
#include "gpio.h"
#include "dma.h"
#include "pwm.h"

#include "ws2811.h"

#include <wiringPi.h>


#define PIN_LDR     29
#define PIN_BTN     28
#define PIN_LED     4
#define PIN_SNOWMAN 2

#define ARRAY_SIZE(stuff)       (sizeof(stuff) / sizeof(stuff[0]))

// defaults for cmdline options
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                12 //wiring pi #26
#define DMA                     10
#define STRIP_TYPE              WS2811_STRIP_GRB        // WS2812/SK6812RGB integrated chip+leds

#define WIDTH                   300
#define HEIGHT                  1
#define LED_COUNT               300 //(WIDTH * HEIGHT)

#define EFFECT_WAIT_MS          50
#define INTERVAL_WAIT_MS        500

#define STATE_OFF               0
#define STATE_DEMO              1
#define STATE_DEBUG             2
#define STATE_SHOW              3

#define TIME_HOUR_START         17
#define TIME_HOUR_END           21

#define STOP_CONDITION_TIME     0
#define STOP_CONDITION_LIGHT    1


#define MAX_BRIGHTNESS          0.4f //float 0-1.0
#define Color                   uint32_t //uint32_t 0xWWRRGGBB

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define bool    int
#define true    1
#define false   0

int width = WIDTH;
int height = HEIGHT;
int led_count = LED_COUNT;
int debug=1;
int stop_condition;

ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .count = LED_COUNT + 1,
            .invert = 0,
            .brightness = 255,
            .strip_type = STRIP_TYPE,
        },
        [1] =
        {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        },
    },
};

Color *led_strip;

uint8_t running = 1;

void ctrl_c_handler(int signum) {
    (void)(signum);
    running = 0;
}


/*! \brief Convert RGB to HSV color space
  
  Converts a given set of RGB values `r', `g', `b' into HSV
  coordinates. The input RGB values are in the range [0, 1], and the
  output HSV values are in the ranges h = [0, 360], and s, v = [0,
  1], respectively.
  
  \param fR Red component, used as input, range: [0, 255]
  \param fG Green component, used as input, range: [0, 255]
  \param fB Blue component, used as input, range: [0, 255]
  \param fH Hue component, used as output, range: [0, 360]
  \param fS Hue component, used as output, range: [0, 1]
  \param fV Hue component, used as output, range: [0, 1]
  
*/
void RGBtoHSV(Color r, Color g, Color b, float* pH, float *pS, float *pV) {
    float fR = 1.0f * r / 255;
    float fG = 1.0f * g / 255;
    float fB = 1.0f * b / 255;
    float fH, fS, fV;
    
    float fCMax = MAX(MAX(fR, fG), fB);
    float fCMin = MIN(MIN(fR, fG), fB);
    float fDelta = fCMax - fCMin;
  
    if(fDelta > 0) {
        if(fCMax == fR) {
          fH = 60 * (fmod(((fG - fB) / fDelta), 6));
        }
        else if(fCMax == fG) {
          fH = 60 * (((fB - fR) / fDelta) + 2);
        }
        else if(fCMax == fB) {
          fH = 60 * (((fR - fG) / fDelta) + 4);
        }

        if(fCMax > 0) {
          fS = fDelta / fCMax;
        }
        else {
          fS = 0;
        }
        fV = fCMax;
    } 
    else {
        fH = 0;
        fS = 0;
        fV = fCMax;
    }
  
    if(fH < 0) {
        fH = 360 + fH;
    }
  
  *pH = fH;
  *pS = fS;
  *pV = fV;
}

/*! \brief Convert HSV to RGB color space
  
  Converts a given set of HSV values `h', `s', `v' into RGB
  coordinates. The output RGB values are in the range [0, 1], and
  the input HSV values are in the ranges h = [0, 360], and s, v =
  [0, 1], respectively.
  
  \param fR Red component, used as output, range: [0, 255]
  \param fG Green component, used as output, range: [0, 255]
  \param fB Blue component, used as output, range: [0, 255]
  \param fH Hue component, used as input, range: [0, 360]
  \param fS Hue component, used as input, range: [0, 1]
  \param fV Hue component, used as input, range: [0, 1]
  
*/
void HSVtoRGB(Color* pR, Color* pG, Color *pB, float fH, float fS, float fV) {
  float fR, fG, fB;
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;
  
  if(0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }
  
  fR += fM;
  fG += fM;
  fB += fM;
  
  *pR=(Color)(255*fR);
  *pG=(Color)(255*fG);
  *pB=(Color)(255*fB);
}

int next_int(int max) {
    int value=rand();
    if (value < 0)
        value = 0 - value;
    return value % max;
}

Color rgb(Color r, Color g, Color b) {
    return (r << 16) | (g << 8) | b;
}

Color next_clr() {
    Color color;
    int sel=next_int(11);
    switch (sel) {
        case 0: color=rgb(0xff, 0x00, 0xff); break;
        case 1: color=rgb(0x00, 0xff, 0x00); break;
        case 2: color=rgb(0x00, 0x00, 0xff); break;
        case 3: color=rgb(0x80, 0x00, 0x00); break;
        case 4: color=rgb(0xff, 0xff, 0x00); break;
        case 5: color=rgb(0x80, 0x80, 0x00); break;
        case 6: color=rgb(0x00, 0xff, 0xff); break;
        case 7: color=rgb(0x00, 0x80, 0x80); break;
        case 8: color=rgb(0x00, 0x00, 0x80); break;
        case 9: color=rgb(0x80, 0x00, 0x80); break;
        case 10: color=rgb(0xff, 0xff, 0xff); break;
    }
    return color;
}

void random_hsv_clr(float *ph, float *ps, float *pv) {
    float h = 4.0f * next_int(90); //0-360
    float s = 4.0f * next_int(25) / 100; //0-1
    float v = 0.5f + 1.0f * next_int(50)/50; //0-1
    
    *ph = h;
    *ps = s;
    *pv = v;
}

Color random_rgb_clr() {
    float h, s, v;
    random_hsv_clr(&h, &s, &v);
    Color r,g,b;
    HSVtoRGB(&r, &g, &b, h, s, v);
    return rgb(r,g,b);
}

void set_pixel_hsv(int pos, float h, float s, float v) {
    Color pixel, r, g, b;
    
    v = v * MAX_BRIGHTNESS;
    HSVtoRGB(&r, &g, &b, h, s, v);
    pixel = (r << 16) | (g << 8) | b;
    
    if (pos >= 0 && pos < LED_COUNT)
        led_strip[pos]=pixel;
}

void set_pixel_rgb(int pos, Color pixel) {
    Color r = (pixel >> 16) & 0xff;
    Color g = (pixel >> 8) & 0xff;
    Color b = pixel & 0xff;
    
    float h, s, v;
    RGBtoHSV(r, g, b, &h, &s, &v);
    
    set_pixel_hsv(pos, h, s, v);
}


void show() {
    for (int i = 0; i < LED_COUNT; ++i){
        ledstring.channel[0].leds[i] = led_strip[i];
    }
    ws2811_render(&ledstring);
}

void turn_off_led_strip() {
    for (int i = 0; i < LED_COUNT; ++i) {
        set_pixel_rgb(i, rgb(0,0,0));
    }
    show();
}

void color_slide() {
    Color color = next_clr();
    int wait_ms=EFFECT_WAIT_MS, slide_size=20;
    int destination = LED_COUNT - slide_size;
    int pos = 0;
    while (destination >= 0) {
        while (pos <= destination) {
            for (int i = pos - slide_size; i < pos; ++i) {
                set_pixel_rgb(i, rgb(0,0,0));
            }
            for (int i = pos; i < pos + slide_size; ++i) {
                set_pixel_rgb(i, color);
            }
            show();
            delay(wait_ms);
            pos += slide_size;
        }
        destination -= slide_size;
        pos = 0;
    }
}

void color_wipe(Color color) {
    int wait_ms=EFFECT_WAIT_MS;
    //"""Wipe color across display a pixel at a time."""
    for (int i = 0; i < LED_COUNT; ++i) {
        set_pixel_rgb(i, color);
        show();
        delay(wait_ms);
    }
}

void theater_chase(Color color) {
    int wait_ms=EFFECT_WAIT_MS, iterations=10;
    //"""Movie theater light style chaser animation."""
    for (int j = 0; j < iterations; ++j) {
        for (int q = 0; q < 3; ++q) {
            for (int i = 0; i < LED_COUNT; i+=3) {
                set_pixel_rgb(i+q, color);
            }
            show();
            delay(wait_ms);
            for (int i = 0; i < LED_COUNT; i+=3) {
                set_pixel_rgb(i+q, 0);
            }
        }
    }
}

void rainbow() {
    int wait_ms=EFFECT_WAIT_MS;
    float step = 360.0f / LED_COUNT;
    for (int offset = LED_COUNT - 1; offset > 0; --offset) {
        for (int i = 0; i < LED_COUNT; ++i) {
            int h = (int)((i + offset) *step);
            set_pixel_hsv(i, h%360, 1, 1);
        }
        show();
        delay(wait_ms);
    }
}

void twinkle() {
    int wait_ms=EFFECT_WAIT_MS;
    int total_on = LED_COUNT/5;
    int led_on[LED_COUNT];
    int available[LED_COUNT];
    for (int r = 0; r < 4000 / wait_ms; ++r) {
        for (int i = 0; i < LED_COUNT; ++i) {
            led_on[i]=0; //mark as off
        }
        //seed
        for (int i = 0; i < total_on; ++i) {
            int pos = 0;
            for (int j = 0; j < LED_COUNT; ++j) {
                if (led_on[j] == 0) {
                    available[pos++]=j;
                }
            }
            int sel = next_int(pos);
            led_on[available[sel]]=next_clr();
        }
        //copy over
        for (int i = 0; i < LED_COUNT; ++i)
            set_pixel_rgb(i, led_on[i]);
        show();
        delay(wait_ms);
    }
}


Color wheel(int pos) {
    //"""Generate rainbow colors across 0-255 positions."""
    if (pos < 85)
        return rgb(pos * 3, 255 - pos * 3, 0);
    else if (pos < 170) {
        pos -= 85;
        return rgb(255 - pos * 3, 0, pos * 3);
    }
    else {
        pos -= 170;
        return rgb(0, pos * 3, 255 - pos * 3);
    }
}

void theater_chase_rainbow() {
    int wait_ms=EFFECT_WAIT_MS;
    //"""Rainbow movie theater light style chaser animation."""
    for (int j = 0; j < 48 ; ++j) {
        for (int q = 0; q < 3; ++q) {
            for (int i=0; i < LED_COUNT; i+= 3) {
                set_pixel_rgb(i+q, wheel((i+j) % 255));
            }
            show();
            delay(wait_ms);
            for (int i = 0; i < LED_COUNT; i+=3) {
                set_pixel_rgb(i+q, 0);
            }
        }
    }
}


void meteorite() {
    float h, s, v;
    int wait_ms=EFFECT_WAIT_MS, meteorite_length=50;
    float *v_gradient = (float*)malloc(sizeof(float) * meteorite_length);
    
    random_hsv_clr(&h, &s, &v);
    float vstep = v / meteorite_length;
    for (int i = 0; i < meteorite_length; ++i)
        v_gradient[i] = v - i * vstep;
    
    int alignment = 0 - meteorite_length + 1;
    while (alignment < meteorite_length * 2) {
        for (int i = 0; i < LED_COUNT; ++i) {
            int pos = i % meteorite_length + alignment;
            if (pos < 0 || pos >= meteorite_length)
                set_pixel_rgb(i, rgb(0,0,0));
            else
                set_pixel_hsv(i, h, s, v_gradient[pos]);
        }
        show();
        delay(wait_ms);
        ++alignment;
    }    
    
    free (v_gradient);
}

void setup_handlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrl_c_handler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

int timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;
}

//daylight: around 270ms
int get_charging_time() {
    int pin = PIN_LDR;
    int val, wait=1300;
    struct timeval t0, t1;

    pinMode (pin, OUTPUT);
    digitalWrite(pin, LOW); 
    delay(1000);
    
    gettimeofday(&t0, 0);
    pinMode (pin, INPUT);
    val=digitalRead(pin);
    
    while ((val == LOW) && (wait > 0)) {
        delay(10);
        wait -= 10;
        val=digitalRead(pin);
    }
    
    gettimeofday(&t1, 0);
    
    return timedifference_msec(t0, t1);
}

bool should_be_on(int *hours_to_go) {
    struct tm *timeinfo ;
    time_t rawtime ;
    
    rawtime = time (NULL) ;
    timeinfo = localtime(&rawtime) ;

    *hours_to_go = TIME_HOUR_START - timeinfo->tm_hour;
    return (timeinfo->tm_hour >= TIME_HOUR_START) && (timeinfo->tm_hour < TIME_HOUR_END);
}

bool should_be_off() {
    struct tm *timeinfo ;
    time_t rawtime ;
    
    rawtime = time (NULL) ;
    timeinfo = localtime(&rawtime) ;
    
    bool turn_off = false;
    if ((stop_condition == STOP_CONDITION_TIME) && (timeinfo->tm_hour >= TIME_HOUR_END)) {
        stop_condition = STOP_CONDITION_LIGHT;
    }
    if ((stop_condition == STOP_CONDITION_LIGHT) && (get_charging_time() > 1000)) {
        turn_off = true;
    }
    return  turn_off;
}

void show_effects(bool random, int sel) {
    if (sel < 0)
        sel = random ? next_int(12) : 0;
    
    if (running && (!random || sel == 0)) {
        if (debug) 
            printf("Color slide ...\n");
        color_slide();
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 1)) {
        if (debug)
            printf("Color wipe R ...\n");
        color_wipe(rgb(255, 0, 0)); // Red
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 2)) {
        if (debug)
            printf("Color wipe G ...\n");
        color_wipe(rgb(0, 255, 0)); // Green
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 3)) {
        if (debug)
            printf("Color wipe B ...\n");
        color_wipe(rgb(0, 0, 255)); // Blue
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 4)) {
        if (debug)
            printf("Color wipe random color\n");
        color_wipe(random_rgb_clr());
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 5)) {
        if (debug)
            printf("Theater chase W ...\n");
        theater_chase(rgb(127, 127, 127)); // White
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 6)) {
        if (debug)
            printf("Theater chase R ...\n");
        theater_chase(rgb(127,   0,   0)); // Red
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 7)) {
        if (debug)
            printf("Theater chase B ...\n");
        theater_chase(rgb(  0,   0, 127)); // Blue
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 8)) {
        if (debug)
            printf("Rainbow...\n");
        rainbow();
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 9)) {
        if (debug)
            printf("Twinkle ...\n");
        twinkle();
        delay(INTERVAL_WAIT_MS);
    }
    
    if (running && (!random || sel == 10)) {
        if (debug)
            printf("Theater chase rainbow ...\n");
        theater_chase_rainbow();
        delay(INTERVAL_WAIT_MS);
    }   
    if (running && (!random || sel == 11)) {
        if (debug)
            printf("Meteorite ...\n");
        meteorite();
        delay(INTERVAL_WAIT_MS);
    }   
}

void will_sart_show() {
    //turn off led
    digitalWrite(PIN_LED, LOW);
    //turn on snowman
    digitalWrite(PIN_SNOWMAN, HIGH);
}

void show_did_end(){
    //turn off led
    digitalWrite(PIN_LED, LOW);
    //turn off snowman
    digitalWrite(PIN_SNOWMAN, LOW);
}

//gcc -Wall 9168.c mailbox.c ws2811.c pwm.c pcm.c dma.c rpihw.c -lwiringPi -lwiringPiDev -lm
int main(int argc, char *argv[]) {
    int state, ticket1, ticket2, hours_to_go;
    ws2811_return_t ret;
    
    srand(time(NULL));

    setup_handlers();
    
    wiringPiSetup();
    pinMode (PIN_BTN, INPUT); 
    pinMode (PIN_LED, OUTPUT); 
    pinMode (PIN_SNOWMAN, OUTPUT); 
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_SNOWMAN, LOW);
    
    if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS) {
        fprintf(stderr, "Initialization failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }
    led_strip = malloc(sizeof(ws2811_led_t) * LED_COUNT + 12);

    state = STATE_OFF;
    ticket1 = ticket2 = 0;
    debug = 1;
    turn_off_led_strip();
   
    if (argc > 1) {
        if (strcmp(argv[1], "demo") == 0) {
            state=STATE_DEMO;
        }
        else if (strcmp(argv[1], "debug") == 0) {
            state=STATE_DEBUG;
        }
        else {
            //autorun at boot up
            debug = 0;
            digitalWrite(PIN_LED, HIGH);
            delay (2 * 60 * 1000);
        }
    }

    while (running) {
        switch (state) {
        case STATE_OFF:
            if (should_be_on(&hours_to_go)) {
                stop_condition=STOP_CONDITION_TIME;
                state = STATE_SHOW;
                will_sart_show();
            }
            else if (digitalRead(PIN_BTN) == HIGH) {
                state = STATE_DEMO;
            }
            else {
                digitalWrite(PIN_LED, (ticket1 & 1) ? HIGH : LOW);
                //flashing the snowman
                if ((hours_to_go > 0) && (hours_to_go <= 1)) {
                    digitalWrite(PIN_SNOWMAN, (ticket2 & 1) ? HIGH : LOW);
                }
                ticket1 = (ticket1 + 1) & 0xff;
                ticket2 = ticket1 >> 1;
                delay(1000);
            }
            break;
        case STATE_DEMO:
            will_sart_show();
            show_effects(false, -1);
            turn_off_led_strip();
            state = STATE_OFF;
            show_did_end();
            break;
        case STATE_DEBUG:
            will_sart_show();
            show_effects(true, 11);
            turn_off_led_strip();
            show_did_end();
/*
            digitalWrite(PIN_LED, HIGH);
            if (digitalRead(PIN_BTN) == HIGH)
                printf("Button!!!!\n");
            printf("Charging time: %d\n", get_charging_time());*/
            delay(100);
            break;
        case STATE_SHOW:
            if (should_be_off()) {
                state = STATE_OFF;
                turn_off_led_strip();
                show_did_end();
                ticket1 = ticket2 = 0;
            }
            else {
                show_effects(true, -1);
            }
            break;
        }
        //printf("Charging time=%d\n", get_charging_time());
    }
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_SNOWMAN, LOW);
    turn_off_led_strip();
    ws2811_fini(&ledstring);
    free (led_strip);
    if (debug)
        printf("Done\n");
    return ret;
}


