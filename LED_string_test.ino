//MIT License
//
//Copyright (c) 2020 Sunil Kumar
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

//Pins are ok for an Arduino Nano. Rotary encoder is assumed to be on pins 2 and 5,
//with common terminal connected to GND. Button is across pin 6 and GND.
//Fair warning - usually do LabVIEW or Java, so some parts may not be optimal.
//
//Should produce an adjustable colourful trail going along the string in one direction
//On pressing the button n times (like double-clicking a mouse), mode will change to mode n
//These are the modes that values of n corrspond to:
//1: RUN/OFF - Moving colourful trail should show on string (on/off only toggles within mode 1)
//2: Test for rotary encoder - rotating should move a single illuminated section along the string
//3: Set trail length - rotating should change length of illuminated region about centre
//4: Set brightness - full string should illuminate, rotating should change brightness
//5: Interval - should look like KITT/a Cylon. Rotating should change update speed
//6: Hue stride - full string should illuminate like a rainbow. Rotating changes colour blending distance
//
//If there's something that doesn't make sense, email helpmewithyoursoftware@outlook.com
//
//Version 1.0

#include <FastLED.h>

//LEDs
#define DATA_PIN 3
#define LEDS_PER_STRING 90
#define NUM_STRINGS 1;

#define MODE_RUN_OFF 1
#define MODE_POS 2
#define MODE_LENGTH 3
#define MODE_BRIGHT 4
#define MODE_INTERVAL 5
#define MODE_HUESTRIDE 6

int MAX_LEDS = LEDS_PER_STRING*NUM_STRINGS;
CRGB leds[LEDS_PER_STRING];
CRGB orig_config_colour = CRGB::Red;
CRGB config_colour = CRGB::Red;
int curr_mode = MODE_RUN_OFF;
int curr_led = 0;
int ctr = 0;
int re_ctr = 0;
int last_re_ctr = 0;

int sat_val = 255;
int max_v = 255;
int v_lim = 255;
int v_trail_len = 10;
//int v_trail_len_max = 5000;
int v_step = 25;
int h_step = 5;
int h_curr = 0;

//Inputs
#define ROTENC_CLK 5
#define ROTENC_DAT 2
#define ROTENC_BUT 6
volatile boolean int_flag = false;
boolean run_off_toggle = true;
boolean cylon_dir = true;
int cylon_trackpos = 0;

//Timing
int del_step = 50;
int del_stride = 10;
int del_max = 250;
unsigned long last_upd;

//Controls
int but_debounce = 50;
int but_was = false;
int clicks = 0;
int click_interval = 400;
unsigned long last_click = 0;
boolean change_det = false;
boolean released = false;

//Comms
int byte_in = 0;
String inputstr = "";
String recvcmd = "";
String precmd = "";
String response_msg = "";
int setcmd = 999;
int getcmd = 999;
int precmdnum = 0;
byte rx_serial;
const int buflen = 20;
char serial_string[buflen];
int serial_str_pointer = 0;
String ser_value_str = "";
int ser_value_int = 0;
byte id_x = 0;
boolean readable = false;
boolean valid = false;

//Commands
const char* NUM_get = "?N";
const char* NUM_set = "!N";
const char* MODE_get = "?M";
const char* MODE_set = "!M";

//Command listing
const char* set_cmds[] = {NUM_set,MODE_set};
const char* get_cmds[] = {NUM_get,MODE_get};

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(ROTENC_CLK, INPUT_PULLUP);
  pinMode(ROTENC_DAT, INPUT_PULLUP);
  pinMode(ROTENC_BUT, INPUT_PULLUP);
  FastLED.addLeds<WS2812, DATA_PIN>(leds, MAX_LEDS);
  FastLED.clear();
  FastLED.show();
  Serial.begin(57600, SERIAL_8N1);
  attachInterrupt(0,INT0_raise,RISING);//INT_0 on pin 2 for nano
  last_upd = millis();
}

void loop() {
  //urgent parts to execute each time
  if(int_flag){
    handle_rot();
    int_flag = false;
  }
  unsigned long loopstart = millis();
  check_serial();
  check_but();
  //do this once every timing interval
  if(loopstart>last_upd+del_step){
    update_leds();
    last_upd = loopstart;
    ctr++;
    if(ctr>MAX_LEDS){
      ctr=0;
    }
  }
}

void check_serial(){
  if (Serial.available() > 0)
  {
    rx_serial = Serial.read();//Anything on the serial port?
    if(serial_str_pointer >= (buflen - 2)){
      //Overflow! emulate carriage return...
      rx_serial = 13;
    }
    switch(rx_serial){
      case 8:
        //Backspace
        serial_str_pointer -= 1;
        // echo back:
        Serial.write(rx_serial);
        break;
      case 42:
        // *
        e_stop();
        //Reset any partial commands
        serial_str_pointer = 0;
        inputstr = "";
        Respond("EMERGENCY STOP!","STOP",readable);
        break;       
      case 13:
      // CR
        serial_str_pointer += 1;
        serial_string[serial_str_pointer] = '\0';
        inputstr = String(serial_string);
        interpret_cmd(inputstr);
        inputstr = "";
        serial_str_pointer = 0;
        for (int k = 0; k < buflen; k++)
        {
          serial_string[k] = 0;
        }
        break;
      case 65:
        handle_A();
        break;
      case 68:
        handle_D();
        break;
      case 69:
        handle_E();
        break;
      case 70:
        handle_F();
        break;        
      case 81:
        handle_Q();
        break;                    
      case 82:
        handle_R();
        break;                  
      case 83:
        handle_S();
        break;  
      case 87:
        handle_W();
        break;
      case 88:
        handle_X();
        break;                  
      case 90:
        handle_Z();
        break;                                          
      default:
        serial_str_pointer += 1;
        //Non-special characters get appended to accumulating input string
        serial_string[serial_str_pointer - 1] = rx_serial;
        Serial.write(rx_serial);
        break;
    }
  }
}

/////////////////////////////////////////////////////////////
//Comms handling
void interpret_cmd(String command) {
  //Do comparison to command set for validity, then run appropriate function if ok
  valid = false;
  setcmd = 999;//Should do default nothing
  getcmd = 999;//Should do default nothing
  //Go through all 'set value' commands
  for (int k = 0; k < (sizeof(set_cmds) / sizeof(char*)); k++) {
    recvcmd = command.substring(inputstr.length() - (strlen(set_cmds[k])));
    precmd = command.substring(0, (inputstr.length() - (strlen(set_cmds[k]))));
    if (recvcmd == String(set_cmds[k])) {
      setcmd = k;
      valid = true;
      precmdnum = precmd.toInt();
    }
  }

  //Go through all 'get value' commands if it wasn't a set command
  if (!valid) {
    for (int k = 0; k < (sizeof(get_cmds) / sizeof(char*)); k++) {
      recvcmd = command.substring((inputstr.length() - (strlen(get_cmds[k]))));
      if (String(get_cmds[k]).equals(recvcmd)) {
        valid = true;
        getcmd = k;
      }
    }
  }

  if (valid == true) {
    //Command is valid
    //Reminder: const char* set_cmds[] = {ANG_set, ACC_set, VMAX_set, R_start, R_stop, R_CW, R_CC, ANG_rel};
    //Now work out what command was to be executed...
    switch (setcmd) {
      case 0:
        set_NUM(precmdnum);
        break; 
      case 1:
        set_MODE(precmdnum);
        break;            
      default:
        //Serial.print("SET COMMAND NOT RECOGNISED\r\n");
        break;
    }

    //Reminder: char* get_cmds[] = {ANG_get, ACC_get, VMAX_get, REV_get};
    //Get?
    switch (getcmd) {
      case 0:
        get_NUM();
        break;
      case 1:
        set_MODE(precmdnum);
        break;         
      default:
        //Serial.print("GET COMMAND NOT RECOGNISED\r\n");
        break;
    }
    Serial.print(" ok\r\n");
  } else {
    Serial.print(" ");
    Serial.print(inputstr);
    Serial.print(" was input - ok\r\n");
  }
}

void Respond(String hum_message, String PC_message, bool human_readable) {
  if (human_readable == true) {
    Serial.print("\n" + hum_message);
  } else {
    Serial.print("\n" + PC_message);
  }
  response_msg = "";
}


/////////////////////////////////////////////////////////////
//Keystroke handling
void handle_A(){
  max_v = constrain((max_v-1),1,v_lim);
}

void handle_D(){
  del_step = constrain((del_step-del_stride),10,del_max);
}

void handle_E(){
  v_trail_len = constrain((v_trail_len-1),1,MAX_LEDS);
}

void handle_F(){
  del_step = constrain((del_step+del_stride),10,del_max);
}

void handle_R(){
  v_trail_len = constrain((v_trail_len+1),1,MAX_LEDS);
}

void handle_S(){
  max_v = constrain((max_v+1),1,v_lim);
}

void handle_W(){
  jump(1);
}

void handle_Q(){
  jump(-1);
}

void handle_X(){
  h_step = constrain((h_step+1),1,255);
}

void handle_Z(){
  h_step = constrain((h_step-1),1,255);
}

/////////////////////////////////////////////////////////////
//Input checking and interrput raising
void check_but(){
  boolean curr_butval = !digitalRead(ROTENC_BUT);//PULLUP INVERT!
  if(millis()>last_click+click_interval){
    //end of old event, execute and reset for new
    if(clicks>0){
      //Serial.print(clicks);
      set_MODE(clicks);
      released = false;
    } 
    clicks = 0;
    last_click = millis();
    but_was = true;//don't forget debouncing! incrementing comes later
  } else {
    if(millis()>last_click+but_debounce && curr_butval == but_was){
      if(curr_butval == but_was){
        change_det = true;//debounced change
        if(but_was == true){
          //debounced change was to pressed
          clicks++;
        } else {
          //reset timer on release
          last_click = millis();
          if(clicks>1){
            released = true;
          }          
        }
        but_was = !curr_butval;//now flip it, wait for transition other way
      }
    }
  }
}

void INT0_raise(){
  int_flag = true;
  delay(but_debounce);
  boolean clk_val = digitalRead(ROTENC_CLK);
  boolean dat_val = digitalRead(ROTENC_DAT);
  if(clk_val){
    if(dat_val){
      re_ctr--;
    } else {
      re_ctr++;  
    } 
  } else {
    if(!dat_val){
      re_ctr--;
    } else {
      re_ctr++;
    }
  }
}

/////////////////////////////////////////////////////////////
//Command actions
void set_NUM(int input_num){
  curr_led = input_num;
}

int get_NUM(){
  line_term();
  Serial.print(curr_led,DEC);
  line_term();
  return curr_led;
}

void set_MODE(int whichmode){
  FastLED.clear();
  if(curr_mode == 1){
    //only toggle on/off in mode 1
    if(whichmode == 1){
      run_off_toggle = !run_off_toggle;
    } else {
      //if coming from another mode, force to on
      run_off_toggle = false;
    }
  }
  curr_mode = whichmode;
}

int get_MODE(){
  line_term();
  Serial.print(curr_mode,DEC);
  line_term();
  return curr_mode;  
}

void line_term(){
  Serial.print("\r\n");
}

/////////////////////////////////////////////////////////////
//General functions
void e_stop(){
  //No emergency stop actions yet
}

void update_leds(){
  switch(curr_mode){
    case MODE_RUN_OFF:
      if(!run_off_toggle){
        FastLED.clear();
      } else {
        v_step = max_v/(v_trail_len);
        for(int i=0;i<v_trail_len;i++){
          leds[constrain((ctr+v_trail_len-i)%MAX_LEDS,0,MAX_LEDS)] = CHSV(h_curr-(h_step*i),sat_val,max_v-(v_step*i));
        }
        leds[(ctr)%MAX_LEDS] = CRGB::Black;
        h_curr+=h_step;
        break;        
      }
      break;
    case MODE_POS:
      highlight(curr_led);      
      break;
    case MODE_LENGTH:
      FastLED.clear();
      for (int i=0; i<MAX_LEDS;i++){
        if(i>(MAX_LEDS-v_trail_len)/2 && i<(MAX_LEDS+v_trail_len)/2){
          leds[i]=config_colour;
        }
      }
      break;
    case MODE_INTERVAL:
      if(cylon_dir){
        if(cylon_trackpos==MAX_LEDS-2){
          cylon_dir = false;
        }
        cylon_trackpos++;
      } else{
        if(cylon_trackpos==1){
          cylon_dir = true;
        }
        cylon_trackpos--;
      }
      highlight(cylon_trackpos);
      break;
    case MODE_BRIGHT:
      //fill_solid(leds,MAX_LEDS,CHSV(config_colour.hue,config_colour.sat,max_v));
      fill_solid(leds,MAX_LEDS,CHSV(0,255,max_v));
      config_colour.setRGB((orig_config_colour.r*max_v)/255,(orig_config_colour.g*max_v)/255,(orig_config_colour.b*max_v)/255);
      break;
    case MODE_HUESTRIDE:
      for(int i=0;i<MAX_LEDS;i++){
        leds[i] = CHSV(h_curr-(h_step*i),sat_val,max_v);
      }
      break;
    default:
      highlight(curr_led);      
      break;
  }
  FastLED.show();
}

void jump(int jumpval){
  curr_led = constrain((curr_led+jumpval),0,MAX_LEDS-1);
}

void highlight(int LEDnum){
  FastLED.clear();
  leds[LEDnum] = config_colour;
}

void handle_rot(){
  boolean dirn = true;
  if(re_ctr>last_re_ctr){
    dirn = false;
  }  
  switch(curr_mode){
    case MODE_RUN_OFF:
      //no response to rotary
      break;
////////////////////
    case MODE_POS:
      if(dirn){
        handle_Q();
      } else {
        handle_W();
      }
      break;
////////////////////
    case MODE_LENGTH:
      if(dirn){
        handle_E();
      } else {
        handle_R();
      }
      break;
////////////////////
    case MODE_BRIGHT:
      if(dirn){
        handle_A();
      } else {
        handle_S();
      }
      break;
////////////////////
    case MODE_INTERVAL:
      if(dirn){
        handle_D();
      } else {
        handle_F();
      }
      break;          
////////////////////
    case MODE_HUESTRIDE:
      if(dirn){
        handle_Z();
      } else {
        handle_X();
      }
      break;                        
  }
  last_re_ctr = re_ctr;
}
