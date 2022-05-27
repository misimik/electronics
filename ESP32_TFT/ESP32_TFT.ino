#include <lvgl.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include "ADS1X15.h"
#include "MCP4725.h"
/* LVGL library might have breaking changes in updates.
 *  Make sure that the the code is in agreement with current
 *  display and input device porting requirements
*/

ESP32Encoder encoder;
const int buttonPin = 33;
const int encoderPinA = 2;
const int encoderPinB = 15;
const int TECpin    = 32;
static ADS1115 ADS(0x48); // Initialize ADC - ADS1115
static MCP4725 MCP(0x60); // Initialize DAC - MCP4725


/*Change to your screen resolution*/
static const uint16_t screenWidth  = 135;
static const uint16_t screenHeight = 240;
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

static lv_obj_t * TEClabel;
static lv_obj_t * reported_temp_label;
static lv_obj_t * secondary_temp_label;
static lv_obj_t * target_temp_slider_label;
static lv_obj_t * base_temp_label;
static lv_obj_t * reported_target_temp_label;
static lv_obj_t * web_label;
static lv_obj_t * secondary_temp_bar;
static lv_obj_t * base_temp_bar;
static lv_obj_t * reported_temp_bar;
static lv_obj_t * reported_target_temp_bar;

/* Global variables */
static int WG_temp_limit_low = 10;
static int WG_temp_limit_high = 90;
static bool tec_status = false;
static bool web_status = true;
static float target_temp = 25;

float temp_to_resistance(float temp, float ref_resistance, float ref_temp, float beta){
  return ref_resistance * exp( beta * ( (1/( temp + 273. ) ) - ( 1/( ref_temp + 273. ) ) ));
}

float resistance_to_temp( float resistance, float ref_resistance, float ref_temp, float beta ){
  return 1/( 1/( ref_temp + 273. ) + 1/beta*log( resistance / ref_resistance ) ) - 273.;
}

float voltage_to_resistance( float voltage, float ref_voltage, float ref_resistance ){
  /* This is for the voltage divider.
  The voltage is measured across  the changing resistance */
  return ref_resistance / ( ref_voltage - voltage ) * voltage;
}

float resistance_to_voltage( float resistance ){
  /* This is for the Wavelength Electronics PTC10K-CH
     Here a 100 uA current is pushed through the thermistor
     and the voltage drop must be equal to the controlling voltage*/
  return resistance / 10000.;
}

void update_target_temp(float target_temp){
//  Thermistor No. 0: R = 9977, T0 = 24.95, beta = 3464
  float target_res = temp_to_resistance( target_temp, 9977, 24.95, 3464. );
  float target_volt = resistance_to_voltage( target_res );
//  4095 levels of 12bit MCP4725 and 3.3 V of the reference voltage (ESP32 LDO)
  MCP.setValue( 4095/3.3 * target_volt );
}

void update_measured_temps(lv_timer_t * timer){
  float f = ADS.toVoltage(1);
  float volt_0 = ADS.readADC(0)*f;
  float volt_1 = ADS.readADC(1)*f;
  float volt_2 = ADS.readADC(2)*f;
  float volt_3 = ADS.readADC(3)*f;
//  Ref voltage: ESP32 LDO = 3.3 V
//  Resistor 1 = 9.86 k Ohm
//  Resistor 2 = 9.91 k Ohm
  float res_0 = volt_0 * 10000;
  float res_1 = voltage_to_resistance( volt_1, 3.3, 9860 );
  float res_2 = voltage_to_resistance( volt_1, 3.3, 9910 );
  float res_3 = volt_3 * 10000;
//  Thermistor 0: T = 9977, T0 = 24.95, beta = 3464
//  Thermistor 2: R = 9962, T0 = 24.89, beta = 3424
//  Thermistor 4: R = 9943, T0 = 24.84, beta = 3450
  float temp_0 = resistance_to_temp( res_0, 9977., 24.95, 3464. );
  float temp_1 = resistance_to_temp( res_1, 9962., 24.89, 3424. );
  float temp_2 = resistance_to_temp( res_2, 9943., 24.84, 3450. );
//  Also uses Thermistor 0
  float temp_3 = resistance_to_temp( res_3, 9977., 24.95, 3464. );

  char buf2[24];

  lv_snprintf(buf2, sizeof(buf2), "Reported WG temp: %d C", (int)temp_0 );
  lv_label_set_text(reported_temp_label, buf2);
  lv_bar_set_value(reported_temp_bar, (int)temp_0, LV_ANIM_OFF);
  
  lv_snprintf(buf2, sizeof(buf2), "Secondary WG temp: %d C", (int)temp_1 );
  lv_label_set_text(secondary_temp_label, buf2);
  lv_bar_set_value(secondary_temp_bar, (int)temp_1, LV_ANIM_OFF);

  lv_snprintf(buf2, sizeof(buf2), "Current base temp: %d C", (int)temp_2 );
  lv_label_set_text(base_temp_label, buf2);
  lv_bar_set_value(base_temp_bar, (int)temp_2, LV_ANIM_OFF);  

  lv_snprintf(buf2, sizeof(buf2), "Rep. targ. WG temp: %d C", (int)temp_3 );
  lv_label_set_text(reported_target_temp_label, buf2);
  lv_bar_set_value(reported_target_temp_bar, (int)temp_3, LV_ANIM_OFF);  

  Serial.print( "Reported Temp:\t" ); Serial.println( temp_0 );
  Serial.print( "Secondary Temp:\t" ); Serial.println( temp_1 );
  Serial.print( "Base Temp:\t" ); Serial.println( temp_2 );
  Serial.print( "Reported target Temp:\t" ); Serial.println( temp_3 );
}

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
    
    lv_disp_flush_ready( disp );
}

/* Reading input device (simulated encoder here) */
void read_encoder(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    static int32_t last_diff = 0;
    int32_t diff = (int32_t)floor(encoder.getCount() / 2.0);
    data->state = (not(digitalRead(buttonPin)) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL);
    data->enc_diff = diff - last_diff;
    last_diff = diff;
}

void TECbtn_event_cb(lv_event_t * e)
{
  Serial.println("Toggled - TEC");
  tec_status = not(tec_status);
  if(tec_status){
    lv_label_set_text(TEClabel, "TEC: ON");
  }
  else{
    lv_label_set_text(TEClabel, "TEC: OFF");
  }
  digitalWrite(TECpin, tec_status);
}

void web_btn_event_cb(lv_event_t * e)
{
  Serial.println("Toggled - WEB");
  web_status = not(web_status);
  if(web_status){
    lv_label_set_text(web_label, "WEB: ON");
  }
  else{
    lv_label_set_text(web_label, "WEB: OFF");
  }
//  digitalWrite(TECpin, tec_status);
}

static void target_temp_slider_event_cb(lv_event_t * e)
{
    lv_obj_t * target_temp_slider = lv_event_get_target(e);
    char buf[24];
    target_temp = lv_slider_get_value(target_temp_slider);
    lv_snprintf(buf, sizeof(buf), "Target WG temp: %d C", (int)target_temp );
    lv_label_set_text(target_temp_slider_label, buf);
    update_target_temp( target_temp );
    
//    lv_obj_align_to(target_temp_slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

//void focus_cb(lv_group_t * group) {
//  // Get the pointer to the focused object
//  static lv_obj_t * oldFocused = NULL; //static makes variable persistent
//  lv_obj_t * newFocused = lv_group_get_focused(group);
//  // Avoid a refocus loop
//  if (newFocused != oldFocused) {
//    // Page is a grandparent to the focused object
//    lv_page_focus(lv_obj_get_parent(lv_obj_get_parent(newFocused)), newFocused, LV_ANIM_ON);
//  }
//  oldFocused = newFocused;
//}

void setup()
{
    lv_init();
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    Wire.begin();
    pinMode(TECpin, OUTPUT);
    pinMode(buttonPin, INPUT);
    
    ESP32Encoder::useInternalWeakPullResistors = UP;
    // Attache pins for use as encoder pins
    encoder.attachHalfQuad(encoderPinA, encoderPinB);

    ADS.setGain(1); // Gain 0: +/- 6V; Gain 1: +/- 4V
    MCP.begin();
    MCP.writeDAC( 4095/3.3 * resistance_to_voltage( temp_to_resistance( 25, 10000, 25, 3450 ) ), true );

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    
    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */
    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
//    uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
//    tft.setTouch( calData );

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;              /*Descriptor of a display driver*/
    lv_disp_drv_init( &disp_drv );              /*Basic initialization*/
    disp_drv.hor_res = screenWidth;             /*Screen horizontal resolution*/
    disp_drv.ver_res = screenHeight;            /*Screen vertical resolution*/
    disp_drv.flush_cb = my_disp_flush;          /*Set your driver function*/
    disp_drv.draw_buf = &draw_buf;              /*Assign the buffer to the display*/
    lv_disp_t * disp;
    disp = lv_disp_drv_register( &disp_drv );   /*Finally register the driver*/
    lv_disp_drv_register( &disp_drv );
//  This together with tft.setRotation allows to correctly display in horizontal mode.
    lv_disp_set_rotation( disp, LV_DISP_ROT_90 );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;            /*Descriptor of a input device driver*/
    lv_indev_drv_init( &indev_drv );            /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_ENCODER;     /*Encoder is a Encoder-like device*/
    indev_drv.read_cb = read_encoder;           /*Set your driver function*/
    lv_indev_t *indev = lv_indev_drv_register( &indev_drv );        /*Finally register the driver*/

    /*Create cont_col as a container for widgets*/
    lv_obj_t * cont_col = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont_col, screenHeight, screenWidth); /*The page will take full screen*/
    lv_obj_align(cont_col, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_flex_flow(cont_col, LV_FLEX_FLOW_COLUMN);

    /* TEC engage indicator */
    lv_obj_t * TECbtn = lv_btn_create(cont_col);     /*Add a button to the current screen*/
    lv_obj_set_size(TECbtn, LV_PCT(100), LV_SIZE_CONTENT );                          /*Set its size*/
    lv_obj_add_event_cb(TECbtn, TECbtn_event_cb, LV_EVENT_CLICKED, NULL);                 /*Assign a callback to the button*/                  
    lv_obj_add_flag(TECbtn, LV_OBJ_FLAG_CHECKABLE);           /*Make button checkable*/
    lv_obj_add_flag(TECbtn, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(TECbtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );  
    TEClabel = lv_label_create(TECbtn);          /*Add a label to the button*/
    lv_label_set_text(TEClabel, "TEC: OFF");                     /*Set the labels text*/
    lv_obj_align_to(TEClabel, TECbtn, LV_ALIGN_CENTER, 0, 0);

    /* WG reported temp indicator */
    reported_temp_label = lv_label_create(cont_col);
    lv_label_set_text(reported_temp_label, "Reported WG temp: 65 C");
    reported_temp_bar =  lv_bar_create(cont_col);
    lv_obj_set_size(reported_temp_bar, LV_PCT(100), 15 );
    lv_obj_center(reported_temp_bar);
    lv_bar_set_value(reported_temp_bar, 65, LV_ANIM_OFF);
    lv_bar_set_range(reported_temp_bar, WG_temp_limit_low, WG_temp_limit_high);
    lv_obj_add_flag(reported_temp_bar, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(reported_temp_bar, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );

    /* WG secondary temp indicator */
    secondary_temp_label = lv_label_create(cont_col);
    lv_label_set_text(secondary_temp_label, "Secondary WG temp: 65 C");
    secondary_temp_bar =  lv_bar_create(cont_col);
    lv_obj_set_size(secondary_temp_bar, LV_PCT(100), 15 );
    lv_obj_center(secondary_temp_bar);
    lv_bar_set_value(secondary_temp_bar, 65, LV_ANIM_OFF);
    lv_bar_set_range(secondary_temp_bar, WG_temp_limit_low, WG_temp_limit_high);
    lv_obj_add_flag(secondary_temp_bar, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(secondary_temp_bar, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );

    /* WG temp control */
    target_temp_slider_label = lv_label_create(cont_col);
    lv_label_set_text(target_temp_slider_label, "Target WG temp: 25 C");
    lv_obj_t * target_temp_slider = lv_slider_create(cont_col);
    lv_obj_set_size(target_temp_slider, LV_PCT(100), 15 );
    lv_slider_set_range(target_temp_slider, WG_temp_limit_low, WG_temp_limit_high);
    lv_slider_set_value(target_temp_slider, 25, LV_ANIM_OFF);
//    lv_obj_center(target_temp_slider);
    lv_obj_add_event_cb(target_temp_slider, target_temp_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_flag(target_temp_slider, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );

    /* Base temperature indicator */
    base_temp_label = lv_label_create(cont_col);
    lv_label_set_text(base_temp_label, "Current base temp: 25 C");
    base_temp_bar =  lv_bar_create(cont_col);
    lv_obj_set_size(base_temp_bar, LV_PCT(100), 15 );
    lv_obj_center(base_temp_bar);
    lv_bar_set_value(base_temp_bar, 25, LV_ANIM_OFF);
    lv_bar_set_range(base_temp_bar, 0, 100);
    lv_obj_add_flag(base_temp_bar, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(base_temp_bar, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );

    /* WG reported target temp indicator */
    reported_target_temp_label = lv_label_create(cont_col);
    lv_label_set_text(reported_target_temp_label, "Rep. targ. WG temp: 65 C");
    reported_target_temp_bar =  lv_bar_create(cont_col);
    lv_obj_set_size(reported_target_temp_bar, LV_PCT(100), 15 );
    lv_obj_center(reported_target_temp_bar);
    lv_bar_set_value(reported_target_temp_bar, 65, LV_ANIM_OFF);
    lv_bar_set_range(reported_target_temp_bar, WG_temp_limit_low, WG_temp_limit_high);
    lv_obj_add_flag(reported_target_temp_bar, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(reported_target_temp_bar, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );

    /* Web server engage indicator */
    lv_obj_t * web_btn = lv_btn_create(cont_col);     /*Add a button to the current screen*/
    lv_obj_set_size(web_btn, LV_PCT(100), LV_SIZE_CONTENT );                          /*Set its size*/
    lv_obj_add_event_cb(web_btn, web_btn_event_cb, LV_EVENT_CLICKED, NULL);                 /*Assign a callback to the button*/                  
    lv_obj_add_flag(web_btn, LV_OBJ_FLAG_CHECKABLE);           /*Make button checkable*/
    lv_obj_add_flag(web_btn, LV_OBJ_FLAG_CLICK_FOCUSABLE );
    lv_obj_add_flag(web_btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS  );  
    web_label = lv_label_create(web_btn);          /*Add a label to the button*/
    lv_label_set_text(web_label, "WEB: ON");                     /*Set the labels text*/
    lv_obj_align_to(web_label, web_btn, LV_ALIGN_CENTER, 0, 0);

//    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    
//    /* Create simple label */
//    lv_obj_t *label = lv_label_create( cont_col );
//    lv_label_set_text( label, LVGL_Arduino.c_str() );
//    lv_obj_align_to( label, cont_col, LV_ALIGN_CENTER, 0, 0 );

    Serial.println( "Setup done" );

    lv_group_t * g = lv_group_create();
// Focus callback makes sure that focused object is visible
    lv_group_set_focus_cb(g, NULL);
// Add all the buttons and sliders to the group to control them
    lv_group_add_obj(g, TECbtn);
    lv_group_add_obj(g, reported_temp_bar);
    lv_group_add_obj(g, secondary_temp_bar);
    lv_group_add_obj(g, target_temp_slider);
    lv_group_add_obj(g, base_temp_bar);
    lv_group_add_obj(g, reported_target_temp_bar);
    lv_group_add_obj(g, web_btn);

// Set the encoder as the input device for the group
    lv_indev_set_group(indev, g);

// Timers - periodically called functions
    static uint32_t user_data = 10;
    lv_timer_t * timer_utt = lv_timer_create( update_measured_temps, 1000, NULL);
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay( 5 );
}
