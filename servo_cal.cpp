#include <string.h>
#include <math.h>
#include <vector>
#include <cstdlib>

#include"pico_explorer.hpp"
#include "pico/stdlib.h"
#include "encoder.hpp"
#include "quadrature_out.pio.h"
#include "st7789.hpp"
#include "pico_graphics.hpp"
#include "pico_vector.hpp"
#include "button.hpp"

using namespace pimoroni;
using namespace encoder;

//--------------------------------------------------
// Constants
//--------------------------------------------------
// The pins used by the encoder
static const pin_pair ENCODER_PINS = {0, 1};
static const uint ENCODER_COMMON_PIN = 2;
static const uint ENCODER_SWITCH_PIN = 3;

// The counts per revolution of the encoder's output shaft
static constexpr float COUNTS_PER_REV = encoder::ROTARY_CPR;

// Set to true if using a motor with a magnetic encoder
static const bool COUNT_MICROSTEPS = false;

// Increase this to deal with switch bounce. 250 Gives a 1ms debounce
static const uint16_t FREQ_DIVIDER = 1;

// Time between each sample, in microseconds
static const int32_t TIME_BETWEEN_SAMPLES_US = 100;

// The full time window that will be stored
static const int32_t WINDOW_DURATION_US = 1000000;

static const int32_t READINGS_SIZE = WINDOW_DURATION_US / TIME_BETWEEN_SAMPLES_US;
static const int32_t SCRATCH_SIZE = READINGS_SIZE / 10;   // A smaller value, for temporarily storing readings during screen drawing

// Whether to output a synthetic quadrature signal
static const bool QUADRATURE_OUT_ENABLED = true;

// The frequency the quadrature output will run at (note that counting microsteps will show 4x this value)
static constexpr float QUADRATURE_OUT_FREQ = 800;

// Which first pin to output the quadrature signal to (e.g. GP6 and GP7)
static const float QUADRATURE_OUT_1ST_PIN = 6;

// How long there should be in microseconds between each screen refresh
static const uint64_t MAIN_LOOP_TIME_US = 50000;

// The zoom level beyond which edge alignment will be enabled to make viewing encoder patterns look nice
static const uint16_t EDGE_ALIGN_ABOVE_ZOOM = 4;
//--------------------------------------------------
// Enums
//--------------------------------------------------
enum DrawState {
  DRAW_LOW = 0,
  DRAW_HIGH,
  DRAW_TRANSITION,
};
//--------------------------------------------------
// Variables
//--------------------------------------------------
ST7789 st7789(PicoExplorer::WIDTH, PicoExplorer::HEIGHT, ROTATE_0, false, get_spi_pins(BG_SPI_FRONT));
PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);
PicoVector vector(&graphics);

Button button_a(PicoExplorer::A);
Button button_b(PicoExplorer::B);
Button button_x(PicoExplorer::X);
Button button_y(PicoExplorer::Y);

//Motor motor1(PicoExplorer::MOTOR1_PINS);

Encoder enc(pio0, 0, ENCODER_PINS, ENCODER_COMMON_PIN, NORMAL_DIR, COUNTS_PER_REV, COUNT_MICROSTEPS, FREQ_DIVIDER);

volatile bool enc_a_readings[READINGS_SIZE];
volatile bool enc_b_readings[READINGS_SIZE];
volatile bool enc_a_scratch[SCRATCH_SIZE];
volatile bool enc_b_scratch[SCRATCH_SIZE];
volatile uint32_t next_reading_index = 0;
volatile uint32_t next_scratch_index = 0;
volatile bool drawing_to_screen = false;
uint16_t current_zoom_level = 1;



////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t draw_plot(Point p1, Point p2, volatile bool (&readings)[READINGS_SIZE], uint32_t reading_pos, bool edge_align) {
  uint32_t reading_window = READINGS_SIZE / current_zoom_level;
  uint32_t start_index_no_modulus = (reading_pos + (READINGS_SIZE - reading_window));
  uint32_t start_index = start_index_no_modulus % READINGS_SIZE;
  int32_t screen_window = std::min(p2.x, (int32_t)PicoExplorer::WIDTH) - p1.x;

  bool last_reading = readings[start_index % READINGS_SIZE];

  uint32_t alignment_offset = 0;
  if(edge_align) {
    // Perform edge alignment by first seeing if there is a window of readings available (will be at anything other than x1 zoom)
    uint32_t align_window = (start_index_no_modulus - reading_pos);

    // Then go backwards through that window
    for(uint32_t i = 1; i < align_window; i++) {
      uint32_t align_index = (start_index + (READINGS_SIZE - i)) % READINGS_SIZE;
      bool align_reading = readings[align_index];

      // Has a transition from high to low been detected?
      if(!align_reading && align_reading != last_reading) {
        // Set the new start index from which to draw from and break out of the search
        start_index = align_index;
        alignment_offset = i;
        break;
      }
      last_reading = align_reading;
    }

    last_reading = readings[start_index % READINGS_SIZE];
  }

  // Go through each X pixel within the screen window
  uint32_t reading_window_start = 0;
  for(int32_t x = 0; x < screen_window; x++) {
    uint32_t reading_window_end = ((x + 1) * reading_window) / screen_window;

    // Set the draw state to be whatever the last reading was
    DrawState draw_state = last_reading ? DRAW_HIGH : DRAW_LOW;

    // Go through the readings in this window to see if a transition from low to high or high to low occurs
    if(reading_window_end > reading_window_start) {
      for(uint32_t i = reading_window_start; i < reading_window_end; i++) {
        bool reading = readings[(i + start_index) % READINGS_SIZE];
        if(reading != last_reading) {
          draw_state = DRAW_TRANSITION;
          break;  // A transition occurred, so no need to continue checking readings
        }
        last_reading = reading;
      }
      last_reading = readings[((reading_window_end - 1) + start_index) % READINGS_SIZE];
    }
    reading_window_start = reading_window_end;

    // Draw a pixel in a high or low position, or a line between the two if a transition
    switch(draw_state) {
      case DRAW_TRANSITION:
        for(uint8_t y = p1.y; y < p2.y; y++)
          graphics.pixel(Point(x + p1.x, y));
        break;
      case DRAW_HIGH:
        graphics.pixel(Point(x + p1.x, p1.y));
        break;
      case DRAW_LOW:
        graphics.pixel(Point(x + p1.x, p2.y - 1));
        break;
    }
  }

  // Return the alignment offset so subsequent encoder channel plots can share the alignment
  return alignment_offset;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool repeating_timer_callback(struct repeating_timer *t) {
  bool_pair state = enc.state();
  if(drawing_to_screen && next_scratch_index < SCRATCH_SIZE) {
    enc_a_scratch[next_scratch_index] = state.a;
    enc_b_scratch[next_scratch_index] = state.b;
    next_scratch_index++;
  }
  else {
    enc_a_readings[next_reading_index] = state.a;
    enc_b_readings[next_reading_index] = state.b;

    next_reading_index++;
    if(next_reading_index >= READINGS_SIZE) {
      next_reading_index = 0;
    }
  }

  return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  stdio_init_all();

#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

  if(ENCODER_SWITCH_PIN != PIN_UNUSED) {
    gpio_init(ENCODER_SWITCH_PIN);
    gpio_set_dir(ENCODER_SWITCH_PIN, GPIO_IN);
    gpio_pull_down(ENCODER_SWITCH_PIN);
  }

  //motor1.init();
  enc.init();

  bool_pair state = enc.state();
  for(uint i = 0; i < READINGS_SIZE; i++) {
    enc_a_readings[i] = state.a;
    enc_b_readings[i] = state.b;
  }

  if(QUADRATURE_OUT_ENABLED) {
    // Set up the quadrature encoder output
    PIO pio = pio1;
    uint offset = pio_add_program(pio, &quadrature_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    quadrature_out_program_init(pio, sm, offset, QUADRATURE_OUT_1ST_PIN, QUADRATURE_OUT_FREQ);
  }
}


int main() {
  setup();
  //stdio_init_all();
  st7789.set_backlight(255);

  Pen WHITE = graphics.create_pen(255, 255, 255);
  Pen BLACK = graphics.create_pen(0, 0, 0);
  Pen BLUE  = graphics.create_pen(0,0,255);
  Pen GREEN  = graphics.create_pen(0,255,0);

  pp_poly_t *poly0 = pp_poly_new();
  pp_poly_t *poly1 = pp_poly_new();

  pp_poly_merge(poly0,ppp_arc((ppp_arc_def){0, 0, 100, 20, 20, 340}));
  pp_poly_merge(poly0,ppp_circle((ppp_circle_def){0, 0, 10, 1}));

  for(int i = -3; i < 4; i++) {
    poly1=ppp_line((ppp_line_def){0 , -80, 0 , -100, 2});
    vector.rotate(poly1,{0,0},i*45);
    pp_poly_merge(poly0, poly1);      
  }

  vector.translate(poly0, {120, 120});

  pp_point_t pointer[] = {{ -80,   0}, { -70,   10}, { -70, 5}, { 0, 5}, { 0, -5}, { -70, -5}, { -70, -10}};

  float angle = 0.0f;
  int value =90;

  while(true) {

    //reset background
    graphics.set_pen(BLUE);
    graphics.clear();
    graphics.set_pen(WHITE);
    graphics.text("Servo Calibrator", Point(35, 0), 320);
    graphics.set_pen(GREEN);
    vector.draw(poly0);

    // create each new pointer
    pp_poly_t *poly2 = pp_poly_new();
    pp_path_add_points(pp_poly_add_path(poly2), pointer, sizeof(pointer) / sizeof(pp_point_t));

    // move new pointer
    vector.rotate(poly2, {0, 0}, angle);
    vector.translate(poly2, {120, 120});

    // draw new pointer
    graphics.set_pen(WHITE);
    vector.draw(poly2);

    // update screen
    st7789.update(&graphics);

    //angle<360.0 ? angle+=1.0f :angle=0.0f;

    int32_t delta = enc.delta();
    if(delta != 0){
      if (delta>0)
        value = MIN(value + 1, 360);
      else
        value = MAX(value - 1, 0);
      }
    
    angle = (float)value;

    printf("angle: %.1f\n",angle);

    pp_poly_free(poly2);
  }
  pp_poly_free(poly0);
  pp_poly_free(poly1);

  return 0;
}
