#include <string.h>
#include <math.h>
#include <vector>
#include <cstdlib>

#include"pico_explorer.hpp"
#include "pico/stdlib.h"
#include "encoder.hpp"
#include "st7789.hpp"
#include "pico_graphics.hpp"
#include "pico_vector.hpp"
#include "button.hpp"

using namespace pimoroni;
using namespace encoder;

ST7789 st7789(PicoExplorer::WIDTH, PicoExplorer::HEIGHT, ROTATE_0, false, get_spi_pins(BG_SPI_FRONT));
PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);
PicoVector vector(&graphics);

int main() {
  stdio_init_all();
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

    angle<360.0 ? angle+=1.0f :angle=0.0f;
    //printf("angle: %.1f\n",angle);

    pp_poly_free(poly2);
  }
  pp_poly_free(poly0);
  pp_poly_free(poly1);
  
  return 0;
}
