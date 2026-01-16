
#include "drivers/bmp280.h"
#include "drivers/ssd1306.h"


#ifndef APP_UI_HANDLER_H_
#define APP_UI_HANDLER_H_

void ui_refresh( bmp280_t *bmp280, ssd1306_t *ssd1306 );

#endif /* APP_UI_HANDLER_H_ */
