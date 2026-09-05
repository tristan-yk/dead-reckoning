/**
  ******************************************************************************
  * @file           : display.h
  * @brief          : SH1106 128x64 OLED with a text framebuffer.
  *
  * Drawing writes into a RAM framebuffer and marks the affected rows dirty.
  * Pushing to the panel is a separate, incremental step: display_service()
  * sends at most one 8-pixel page per call, because a full frame is about
  * 10 ms of blocking I2C even at 400 kHz and the filter cannot give that up
  * inside a 10 ms tick. Pages that have not changed cost nothing, so a screen
  * showing steady values stops transferring entirely.
  ******************************************************************************
  */

#ifndef DISPLAY_H
#define DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64
#define DISPLAY_PAGES  (DISPLAY_HEIGHT / 8)
#define DISPLAY_COLS   (DISPLAY_WIDTH / 6) /* 21 characters per row */
#define DISPLAY_ROWS   DISPLAY_PAGES       /* one text row per page */

bool display_init(void);

/* Clears the framebuffer. Nothing reaches the panel until display_service. */
void display_clear(void);

/* Draws text at a character cell, clipped to the panel. Row is 0..7, column is
 * 0..20. Characters outside the font render as blanks. */
void display_text(uint8_t row, uint8_t col, const char *text);

/* Draws text and pads the rest of the row with spaces, so a shorter string
 * cannot leave fragments of the previous one behind. */
void display_line(uint8_t row, const char *text);

/* Sends at most one changed page. Call once per tick; a full refresh takes at
 * most DISPLAY_PAGES calls. Returns true if a page was transferred. */
bool display_service(void);

/* True while any page still differs from what the panel is showing. */
bool display_dirty(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
