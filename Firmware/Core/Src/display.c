/**
  ******************************************************************************
  * @file           : display.c
  * @brief          : SH1106 128x64 OLED with a text framebuffer.
  ******************************************************************************
  */

#include "display.h"

#include <string.h>

#include "font5x7.h"
#include "i2c_bus.h"
#include "main.h"

#define OLED_ADDR 0x3C

/* This panel's RAM is wider than its glass, and column 0 of the display sits at
 * column 2 of RAM. Without the offset the last two columns wrap. */
#define COL_OFFSET 2

/* One glyph cell: five columns of font plus a one-column gap. */
#define CELL_WIDTH 6

/* Bytes per data transfer. The SH1106 accepts an arbitrary run after the 0x40
 * control byte, but chunking bounds how long a single I2C transaction blocks. */
#define CHUNK 32

static uint8_t framebuffer[DISPLAY_PAGES][DISPLAY_WIDTH];

/* One bit per page, set when the framebuffer no longer matches the panel. */
static uint8_t dirty_pages;

/* Where the round-robin scan resumes, so no page can be starved by a page
 * above it being rewritten every frame. */
static uint8_t next_page;

static bool send_command(uint8_t cmd)
{
  uint8_t packet[2] = {0x00, cmd}; /* 0x00 selects the command stream */
  return i2c_transmit(OLED_ADDR, packet, sizeof(packet), 20);
}

static bool send_data(const uint8_t *data, uint16_t len)
{
  uint8_t buffer[1 + CHUNK];
  buffer[0] = 0x40; /* 0x40 selects the data stream */

  while (len > 0u) {
    uint16_t chunk = (len > CHUNK) ? CHUNK : len;
    memcpy(&buffer[1], data, chunk);
    if (!i2c_transmit(OLED_ADDR, buffer, (uint16_t)(1 + chunk), 20)) {
      return false;
    }
    data += chunk;
    len = (uint16_t)(len - chunk);
  }
  return true;
}

bool display_init(void)
{
  HAL_Delay(50); /* the panel ignores commands until its regulator settles */

  static const uint8_t sequence[] = {
    0xAE,             /* display off while configuring          */
    0xD5, 0x80,       /* display clock divide / oscillator      */
    0xA8, 0x3F,       /* multiplex ratio for 64 rows            */
    0xD3, 0x00,       /* no display offset                      */
    0x40,             /* start line 0                           */
    0xAD, 0x8B,       /* enable the built-in DC-DC converter    */
    0xA1,             /* segment remap, so column 0 is at left  */
    0xC8,             /* reverse COM scan, so row 0 is at top   */
    0xDA, 0x12,       /* alternate COM pin configuration        */
    0x81, 0x7F,       /* contrast                               */
    0xD9, 0x22,       /* pre-charge period                      */
    0xDB, 0x20,       /* VCOMH deselect level                   */
    0xA4,             /* follow RAM rather than all-on          */
    0xA6,             /* normal, not inverted                   */
    0xAF,             /* display on                             */
  };

  for (unsigned i = 0; i < sizeof(sequence); i++) {
    if (!send_command(sequence[i])) {
      return false;
    }
  }

  display_clear();
  return true;
}

void display_clear(void)
{
  memset(framebuffer, 0, sizeof(framebuffer));
  dirty_pages = (uint8_t)((1u << DISPLAY_PAGES) - 1u);
}

void display_text(uint8_t row, uint8_t col, const char *text)
{
  if (row >= DISPLAY_ROWS || text == NULL) {
    return;
  }

  uint8_t *page = framebuffer[row];
  bool changed = false;

  for (const char *p = text; *p != '\0' && col < DISPLAY_COLS; p++, col++) {
    unsigned char c = (unsigned char)*p;
    /* Anything outside the table renders as a blank cell rather than as
     * whatever byte happens to follow the array. */
    const uint8_t *glyph = NULL;
    if (c >= FONT_FIRST_CHAR && c <= FONT_LAST_CHAR) {
      glyph = font5x7[c - FONT_FIRST_CHAR];
    }

    uint16_t x = (uint16_t)(col * CELL_WIDTH);
    for (unsigned i = 0; i < CELL_WIDTH; i++) {
      uint8_t bits = 0;
      if (glyph != NULL && i < FONT_WIDTH) {
        bits = glyph[i];
      }
      if (page[x + i] != bits) {
        page[x + i] = bits;
        changed = true;
      }
    }
  }

  if (changed) {
    dirty_pages |= (uint8_t)(1u << row);
  }
}

void display_line(uint8_t row, const char *text)
{
  char padded[DISPLAY_COLS + 1];
  unsigned i = 0;

  while (i < DISPLAY_COLS && text[i] != '\0') {
    padded[i] = text[i];
    i++;
  }
  /* Pad rather than truncate, so a value that gets shorter - "-10.5" becoming
   * "-9.5" - cannot leave the old trailing character on screen. */
  while (i < DISPLAY_COLS) {
    padded[i++] = ' ';
  }
  padded[DISPLAY_COLS] = '\0';

  display_text(row, 0, padded);
}

bool display_dirty(void)
{
  return dirty_pages != 0u;
}

bool display_service(void)
{
  if (dirty_pages == 0u) {
    return false;
  }

  for (unsigned attempt = 0; attempt < DISPLAY_PAGES; attempt++) {
    uint8_t page = next_page;
    next_page = (uint8_t)((next_page + 1u) % DISPLAY_PAGES);

    if ((dirty_pages & (1u << page)) == 0u) {
      continue;
    }

    if (!send_command((uint8_t)(0xB0 | page)) ||
        !send_command((uint8_t)(0x00 | (COL_OFFSET & 0x0F))) ||
        !send_command((uint8_t)(0x10 | ((COL_OFFSET >> 4) & 0x0F)))) {
      return false; /* leave the page dirty so it is retried */
    }
    if (!send_data(framebuffer[page], DISPLAY_WIDTH)) {
      return false;
    }

    dirty_pages &= (uint8_t)~(1u << page);
    return true;
  }

  return false;
}
