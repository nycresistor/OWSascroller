// CLOCK PIN: A0 (22)
// DATA 1: A1 (23)
// DATA 2: A2 (24)
// DATA 3: A3 (25)
// CLOCK PIN: A4 (26)
// CLOCK PIN: A5 (27)

// Connector pins 1-7 are tied to digital output pins 10-4 (desc)
#define ROW_START_PIN 4
// ROW 0: 10
// ROW 1: 9
// ROW 2: 8
// ROW 3: 7
// ROW 4: 6
// ROW 5: 5
// ROW 6: 4

// CLOCK: 2 - PD2
#define CLOCK_PIN 2
// DATA: 3 - PD3
#define DATA_PIN 3

inline void clockHigh() {
  PORTD |= _BV(CLOCK_PIN);
}

inline void clockLow() {
  PORTD &= ~_BV(CLOCK_PIN);
}

inline void dataHigh() {
  PORTD |= _BV(DATA_PIN);
}

inline void dataLow() {
  PORTD &= ~_BV(DATA_PIN);
}

#define GREETING "default message"

const static int columns = 95;
const static int rows = 7;

static int active_row = -1;

#include <avr/pgmspace.h>
#include "hfont.h"
#include <EEPROM.h>
#include <stdint.h>

typedef enum {
  LEFT,
  RIGHT,
  UP,
  DOWN,
  NONE
} Direction;

typedef enum {
  SCROLLING
} Mode;

inline int rowPin(const int row) {
  return ROW_START_PIN + rows - (row+1);
}

Mode mode = SCROLLING;
Direction dir = LEFT;

// Scroll delay is in complete display refreshes per frame.
int scroll_delay = 8;

uint8_t b1[columns];
uint8_t b2[columns];

class Bitmap {
  uint8_t* data;
  uint8_t* dsply;
public:
  Bitmap() {
    data = b1;
    dsply = b2;
  }
  void erase() {
    for (int i = 0; i < columns; i++) data[i] = 0;
  }
  void writeStr(char* p, int x, int y) {
    while (*p != '\0') {
      x = writeChar(*p,x,y);
      p++;
      x++;
    }
  }

  int charWidth(char c) {
    if (c == ' ') return 2;
    if ((c & 0x80) != 0) return (c & 0x7f);
    int coff = (int)c * 8;
    uint8_t row = pgm_read_byte(charData+coff);
    int width = 0;
    if (row == 0) {
      return 0;
    }
    while (row != 1) {
      coff++;
      width++;
      row = pgm_read_byte(charData+coff);
    }
    return width;
  }

  int stringWidth(const char* s) {
    int textLen = 0;
    while (*s != '\0') {
      textLen += charWidth(*s);
      s++;
    }
    return textLen;
  }

  int writeChar(char c, int x, int y, bool wrap = true) {
    int coff = (int)c * 8;
    uint8_t row = pgm_read_byte(charData+coff);
    if (c == ' ') return x+2;
    if ((c & 0x80) != 0) return x + (c & 0x7f);
    if (row == 0) {
      return x;
    }
    uint8_t mask = 0xfe >> y;
    while (row != 1) {
      row = row >> y;
      if (wrap) {
        x = x % columns;
        if (x < 0) { x = x + columns; }
      }
      if (x >= 0 && x < columns) {
        data[x] = row | (data[x] & mask);
      }
      coff++;
      x++;
      row = pgm_read_byte(charData+coff);
    }
    return x;
  }
  void flip() {
    uint8_t* d;
    cli();
    d = dsply;
    dsply = data;
    data = d;
    sei();
  }
  uint8_t* getDisplay() { return dsply; }
};

static Bitmap b;

int onRow = -1;

// Handle descending pin order
inline void rowOff() {
  if (onRow != -1) {
    digitalWrite(rowPin(onRow),LOW);
    onRow = -1;
  }
}


inline void rowOn(int row) {
  onRow = row;
  digitalWrite(rowPin(onRow),HIGH);
}

void setup() {
  b.erase();
  b.flip();
  for (int i = 0; i < rows; i++) {
    pinMode(rowPin(i),OUTPUT);
    digitalWrite(rowPin(i),LOW);
  }
  onRow = -1;

  pinMode(CLOCK_PIN,OUTPUT);
  pinMode(DATA_PIN,OUTPUT);

  // 2ms per row/interrupt
  // clock: 16MHz
  // target: 500Hz
  // 32000 cycles per interrupt
  // Prescaler: 1/64 OC: 500
  // CS[2:0] = 0b011
  // WGM[3:0] = 0b0100 CTC mode (top is OCR1A)
  
  TCCR1A = 0b00000000;
  TCCR1B = 0b00001011;
  TIMSK1 = _BV(OCIE1A);
  OCR1A = 200;

  Serial.begin(9600);
  Serial.println("Meet the press, suckers.");
  
  delay(100);
}

static unsigned int curRow = 0;

#define BUFFER_SIZE 512
#define MESSAGE_TICKS (columns*20)

static char message[BUFFER_SIZE];
static int buf_start = 0;
static int buf_end = 0;

const static uint16_t DEFAULT_MSG_OFF = 0x0;

enum {
  CODE_OK = 0,
  CODE_ERROR = -1
};

static int xend = 0;
static int xoff = 0;
static int yoff = 0;

volatile static int frames = 0;

void loop() {
  boolean spacer = true;
  while (frames < scroll_delay) {
    int nextChar = Serial.read();
    if (spacer && nextChar != -1) {
      spacer = false;
      if (xend < columns) {
        int w = (columns - xend);
        if (w > columns) w = columns;
        xend = columns;
        message[buf_end] = 0x80 | w;
        buf_end = (buf_end + 1)%BUFFER_SIZE;
        if (buf_end == buf_start) {
          buf_start = (buf_start+1)%BUFFER_SIZE;
        }
      }
    }
    while (nextChar != -1) {
      message[buf_end] = nextChar;
      buf_end = (buf_end + 1)%BUFFER_SIZE;
      if (buf_end == buf_start) {
        buf_start = (buf_start+1)%BUFFER_SIZE;
      }
      Serial.write(nextChar);
      nextChar = Serial.read();
    }
  }
  frames = 0;
  b.flip();
  b.erase();
  if (mode == SCROLLING) {
    int s = buf_start;
    int e = buf_end;
    int x = xoff;
    while (s != e) {
      x = b.writeChar(message[s],x,yoff,false);
      if (x < 0) {
        // character can be removed from queue
        buf_start = (buf_start + 1)%BUFFER_SIZE;
        xoff = x + 1;
      }
      x++;
      s = (s+1)%BUFFER_SIZE;
    }
    xend = x;
    if (xend <= 0) { xend = 0; xoff = 0; }
    switch (dir) {
    case LEFT: xoff--; break;
    case RIGHT: xoff++; break;
    case UP: yoff--; break;
    case DOWN: yoff++; break;
    }

    //if (xoff < 0) { xoff += columns; }
    if (xoff >= columns) { xoff -= columns; }
    if (yoff < 0) { yoff += 7; }
    if (yoff >= 7) { yoff -= 7; }
  }
}

inline void donops() {
    __asm__("nop\n\t");
    __asm__("nop\n\t");
    __asm__("nop\n\t");
}

ISR(TIMER1_COMPA_vect)
{
  uint8_t row = curRow % 7;
  uint8_t* p = b.getDisplay();
  uint8_t mask = 1 << (7-row);

  rowOff();
  for (int i = 0; i < columns; i++) {
    donops();
    clockLow();
    donops();
    if ((p[i] & mask) != 0) { dataHigh(); } else { dataLow(); }
    //if ((i+row) % 2 == 0) { dataLow(); } else { dataHigh(); }
    donops();
    clockHigh();
    donops();
  }
  rowOn(curRow%7);
  curRow++;
  if (curRow >= 7) {
    curRow = 0;
    frames++;
  }
}
