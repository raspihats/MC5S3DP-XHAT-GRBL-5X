/*
  eeprom.c - Grbl EEPROM API, backed by a RAM copy of the EEPROM emulation
  Part of Grbl

  The board has no EEPROM chip. Grbl's EEPROM functions (eeprom_get_char() etc.,
  names kept from upstream Grbl) are served by ST's EEPROM emulation
  (Src/eeprom_emulation.c), which stores the bytes in internal flash sectors 2-3.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Why a RAM copy:
  The STM32F411 has a single flash bank, and code and interrupt handlers run
  from flash. Each flash write stalls the CPU briefly, and when the active
  emulation sector fills up it is erased, stalling the CPU (interrupts included)
  for ~250-500 ms. At 115200 baud that drops thousands of received bytes, which
  can make Grbl miss a newline and leave the host waiting for an 'ok' forever.

  So reads and writes go to RAM (ee_ram), and changed bytes are written to flash
  later by eeprom_service(), only while the machine is idle and the host has not
  sent a line for EEPROM_FLUSH_QUIET_MS. Bytes whose value did not change are
  never written, so the emulation sectors fill up (and get erased) far less often.

  Trade-off: a setting changed less than about EEPROM_FLUSH_QUIET_MS before a
  power loss is not saved.
*/

#include "grbl.h"
#include "eeprom_emulation.h"

#define EEPROM_SIZE NB_OF_VAR

// Wait this long after the last received line character (and in an idle state)
// before writing changed settings to flash.
#define EEPROM_FLUSH_QUIET_MS 500

static uint8_t ee_ram[EEPROM_SIZE];    // Current values, used by Grbl.
static uint8_t ee_flash[EEPROM_SIZE];  // Values as currently stored in flash.
static uint8_t ee_dirty = 0;           // ee_ram may differ from ee_flash.
static uint32_t ee_retry_ms = 0;       // Earliest time to retry after a failed flush.


// Initializes the EEPROM emulation and loads every stored byte into RAM.
// Must be called before any other eeprom_* function.
void eeprom_init()
{
  EE_Init();
  EE_ReadAll(ee_flash, EEPROM_SIZE);
  memcpy(ee_ram, ee_flash, EEPROM_SIZE);
  ee_dirty = 0;
}


unsigned char eeprom_get_char(unsigned int addr)
{
  if (addr >= EEPROM_SIZE) { return 0xFF; }
  return ee_ram[addr];
}


void eeprom_put_char(unsigned int addr, unsigned char new_value)
{
  if (addr >= EEPROM_SIZE) { return; }
  if (ee_ram[addr] == new_value) { return; } // Unchanged: nothing to write.
  ee_ram[addr] = new_value;
  ee_dirty = 1;
}


// Writes every byte that differs from flash. May stall the CPU (see top of file).
static void eeprom_flush()
{
  uint8_t failed = 0;
  unsigned int addr;
  for (addr = 0; addr < EEPROM_SIZE; addr++) {
    if (ee_ram[addr] != ee_flash[addr]) {
      if (EE_WriteVariable(addr, ee_ram[addr]) == FLASH_COMPLETE) {
        ee_flash[addr] = ee_ram[addr];
      } else {
        failed = 1;
      }
    }
  }
  ee_dirty = failed;
  if (failed) { ee_retry_ms = sys_tick_ms + EEPROM_FLUSH_QUIET_MS; }
}


// Called from the main loop between lines. Writes pending changes to flash
// once it is safe to stall the CPU.
void eeprom_service()
{
  if (!ee_dirty) { return; }

  // No motion of any kind: only idle, alarm or sleep, with nothing planned.
  if ((sys.state != STATE_IDLE) && (sys.state != STATE_ALARM) && (sys.state != STATE_SLEEP)) { return; }
  if (plan_get_current_block() != NULL) { return; }

  // No line data waiting or arriving.
  if (serial_get_rx_buffer_count() != 0) { return; }
  uint32_t now = sys_tick_ms;
  if ((uint32_t)(now - serial_rx_last_line_ms) < EEPROM_FLUSH_QUIET_MS) { return; }
  if ((int32_t)(now - ee_retry_ms) < 0) { return; }

  eeprom_flush();
}


// Extensions added as part of Grbl

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) {
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++));
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) {
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += data;
    *(destination++) = data;
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
