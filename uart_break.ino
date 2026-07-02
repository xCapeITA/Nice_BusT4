/*
  Nice BusT4 - Generazione hardware del Break a 519µs
  Adattato per Wemos D1 mini / ESP8266 (Arduino Core 3.0.2)
  
  Il timeout di 90 microsecondi corregge il bug hardware del Core 
  evitando che il baudrate cambi prima dell'effettiva uscita del bit di STOP.
*/

#include <Arduino.h>

extern "C" {
#include "uart.h"
}

unsigned long timing = 0; 

#define _UART_NO UART0
#define TX_P 1
const uint32_t baud_work = 19200;
const uint32_t baud_break = 9200;

// Buffer di test con comandi reali intercettati dal protocollo Nice BusT4
const uint8_t master_tx_buf[] = {0x55, 0x0C, 0x00, 0x03, 0x00, 0x81, 0x01, 0x05, 0x86, 0x01, 0x82, 0x01, 0x64, 0xE6, 0x0C}; // Esempio: Passo-Passo (SBS)
const uint8_t break_tx_buf[] = {0x00};

uart_t* _uart = nullptr;

void send_break_calibrated() {
  // 1. Svuota i buffer FIFO per evitare sovrapposizioni
  uart_flush(_uart);
  
  // 2. Imposta il baudrate ridotto (9200 baud) per allargare la finestra temporale dello zero
  uart_set_baudrate(_uart, baud_break);
  
  // 3. Invia il byte 0x00 in linea
  uart_write(_uart, (const char *)break_tx_buf, sizeof(break_tx_buf));
  
  // 4. Attende che la FIFO hardware sia vuota
  uart_wait_tx_empty(_uart);
  
  // 5. RITARDO DI CALIBRAZIONE: 90 microsecondi determinanti.
  // Permette allo Shift Register fisico di completare l'uscita dello stato logico basso,
  // stabilizzando la durata del Break esattamente intorno ai 519µs-520µs totali in linea.
  delayMicroseconds(90); 
  
  // 6. Ripristina il baudrate di lavoro nominale del BusT4 (19200 baud)
  uart_set_baudrate(_uart, baud_work);
  
  // 7. Spedisce il pacchetto dati effettivo senza generare glitch o picchi spuri
  uart_write(_uart, (const char *)master_tx_buf, sizeof(master_tx_buf));
  uart_wait_tx_empty(_uart);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // Allocazione del buffer seriale a 256 byte per garantire la piena compatibilità
  _uart = uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
}

void loop() {
  unsigned long now = millis();
  // Ciclo di test: invio della sequenza di attivazione ogni 2 secondi
  if (now - timing > 2000) {
    timing = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    send_break_calibrated(); 
  }
}
