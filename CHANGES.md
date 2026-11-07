# Correzioni al componente bus_t4

## Bug critici
- **Iteratori di vettori diversi** in `parse_status_packet` (HWR, FRM, DSC): `assign` usava l'inizio di `rx_message_` e la fine di `hardware_`. Comportamento indefinito, causa di crash dopo l'aggancio al bus. Ora tutti i payload passano da `payload_of(data)`, che controlla la lunghezza.
- **Loop UART senza limite** in `loop()`: con linea rumorosa il ciclo di lettura non termina e il watchdog riavvia il chip. Aggiunto un budget di 64 byte per ciclo e un controllo sul puntatore nullo.
- **Overflow di `length`** in `validate_message_`: `uint8_t` andava in overflow con `packet_size >= 253`. Ora `uint16_t`.
- **Inizializzazione UART non verificata**: `setup()` ora segnala errore con `mark_failed()` se `uart_init` restituisce nullo.
- **`gen_inf_cmd`**: copia limitata alla dimensione reale del vettore dati.

## Pulizia
- `#include "esphome.h"` rimosso dall'header (include da componente custom, non necessario).
- `cover.py` non scrive più su membri `protected` dal codice generato: usa i setter pubblici `set_addr_to` e `set_addr_from`.
- `automation.h` rimosso: non era incluso né usato.

## YAML
- `logger: baud_rate: 0`: la seriale di log è disattivata perché condivide la UART0 con il bus. I log restano disponibili dalla dashboard e via API.

## Da verificare
- Compilazione con la versione di ESPHome in uso (non testata in questa sessione).
- Boot a banco con bus scollegato.
- Risposte HWR, FRM, DSC con centrale collegata.
