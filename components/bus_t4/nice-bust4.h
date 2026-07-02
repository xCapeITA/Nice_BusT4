/*
  Nice BusT4
  Scambio dati via UART alla velocità di 19200 8n1
  Prima di ogni pacchetto dati viene inviato un break della durata di 519us (10 bit)
  Il contenuto del pacchetto compreso è descritto nella struttura packet_cmd_body_t

  Per Oview all'indirizzo viene sempre sommato 0x80.
  L'indirizzo del controllore del cancello rimane invariato.

  Connessione:
  BusT4                     ESP8266 / ESP32
  Morsettiera dispositivo   Rx Tx GND
  9  7  5  3  1  
  10 8  6  4  2
  alloggiamento cavo
             1 ---------- Rx
             2 ---------- GND
             4 ---------- Tx
             5 ---------- +24V

  Dal manuale nice_dmbm_integration_protocol.pdf:
  • ADR: indirizzo della rete NICE dove risiedono i dispositivi da controllare (da 1 a 63 in HEX).
         Se il destinatario è un modulo di integrazione su barra DIN vale 0, se è un motore smart vale 1.
  • EPT: indirizzo del motore Nice all'interno della rete ADR (da 1 to 127 in HEX).
  • CMD: comando da inviare a destinazione.
  • PRF: comando di impostazione profilo.
  • FNC: funzione da inviare a destinazione.
  • EVT: evento inviato a destinazione.
*/

#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"
#include "esphome/core/helpers.h"
#include <queue>
#include <vector>
#include <string>

#ifdef ARDUINO_ARCH_ESP8266
#include <HardwareSerial.h>
#else
#include <driver/uart.h>
#endif

namespace esphome {
namespace bus_t4 {

using namespace esphome::cover;

static const int _UART_NO = 0; 
static const int TX_P = 1;         
static const uint32_t BAUD_BREAK = 9200; 
static const uint32_t BAUD_WORK = 19200; 
static const uint8_t START_CODE = 0x55; 

static const float CLOSED_POSITION_THRESHOLD = 0.007f;  // Soglia percentuale sotto la quale il cancello è considerato chiuso
static const uint32_t POSITION_UPDATE_INTERVAL = 500;   // Intervallo di aggiornamento della posizione in ms

/* 
  Tipo di messaggio dei pacchetti (6° byte dei pacchetti CMD e INF)
*/
enum mes_type : uint8_t {
  CMD = 0x01,  // Invio comandi all'automazione
  INF = 0x08,  // Richiede o imposta informazioni sul dispositivo
};

/* 
  Menu dei comandi nell'gerarchia Oview (9° byte dei pacchetti CMD)
*/
enum cmd_mnu : uint8_t {
  CONTROL = 0x01,
};

/* Utilizzato nelle risposte STA */
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
  OPENED      = 0x04,
  CLOSED      = 0x05,
  ENDTIME     = 0x06,  // Manovra terminata per timeout
  STOPPED     = 0x08,
  PART_OPENED = 0x10,  // Apertura parziale
};

/* Errori */
enum errors_byte : uint8_t {
  NOERR = 0x00, // Nessun errore
  FD    = 0xFD, // Comando non disponibile per questo dispositivo
};

/* Tipi di motore */
enum motor_type : uint8_t {
  SLIDING   = 0x01, 
  SECTIONAL = 0x02,
  SWING     = 0x03,
  BARRIER   = 0x04,
  UPANDOVER = 0x05, 
};

/* Destinatario o mittente (9° byte) */
enum whose_pkt : uint8_t {
  FOR_ALL = 0x00,  // Pacchetto per/da tutti
  FOR_CU  = 0x04,  // Pacchetto per/dalla centrale di comando
  FOR_OXI = 0x0A,  // Pacchetto per/dal ricevitore OXI
};
  
/* 10° byte dei pacchetti GET/SET EVT */
enum command_pkt : uint8_t {
  TYPE_M        = 0x00,  // Richiesta tipo di motore
  INF_STATUS    = 0x01,  // Stato del cancello (Aperto/Chiuso/Fermo)  
  WHO           = 0x04,  // Chi è online?     
  MAC           = 0x07,  // Indirizzo MAC
  MAN           = 0x08,  // Produttore
  PRD           = 0x09,  // Prodotto
  INF_SUPPORT   = 0x10,  // Comandi INF disponibili
  HWR           = 0x0a,  // Versione hardware
  FRM           = 0x0b,  // Versione firmware
  DSC           = 0x0c,  // Descrizione
  CUR_POS       = 0x11,  // Posizione corrente condizionata dell'automazione
  MAX_OPN       = 0x12,  // Apertura massima possibile dell'encoder
  POS_MAX       = 0x18,  // Posizione massima (apertura) dell'encoder
  POS_MIN       = 0x19,  // Posizione minima (chiusura) dell'encoder  
  INF_P_OPN1    = 0x21,  // Apertura parziale 1 
  INF_P_OPN2    = 0x22,  // Apertura parziale 2
  INF_P_OPN3    = 0x23,  // Apertura parziale 3
  INF_SLOW_OPN  = 0x24,  // Rallentamento in apertura
  INF_SLOW_CLS  = 0x25,  // Rallentamento in chiusura 
  OPN_OFFSET    = 0x28,  // Ritardo apertura
  CLS_OFFSET    = 0x29,  // Ritardo chiusura
  OPN_DIS       = 0x2a,  // Scarico apertura (Open discharge)
  CLS_DIS       = 0x2b,  // Scarico chiusura (Close discharge)
  REV_TIME      = 0x31,  // Durata inversione (Brief inversion value)
  OPN_PWR       = 0x4A,  // Controllo forza - Forza apertura    
  CLS_PWR       = 0x4B,  // Controllo forza - Forza chiusura        
  SPEED_OPN     = 0x42,  // Configurazione velocità - Velocità apertura                  
  SPEED_CLS     = 0x43,  // Configurazione velocità - Velocità chiusura          
  SPEED_SLW_OPN = 0x45,  // Configurazione velocità - Velocità rallentamento apertura  
  SPEED_SLW_CLS = 0x46,  // Configurazione velocità - Velocità rallentamento chiusura  
  OUT1          = 0x51,  // Configurazione uscite   
  OUT2          = 0x52,  // Configurazione uscite     
  LOCK_TIME     = 0x5A,  // Tempo di lavoro dell'elettroserratura
  S_CUP_TIME    = 0x5C,  // Tempo di lavoro della ventosa (Suction Cup Time)    
  LAMP_TIME     = 0x5B,  // Tempo di luce di cortesia (Courtesy light Time)
  COMM_SBS      = 0x61,  // Configurazione comandi - Passo Passo    
  COMM_POPN     = 0x62,  // Configurazione comandi - Apri parzialmente      
  COMM_OPN      = 0x63,  // Configurazione comandi - Apri         
  COMM_CLS      = 0x64,  // Configurazione comandi - Chiudi    
  COMM_STP      = 0x65,  // Configurazione comandi - STOP      
  COMM_PHOTO    = 0x68,  // Configurazione comandi - Foto      
  COMM_PHOTO2   = 0x69,  // Configurazione comandi - Foto2
  COMM_PHOTO3   = 0x6A,  // Configurazione comandi - Foto3
  COMM_OPN_STP  = 0x6B,  // Configurazione comandi - Stop in apertura    
  COMM_CLS_STP  = 0x6C,  // Configurazione comandi - Stop in chiusura   
  IN1           = 0x71,  // Configurazione ingressi
  IN2           = 0x72,  // Configurazione ingressi
  IN3           = 0x73,  // Configurazione ingressi
  IN4           = 0x74,  // Configurazione ingressi
  COMM_LET_OPN  = 0x78,  // Configurazione comandi - Ostacolo in apertura       
  COMM_LET_CLS  = 0x79,  // Configurazione comandi - Ostacolo in chiusura           
  AUTOCLS       = 0x80,  // Chiusura automatica
  P_TIME        = 0x81,  // Tempo di pausa
  PH_CLS_ON     = 0x84,  // Chiudi dopo foto - Attivo    
  PH_CLS_VAR    = 0x86,  // Chiudi dopo foto - Modalità      
  PH_CLS_TIME   = 0x85,  // Chiudi dopo foto - Tempo di attesa       
  ALW_CLS_ON    = 0x88,  // Chiudi sempre - Attivo      
  ALW_CLS_VAR   = 0x8A,  // Chiudi sempre - Modalità    
  ALW_CLS_TIME  = 0x89,  // Chiudi sempre - Tempo di attesa       
  STAND_BY_ACT  = 0x8c,  // Standby - Attivo ON / OFF
  WAIT_TIME     = 0x8d,  // Standby - Tempo di attesa
  STAND_BY_MODE = 0x8e,  // Standby - Modalità (safety = 0x00, bluebus=0x01, all=0x02)
  START_ON      = 0x90,  // Configurazione avvio - Attivo      
  START_TIME    = 0x91,  // Configurazione avvio - Tempo di spunto              
  SLOW_ON       = 0xA2,  // Rallentamento
  DIS_VAL       = 0xA4,  // Valore non valido (Disable value)
  BLINK_ON       = 0x94,  // Pre-lampeggio - Attivo        
  BLINK_OPN_TIME = 0x95,  // Pre-lampeggio - Tempo in apertura              
  BLINK_CLS_TIME = 0x99,  // Pre-lampeggio - Tempo in chiusura
  OP_BLOCK       = 0x9a,  // Blocco motore (Operator block)
  KEY_LOCK       = 0x9c,  // Blocco pulsanti
  T_VAL          = 0xB1,  // Soglia allarme manutenzione (numero di manovre)
  P_COUNT        = 0xB2,  // Contatore parziale
  C_MAIN         = 0xB4,  // Cancella manutenzione
  DIAG_BB        = 0xD0,  // Diagnostica dispositivi BlueBus    
  INF_IO         = 0xD1,  // Stato ingressi-uscite  
  DIAG_PAR       = 0xD2,  // Diagnostica altri parametri  
  
  CUR_MAN        = 0x02,  // Manovra corrente
  SUBMNU         = 0x04,  // Sottomenu
  STA            = 0xC0,  // Stato in movimento
  MAIN_SET       = 0x80,  // Parametri principali
  RUN            = 0x82,  // Comando da eseguire    
};  

/* Tipo di comando Run (11° byte dei pacchetti EVT) */
enum run_cmd : uint8_t {
  SET          = 0xA9,  // Richiesta di modifica parametri
  GET          = 0x99,  // Richiesta di ottenimento parametri
  GET_SUPP_CMD = 0x89,  // Ottieni comandi supportati
};

/* Comando effettivo da eseguire (11° byte del pacchetto CMD) */
enum control_cmd : uint8_t { 
  SBS         = 0x01,  // Passo Passo
  STOP        = 0x02,  // Stop
  OPEN        = 0x03,  // Apri
  CLOSE       = 0x04,  // Chiudi
  P_OPN1      = 0x05,  // Apertura parziale 1 (Modalità pedonale)
  P_OPN2      = 0x06,  // Apertura parziale 2
  P_OPN3      = 0x07,  // Apertura parziale 3
  RSP         = 0x19,  // Risposta interfaccia di conferma ricezione comando  
  EVT         = 0x29,  // Risposta interfaccia contenente le informazioni richieste
  P_OPN4      = 0x0b,  // Apertura parziale 4 - Condominiale
  P_OPN5      = 0x0c,  // Apertura parziale 5 - Priorità Passo Passo
  P_OPN6      = 0x0d,  // Apertura parziale 6 - Apri e Blocca
  UNLK_OPN    = 0x19,  // Sblocca e Apri
  CLS_LOCK    = 0x0E,  // Chiudi e Blocca
  UNLCK_CLS   = 0x1A,  // Sblocca e Chiudi
  LOCK        = 0x0F,  // Blocca
  UNLOCK      = 0x10,  // Sblocca
  LIGHT_TIMER = 0x11,  // Timer luce di cortesia
  LIGHT_SW    = 0x12,  // Luce cortesia ON/OFF
  HOST_SBS    = 0x13,  // Master Passo Passo
  HOST_OPN    = 0x14,  // Master Apri
  HOST_CLS    = 0x15,  // Master Chiudi
  SLAVE_SBS   = 0x16,  // Slave Passo Passo
  SLAVE_OPN   = 0x17,  // Slave Apri
  SLAVE_CLS   = 0x18,  // Slave Chiudi
  AUTO_ON     = 0x1B,  // Chiusura automatica attiva
  AUTO_OFF    = 0x1C,  // Chiusura automatica non attiva   
};

enum position_hook_type : uint8_t {
  IGNORE    = 0x00,
  STOP_UP   = 0x01,
  STOP_DOWN = 0x02
};

class NiceBusT4 : public Component, public Cover {
 public:
    // Impostazioni flag dell'automazione
    bool autocls_flag;   // Chiusura automatica - L1
    bool photocls_flag;  // Chiudi dopo foto - L2
    bool alwayscls_flag; // Chiudi sempre - L3
    bool init_ok = false; 
    bool is_walky = false; 
    bool is_robus = false; 
    bool is_ro = false; 
    
    void setup() override;
    void loop() override;
    void dump_config() override; 

    void send_raw_cmd(const std::string &data);
    void send_cmd(uint8_t data) { this->tx_buffer_.push(gen_control_cmd(data)); }  
    void send_inf_cmd(const std::string &to_addr, const std::string &whose, const std::string &command, const std::string &type_command, const std::string &next_data, bool data_on, const std::string &data_command); 
    void set_mcu(const std::string &command, const std::string &data_command); 
    
    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    cover::CoverTraits get_traits() override;

 protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  
    void update_position(uint16_t newpos);  

    uint32_t last_position_time{0};  
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    CoverOperation last_published_op;  
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; 
  
    bool init_cu_flag = false;  
    bool init_oxi_flag = false; 

    // Variabili UART e posizionamento
    uint8_t _uart_nr;
#ifdef ARDUINO_ARCH_ESP8266
    HardwareSerial* _uart = nullptr;
#else
    int _uart_port_num = 0;
    void* _uart = nullptr; 
#endif
    uint16_t _max_opn = 0;  
    uint16_t _pos_opn = 2048;  
    uint16_t _pos_cls = 0;  
    uint16_t _pos_usl = 0;  
    
    // Indirizzamento di default della rete BusT4 per il modulo lupanosix_blu
    uint8_t addr_from[2] = {0x00, 0x66}; 
    uint8_t addr_to[2]; 
    uint8_t addr_oxi[2]; 

    std::vector<uint8_t> raw_cmd_prepare(const std::string &data);             
  
    // Generatori di comandi INF sovraccaricati e ottimizzati
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);  
    
    inline std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {
        return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0);
    } 
    
    inline std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data) {
        return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());
    } 
    
    inline std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data) {
        return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);
    } 
          
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);         
  
    void init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device);
    void send_array_cmd(std::vector<uint8_t> data);  
    void send_array_cmd(const uint8_t *data, size_t len);

    void parse_status_packet(const std::vector<uint8_t> &data); 
    
    void handle_char_(uint8_t c);                                         
    void handle_datapoint_(const uint8_t *buffer, size_t len);          
    bool validate_message_();                                         

    std::vector<uint8_t> rx_message_;                          
    std::queue<std::vector<uint8_t>> tx_buffer_;              
    bool ready_to_tx_{true};                              
  
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;  
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description; 
};

} // namespace bus_t4
} // namespace esphome
