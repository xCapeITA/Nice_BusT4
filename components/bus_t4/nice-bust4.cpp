#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);
  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
      } else { 
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Posizione richiesta: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {
  _uart = reinterpret_cast<HardwareSerial*>(uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false));
}

void NiceBusT4::loop() {
  uint32_t now = millis();

  if ((now - this->last_update_) > 10000) {
    std::vector<uint8_t> unknown = {0x55, 0x55};
    if (!this->init_ok) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00));
    } else if (this->class_gate_ == 0x55 || this->manufacturer_ == unknown) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);
    }
    this->last_update_ = now;
  }

  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  }

  // Lettura UART con cast nativo per prevenire errori di tipo tra core C e classi C++
  uart_t* native_uart = reinterpret_cast<uart_t*>(_uart);
  while (uart_rx_available(native_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(native_uart);
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }

  if (this->ready_to_tx_ && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->ready_to_tx_ = false;
  }

  if (!is_robus && init_ok && (current_operation != COVER_OPERATION_IDLE)) {
    if (now - last_position_time > POSITION_UPDATE_INTERVAL) {
      last_position_time = now;
      request_position();
    }
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  uint8_t *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  if (at == 0) return new_byte == 0x00;
  if (at == 1) return new_byte == START_CODE;
  if (at == 2) return true;

  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3);

  if (at <= 8) return true;

  if (at == 9) {
    uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Checksum 1 invalido %02X!=%02X", data[9], crc1);
      return false;
    }
    return true;
  }

  if (at < length) return true;

  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2) {
    ESP_LOGW(TAG, "Checksum 2 invalido %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  if (data[length] != packet_size) {
    ESP_LOGW(TAG, "Dimensione errata del messaggio %02X!=%02X", data[length], packet_size);
    return false;
  }

  rx_message_.erase(rx_message_.begin());
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG, "Pacchetto ricevuto: %s", pretty_cmd.c_str());

  parse_status_packet(rx_message_);
  return false; 
}

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  if (data.size() < 14) return;

  if ((data[1] == 0x0d) && (data[13] == 0xFD)) {
    ESP_LOGE(TAG, "Comando non disponibile per questo dispositivo.");
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) {
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG, "Dati HEX: %s", pretty_data.c_str());

    if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == GET - 0x80)) {
      switch (data[10]) {
        case TYPE_M:
          if (data.size() > 14) {
            switch (data[14]) {
              case SLIDING:   this->class_gate_ = SLIDING; break;
              case SECTIONAL: this->class_gate_ = SECTIONAL; break;
              case SWING:     this->class_gate_ = SWING; break;
              case BARRIER:   this->class_gate_ = BARRIER; break;
              case UPANDOVER: this->class_gate_ = UPANDOVER; break;
            }
          }
          break;
        case INF_IO:
          if (data.size() > 16) {
            switch (data[16]) {
              case 0x00: ESP_LOGI(TAG, "Finecorsa in errore"); break;
              case 0x01: ESP_LOGI(TAG, "Finecorsa in chiusura"); this->position = COVER_CLOSED; break;
              case 0x02: ESP_LOGI(TAG, "Finecorsa in apertura"); this->position = COVER_OPEN; break;
            }
          }
          this->publish_state_if_changed();
          break;

        case MAX_OPN:
          if (data.size() > 15) {
            if (is_walky) {
              this->_max_opn = data[15];
              this->_pos_opn = data[15];
            } else {
              this->_max_opn = (data[14] << 8) + data[15];
            }
            ESP_LOGI(TAG, "Posizione massima encoder: %d", this->_max_opn);
          }
          break;

        case POS_MIN:
          if (data.size() > 15) {
            this->_pos_cls = (data[14] << 8) + data[15];
            ESP_LOGI(TAG, "Posizione cancello chiuso: %d", this->_pos_cls);
          }
          break;

        case POS_MAX:
          if (data.size() > 15) {
            uint16_t pos_opn_val = (data[14] << 8) + data[15];
            if (pos_opn_val > 0x00) this->_pos_opn = pos_opn_val;
            ESP_LOGI(TAG, "Posizione cancello aperto: %d", this->_pos_opn);
          }
          break;

        case CUR_POS:
          if (data.size() > 15) {
            if (is_walky) update_position(data[15]);
            else update_position((data[14] << 8) + data[15]);
          }
          break;

        case INF_STATUS:
          if (data.size() > 14) {
            switch (data[14]) {
              case OPENED:
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case CLOSED:
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case 0x01:
              case 0x00:
              case 0x0b:
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              case STA_OPENING:
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
            }
          }
          this->publish_state_if_changed();
          break;

        case AUTOCLS:
          if (data.size() > 14) {
            this->autocls_flag = data[14];
            ESP_LOGCONFIG(TAG, "Chiusura automatica - L1: %s", autocls_flag ? "Si" : "No");
          }
          break;
        case PH_CLS_ON:  if (data.size() > 14) this->photocls_flag = data[14]; break;
        case ALW_CLS_ON: if (data.size() > 14) this->alwayscls_flag = data[14]; break;
      }
    }

    if ((data[6] == INF) && (data[11] == GET - 0x81)) {
      tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
    }
  }

  if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == SET - 0x80) && (data[13] == NOERR)) {
    switch (data[10]) {
      case AUTOCLS:   tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); break;
      case PH_CLS_ON: tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); break;
      case ALW_CLS_ON:tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); break;
    }
  }

  if ((data[6] == INF) && (data[9] == FOR_ALL) && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) {
    switch (data[10]) {
      case MAN: this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2); break;
      case PRD:
        if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
          this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
        } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
          this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57};
          std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00};
          if (this->product_ == wla1) this->is_walky = true;
          if (this->product_ == ROBUSHSR10) this->is_robus = true;
        }
        break;
      case HWR:
        if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
          this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
        } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
          this->hardware_.assign(this->rx_message_.begin() + 14, this->hardware_.end() - 2);
        }
        break;
      case FRM:
        if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
          this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
        } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
          this->firmware_.assign(this->rx_message_.begin() + 14, this->hardware_.end() - 2);
        }
        break;
      case DSC:
        if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
          this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
        } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
          this->description_.assign(this->rx_message_.begin() + 14, this->hardware_.end() - 2);
        }
        break;
      case WHO:
        if (data[12] == 0x01) {
          if (data[14] == 0x04) {
            this->addr_to[0] = data[4]; this->addr_to[1] = data[5]; this->init_ok = true;
          } else if (data[14] == 0x0A) {
            this->addr_oxi[0] = data[4]; this->addr_oxi[1] = data[5];
            init_device(data[4], data[5], data[14]);
          }
        }
        break;
    }
  }

  if (data[1] > 0x0d) {
    switch (data[9]) {
      case FOR_CU:
        switch (data[10] + 0x80) {
          case RUN:
            if (data[11] >= 0x80) {
              switch (data[11] - 0x80) {
                case OPEN:  this->current_operation = COVER_OPERATION_OPENING; break;
                case CLOSE: this->current_operation = COVER_OPERATION_CLOSING; break;
                case STOPPED:
                case ENDTIME:
                  this->current_operation = COVER_OPERATION_IDLE; request_position(); break;
              }
            } else {
              switch (data[11]) {
                case STA_OPENING: this->current_operation = COVER_OPERATION_OPENING; break;
                case STA_CLOSING: this->current_operation = COVER_OPERATION_CLOSING; break;
                case CLOSED:      this->current_operation = COVER_OPERATION_IDLE; this->position = COVER_CLOSED; break;
                case OPENED:
                  this->current_operation = COVER_OPERATION_IDLE; this->position = COVER_OPEN;
                  if (this->_max_opn == 0) this->_max_opn = this->_pos_opn = this->_pos_usl;
                  break;
                case STOPPED:
                case PART_OPENED:
                  this->current_operation = COVER_OPERATION_IDLE; request_position(); break;
              }
            }
            this->publish_state_if_changed();
            break;

          case STA:
            switch (data[11]) {
              case STA_OPENING: case 0x83: this->current_operation = COVER_OPERATION_OPENING; break;
              case STA_CLOSING: case 0x84: this->current_operation = COVER_OPERATION_CLOSING; break;
              case CLOSED:  this->current_operation = COVER_OPERATION_IDLE; this->position = COVER_CLOSED; break;
              case OPENED:  this->current_operation = COVER_OPERATION_IDLE; this->position = COVER_OPEN; break;
              case STOPPED: this->current_operation = COVER_OPERATION_IDLE; request_position(); break;
            }
            if (data.size() > 13) update_position((data[12] << 8) + data[13]);
            break;
        }
        break;
    }
  }

  if ((data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[13] == NOERR)) {
    switch (data[11]) {
      case STA_OPENING: this->current_operation = COVER_OPERATION_OPENING; break;
      case STA_CLOSING: this->current_operation = COVER_OPERATION_CLOSING; break;
      case OPENED:      this->position = COVER_OPEN; this->current_operation = COVER_OPERATION_IDLE; break;
      case CLOSED:      this->position = COVER_CLOSED; this->current_operation = COVER_OPERATION_IDLE; break;
      case STOPPED:     this->current_operation = COVER_OPERATION_IDLE; break;
    }
    this->publish_state();
  }
}

void NiceBusT4::dump_config() {
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Produttore: %s", manuf_str.c_str());
  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Unità: %s", prod_str.c_str());
}

std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]};
  frame.push_back(CMD); frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1); frame.push_back(CONTROL); frame.push_back(RUN);
  frame.push_back(control_cmd); frame.push_back(0x64);
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size); frame.insert(frame.begin(), f_size); frame.insert(frame.begin(), START_CODE);
  return frame;
}

std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]};
  frame.push_back(INF); frame.push_back(0x06 + len);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1); frame.push_back(whose); frame.push_back(inf_cmd);
  frame.push_back(run_cmd); frame.push_back(next_data); frame.push_back(len);
  if (len > 0 && !data.empty()) {
    frame.insert(frame.end(), data.begin(), data.begin() + len);
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) crc2 = crc2 ^ frame[i];
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size); frame.insert(frame.begin(), f_size); frame.insert(frame.begin(), START_CODE);
  return frame;
}

void NiceBusT4::send_raw_cmd(const std::string &data) {
  std::vector<uint8_t> v_cmd = raw_cmd_prepare(data);
  if (!v_cmd.empty()) send_array_cmd(&v_cmd[0], v_cmd.size());
}

std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(const std::string &data) {
  std::string clean_data = data;
  clean_data.erase(std::remove_if(clean_data.begin(), clean_data.end(), [](const unsigned char ch) {
    return !(std::isxdigit(ch));
  }), clean_data.end());

  std::vector<uint8_t> frame;
  if (clean_data.size() % 2 != 0) return frame;

  frame.reserve(clean_data.size() / 2);
  for (size_t i = 0; i < clean_data.size(); i += 2) {
    std::string sub_str = clean_data.substr(i, 2);
    char hexstoi = (char)std::strtol(sub_str.c_str(), nullptr, 16);
    frame.push_back(hexstoi);
  }
  return frame;
}

void NiceBusT4::send_array_cmd(std::vector<uint8_t> data) {
  if (!data.empty()) send_array_cmd(data.data(), data.size());
}

void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  char br_ch = 0x00;
  uart_t* native_uart = reinterpret_cast<uart_t*>(_uart);

  uart_flush(native_uart);
  uart_set_baudrate(native_uart, BAUD_BREAK);
  uart_write(native_uart, &br_ch, 1);
  uart_wait_tx_empty(native_uart);
  delayMicroseconds(90); 
  
  uart_set_baudrate(native_uart, BAUD_WORK);
  uart_write(native_uart, (const char *)data, len);
  uart_wait_tx_empty(native_uart);

  std::string pretty_cmd = format_hex_pretty(data, len);
  ESP_LOGI(TAG, "Inviato: %s", pretty_cmd.c_str());
}

void NiceBusT4::send_inf_cmd(const std::string &to_addr, const std::string &whose, const std::string &command, const std::string &type_command, const std::string &next_data, bool data_on, const std::string &data_command) {
  std::vector<uint8_t> v_to_addr = raw_cmd_prepare(to_addr);
  std::vector<uint8_t> v_whose = raw_cmd_prepare(whose);
  std::vector<uint8_t> v_command = raw_cmd_prepare(command);
  std::vector<uint8_t> v_type_command = raw_cmd_prepare(type_command);
  std::vector<uint8_t> v_next_data = raw_cmd_prepare(next_data);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);

  if (v_to_addr.size() < 2 || v_whose.empty() || v_command.empty() || v_type_command.empty() || v_next_data.empty()) return;

  if (data_on && !v_data_command.empty()) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  }
}

void NiceBusT4::set_mcu(const std::string &command, const std::string &data_command) {
  std::vector<uint8_t> v_command = raw_cmd_prepare(command);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);
  if (!v_command.empty()) {
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
}

void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
    if (is_walky) tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00));
  }
}

void NiceBusT4::request_position(void) {
  if (is_walky) tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  if ((_pos_opn - _pos_cls) != 0) {
    position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  }
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();

  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || 
      (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
    send_cmd(STOP);
    position_hook_type = IGNORE;
  }
}

void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

} 
}
