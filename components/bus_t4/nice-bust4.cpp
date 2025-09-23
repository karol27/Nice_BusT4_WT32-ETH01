#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // to use auxiliary functions for working with strings
#include "driver/uart.h"           // functions for ESP32 board type 

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

// uint8_t moja_zmienna = 0;

// NiceBusT4::NiceBusT4(text_sensor::TextSensor *sensor) {  // Konstruktor przypisujący wskaźnik do text_sensor
  // this->pause_time_sensor = sensor;
// }

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}


/*
  OVIEW command dumps

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c
*/

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

      } else { // Arbitrary position
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Required drive position: %d", position_hook_value);
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


 // _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_6E2, TX_P, 256, false); //for ESP8266
  _uart =  uartBegin(_UART_NO, BAUD_WORK, SERIAL_8N1, RX_PIN, TX_PIN, 256, 256, false, 112); //for WT32
  // who's online?
//  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));

  // ESP_LOGD("setup", "Wywołanie setup()");
  // if (this->pause_time_sensor == nullptr) {
    // // Użycie operatora & do uzyskania wskaźnika do id(pause_time_sensor)
    // this->pause_time_sensor = &id(pause_time_sensor);
    // ESP_LOGD("setup", "pause_time_sensor został przypisany w setup: %p", this->pause_time_sensor);
  // } else {
    // ESP_LOGW("setup", "pause_time_sensor już został przypisany");
  // }

}

void NiceBusT4::loop() {

  if ((millis() - this->last_update_) > 10000) {    // every 10 seconds // If the drive is not detected the first time, we will try later
      std::vector<uint8_t> unknown = {0x55, 0x55};
      if (this->init_ok == false) {
        ESP_LOGI(TAG, "  Initialize device");
        ESP_LOGI(TAG, "  Who is online request");
        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        ESP_LOGI(TAG, "  Product request");
        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
      } else if (this->class_gate_ == 0x55) {
        ESP_LOGI(TAG, "  Initialize device - class_gate == 0x55");
        init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        // this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        // this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
      } else if (this->manufacturer_ == unknown)  {
        ESP_LOGI(TAG, "  Initialize device - manufacturer");
        init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        // this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        // this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
      }
      this->last_update_ = millis();
  }  // if  every minute


  // allow sending every 100 ms
  uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  } 


  while (uartAvailable(_uart) > 0) {
    //uint8_t c = (uint8_t)uart_Read(_uart);                // read the byte for ESP8266
    uint8_t c = (uint8_t)uartRead(_uart);                // read the byte for ESP32
    this->handle_char_(c);                                     // send the byte for processing
    this->last_uart_byte_ = now;
  } //while

  if (this->ready_to_tx_) {   // if possible send
    if (!this->tx_buffer_.empty()) {  // if you have anything to send
      this->send_array_cmd(this->tx_buffer_.front()); // send the first command in the queue
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
  }

  // Poll of current actuator position
  if (!is_robus) {
  
  now = millis();
  if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
    last_position_time = now;
    request_position();
  } 
  } // not robus

} //loop


void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);                      // throw a byte at the end of the received message
  if (!this->validate_message_()) {                    // check the resulting message
    this->rx_message_.clear();                         // if the verification fails, then the message is garbage, you need to delete it
  }
}


bool NiceBusT4::validate_message_() {                    // checking the received message
  uint32_t at = this->rx_message_.size() - 1;       // number of the last byte received
  uint8_t *data = &this->rx_message_[0];               // pointer to the first byte of the message
  uint8_t new_byte = data[at];                      // last byte received

  // Byte 0: HEADER1 (always 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (always 0x55)
  if (at == 1)
    return new_byte == START_CODE;

  // Byte 2: packet_size - number of bytes further + 1
  // No verification carried out

  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3); // the length of the expected message is clear


  // Byte 3: Series (series) to whom package
  // No verification carried out
  //  uint8_t command = data[3];
  if (at == 3)
    return true;

 // Byte 4: Address to whom the package
 // Byte 5: Series (row) from whom the packet
 // Byte 6: Address of who the packet is from
 // Byte 7: Message type CMD or INF
 // Byte 8: The number of bytes further minus the two CRC bytes at the end.

  if (at <= 8)
    // No verification carried out
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR the previous six bytes
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  // Byte 10:
  // ...

  // wait until all the package data arrives
  if (at  < length)
    return true;

  // consider crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  //  if (at  ==  length) {
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
  }

  // If you got here, the correct message was received and is in the rx_message_ buffer

 // Remove 0x00 at the beginning of the message
  rx_message_.erase(rx_message_.begin());

  // to output the package to the log
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Package received: %S ", pretty_cmd.c_str() );

  // here we do something with the message
  parse_status_packet(rx_message_);

  // return false to reset rx buffer
  return false;
}


// void NiceBusT4::set_pause_time(uint8_t nowy_pause_time) {
  // pause_time = nowy_pause_time;

  // // Konwersja uint8_t na string
  // std::string pause_time_str = std::to_string(pause_time);

  // // Aktualizuj text_sensor bezpośrednio
  // id(moj_text_sensor).publish_state(pause_time_str.c_str());
// }

// parse the received packages
void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  // ESP_LOGD("debug", "Wywołanie parse_status_packet");
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // error
    ESP_LOGE(TAG,  "Command not available for this device" );
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
  //  ESP_LOGD(TAG, "EVT packet with data received. Last cell %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Data string: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %S ", pretty_data.c_str() );
    // ESP_LOGI(TAG,  "Dane wektorowe %S ", vec_data.c_str() );
    // We received a package with EVT data, we are starting to disassemble it

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // interested in completed responses to GET requests that arrived without errors from the drive
      ESP_LOGI(TAG,  "Request response received %X ", data[10] );
      switch (data[10]) { // cmd_submnu
        case TYPE_M:
          //           ESP_LOGI(TAG,  "type of drive %X",  data[14]);
          switch (data[14]) { //14
            case SLIDING:
              this->class_gate_ = SLIDING;
              //        ESP_LOGD(TAG, "Gate type: Sliding %#X ", data[14]);
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              //        ESP_LOGD(TAG, "Gate type: Sectional %#X ", data[14]);
              break;
            case SWING:
              this->class_gate_ = SWING;
              //        ESP_LOGD(TAG, "Gate type: Swing %#X ", data[14]);
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              //        ESP_LOGD(TAG, "Gate type: Barrier %#X ", data[14]);
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              //        ESP_LOGD(TAG, "Gate type: Up and over %#X ", data[14]);
              break;
          }  // switch 14
          break; //  TYPE_M
        case INF_IO: // response to a request for the position of the sliding gate limit switch
          switch (data[16]) { //16
            case 0x00:
              ESP_LOGI(TAG, "  The limit switch did not work ");
              break; // 0x00
            case 0x01:
              ESP_LOGI(TAG, "  Closing limit switch ");
              this->position = COVER_CLOSED;
              break; //  0x01
            case 0x02:
              ESP_LOGI(TAG, "  Opening limit switch ");
              this->position = COVER_OPEN;
              break; // 0x02

          }  // switch 16
          this->publish_state_if_changed();  // publish the status

          break; //  INF_IO


        //encoder maximum opening position, opening, closing

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {  
            this->_max_opn = (data[14] << 8) + data[15];
            max_encoder_position = this->_max_opn;
          }
          ESP_LOGI(TAG, "Maximum encoder position: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Closed gate position: %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15])>0x00) { // if the response from the actuator contains data about the opening position
          this->_pos_opn = (data[14] << 8) + data[15];}
          ESP_LOGI(TAG, "Gate open position: %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
            current_position = (data[14] << 8) + data[15];
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  The gate is open");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  The gate is closed");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  The gate is stopped");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  Gate status unknown");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
             case 0x0b:
              ESP_LOGI(TAG, "  Search for provisions done");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
              case STA_OPENING:
              ESP_LOGI(TAG, "  Opening in progress");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
              case STA_CLOSING:
              ESP_LOGI(TAG, "  Closing in progress");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }  // switch
          this->publish_state_if_changed();  // publish the status
          break;

          //      default: // cmd_mnu
        case AUTOCLS:
          this->autocls_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Auto close - L1: %S ", autocls_flag ? "Yes" : "No");  
          break;
          
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Close after photo - L2: %S ", photocls_flag ? "Yes" : "No");
          break;  
          
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Always close - L3: %S ", alwayscls_flag ? "Yes" : "No");
          break;     

        case STANDBY_ON:
          this->standby_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Stand-by - L4: %S ", standby_flag ? "Yes" : "No");
          break;
        
        case START_ON:
          this->peak_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Peak - L5: %S ", peak_flag ? "Yes" : "No");
          break; 
        
        case BLINK_ON:
          this->preflashing_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Pre-flasing - L6: %S ", preflashing_flag ? "Yes" : "No");
          break; 

        // case SLAVE_ON:
          // this->close_to_popen_flag = data[14];
          // ESP_LOGCONFIG(TAG, "  Close becomes Partial open- L7: %S ", close_to_popen_flag ? "Yes" : "No");
          // break; 
        
        case SLAVE_ON:
          this->slavemode_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Slave mode - L8: %S ", slavemode_flag ? "Yes" : "No");
          break; 

        // level2 settings:
        case P_TIME:
          this->pause_time = data[14];
          // auto *sensor = App.get_text_sensors("pause_time_sensor"); // Pobierz wskaźnik do text_sensor o nazwie "pause_time_sensor"
          // if (sensor != nullptr) {
            // sensor->publish_state(String(pause_time).c_str());  // Wywołanie funkcji w ESPHome, która zaktualizuje text_sensor
          // }
          // id(pause_time_sensor).publish_state(String(pause_time).c_str()); //Update sensor with id pause_time_sensor
          // id(pause_time_number).set_value(pause_time);
          // if (pause_time_sensor != nullptr) { // Aktualizacja wartości sensora, jeśli wskaźnik jest poprawnie ustawiony
            // ESP_LOGD("debug", "Aktualizacja pause_time_sensor: %p", pause_time_sensor);
            // pause_time_sensor->publish_state(String(pause_time).c_str());
          // } else {
            // ESP_LOGW("debug", "pause_time_sensor jest nullptr w parse_status_packet");
          // }
          //Kolejna metoda
          // std::string pause_time_str = std::to_string(pause_time);
          // id(pause_time_sensor).publish_state(pause_time_str.c_str());
          // if (this->pause_time_sensor != nullptr) {
            // std::string pause_time_str = std::to_string(this->pause_time);
            // this->pause_time_sensor->publish_state(pause_time_str);
          // } else {
            // ESP_LOGW("bus_t4", "pause_time_sensor is not set!");
          // }
          
          ESP_LOGCONFIG(TAG, "  Pause time - settings level 2, L1: %u", pause_time ); //in seconds
          break;
        
        case COMM_SBS:
          this->step_by_step_mode = data[14];
          ESP_LOGCONFIG(TAG, "  Step by step mode - settings level 2, L2: %u", step_by_step_mode ); 
          break;
          
        case SPEED_OPN:
          this->motor_speed_open = data[14];
          ESP_LOGCONFIG(TAG, "  Motor speed open - settings level 2, L3: %u", motor_speed_open ); 
          break;
          
        case SPEED_CLS:
          this->motor_speed_close = data[14];
          ESP_LOGCONFIG(TAG, "  Motor speed close - settings level 2, L3: %u", motor_speed_close ); 
          break;

        case OUT2:
          this->out2 = data[14];
          ESP_LOGCONFIG(TAG, "  GOI mode - settings level 2, L4: %u", out2 ); 
          break;

        case OPN_PWR:
          this->motor_force_open = data[14];
          ESP_LOGCONFIG(TAG, "  Motor force open - settings level 2, L5: %u", motor_force_open ); 
          break;

        case CLS_PWR:
          this->motor_force_close = data[14];
          ESP_LOGCONFIG(TAG, "  Motor force close - settings level 2, L4: %u", motor_force_close ); 
          break;

        case P_COUNT:
          this->p_count = (data[14] << 24) + (data[15] << 16) + (data[16] << 8) + data[17];
          ESP_LOGCONFIG(TAG, "  Number of cycles: %u", p_count ); 
          break;
					
        case OP_BLOCK:
          this->op_block_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Operator blocking: %S ", op_block_flag ? "Yes" : "No");
          break; 
      } // switch cmd_submnu
    } // if completed responses to GET requests received without errors from the drive

     if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) { // interested in incomplete responses to GET requests that came without errors from everyone
       ESP_LOGI(TAG,  "Received an incomplete response to request %X, continued at offset %X", data[10], data[12] );
       // repeat the command with the new offset
       tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));

     } // incomplete responses to GET requests that arrived without errors from the drive


    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // I'm interested in responses to SET requests that came without errors from the drive   
      switch (data[10]) { // cmd_submnu
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Auto close
          break;
          
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Close after Photo
          break;  
          
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // Always close
          break;  

        case STANDBY_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, STANDBY_ON, GET)); // Standby
          break;
        
        case START_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, START_ON, GET)); // peak
          break;
        
        case BLINK_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, BLINK_ON, GET)); // Pre-flasing
          break;   

        // case BLINK_ON:
          // tx_buffer_.push(gen_inf_cmd(FOR_CU, BLINK_ON, GET)); // Close becomes partial open
          // break;

        case SLAVE_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, SLAVE_ON, GET)); // Pre-flasing
          break;

        // level 2 settings
        case P_TIME:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, P_TIME, GET)); // pause time
         break;

        case COMM_SBS:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, COMM_SBS, GET)); // pause time
         break;

        case SPEED_OPN:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, SPEED_OPN, GET)); // pause time
         break;
         
        case SPEED_CLS:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, SPEED_CLS, GET)); // pause time
         break;

        case OUT2:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, OUT2, GET)); // pause time
         break;

        case OPN_PWR:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, OPN_PWR, GET)); // open force
         break;

        case CLS_PWR:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, CLS_PWR, GET)); // close force
         break;

        case P_COUNT:
         tx_buffer_.push(gen_inf_cmd(FOR_CU, P_COUNT, GET)); // number of cycles
         break;
				 
				case OP_BLOCK:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, OP_BLOCK, GET)); // Standby
          break;
      }// switch cmd_submnu
    }// if responses to SET requests received without errors from the drive

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // interested in FOR_ALL responses to GET requests that arrived without errors

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Manufacturer: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet is from the receiver
//            ESP_LOGCONFIG(TAG, "  Receiver: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // if the packet is from the receiver
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the package is from the drive controller
//            ESP_LOGCONFIG(TAG, "  Drive unit: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57}; // to understand that Walky drive
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00}; // to understand that the ROBUSHSR10 drive
            if (this->product_ == wla1) { 
              this->is_walky = true;
         //     ESP_LOGCONFIG(TAG, "  WALKY drive!: %S ", str.c_str());
                                        }
            if (this->product_ == ROBUSHSR10) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  Drive unit ROBUS!: %S ", str.c_str());
                                        }     

          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet is from the receiver
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the package is from the drive controller          
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet is from the receiver
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the package is from the drive controller          
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // if the packet is from the receiver
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // if the package is from the drive controller          
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // drive unit
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
     //         init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // receiver
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }  // switch

    }  // if  FOR_ALL responses to GET requests that arrived without errors

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) { //  packets from the receiver with information about the list of remote controls that arrived without errors
      ESP_LOGCONFIG(TAG, "Remote control number: %X%X%X%X, command: %X, button: %X, mode: %X, click counter: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }  // if

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) { //  packets from the receiver with information about the remote control button read
      ESP_LOGCONFIG(TAG, "button %X, remote control number: %X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }  // if

  } //  if evt



  //else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // otherwise the Response packet is a confirmation of the received command
  else if (data[1] > 0x0d) {  // otherwise the Response packet is a confirmation of the received command
    ESP_LOGD(TAG, "RSP packet received");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Data string: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %S ", pretty_data.c_str() );
    // ESP_LOGI(TAG,  "Dane wektorowe %S ", vec_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG, "Drive Controller Package");
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG, "RUN submenu");
      if (data[11] >= 0x80) {
        switch (data[11] - 0x80) {  // sub_run_cmd1
          case SBS:
            ESP_LOGI(TAG, "Command: Step by step");
            break;
          case STOP:
            ESP_LOGI(TAG, "Command: STOP");
            break;
          case OPEN:
            ESP_LOGI(TAG, "Command: OPEN");
            this->current_operation = COVER_OPERATION_OPENING;
            break;
          case CLOSE:
            ESP_LOGI(TAG, "Command: CLOSE");
            this->current_operation = COVER_OPERATION_CLOSING;
            break;
          case P_OPN1:
            ESP_LOGI(TAG, "Command: Partial opening 1");
            break;
          case STOPPED:
            ESP_LOGI(TAG, "Command: Stopped");
            this->current_operation = COVER_OPERATION_IDLE;
            request_position();
            break;
          case ENDTIME:
            ESP_LOGI(TAG, "Operation timed out");
            this->current_operation = COVER_OPERATION_IDLE;
            request_position();
            break;
          default:
            ESP_LOGI(TAG, "Unknown command: %X", data[11]);
        }  // switch sub_run_cmd1
      } else {
        switch (data[11]) {  // sub_run_cmd2
          case STA_OPENING:
            ESP_LOGI(TAG, "Operation: Opens");
            this->current_operation = COVER_OPERATION_OPENING;
            break;
          case STA_CLOSING:
            ESP_LOGI(TAG, "Operation: Closed");
            this->current_operation = COVER_OPERATION_CLOSING;
            break;
          case CLOSED:
            ESP_LOGI(TAG, "Operation: Closed");
            this->current_operation = COVER_OPERATION_IDLE;
            this->position = COVER_CLOSED;
            break;
          case OPENED:
            ESP_LOGI(TAG, "Operation: Open");
            this->current_operation = COVER_OPERATION_IDLE;
            this->position = COVER_OPEN;
            // calibrate opened position if the motor does not report max supported position (Road 400)
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
            break;
          case STOPPED:
            ESP_LOGI(TAG, "Operation: Stopped");
            this->current_operation = COVER_OPERATION_IDLE;
            request_position();
            break;
          case PART_OPENED:
            ESP_LOGI(TAG, "Operation: Partially open");
            this->current_operation = COVER_OPERATION_IDLE;
            request_position();
            break;
          default:
            ESP_LOGI(TAG, "Unknown operation: %X", data[11]);
        }  // switch sub_run_cmd2
      }
      this->publish_state_if_changed();  // publish the status
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Submenu Status in motion" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
              case 0x83: // Road 400
                ESP_LOGI(TAG, "Movement: Opens" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84: // Road 400
                ESP_LOGI(TAG,  "Movement: Closes" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Traffic: Closed" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Traffic: Open");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Traffic: Stopped");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Movement: %X", data[11] );

                
            } // switch sub_run_cmd2

            update_position((data[12] << 8) + data[13]);
            break; //STA

          default: // sub_inf_cmd
            ESP_LOGI(TAG,  "Submenu %X", data[10] );
        }  // switch sub_inf_cmd

        break; // Drive Controller Package
      case CONTROL:
        ESP_LOGI(TAG,  "CONTROL package" );
        break; // CONTROL
      case FOR_ALL:
        ESP_LOGI(TAG,  "Package for everyone" );
        break; // FOR_ALL
      case 0x0A:
        ESP_LOGI(TAG,  "Receiver package" );
        break; // receiver package
      default: // cmd_mnu
        ESP_LOGI(TAG,  "Menu %X", data[9] );
    }  // switch  cmd_mnu


  } // else


  ///////////////////////////////////////////////////////////////////////////////////


  // RSP is a response (ReSPonce) to simply receiving a CMD command rather than executing it. It also reports the completion of the operation.
  /* if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[12] == 0x19)) { // we recognize the status packet by its content in certain bytes
     //  ESP_LOGD(TAG, "RSP packet received. cmd = %#x", data[11]);

     switch (data[11]) {
       case OPENING:
         this->current_operation = COVER_OPERATION_OPENING;
         ESP_LOGD(TAG, "Status: Opening");
         break;
       case CLOSING:
         this->current_operation = COVER_OPERATION_CLOSING;
         ESP_LOGD(TAG, "Status: Closed");
         break;
       case OPENED:
         this->position = COVER_OPEN;
         ESP_LOGD(TAG, "Status: Open");
         this->current_operation = COVER_OPERATION_IDLE;
         break;


       case CLOSED:
         this->position = COVER_CLOSED;
         ESP_LOGD(TAG, "Status: Closed");
         this->current_operation = COVER_OPERATION_IDLE;
         break;
       case STOPPED:
         this->current_operation = COVER_OPERATION_IDLE;
         ESP_LOGD(TAG, "Status: Stopped");
         break;

     }  // switch

     this->publish_state();  // publish the status

    } //if
  */
  /*
    // status after reaching limit switches
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) &&  (data[12] == 0x00)) { // we recognize the status packet by its content in certain bytes
      ESP_LOGD(TAG, "Received package of limit switches. Status = %#x", data[11]);
      switch (data[11]) {
        case OPENED:
          this->position = COVER_OPEN;
          ESP_LOGD(TAG, "Status: Open");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          ESP_LOGD(TAG, "Status: Closed");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Status: Opening");
          break;
        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Status: Closed");
          break;
      } //switch
      this->publish_state();  // publish the status
    } //if
  */
  // STA = 0x40,   // status in motion
  /*
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == STA) ) { // we recognize the status packet by its content in certain bytes
      uint16_t ipos = (data[12] << 8) + data[13];
      ESP_LOGD(TAG, "Current maneuver: %#X Position: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
      this->position = ipos / 2100.0f; // passing the position to the component

      switch (data[11]) {
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Status: Opening");
          break;

        case OPENING2:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Status: Opening");
          break;

        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Status: Closed");
          break;
        case CLOSING2:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Status: Closed");
          break;
        case OPENED:
          this->position = COVER_OPEN;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Status: Open");
          //      this->current_operation = COVER_OPERATION_OPENING;
          //    ESP_LOGD(TAG, "Status: Opening");
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Status: Closed");
          //      this->current_operation = COVER_OPERATION_CLOSING;
          //ESP_LOGD(TAG, "Status: Closed");
          break;
        case STOPPED:
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Status: Stopped");
          break;

      }  // switch

      this->publish_state();  // publish the status

    } //if
  */


  ////////////////////////////////////////////////////////////////////////////////////////
} // function







void NiceBusT4::dump_config() {    //  add information about the connected controller to the log
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
  switch (this->class_gate_) {
    case SLIDING:
      ESP_LOGCONFIG(TAG, "  Type: Sliding Gate");
      break;
    case SECTIONAL:
      ESP_LOGCONFIG(TAG, "  Type: Sectional doors");
      break;
    case SWING:
      ESP_LOGCONFIG(TAG, "  Type: Swing Gate");
      break;
    case BARRIER:
      ESP_LOGCONFIG(TAG, "  Type: Barrier");
      break;
    case UPANDOVER:
      ESP_LOGCONFIG(TAG, "  Type: Up and Over Gates");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Type: Unknown Gate, 0x%02X", this->class_gate_);
  } // switch


  ESP_LOGCONFIG(TAG, "  Maximum encoder or timer position: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Gate open position: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Closed gate position: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Manufacturer: %S ", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Drive unit: %S ", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Drive hardware: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Drive firmware: %S ", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Drive description: %S ", dsc_str.c_str());


  ESP_LOGCONFIG(TAG, "  Gateway address: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Drive address: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Receiver address: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Receiver: %S ", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Receiver hardware: %S ", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Receiver firmware: %S ", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Receiver Description: %S ", oxi_dsc_str.c_str());

  //settings - level 1
  ESP_LOGCONFIG(TAG, "  Auto close - L1: %S ", autocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Close after photo - L2: %S ", photocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Always close - L3: %S ", alwayscls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Stand-by - L4: %S ", standby_flag ? "Yes" : "No");  
  ESP_LOGCONFIG(TAG, "  Peak - L5: %S ", peak_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Pre-flasing - L6: %S ", preflashing_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Close becomes Partial open - L7: %S ", close_to_popen_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Slave mode - L8: %S ", slavemode_flag ? "Yes" : "No");

  //settings - level 2
  ESP_LOGCONFIG(TAG, "  Pause time level - level 2, L1: %u ", pause_time);
  ESP_LOGCONFIG(TAG, "  Step by step mode - level 2, L2: %u ", step_by_step_mode);
  ESP_LOGCONFIG(TAG, "  Motor speed open - level 2, L3: %u ", motor_speed_open);
  ESP_LOGCONFIG(TAG, "  Motor speed close - level 2, L3: %u ", motor_speed_close);
  ESP_LOGCONFIG(TAG, "  GOI mode - level 2, L4: %u ", out2);
  ESP_LOGCONFIG(TAG, "  Motor force open - level 2, L5: %u ", motor_force_open);
  ESP_LOGCONFIG(TAG, "  Motor force close - level 2, L5: %u ", motor_force_close);
  ESP_LOGCONFIG(TAG, "  Number of cycles: %u ", p_count);
	ESP_LOGCONFIG(TAG, "  Operator blocking: %S ", op_block_flag ? "Yes" : "No");  

}



//formation of a management command
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]}; // tytuł
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // OFFSET CMD, DPRO924 refused to work with 0x00, although other drives responded to commands
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // to output the command to the log
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "Command formed: %S ", pretty_cmd.c_str() );

  return frame;
}

// generating an INF command with and without data
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]}; // tytuł
  frame.push_back(INF);  // 0x08 mes_type
  frame.push_back(0x06 + len); // mes_size
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data); // next_data
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end()); // blok danych
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // to output the command to the log
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "INF package generated: %S ", pretty_cmd.c_str() );

  return frame;

}


void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < uint8_t > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());
}

//  Here you need to add a check for incorrect data from the user
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(std::string data) { // preparing user-entered data for sending
  // remove everything except hexadecimal letters and numbers
  data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)) );
  }), data.end()); 

  //assert (data.size () % 2 == 0); // check parity
  std::vector < uint8_t > frame;
  frame.resize(0); // reset the command size

  for (uint8_t i = 0; i < data.size (); i += 2 ) {          // fill the command array
    std::string sub_str(data, i, 2);                        // take 2 bytes from the command
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16);  // convert to number
    frame.push_back(hexstoi);                               // write the number to the line element of the new command
  }
  
  return frame;
}


void NiceBusT4::send_array_cmd(std::vector<uint8_t> data) {          // sends break + command prepared earlier in the array
  return send_array_cmd((const uint8_t *)data.data(), data.size());
}
void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  // sending data to uart

  char br_ch = 0x00;                            // for break
  uartFlush(_uart);                             // clear uart
  uartSetBaudRate(_uart, BAUD_BREAK);           // lower the body rate
  //uart_write(_uart, &br_ch, 1);               // for ESP8266                     // send zero at low speed, long zero
  uart_write_bytes(UART_NUM_1, &br_ch, 1);      // for ESP32    // send zero at low speed, long zero
  //uart_write(_uart, (char *)&dummy, 1);
  //uart_wait_tx_empty(_uart);                  // for ESP8266   // We wait until the sending is completed. There is an error here in the uart.h library (esp8266 core 3.0.2), waiting is not enough for further uart_set_baudrate().
  uart_wait_tx_done(UART_NUM_1,100);            // for ESP32      // We wait until the sending is completed. There is an error here in the uart.h library (esp8266 core 3.0.2), waiting is not enough for further uart_set_baudrate().
  delayMicroseconds(90);                        // add a delay to the wait, otherwise the speed will switch before sending. With delay on d1-mini I got a perfect signal, break = 520us
  uartSetBaudRate(_uart, BAUD_WORK);            // we return the working body rate
  //uart_write(_uart, (char *)&data[0], len);             // for ESP8266   // send the main package
  uart_write_bytes(UART_NUM_1, (char *)&data[0], len);    // for ESP32      // send the main package
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  //uart_wait_tx_empty(_uart);          // for ESP8266     // waiting for the sending to complete
  uart_wait_tx_done(UART_NUM_1,100);    // for ESP32        // waiting for the sending to complete
  delayMicroseconds(90);
  //delayMicroseconds(150); //for ESP32


  std::string pretty_cmd = format_hex_pretty((uint8_t*)&data[0], len);                    // to output the command to the log
  ESP_LOGI(TAG,  "Sent: %S ", pretty_cmd.c_str() );

}

// generating and sending inf commands from yaml configuration
void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
  std::vector < uint8_t > v_to_addr = raw_cmd_prepare (to_addr);
  std::vector < uint8_t > v_whose = raw_cmd_prepare (whose);
  std::vector < uint8_t > v_command = NiceBusT4::raw_cmd_prepare (command);
  std::vector < uint8_t > v_type_command = raw_cmd_prepare (type_command);
  std::vector < uint8_t > v_next_data = raw_cmd_prepare (next_data);
  std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);


  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  } // else
}

// generating and sending installation commands to the drive controller from yaml configuration with minimal parameters
void NiceBusT4::set_mcu(std::string command, std::string data_command) {
    std::vector < uint8_t > v_command = raw_cmd_prepare (command);
    std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
  
// device initialization
void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == FOR_CU) {
    ESP_LOGI(TAG, "Checkinf motor settings");
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // drive type request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // manufacturer's request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  firmware request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //product request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //hardware request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //opening position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // closing position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //request description
    if (is_walky)  // request for maximum value for encoder
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();  // current position request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //Gate status (Open/Closed/Stopped)
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00)); // Auto close
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00)); // Close after Photo
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00)); // Always close
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, STANDBY_ON, GET, 0x00)); // Stand by    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, START_ON, GET, 0x00)); // Peak
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, BLINK_ON, GET, 0x00)); // Pre-flashing
    //tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, CLOSE_TO_POPEN_ON, GET, 0x00)); // Close to Partial open
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, SLAVE_ON, GET, 0x00)); // Slave mode

    //level 2 settings  
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, P_TIME, GET, 0x00));    // Pause time
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, COMM_SBS, GET, 0x00));  // Step by step mode
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, SPEED_OPN, GET, 0x00)); // Speed open
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, SPEED_CLS, GET, 0x00)); // Speed close
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, OUT2, GET, 0x00));      // GOI mode
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, OPN_PWR, GET, 0x00));   // open force
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, CLS_PWR, GET, 0x00));   // close force

    //other settings/informations
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, P_COUNT, GET, 0x00)); // Number of cycles
		tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, OP_BLOCK, GET, 0x00)); // Stand by  
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); // product request
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); // hardware request    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); // firmware request    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); // request description  
  }

 //  for(int licznik = 0x70; licznik <= 0x9F; ++licznik) {
  // send_inf_cmd("0003", "04", licznik, "a9", "00", true, "01");
  // send_inf_cmd("0003", "04", licznik, "99", "00", true, "01");     
 //        delayMicroseconds(1000000);
 //    }
}

// Querying the conditional current position of the actuator
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Update current actuator position
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Conditional gate position: %d, position at %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();  // publish the status
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
    ESP_LOGI(TAG, "The required position has been reached. Stopping the gate");
    send_cmd(STOP);
    position_hook_type = IGNORE;
  }
}

// Publish gate status when changed
void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
