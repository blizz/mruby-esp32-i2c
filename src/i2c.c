#include <mruby.h>
#include <mruby/array.h>
#include <mruby/string.h>
#include <mruby/value.h>
#include <mruby/variable.h>

#include "driver/i2c.h"
#include "string.h"

#define E_I2C_ERROR (mrb_class_get(mrb, "I2CError"))

#define I2C_MAX_READ_BUFFER_LENGTH         100
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE    0           /*!< I2C master do not need buffer             from NKOLBAN example */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE    0           /*!< I2C master do not need buffer             from NKOLBAN example */
#define ACK_CHECK_EN                       0x1           /*!< I2C master will check ack from slave      from NKOLBAN example */
#define ACK_CHECK_DIS                      0x0           /*!< I2C master will not check ack from slave  from NKOLBAN example */
#define ACK_VAL                            0x0           /*!< I2C ack value                             from NKOLBAN example */
#define NACK_VAL                           0x1           /*!< I2C nack value                            from NKOLBAN example */

#define BH1750_SENSOR_ADDR                 0x48             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x00             /*!< Command to set measure mode */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define I2C_TYPE_CHAR                      1
#define I2C_TYPE_SIGNED_CHAR               2
#define I2C_TYPE_UNSIGNED_CHAR             3
#define I2C_TYPE_SHORT                     4 
#define I2C_TYPE_UNSIGNED_SHORT            5
#define I2C_TYPE_LONG                      6
#define I2C_TYPE_UNSIGNED_LONG             7
#define I2C_TYPE_LONG_LONG                 8
#define I2C_TYPE_UNSIGNED_LONG_LONG        9
#define I2C_TYPE_FLOAT                    10 
#define I2C_TYPE_DOUBLE                   11
#define I2C_TYPE_LONG_DOUBLE              12 
#define I2C_TYPE_STRING                   13 
#define I2C_TYPE_END                      14

static mrb_value
mrb_esp32_i2c_init(mrb_state *mrb, mrb_value self)
{

  printf("mrb_esp32_i2c_init\n");

  mrb_int mode, port, scl, sda, freq;
  mrb_bool scl_pullup, sda_pullup;
  i2c_config_t conf;

  mrb_get_args(mrb, "iiibibi", &port, &mode, &scl, &scl_pullup, &sda, &sda_pullup, &freq);

  printf("mrb_esp32_i2c_init       port=%d\n",port);
  printf("mrb_esp32_i2c_init       mode=%d\n",mode);
  printf("mrb_esp32_i2c_init        scl=%d\n",scl);
  printf("mrb_esp32_i2c_init scl_pullup=%d\n",scl_pullup);
  printf("mrb_esp32_i2c_init        sda=%d\n",sda);
  printf("mrb_esp32_i2c_init sda_pullup=%d\n",sda_pullup);
  printf("mrb_esp32_i2c_init       freq=%d\n",freq);

  conf.mode = mode;
  conf.scl_io_num = scl;
  conf.sda_io_num = sda;
  conf.scl_pullup_en = scl_pullup;
  conf.sda_pullup_en = sda_pullup;
  conf.master.clk_speed = freq;

  i2c_param_config(   port, &conf);
  i2c_driver_install( port, 
                      mode, 
                      I2C_EXAMPLE_MASTER_RX_BUF_DISABLE, 
                      I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 
                      0);

  return self;
}

static mrb_value
mrb_esp32_i2c_deinit(mrb_state *mrb, mrb_value self)
{
  mrb_value port;

  port = mrb_iv_get(mrb, self, mrb_intern_lit(mrb, "@port"));
  i2c_driver_delete(mrb_fixnum(port));
  return mrb_nil_value();
}

static mrb_value
mrb_esp32_i2c_send(mrb_state *mrb, mrb_value self) {
    mrb_value data, port;
    mrb_int addr;
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    mrb_get_args(mrb, "Si", &data, &addr);
    port = mrb_iv_get(mrb, self, mrb_intern_lit(mrb, "@port"));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1 ) | I2C_MASTER_WRITE, 1);
    if (!mrb_nil_p(data)) {
      i2c_master_write(cmd, RSTRING_PTR(data), RSTRING_LEN(data), 1);
    }
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(mrb_fixnum(port), cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return mrb_fixnum_value(err);
}

static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    printf( "conf.mode = %d\n", conf.mode );
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    printf( "conf.sda_io_num = %d\n", conf.sda_io_num );
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    printf( "conf.sda_pullup_en = %d\n", conf.sda_pullup_en );
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    printf( "conf.scl_io_num = %d\n",  conf.scl_io_num);
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    printf( "conf.scl_pullup_en = %d\n", conf.scl_pullup_en );
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    printf( "conf.master.clk_speed = %d\n", conf.master.clk_speed );
    i2c_param_config(i2c_master_port, &conf);
    printf( "i2c_master_port = %d\n",  i2c_master_port);
    printf( "I2C_EXAMPLE_MASTER_RX_BUF_DISABLE = %d\n", I2C_EXAMPLE_MASTER_RX_BUF_DISABLE );
    printf( "I2C_EXAMPLE_MASTER_TX_BUF_DISABLE = %d\n", I2C_EXAMPLE_MASTER_TX_BUF_DISABLE );
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_example_master_sensor_test(mrb_state *mrb, mrb_value self)
{
    i2c_port_t i2c_num = 1;
    uint8_t    datah;
    uint8_t    datal;
    uint8_t*   data_h = &datah;
    uint8_t*   data_l = &datal;

    i2c_example_master_init();

    int ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    printf( "send address = %d %d\n", BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN );
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    printf( "send cmd = %d %d\n", BH1750_CMD_START, ACK_CHECK_EN );
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    printf( "send i2c_num = %d\n", i2c_num );
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    printf( "send address = %d %d\n", BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN );
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret == ESP_OK) {
            printf("******xxx*************\n");
            printf("*******************\n");
            printf("data_h: %02x\n", datah);
            printf("data_l: %02x\n", datal);
            printf("sensor val: %f\n", (datah << 8 | datal) / 1.2);
    } else {
            printf("No ack, sensor not connected...skip...\n");
    }

    return ret;
}

static mrb_value
mrb_esp32_i2c_send_receive(mrb_state *mrb, mrb_value self) {
    mrb_value        send_string, port;
    mrb_int          addr, read_len, receive_data_type;
    i2c_cmd_handle_t cmd;
    esp_err_t        err;
    char            *buffer; //[I2C_MAX_READ_BUFFER_LENGTH+1];

             char                    char_val;
             char                    char_val_ary[2] = "\0\0";
      signed char             signed_char_val;
    unsigned char           unsigned_char_val;
             short                  short_val;
    unsigned short         unsigned_short_val;
             long                    long_val;
    unsigned long           unsigned_long_val;
        long long               long_long_val;
    unsigned long long unsigned_long_long_val;
             float                  float_val;
             double                double_val;
        long double           long_double_val;

    printf("mrb_esp32_i2c_send_receive\n");

    mrb_get_args(mrb, "Siii", &send_string, &addr, &receive_data_type, &read_len);

    //send_data = (uint8_t) *RSTRING_PTR(send_string);

    printf("mrb_esp32_i2c_send_receive read_len = %d\n", read_len);
    printf("mrb_esp32_i2c_send_receive send_data = %s\n", RSTRING_PTR(send_string));

    if ( read_len > I2C_MAX_READ_BUFFER_LENGTH) {
      return mrb_fixnum_value(ESP_ERR_INVALID_SIZE);
    }
    if (( 0 == receive_data_type ) || ( I2C_TYPE_END <= receive_data_type )) {
      return mrb_fixnum_value(ESP_ERR_INVALID_ARG);
    }

    if (I2C_TYPE_STRING == receive_data_type)
      buffer = (char*) calloc( 1, read_len + 1 ); 

    port = mrb_iv_get(mrb, self, mrb_intern_lit(mrb, "@port"));

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    printf("mrb_esp32_i2c_send_receive addr=%d\n",(addr << 1 ) | I2C_MASTER_WRITE);
    printf("mrb_esp32_i2c_send_receive portTICK_RATE_MS=%d\n",portTICK_RATE_MS);
    i2c_master_write_byte(cmd, (addr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (1 == RSTRING_LEN(send_string)) {
      printf("mrb_esp32_i2c_send_receive single_byte_send=0\n");
      i2c_master_write_byte(cmd, *RSTRING_PTR(send_string), ACK_CHECK_EN);
    } else {
      if (!mrb_nil_p(send_string)) {
        i2c_master_write(cmd, RSTRING_PTR(send_string), RSTRING_LEN(send_string), ACK_CHECK_EN);
      }
    }
    i2c_master_stop(cmd);

    printf( "send i2c_num = %d\n", mrb_fixnum(port) );

    err = i2c_master_cmd_begin(mrb_fixnum(port), cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
      printf( "Returning error A = %d\n", err);
      if (I2C_TYPE_STRING == receive_data_type)
        free(buffer);
      return mrb_fixnum_value(err);
    }

    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    printf( "send address = %d %d\n", (addr << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN );
    i2c_master_write_byte(cmd, (addr << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);

    printf( "A\n");

    uint8_t           high_byte;
    uint8_t upper_mid_high_byte;
    uint8_t  upper_mid_low_byte;
    uint8_t       mid_high_byte;
    uint8_t        mid_low_byte;
    uint8_t lower_mid_high_byte;
    uint8_t  lower_mid_low_byte;
    uint8_t            low_byte;
    uint8_t *dptr;
    int      ack;
    printf( "B\n");
    switch (receive_data_type) {
      case I2C_TYPE_CHAR               :
        i2c_master_read_byte(cmd, &char_val, NACK_VAL);
        break;
      case I2C_TYPE_SIGNED_CHAR        :
        i2c_master_read_byte(cmd, &signed_char_val, NACK_VAL);
        break;
      case I2C_TYPE_UNSIGNED_CHAR      :
        i2c_master_read_byte(cmd, &unsigned_char_val, NACK_VAL);
        break;
      case I2C_TYPE_SHORT              :
    printf( "C\n");
        i2c_master_read_byte(cmd, &high_byte,  ACK_VAL);
        i2c_master_read_byte(cmd,  &low_byte, NACK_VAL);
        break;
      case I2C_TYPE_UNSIGNED_SHORT     :
        i2c_master_read_byte(cmd, &high_byte,  ACK_VAL);
        i2c_master_read_byte(cmd,  &low_byte, NACK_VAL);
        break;
      case I2C_TYPE_LONG               :
        i2c_master_read_byte(cmd,     &high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,      &low_byte,  NACK_VAL);
        //long_val = (high_byte << 24 | mid_high_byte << 16 | mid_low_byte << 8 | low_byte);
        break;
      case I2C_TYPE_UNSIGNED_LONG      :
        i2c_master_read_byte(cmd,     &high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,      &low_byte,  NACK_VAL);
        //unsigned_long_val = (high_byte << 24 | mid_high_byte << 16 | mid_low_byte << 8 | low_byte);
        break;
      case I2C_TYPE_LONG_LONG          : 
        i2c_master_read_byte(cmd,           &high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &upper_mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &upper_mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,       &mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,        &mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &lower_mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &lower_mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,            &low_byte,  NACK_VAL);
        //long_long_val = ((long long)high_byte << 56 | (long long)upper_mid_high_byte << 48 | (long long)upper_mid_low_byte << 40 | (long long)mid_high_byte << 32 | (long long)mid_low_byte << 24 | (long long)lower_mid_high_byte << 16 | (long long)lower_mid_low_byte << 8 | (long long)low_byte);
        break;
      case I2C_TYPE_UNSIGNED_LONG_LONG :
        i2c_master_read_byte(cmd,           &high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &upper_mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &upper_mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,       &mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,        &mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd, &lower_mid_high_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,  &lower_mid_low_byte,   ACK_VAL);
        i2c_master_read_byte(cmd,            &low_byte,  NACK_VAL);
        //unsigned_long_long_val = ((unsigned long long)high_byte << 56 | (unsigned long long)upper_mid_high_byte << 48 | (unsigned long long)upper_mid_low_byte << 40 | (unsigned long long)mid_high_byte << 32 | (unsigned long long)mid_low_byte << 24 | (unsigned long long)lower_mid_high_byte << 16 | (unsigned long long)lower_mid_low_byte << 8 | (unsigned long long)low_byte);
        break;
      case I2C_TYPE_FLOAT              :
        dptr = (uint8_t*) &float_val;
        ack  = ACK_VAL;
        for (int i=0; i < sizeof(float_val); i++ ) {
          if (i == (read_len - 1))
            ack = NACK_VAL;
          i2c_master_read_byte(cmd, dptr++, ack);
        }
        break;
      case I2C_TYPE_DOUBLE             :
        dptr = (uint8_t*) &double_val;
        ack  = ACK_VAL;
        for (int i=0; i < sizeof(double_val); i++ ) {
          if (i == (read_len - 1))
            ack = NACK_VAL;
          i2c_master_read_byte(cmd, dptr++, ack);
        }
        break;
      case I2C_TYPE_LONG_DOUBLE        :
        dptr = (uint8_t*) &long_double_val;
        ack  = ACK_VAL;
        for (int i=0; i < sizeof(long_double_val); i++ ) {
          if (i == (read_len - 1))
            ack = NACK_VAL;
          i2c_master_read_byte(cmd, dptr++, ack);
        }
        break;
      case I2C_TYPE_STRING             :
        dptr = (uint8_t*) buffer;
        ack  = ACK_VAL;
        for (int i=0; i < read_len; i++ ) {
          if (i == (read_len - 1))
            ack = NACK_VAL;
          i2c_master_read_byte(cmd, dptr++, ack);
        }
        break;
    }

    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(mrb_fixnum(port), cmd, 1000 / portTICK_RATE_MS);

    printf("data_h: %02x\n", high_byte);
    printf("data_l: %02x\n", low_byte);
    printf("sensor val: %f\n", (high_byte << 8 | low_byte) / 1.2);


    i2c_cmd_link_delete(cmd);

    if (0 != err) {
      printf( "Returning error B = %d\n", err);
      if (I2C_TYPE_STRING == receive_data_type)
        free(buffer);
      return mrb_fixnum_value(err);
    } else {

      mrb_value newval;
      switch (receive_data_type) {
        case I2C_TYPE_CHAR               :
          char_val_ary[0] = char_val;
          return mrb_str_new_cstr(mrb,char_val_ary);
        case I2C_TYPE_SIGNED_CHAR        :
          return mrb_fixnum_value(signed_char_val);
        case I2C_TYPE_UNSIGNED_CHAR      :
          return mrb_fixnum_value(unsigned_char_val);
        case I2C_TYPE_SHORT              :
          short_val = (high_byte << 8 | low_byte);
          return mrb_fixnum_value(short_val);
        case I2C_TYPE_UNSIGNED_SHORT     :
          unsigned_short_val = (high_byte << 8 | low_byte);
          return mrb_fixnum_value(unsigned_short_val);
        case I2C_TYPE_LONG               :
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_UNSIGNED_LONG      :
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_LONG_LONG          : 
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_UNSIGNED_LONG_LONG :
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_FLOAT              :
          if (4 == sizeof(float_val)) {
            newval.value.f = float_val;
            return newval;
          }
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_DOUBLE             :
          if (4 == sizeof(double_val)) {
            newval.value.f = double_val;
            return newval;
          }
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_LONG_DOUBLE        :
          return mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED);
        case I2C_TYPE_STRING             :
          printf( "Returning binary data C\n");
          printf("sensor val: %f\n", ( ((uint8_t) buffer[0]) << 8 | ((uint8_t) buffer[1]) ) / 1.2);
          newval = mrb_str_new_cstr(mrb,buffer);
          free(buffer);
          return newval;
      }
    }
  return mrb_fixnum_value(ESP_FAIL);
}

void
mrb_mruby_esp32_i2c_gem_init(mrb_state* mrb)
{
  struct RClass *esp32, *i2c, *constants;

  esp32 = mrb_define_module(mrb, "ESP32");

  i2c = mrb_define_class_under(mrb, esp32, "I2C", mrb->object_class);
  mrb_define_method(mrb, i2c, "_init",                          mrb_esp32_i2c_init,             MRB_ARGS_REQ(7));
  mrb_define_method(mrb, i2c, "deinit",                         mrb_esp32_i2c_deinit,           MRB_ARGS_NONE());
  mrb_define_method(mrb, i2c, "send",                           mrb_esp32_i2c_send,             MRB_ARGS_REQ(2));
  mrb_define_method(mrb, i2c, "send_receive",                   mrb_esp32_i2c_send_receive,     MRB_ARGS_ARG(3,1));
  //mrb_define_method(mrb, i2c, "i2c_example_master_sensor_test", i2c_example_master_sensor_test, MRB_ARGS_NONE());

  mrb_define_const(mrb, i2c, "CHAR",                mrb_fixnum_value( I2C_TYPE_CHAR ));
  mrb_define_const(mrb, i2c, "SIGNED_CHAR",         mrb_fixnum_value( I2C_TYPE_SIGNED_CHAR ));
  mrb_define_const(mrb, i2c, "UNSIGNED_CHAR",       mrb_fixnum_value( I2C_TYPE_UNSIGNED_CHAR ));
  mrb_define_const(mrb, i2c, "SHORT",               mrb_fixnum_value( I2C_TYPE_SHORT ));
  mrb_define_const(mrb, i2c, "UNSIGNED_SHORT",      mrb_fixnum_value( I2C_TYPE_UNSIGNED_SHORT ));
  mrb_define_const(mrb, i2c, "LONG",                mrb_fixnum_value( I2C_TYPE_LONG ));
  mrb_define_const(mrb, i2c, "UNSIGNED_LONG",       mrb_fixnum_value( I2C_TYPE_UNSIGNED_LONG ));
  mrb_define_const(mrb, i2c, "LONG_LONG",           mrb_fixnum_value( I2C_TYPE_LONG_LONG ));
  mrb_define_const(mrb, i2c, "UNSIGNED_LONG_LONG",  mrb_fixnum_value( I2C_TYPE_UNSIGNED_LONG_LONG ));
  mrb_define_const(mrb, i2c, "FLOAT",               mrb_fixnum_value( I2C_TYPE_FLOAT ));
  mrb_define_const(mrb, i2c, "DOUBLE",              mrb_fixnum_value( I2C_TYPE_DOUBLE ));
  mrb_define_const(mrb, i2c, "LONG_DOUBLE",         mrb_fixnum_value( I2C_TYPE_LONG_DOUBLE ));
  mrb_define_const(mrb, i2c, "STRING",              mrb_fixnum_value( I2C_TYPE_STRING ));

  constants = mrb_define_module_under(mrb, i2c, "Constants");

  mrb_define_const(mrb, constants, "PORT0",  mrb_fixnum_value( I2C_NUM_0 ));
  mrb_define_const(mrb, constants, "PORT1",  mrb_fixnum_value( I2C_NUM_1 ));

  mrb_define_const(mrb, constants, "SDA0",   mrb_fixnum_value( GPIO_NUM_18 ));
  mrb_define_const(mrb, constants, "SCL0",   mrb_fixnum_value( GPIO_NUM_19 ));
  mrb_define_const(mrb, constants, "SDA1",   mrb_fixnum_value( GPIO_NUM_25 ));
  mrb_define_const(mrb, constants, "SCL1",   mrb_fixnum_value( GPIO_NUM_26 ));

  mrb_define_const(mrb, constants, "MASTER", mrb_fixnum_value( I2C_MODE_MASTER ));
  mrb_define_const(mrb, constants, "SLAVE",  mrb_fixnum_value( I2C_MODE_SLAVE ));

}

void
mrb_mruby_esp32_i2c_gem_final(mrb_state* mrb)
{
}
