#include <ESP8266WiFi.h>

// GPIO16, or D0, is not available since it is in its own pin register
#define GPIO_TDI    D6
#define GPIO_TDO    D4
#define GPIO_TCK    D7
#define GPIO_TMS    D5

//#define VERBOSE
#define MAX_WRITE_SIZE  512

#define XVCD_AP_MODE
#define XVCD_STATIC_IP

// AP mode or STA mode:
const char *ssid = "jtag";
const char *pw = "pippo123";

IPAddress ip(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress netmask(255, 255, 255, 0);
const int port = 2542;

WiFiServer server(port);
WiFiClient client;

int jtag_state;

// JTAG buffers
uint8_t cmd[16];
uint8_t buffer[1024], result[512];

enum {

  test_logic_reset, run_test_idle,

  select_dr_scan, capture_dr, shift_dr,
  exit1_dr, pause_dr, exit2_dr, update_dr,

  select_ir_scan, capture_ir, shift_ir,
  exit1_ir, pause_ir, exit2_ir, update_ir,

  num_states
};

const int next_state[num_states][2] = {

  [test_logic_reset] = {run_test_idle, test_logic_reset},
  [run_test_idle] = {run_test_idle, select_dr_scan},

  [select_dr_scan] = {capture_dr, select_ir_scan},
  [capture_dr] = {shift_dr, exit1_dr},
  [shift_dr] = {shift_dr, exit1_dr},
  [exit1_dr] = {pause_dr, update_dr},
  [pause_dr] = {pause_dr, exit2_dr},
  [exit2_dr] = {shift_dr, update_dr},
  [update_dr] = {run_test_idle, select_dr_scan},

  [select_ir_scan] = {capture_ir, test_logic_reset},
  [capture_ir] = {shift_ir, exit1_ir},
  [shift_ir] = {shift_ir, exit1_ir},
  [exit1_ir] = {pause_ir, update_ir},
  [pause_ir] = {pause_ir, exit2_ir},
  [exit2_ir] = {shift_ir, update_ir},
  [update_ir] = {run_test_idle, select_dr_scan}
};

int sread(void *target, int len) {

  uint8_t *t = (uint8_t *) target;

  while (len) {

    int r = client.read(t, len);
    if (r <= 0)
      return r;
    t += r;
    len -= r;
  }

  return 1;
}

int srcmd(void * target, int maxlen) {
  uint8_t *t = (uint8_t *) target;

  while (maxlen) {
    int r = client.read(t, 1);
    if (r <= 0)
      return r;

    if (*t == ':') {
      return 1;
    }

    t += r;
    maxlen -= r;
  }

  return 0;
}

void ICACHE_RAM_ATTR bit_shift(int len, int nr_bytes) {
  // result[bp] & (1 << j)
  int i, j;
  uint32_t wsbuf, wcbuf;
  uint8_t *pms, *pdi, *pwr;
  uint8_t tmsbuf, tdibuf, resbuf;
  
  pms = buffer;
  pdi = buffer + nr_bytes;
  pwr = result;

  for (i = 0; i < len; ) {
    tmsbuf = *pms, tdibuf = *pdi;
    resbuf = 0;

    for (j = 0; i < len && j < 8; i++, j++) {
      int tms = tmsbuf & 0x01;
      int tdi = tdibuf & 0x01;

      // Before: read, change, rise, fall
      // Modifided: change, fall, read, rise

      wsbuf = 0;
      wcbuf = 1 << GPIO_TCK; // Embed TCK fall here

      if (tms) {
        wsbuf |= (1 << GPIO_TMS);
      } else {
        wcbuf |= (1 << GPIO_TMS);
      }

      if (tdi) {
        wsbuf |= (1 << GPIO_TDI);
      } else {
        wcbuf |= (1 << GPIO_TDI);
      }
      
      // Change + fall
      if (wsbuf) GPOS = wsbuf;
      GPOC = wcbuf;
      // Read
      resbuf |= ((GPI & (1<<GPIO_TDO)) >> GPIO_TDO) << j;
      // Rise
      GPOS = (1<<GPIO_TCK);

      // Track the state.
      jtag_state = next_state[jtag_state][tms];

      tmsbuf >>= 1;
      tdibuf >>= 1;
    }
    *pwr = resbuf;

    pms++, pdi++, pwr++;
  }
}

void setup() {

//  ESP.wdtDisable();
//  ESP.wdtEnable(WDTO_8S);

  delay(1000);

  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println();
#ifdef XVCD_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask);
  WiFi.softAP(ssid, pw);
#else
#ifdef XVCD_STATIC_IP
  WiFi.config(ip, gateway, netmask);
#endif
  WiFi.begin(ssid, pw);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
  
  Serial.print("Starting XVC Server on port ");
  Serial.println(port);

  pinMode(GPIO_TDI, OUTPUT);
  pinMode(GPIO_TDO, INPUT);
  pinMode(GPIO_TCK, OUTPUT);
  pinMode(GPIO_TMS, OUTPUT);

  server.begin();
  server.setNoDelay(true);
}

void loop() {

  start: if (!client.connected()) {
    
    // try to connect to a new client
    client = server.available();
    
  } else {
    
    // read data from the connected client
    if (client.available()) {

      while (client.connected()) {
      
        int seen_tlr = 0;
      
        do {
      
          if (srcmd(cmd, 8) != 1)
            goto start;
      
          if (memcmp(cmd, "getinfo:", 8) == 0) {
#ifdef VERBOSE
            Serial.write("XVC_info\n");
#endif
            client.write("xvcServer_v1.0:");
            client.print(MAX_WRITE_SIZE);
            client.write("\n");
            goto start;
          }

          if (memcmp(cmd, "settck:", 7) == 0) {
#ifdef VERBOSE
            Serial.write("XVC_tck\n");
#endif
            int ntck;
            if (sread(&ntck, 4) != 1) {
              Serial.println("reading tck failed\n");
              goto start;
            }
            // Actually TCK frequency is fixed, but replying a fixed TCK will halt hw_server
            client.write((const uint8_t *)&ntck, 4);
            goto start;
          }

          if (memcmp(cmd, "shift:", 6) != 0) {
            cmd[15] = '\0';
            Serial.print("invalid cmd ");
            Serial.println((char *)cmd);
            goto start;
          }

          int len;
          if (sread(&len, 4) != 1) {

            Serial.println("reading length failed\n");
            goto start;
          }
      
          int nr_bytes = (len + 7) / 8;

#ifdef VERBOSE

            Serial.print("len = ");
            Serial.print(len);
            Serial.print(" nr_bytes = ");
            Serial.println(nr_bytes);
#endif
      
          if (nr_bytes * 2 > sizeof(buffer)) {

            Serial.println("buffer size exceeded");
            goto start;
          }
      
          if (sread(buffer, nr_bytes * 2) != 1) {

            Serial.println("reading data failed\n");
            goto start;
          }
             
          memset((uint8_t *)result, 0, nr_bytes);
      
#ifdef VERBOSE

            Serial.print("#len=");
            Serial.print(nr_bytes);
            Serial.print("#");
      
            for (i = 0; i < nr_bytes * 2; ++i)
              Serial.print(buffer[i], HEX);
      
            Serial.println();
#endif
      
          // Only allow exiting if the state is rti and the IR
          // has the default value (IDCODE) by going through test_logic_reset.
          // As soon as going through capture_dr or capture_ir no exit is
          // allowed as this will change DR/IR.
      
          seen_tlr = (seen_tlr || jtag_state == test_logic_reset) && (jtag_state != capture_dr) && (jtag_state != capture_ir);
      
          //
          // Due to a weird bug(??) xilinx impacts goes through another "capture_ir"/"capture_dr" cycle after
          // reading IR/DR which unfortunately sets IR to the read-out IR value.
          // Just ignore these transactions.
      
          if ((jtag_state == exit1_ir && len == 5 && buffer[0] == 0x17) || (jtag_state == exit1_dr && len == 4 && buffer[0] == 0x0b)) {

#ifdef VERBOSE
              Serial.print("ignoring bogus jtag state movement in jtag_state ");
              Serial.println(jtag_state);
#endif
          } else
            bit_shift(len, nr_bytes);
  
          if(client) {
            
            if(client.write((const uint8_t *)result, nr_bytes) != nr_bytes) {

              Serial.print("write error - nbytes =");
              Serial.println(nr_bytes);
              goto start;
            }
            
          } else {
  
            Serial.println("client error !!");
          }

#ifdef VERBOSE
            Serial.print("jtag state ");
            Serial.println(jtag_state);
#endif
      
        } while (!(seen_tlr && jtag_state == run_test_idle));
      }
    }
  }
}
