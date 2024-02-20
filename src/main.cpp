#include <Arduino.h>
#include <ETH.h>
#include <HTTPClient.h>
#include <Wire.h>

#include <chrono>
#include <string>
#include <string_view>

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "TinyGPSPlus.h"
#include "api_key.h"
#include "sml2"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define SCREEN_ADDRESS \
  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1};

template <typename Duration>
auto increment_time(TinyGPSTime gpsTime, Duration increment) {
  using namespace std::chrono;
  return hh_mm_ss{hours{gpsTime.hour()} + minutes{gpsTime.minute()} +
                  seconds{gpsTime.second()} + increment};
}

template <typename Duration>
auto increment_date(TinyGPSDate gpsDate, Duration increment) {
  using namespace std::chrono;
  return year_month_day{sys_days{year{gpsDate.year()} / month{gpsDate.month()} /
                                 day{gpsDate.day()}} +
                        increment};
}

struct network_event {
  WiFiEvent_t ev{};
};

auto start_network = [] {
  Serial.println("ETH Started");
  ETH.setHostname("h2");
};

auto connect_network = [] { Serial.println("ETH Connected"); };
auto disconnect_network = [] { Serial.println("ETH Disconnected"); };

auto got_ip = [] {
  Serial.print("ETH MAC: ");
  Serial.print(ETH.macAddress());
  Serial.print(", IPv4: ");
  Serial.print(ETH.localIP());
  if (ETH.fullDuplex()) {
    Serial.print(", FULL_DUPLEX");
  }
  Serial.print(", ");
  Serial.print(ETH.linkSpeed());
  Serial.print("Mbps");
  Serial.print(", DNS: ");
  Serial.println(ETH.dnsIP());
};

template <WiFiEvent_t ev>
struct when {
  constexpr bool operator()(const network_event& net) const {
    return net.ev == ev;
  }
};

sml::sm connection = [] {
  using namespace sml::dsl;
  return transition_table{
      // clang-format off
    *"uninitialized"_s + event<network_event>[when<ARDUINO_EVENT_ETH_START>{}] / start_network = "disconnected"_s,
     "disconnected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_CONNECTED>{}] / connect_network = "connected"_s,
     "connected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_DISCONNECTED>{}] / disconnect_network = "disconnected"_s,
     "connected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_GOT_IP>{}] / got_ip = "ready"_s,
     "ready"_s + event<network_event>[when<ARDUINO_EVENT_ETH_DISCONNECTED>{}] / disconnect_network = "disconnected"_s,
      // clang-format on
  };
};

volatile SemaphoreHandle_t ppsSemaphore{};
TinyGPSPlus gnss{};
struct have_fix {};
struct pps_pulse {};

auto debug_fixed = [] { Serial.print("fixed!\n"); };
auto debug_updating = [] { Serial.print("updating!\n"); };

auto pps = [] { xSemaphoreGiveFromISR(ppsSemaphore, nullptr); };

constexpr void add_query_parameter(std::string& uri, std::string_view key,
                                   std::string_view value) {
  uri.reserve(uri.size() + key.size() + value.size() + 2);
  uri.append({'&'});
  uri.append(key.cbegin(), key.cend());
  uri.append({'='});
  uri.append(value.cbegin(), value.cend());
}

void add_query_parameter(std::string& uri, std::string_view key, double value) {
  add_query_parameter(uri, key, std::to_string(value));
}

TaskHandle_t http_stuff{};

auto tz_api_query = [] (void*) {
  std::string api = "http://api.timezonedb.com/v2.1/get-time-zone?";
  add_query_parameter(api, "key", api_key);
  add_query_parameter(api, "format", "json");
  add_query_parameter(api, "by", "position");
  add_query_parameter(api, "lat", gnss.location.lat());
  add_query_parameter(api, "lng", gnss.location.lng());
  HTTPClient http{};
  Serial.println(api.c_str());
  http.begin(api.c_str());
  auto httpResponseCode = http.GET();
  if (httpResponseCode < 0) {
    Serial.println("Failed to get tzapi data :(");
    vTaskDelete(http_stuff);
    return;
  }
  Serial.println(http.getString());
  http.end();
  vTaskDelete(http_stuff);
};

auto determine_tz_from_position = [] {
    xTaskCreatePinnedToCore(tz_api_query, "api_query", 10000, nullptr, 0, &http_stuff, 0);
};

auto is_connection_ready = [] {
  using namespace sml::dsl;
  return connection.is("ready"_s);
};

sml::sm timesync = [] {
  using namespace sml::dsl;
  return transition_table{
      // clang-format off
    *"init"_s + event<have_fix> / debug_fixed = "fixed"_s,
     "fixed"_s + event<pps_pulse>[is_connection_ready] / determine_tz_from_position = "updating"_s,
     "updating"_s + event<pps_pulse> / pps,
      // clang-format on
  };
};

auto process_nmea = [] {
  while (Serial2.available()) {
    gnss.encode(Serial2.read());
  }
  if (gnss.sentencesWithFix() > 0 && gnss.time.isValid() &&
      gnss.location.isValid()) {
    timesync.process_event(have_fix{});
  }
};

void setup() {
  // Debug console setup
  Serial.begin(9600);

  // OLED setup
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {
    }  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("Waiting for fix...");
  display.display();
  display.setTextSize(2);

  // GNSS module setup
  Serial2.begin(9600, SERIAL_8N1, IO14, IO15);
  Serial2.print("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  Serial2.print("$PMTK255,1*2D\r\n");
  Serial2.flush();
  Serial2.onReceive(process_nmea, false);

  // ETH setup
  WiFi.onEvent(
      [](WiFiEvent_t ev) { connection.process_event(network_event{ev}); });
  ETH.begin();

  // PPS setup
  constexpr auto pps_pin = IO36;
  ppsSemaphore = xSemaphoreCreateBinary();
  pinMode(pps_pin, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(pps_pin),
      [] { timesync.process_event(pps_pulse{}); }, FALLING);
}

auto update_display = [] {
  display.clearDisplay();
  using namespace std::chrono;
  using namespace std::chrono_literals;
  display.setCursor(0, 0);
  auto display_time = increment_time(gnss.time, 1s);
  auto display_date = increment_date(
      gnss.date,
      (gnss.time.hour() > display_time.hours().count()) ? days{1} : days{0});
  display.printf("%04d-%02d-%02d\n%02lld:%02lld:%02lld",
                 int{display_date.year()}, unsigned{display_date.month()},
                 unsigned{display_date.day()}, display_time.hours().count(),
                 display_time.minutes().count(),
                 display_time.seconds().count());
  display.display();
};

void loop() {
  // this will turn into a dispatch loop on an event queue driven by the various
  // state machines
  if (xSemaphoreTake(ppsSemaphore, 0) == pdTRUE) {
    update_display();
  }
}