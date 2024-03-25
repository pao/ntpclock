#include <Arduino.h>
#include <ETH.h>
#include <HTTPClient.h>
#include <Wire.h>

#include <chrono>
#include <string>
#include <string_view>
#include <tuple>

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "ArduinoJson.h"
#include "TinyGPSPlus.h"
#include "WiFiNTPServer.h"
#include "api_key.h"
#include "sml2"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define SCREEN_ADDRESS \
  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1};

auto to_chrono(TinyGPSDate gpsDate, TinyGPSTime gpsTime) {
  using namespace std::chrono;
  return sys_days{year{gpsDate.year()} / month{gpsDate.month()} /
                  day{gpsDate.day()}} +
         hours{gpsTime.hour()} + minutes{gpsTime.minute()} +
         seconds{gpsTime.second()};
}

void to_tm(struct tm* ref_time,
           std::chrono::sys_time<std::chrono::milliseconds> utc_time) {
  const auto tt = std::chrono::system_clock::to_time_t(utc_time);
  *ref_time = *std::gmtime(&tt);
}

auto format_time(std::chrono::sys_seconds time) {
  using namespace std::chrono;
  auto date = year_month_day{floor<days>(time)};
  return std::tuple{date, hh_mm_ss{time - sys_days{date}}};
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
volatile SemaphoreHandle_t ref_time_mutex{};
TinyGPSPlus gnss{};
struct got_fix {};
struct pps_pulse {};
struct got_tzoffset {};
struct failed_tzoffset {};

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
TaskHandle_t ntp_responder{};

struct tz_params_t {
  std::chrono::seconds offset{0};
  std::chrono::sys_seconds valid_until_utc{};
  bool is_dst{0};
  bool has_dst{0};
};

tz_params_t tz_params{};

std::chrono::sys_seconds current_time_utc{};

// forward declaration
auto tz_api_query(void*) -> void;
auto serve_ntp(void*) -> void;

auto determine_tz_from_position = [] {
  if (ntp_responder) {
    vTaskDelete(ntp_responder);
    ntp_responder = nullptr;
  }
  xTaskCreatePinnedToCore(tz_api_query, "api_query", 10000, nullptr, 0,
                          &http_stuff, 0);
};

auto serve_ntp_clients = [] {
  xTaskCreate(serve_ntp, "serve ntp", 10000, nullptr, 0, &ntp_responder);
};

auto is_connection_ready = [] {
  using namespace sml::dsl;
  return connection.is("ready"_s);
};

auto tz_not_valid = [] {
  return current_time_utc >= tz_params.valid_until_utc;
};

// immediately change DST until we can query the API again to be sure
auto toggle_dst = [] {
  if (!tz_params.has_dst) {
    return;
  }
  using namespace std::chrono_literals;
  tz_params.offset += tz_params.is_dst ? -3600s : 3600s;
  tz_params.is_dst = !tz_params.is_dst;
};

sml::sm timesync = [] {
  using namespace sml::dsl;
  return transition_table{
      // clang-format off
    *"init"_s + event<got_fix> / debug_fixed = "no_tz"_s,
     "no_tz"_s + event<pps_pulse>[is_connection_ready] / []{pps(); determine_tz_from_position();} = "updating"_s,
     "no_tz"_s + event<pps_pulse> / pps,
     "updating"_s + event<pps_pulse> / pps,
     "updating"_s + event<got_tzoffset> / serve_ntp_clients = "have_tz"_s,
     "updating"_s + event<failed_tzoffset> = "no_tz"_s,
     "have_tz"_s + event<pps_pulse>[tz_not_valid] / []{toggle_dst(); pps();} = "no_tz"_s,
     "have_tz"_s + event<pps_pulse> / pps,
      // clang-format on
  };
};

long tz_query_backoff{0};
static constexpr long tz_query_backoff_increment{1000};
static constexpr long tz_query_backoff_max{300 * 1000};

auto tz_api_query(void*) -> void {
  delay(tz_query_backoff);
  tz_query_backoff =
      std::min((tz_query_backoff + tz_query_backoff_increment) * 2,
               tz_query_backoff_max);
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
    timesync.process_event(failed_tzoffset{});
    vTaskDelete(http_stuff);
    return;
  }
  JsonDocument doc{};

  // Parse JSON object
  DeserializationError error = deserializeJson(doc, http.getString());
  http.end();
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    timesync.process_event(failed_tzoffset{});
    vTaskDelete(http_stuff);
    return;
  }

  tz_params.offset = std::chrono::seconds{doc["gmtOffset"].as<long>()};
  tz_params.has_dst = (doc["dst"].as<const char*>() != nullptr);
  tz_params.is_dst = (doc["dst"].as<std::string>() == "1");
  tz_params.valid_until_utc = std::chrono::sys_seconds{
      std::chrono::seconds{doc["zoneEnd"] | std::numeric_limits<long>::max()}};
  Serial.printf("offset: %ld, dst?: %d, dst: %d\n", doc["gmtOffset"].as<long>(),
                tz_params.has_dst, tz_params.is_dst);

  tz_query_backoff = 0;
  timesync.process_event(got_tzoffset{});

  vTaskDelete(http_stuff);
};

auto process_nmea = [] {
  while (Serial2.available()) {
    gnss.encode(Serial2.read());
  }
  if (gnss.sentencesWithFix() > 0 && gnss.time.isValid() &&
      gnss.location.isValid()) {
    timesync.process_event(got_fix{});
  }
};

WiFiNTPServer ntp_server("GPS", L_NTP_STRAT_PRIMARY);
unsigned long micros_at_last_pps{};
tm ref_time_at_last_pps{};
bool in_between_updates{true};

auto serve_ntp(void*) -> void {
  while (true) {
    if (!in_between_updates && xSemaphoreTake(ref_time_mutex, 0) == pdTRUE) {
      ntp_server.setReferenceTime(ref_time_at_last_pps, micros_at_last_pps);
      xSemaphoreGive(ref_time_mutex);
    }
    ntp_server.update();
  }
}

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

  ref_time_mutex = xSemaphoreCreateMutex();
  ntp_server.setServerPrecision(0.0001);
  ntp_server.begin();

  // PPS setup
  constexpr auto pps_pin = IO36;
  ppsSemaphore = xSemaphoreCreateBinary();
  pinMode(pps_pin, INPUT_PULLDOWN);
  attachInterrupt(
      digitalPinToInterrupt(pps_pin),
      [] {
        if (xSemaphoreTake(ref_time_mutex, 0) == pdTRUE) {
          micros_at_last_pps = esp_timer_get_time();
          in_between_updates = true;
          xSemaphoreGive(ref_time_mutex);
        }
        timesync.process_event(pps_pulse{});
      },
      RISING);
}

auto update_display = [] {
  display.clearDisplay();
  using namespace std::chrono;
  using namespace std::chrono_literals;
  current_time_utc = to_chrono(gnss.date, gnss.time) + 1s;
  if (xSemaphoreTake(ref_time_mutex, 0) == pdTRUE) {
    to_tm(&ref_time_at_last_pps, current_time_utc);
    in_between_updates = false;
    xSemaphoreGive(ref_time_mutex);
  }
  auto [display_date, display_time] =
      format_time(current_time_utc + tz_params.offset);
  display.setCursor(0, 0);
  display.printf("%04d-%02d-%02d\n%02lld:%02lld:%02lld",
                 int{display_date.year()}, unsigned{display_date.month()},
                 unsigned{display_date.day()}, display_time.hours().count(),
                 display_time.minutes().count(),
                 display_time.seconds().count());
  display.display();
};

void loop() {
  // this will turn into a dispatch loop on an event queue driven by the
  // various state machines
  if (xSemaphoreTake(ppsSemaphore, 0) == pdTRUE) {
    update_display();
  }
}