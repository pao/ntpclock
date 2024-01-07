#include <Arduino.h>
#include <string>

#include "sml2"

struct toggle{};
struct button_pressed{};

std::string button_state{"unpressed"};

sml::sm blinker = []
{
  auto switch_off = []
  { Serial.println(("off. button is " + button_state).c_str()); };
  auto switch_on = []
  { Serial.println(("ON! button is " + button_state).c_str()); };

  auto set_button = [] { button_state = "pressed"; };
  auto reset_button = [] { button_state = "reset"; };

  using namespace sml::dsl;
  return transition_table{
      *"on"_s + event<toggle> / switch_off = "off"_s,
      "off"_s + event<toggle> / switch_on = "on"_s,

      *"unpressed"_s + event<button_pressed> / set_button = "pressed"_s,
      "pressed"_s + event<button_pressed> / reset_button = "unpressed"_s,
  };
};

auto interruptPin = 4;

void button() {
  blinker.process_event(button_pressed{});
}

void setup()
{
  Serial.begin(9600);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), button, FALLING);
}

void loop()
{
  //Serial.println("Hello world!");
  blinker.process_event(toggle{});
  delay(1000);
}