#include <libhal-actuator/rc_servo.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{
  // TODO: for pca get_pwm16_channel in profiles
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto servo_settings = resources::rc_servo_settings();
  std::array<hal::actuator::rc_servo16, 4> servos{
    // shoulder_servo
    // elbow_servo
    // pitch_servo
    // roll_servo
  };
  // shoulder_servo
  // elbow_servo
  // pitch_servo
  // roll_servo
  // auto motor settings?
  
  bool mimic_attached = false;
  std::pair servo_output_range = { 0.06,
                                   0.89 };  // The values from 0 to 180 in ADC
  std::pair output_percent = { 0, 180 };
  float tolerance = 2.0f;  // TESTING degrees

  // Set up ADC expander!
  if (!mimic_attached) {
    // sending arm movements by reading adc
    // for adc channel in adcs
    //
  }

  // writing pwm to servos, meaning mimicking arm
  // movements
  std::span<char> mimic_state_buffer{};
  if (mimic_attached) {
    // read serial
    size_t index = 0;
    bool started_reading = false;
    while (index < mimic_state_buffer.size() - 1) {
      std::array<hal::byte, 1> single_byte;
      hal::read(*console, single_byte, hal::never_timeout());

      char c = static_cast<char>(single_byte[0]);
      // Skip leading whitespace
      if (!started_reading && (c == '\r' || c == '\n' || c == ' ')) {
        continue;
      }

      started_reading = true;
      if (c == '\r') {
        continue;  // Skip carriage return
      }
      if (c == '\n') {
        mimic_state_buffer[index] = '\0';
        break;
      }
      mimic_state_buffer[index] = c;
      index++;
    }

    // For right now, separate track
    // TRACK LOGIC HERE

    // separating the buffer into respective portions
    int i = 0;
    for (auto servo : servos) {
      std::span<char> servo_buffer = mimic_state_buffer.subspan(i * 1, i * 4);
      float deg_value = std::strtof(servo_buffer.data(), nullptr);
      hal::degrees angle = deg_value;
      // If degree is invalid
      if (angle > servo_settings.max_angle ||
          angle < servo_settings.min_angle) {
        hal::print<64>(*console,
                       "Invalid angle for %s. Value was %f.\nExpected value "
                       "between %f and %f",
                       servo.c_str(),  // check if this works
                       deg_value,
                       servo_settings.min_angle,
                       servo_settings.max_angle);
        hal::print(*console, "\n\n");
        continue;
      }

      // Set servo to degree value
      servo->position(angle);
      hal::delay(*clock,
                 2000ms);  // blocking? what is the proper way to approach this

      // TESTING use ADC to look at accuracy
      float servo_pos = servo_feedback[i]->read();  // THIS WILL BE DONE
      float servo_percent_pos =
        hal::map(servo_pos, servo_output_range, output_percent);
      hal::print<64>(*console, "ADC: %f\n", servo_pos);
      hal::print<64>(*console, "ADC Mapped: %f\n", servo_percent_pos);
      if (std::abs(servo_percent_pos - deg_value) < tolerance) {
        hal::print(*console, "Servo angle reached!\n");
      }

      i++;
    }
  }
}  // namespace sjsu::mimic
