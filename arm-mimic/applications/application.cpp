#include <libhal-actuator/rc_servo.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-expander/pca9685.hpp>
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

  // Set up PCA9685
  auto i2c = resources::i2c();
  auto servo_settings = resources::servo_settings();
  hal::expander::pca9685 pca9685(*i2c, 0b100'0000);
  auto pwm0 = pca9685.get_pwm_channel<0>();
  auto pwm1 = pca9685.get_pwm_channel<1>();
  auto pwm2 = pca9685.get_pwm_channel<2>();
  auto pwm3 = pca9685.get_pwm_channel<3>();
  auto pwm4 = pca9685.get_pwm_channel<4>();

  std::array<hal::actuator::rc_servo, 5> servos{
    { hal::actuator::rc_servo(pwm0, servo_settings),
      hal::actuator::rc_servo(pwm1, servo_settings),
      hal::actuator::rc_servo(pwm2, servo_settings),
      hal::actuator::rc_servo(pwm3, servo_settings),
      hal::actuator::rc_servo(pwm4, servo_settings) }
  };

  // Set up ADC1283

  // hal::actuator::rc_servo shoulder_servo(pwm0, servo_settings);
  // hal::actuator::rc_servo elbow_servo(pwm1, servo_settings);
  // hal::actuator::rc_servo pitch_servo(pwm2, servo_settings);
  // hal::actuator::rc_servo roll_servo(pwm3, servo_settings);
  // hal::actuator::rc_servo clamp_servo(pwm4, servo_settings);

  // std::array<hal::actuator::rc_servo, 5> servos{
  //   shoulder_servo, elbow_servo, pitch_servo, roll_servo, clamp_servo
  // };
  // shoulder_servo
  // elbow_servo
  // pitch_servo
  // roll_servo
  // auto motor settings?

  bool controller_mode = false;
  std::pair servo_output_range = { 0.183,
                                   0.711 };  // The values from 0 to 180 in ADC
  std::pair output_percent = { 0, 180 };
  float tolerance = 2.0f;  // TESTING degrees

  // Set up ADC expander!
  if (controller_mode) {
    // sending arm movements by reading adc
    // for adc channel in adcs
    //
  }

  // writing pwm to servos, meaning mimicking arm
  // movements
  std::span<char> mimic_state_buffer{};
  if (!controller_mode) {
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
      // std::span<char> servo_buffer = mimic_state_buffer.subspan(i * 1, i *
      // 4);
      std::span<char> servo_buffer = mimic_state_buffer.subspan(i * 1, i * 1);
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
      servo.position(angle);
      hal::delay(*clock,
                 2000ms);  // blocking? what is the proper way to approach this

      // TESTING use ADC to look at accuracy
      float servo_pos = servo_feedback[i].read();  // THIS WILL BE DONE
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
