#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::collections::vec_deque::VecDeque;
use alloc::string::ToString;
use alloc::vec::Vec;
use core::cmp::min;
use core::ops::Sub;
use core::{fmt::Display, iter::Cycle, slice::Iter};
use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Pull},
    main,
    rmt::Rmt,
    time::{Duration, Instant, Rate},
};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use hd44780_driver::bus::I2CBus;
use hd44780_driver::charset::Fallback;
use hd44780_driver::memory_map::StandardMemoryMap;
use hd44780_driver::{
    Cursor, CursorBlink, Display as LcdDisplay, DisplayMode, HD44780, charset::CharsetA00,
    memory_map::MemoryMap1602, setup::DisplayOptionsI2C,
};
use smart_leds::{Brightness, Gamma, RGB8, SmartLedsWrite, brightness, colors, gamma};

use log::info;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("PANIC: {info}");

    // просто висим
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Clone, Copy)]
enum Event {
    Tick(Duration),
    ButtonPressed { in_time: Duration },
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Green,
    Yellow,
    Red,
}

impl From<State> for LedFrame {
    fn from(val: State) -> Self {
        brightness(gamma([RGB8::from(val)].into_iter()), 255)
    }
}

impl Display for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            State::Green => f.write_str("GREEN"),
            State::Yellow => f.write_str("YELLOW"),
            State::Red => f.write_str("RED"),
        }
    }
}

impl From<State> for RGB8 {
    fn from(val: State) -> Self {
        match val {
            State::Green => colors::GREEN,
            State::Yellow => colors::YELLOW,
            State::Red => colors::RED,
        }
    }
}

enum Command {
    ChangeFrame(LedFrame),
    NewLcdText(LcdContent),
}

struct LcdContent {
    state: State,
    timer: Duration,
}

struct TrafficLight {
    remaning: Duration,
    cycle: Cycle<Iter<'static, (State, Duration)>>,
    lead_time: Duration,
    state: State,
}

type LedFrame = Brightness<Gamma<core::array::IntoIter<RGB8, 1>>>;
type Lcd<'a> = HD44780<
    I2CBus<esp_hal::i2c::master::I2c<'a, Blocking>>,
    StandardMemoryMap<16, 2>,
    Fallback<CharsetA00, 63>,
>;
impl TrafficLight {
    fn new(colors: &'static [(State, Duration)], lead_time: Duration) -> Self {
        Self {
            state: State::Green,
            remaning: Duration::ZERO,
            cycle: colors.iter().cycle(),
            lead_time,
        }
    }

    pub fn handle(&mut self, event: Event) -> Vec<Command> {
        match event {
            Event::Tick(now) if now >= self.remaning => {
                let (c, d) = self.cycle.next().expect("cycle is ended!");
                self.remaning = now + *d;
                self.state = *c;
                info!("{:<7} - next at {}", c, self.remaning.as_millis());
                let mut v = Vec::with_capacity(2);
                // Это лучше, чем когда clippy ругается на large_stack_frames
                #[allow(clippy::vec_init_then_push)]
                {
                    v.push(Command::ChangeFrame(LedFrame::from(*c)));
                    v.push(Command::NewLcdText(LcdContent {
                        state: self.state,
                        timer: self.remaning.sub(now),
                    }));
                }
                v
            }
            Event::Tick(now) => {
                if now.as_millis().is_multiple_of(1000) {
                    return Vec::from([Command::NewLcdText(LcdContent {
                        state: self.state,
                        timer: self.remaning.sub(now),
                    })]);
                }
                Vec::default()
            }
            Event::ButtonPressed { in_time } => {
                info!("Button pressed at {}", in_time.as_millis());
                match self.state {
                    State::Green => {
                        info!("GREEN will be disable at {}", self.remaning.as_millis());
                        self.remaning = core::cmp::min(self.remaning, in_time + self.lead_time);
                    }
                    State::Yellow | State::Red => info!("ignored"),
                }
                Vec::default()
            }
        }
    }
}
const RULES: &[(State, Duration)] = &[
    (State::Green, Duration::from_secs(10)),
    (State::Yellow, Duration::from_secs(2)),
    (State::Red, Duration::from_secs(10)),
];
const LEAD_TIME: Duration = Duration::from_secs(2);

const DEBOUNCE: Duration = Duration::from_millis(200);

fn lcd_update_full(lcd: &mut Lcd, delay: &mut Delay, line1: &str, line2: &str) {
    lcd.clear(delay).ok();

    // первая строка
    lcd.set_cursor_xy((0, 0), delay).ok();
    lcd.write_str(&line1[..min(16, line1.len())], delay).ok();

    // вторая строка
    lcd.set_cursor_xy((0, 1), delay).ok();
    lcd.write_str(&line2[..min(16, line2.len())], delay).ok();
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    info!("LedFrame size: {}", core::mem::size_of::<LedFrame>());
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    info!("Start up");
    let rmt = Rmt::new(p.RMT, Rate::from_mhz(80)).expect("init rmt");

    let mut rmt_buffer = smart_led_buffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, p.GPIO8, &mut rmt_buffer);

    let lead_button = Input::new(p.GPIO0, InputConfig::default().with_pull(Pull::Up));

    let i2c = I2c::new(p.I2C0, esp_hal::i2c::master::Config::default())
        .expect("can init i2c")
        .with_sda(p.GPIO6)
        .with_scl(p.GPIO7);
    let mut delay = Delay::new();
    let mut options = DisplayOptionsI2C::new(MemoryMap1602::new())
        .with_i2c_bus(i2c, 0x27)
        .with_charset(CharsetA00::QUESTION_FALLBACK);

    let mut display = loop {
        match HD44780::new(options, &mut delay) {
            Err((options_back, error)) => {
                info!("Error creating LCD Driver: {error}");
                options = options_back;
                delay.delay_millis(500);
                // try again
            }
            Ok(display) => break display,
        }
    };

    // Disable cursor
    display
        .set_display_mode(
            DisplayMode {
                display: LcdDisplay::On,
                cursor_visibility: Cursor::Visible,
                cursor_blink: CursorBlink::On,
            },
            &mut delay,
        )
        .unwrap();

    info!("init display");
    display.clear(&mut delay).unwrap();
    display.reset(&mut delay).unwrap();

    display
        .write_str("Traffic Light started", &mut delay)
        .unwrap();

    let mut tl = TrafficLight::new(RULES, LEAD_TIME);

    let start = Instant::now();
    let mut last_press_gpio0 = Duration::ZERO;
    info!("start loop");
    let mut events = VecDeque::with_capacity(10);

    loop {
        let current_loop = start.elapsed();

        if current_loop.as_millis().is_multiple_of(1000) {
            info!("elapsed {}", current_loop.as_millis(),);
        };
        if current_loop.as_millis().is_multiple_of(10) {
            events.push_back(Event::Tick(current_loop));
        };
        if lead_button.is_high() && current_loop - last_press_gpio0 >= DEBOUNCE {
            last_press_gpio0 = current_loop;
            events.push_back(Event::ButtonPressed {
                in_time: current_loop,
            });
        }
        while let Some(e) = events.pop_front() {
            for c in tl.handle(e) {
                match c {
                    Command::ChangeFrame(f) => led.write(f).ok(),
                    Command::NewLcdText(LcdContent { state, timer }) => {
                        lcd_update_full(
                            &mut display,
                            &mut delay,
                            state.to_string().as_str(),
                            timer.as_secs().to_string().as_str(),
                        );
                        None
                    }
                };
            }
        }
    }
}
