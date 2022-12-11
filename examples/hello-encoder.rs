use std::sync::Mutex;
use std::thread;
use std::time::Duration;

use esp_idf_hal::gpio::{Gpio18, Gpio19, Input, PinDriver, Pull};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer::config::Config;
use esp_idf_hal::timer::TimerDriver;
use esp_idf_sys::{timer_autoreload_t_TIMER_AUTORELOAD_EN, timer_set_auto_reload};
use heapless::spsc::{Producer, Queue};
use rotary_encoder_embedded::{standard::StandardMode, Direction, RotaryEncoder};

struct RotaryEncoderState {
    encoder: RotaryEncoder<
        StandardMode,
        PinDriver<'static, Gpio18, Input>,
        PinDriver<'static, Gpio19, Input>,
    >,
    queue_producer: Producer<'static, Direction, 32>,
}

static ROTARY_ENCODER: Mutex<Option<RotaryEncoderState>> = Mutex::new(None);

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    // Initialize the GPIO pins for the rotary encoder in pull-up input mode
    let peripherals = Peripherals::take().unwrap();
    let mut gpio18 = PinDriver::input(peripherals.pins.gpio18)?;
    gpio18.set_pull(Pull::Up)?;
    let mut gpio19 = PinDriver::input(peripherals.pins.gpio19)?;
    gpio19.set_pull(Pull::Up)?;

    // A pretty hacky way to get a queue where the interrupt can write rotation events,
    // so the main thread can consume them and do something. The interrupt handler
    // can for example not write to stdout.
    let queue = Box::leak(Box::new(Queue::<Direction, 32>::new()));
    let (queue_producer, mut queue_consumer) = queue.split();

    let rotary_encoder = RotaryEncoder::new(gpio18, gpio19).into_standard_mode();
    let rotary_encoder_state = RotaryEncoderState {
        encoder: rotary_encoder,
        queue_producer,
    };
    ROTARY_ENCODER
        .lock()
        .unwrap()
        .replace(rotary_encoder_state);

    // Compute timer divider to fire at 9500 Hz
    // Then have the timer count to 10 before firing an the interrupt,
    // resulting in checking the rotary encoder at 950 Hz
    let apb_freq = unsafe { esp_idf_sys::rtc_clk_apb_freq_get() };
    let timer_divider = apb_freq / 9500;

    let timer_config = Config {
        divider: timer_divider,
        xtal: false,
    };
    let mut timer = TimerDriver::new(peripherals.timer00, &timer_config)?;
    unsafe {
        timer_set_auto_reload(
            timer.group(),
            timer.index(),
            timer_autoreload_t_TIMER_AUTORELOAD_EN,
        )
    };
    timer.set_counter(0)?;
    timer.set_alarm(10)?;
    timer.enable_alarm(true)?;
    unsafe { timer.subscribe(interrupt)? };
    timer.enable_interrupt()?;
    timer.enable(true)?;

    loop {
        while let Some(direction) = queue_consumer.dequeue() {
            println!("{:?}", direction);
        }
        thread::sleep(Duration::from_millis(10));
    }
}

fn interrupt() {
    let mut rotary_encoder_state = ROTARY_ENCODER.lock().unwrap();
    let rotary_encoder_state = rotary_encoder_state.as_mut().unwrap();

    rotary_encoder_state.encoder.update();
    let new_direction = match rotary_encoder_state.encoder.direction() {
        Direction::Clockwise => Direction::Clockwise,
        Direction::Anticlockwise => Direction::Anticlockwise,
        Direction::None => return,
    };
    let _ = rotary_encoder_state.queue_producer.enqueue(new_direction);
}
