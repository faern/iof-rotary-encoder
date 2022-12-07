use std::cell::RefCell;
use std::thread;
use std::time::Duration;

use esp_idf_hal::gpio::{Gpio18, Gpio19, InterruptType, Pull, SubscribedInput};
use esp_idf_hal::interrupt::Mutex;
use esp_idf_hal::peripherals::Peripherals;
use heapless::spsc::{Producer, Queue};
use rotary_encoder_embedded::{standard::StandardMode, Direction, RotaryEncoder};

struct RotaryEncoderState {
    encoder: RotaryEncoder<StandardMode, Gpio18<SubscribedInput>, Gpio19<SubscribedInput>>,
    last_direction: Direction,
    queue_producer: Producer<'static, Direction, 32>,
}

static ROTARY_ENCODER: Mutex<RefCell<Option<RotaryEncoderState>>> = Mutex::new(RefCell::new(None));

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    // Initialize the GPIO pins for the rotary encoder and configure them to trigger
    // the interrupt on any edge.
    let peripherals = Peripherals::take().unwrap();
    let mut gpio18 = peripherals.pins.gpio18.into_input()?;
    let mut gpio19 = peripherals.pins.gpio19.into_input()?;
    gpio18.set_pull_up()?;
    gpio19.set_pull_up()?;
    let gpio18 = unsafe { gpio18.into_subscribed(interrupt, InterruptType::AnyEdge) }?;
    let gpio19 = unsafe { gpio19.into_subscribed(interrupt, InterruptType::AnyEdge) }?;

    // A pretty hacky way to get a queue where the interrupt can write rotation events,
    // so the main thread can consume them and do something. The interrupt handler
    // can for example not write to stdout.
    let queue = Box::leak(Box::new(Queue::<Direction, 32>::new()));
    let (queue_producer, mut queue_consumer) = queue.split();

    let rotary_encoder = RotaryEncoder::new(gpio18, gpio19).into_standard_mode();
    let rotary_encoder_state = RotaryEncoderState {
        encoder: rotary_encoder,
        last_direction: Direction::None,
        queue_producer,
    };
    ROTARY_ENCODER
        .lock()
        .borrow_mut()
        .replace(rotary_encoder_state);

    loop {
        while let Some(direction) = queue_consumer.dequeue() {
            println!("{:?}", direction);
        }
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(10));
    }
}

fn interrupt() {
    let rotary_encoder_state = ROTARY_ENCODER.lock();
    let mut rotary_encoder_state = rotary_encoder_state.borrow_mut();
    // Will panic if the interrupt happens before the state is inserted in main. But I don't care for this example.
    let rotary_encoder_state = rotary_encoder_state.as_mut().unwrap();

    rotary_encoder_state.encoder.update();
    let new_direction = match rotary_encoder_state.encoder.direction() {
        Direction::Clockwise => Direction::Clockwise,
        Direction::Anticlockwise => Direction::Anticlockwise,
        Direction::None => return,
    };
    // Only issue the rotation event if we moved twice in the same direction.
    // The PEC11R encoders I use have two rotation events per dent from this library.
    if new_direction == rotary_encoder_state.last_direction {
        rotary_encoder_state.last_direction = Direction::None;
        let _ = rotary_encoder_state.queue_producer.enqueue(new_direction);
    } else {
        rotary_encoder_state.last_direction = new_direction;
    }
}
