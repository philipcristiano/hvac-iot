#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use log::info;


use sht4x_ng::Sht4x;
use esp_hal::{delay::Delay, main, rmt::Rmt, time::RateExtU32};
use esp_hal::i2c::master;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut i2c = master::I2c::new(
        peripherals.I2C0,
        master::Config::default(),
    ).unwrap()
    .with_sda(peripherals.GPIO25)
    .with_scl(peripherals.GPIO11)
    .into_async();

    let mut delay = embassy_time::Delay;
    let mut sht40: Sht4x<_, embassy_time::Delay> = Sht4x::new(i2c);
    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let freq = 32.MHz();
    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();
    // TODO: Spawn some tasks
    let _ = spawner;
    let led_pin = peripherals.GPIO8;
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;
    //let delay = Delay::new();
    loop {
        let serial = sht40.serial_number(&mut delay).await.unwrap();
        let measure = sht40.measure(sht4x_ng::Precision::Low, &mut delay).await.unwrap();
        info!("Hello world! {serial}: {measure:?}");

        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = [hsv2rgb(color)];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();
        };
        Timer::after(Duration::from_secs(1)).await;
        //Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}
