#![no_std]
#![no_main]

use embassy_futures::join::join;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
//use {defmt_rtt as _}; // global logger

use bt_hci::controller::ExternalController;
use log::{info};

use sht4x_ng::Sht4x;
use esp_hal::{delay::Delay, main, rmt::Rmt, time::RateExtU32};
use esp_wifi::ble::controller::BleConnector;
use esp_hal::{timer::timg::TimerGroup};
use esp_hal::i2c::master;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
pub const L2CAP_MTU: usize = 255;
use trouble_host::prelude::*;
const BTHOME_SVC_UUID: u16 = 0xFCD2; // BTHome service UUID


extern crate alloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    info!("{_info}");
    loop {}
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    // generator version: 0.2.2

    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let i2c = master::I2c::new(
        peripherals.I2C0,
        master::Config::default(),
    ).unwrap()
    .with_scl(peripherals.GPIO4)
    .with_sda(peripherals.GPIO5)
    .into_async();

    let mut delay = embassy_time::Delay;
    let mut sht40: Sht4x<_, embassy_time::Delay> = Sht4x::new(i2c);
    esp_alloc::heap_allocator!(72 * 1024);

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let timer_group1 = TimerGroup::new(peripherals.TIMG1);
    //let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);

    log::info!("Embassy initialized!");
    log::info!("Start wifi init");
    let maybe_init = esp_wifi::init(
        timer_group1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    );
    let init = if let Ok(i) = maybe_init {
        log::info!("wifi config initialized");
        i

    } else {
        panic!("wifi init failed");

    };
    esp_hal_embassy::init(timer_group0.timer0);
    let freq = 32.MHz();
    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

    info!("Start bluetooth init");

    // Bluetooth
    //
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let address: Address = Address::random([0x00, 0x04, 0x1a, 0x05, 0xe4, 0xfe]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, runner, ..
    } = stack.build();
    //spawner.must_spawn(ble_task(runner));


    // LED
    //
    //let led_pin = peripherals.GPIO8;

    //let delay = Delay::new();
    // Buffer for BTHome advertisement data
    let mut adv_data = [0; 31];

    info!("About to start embassy join");
    let _ = join(ble_task(runner), async {
    loop {
        info!("beginning loop iter");
        //let mut data;
        let serial = sht40.serial_number(&mut delay).await.unwrap();
        let measure = sht40.measure(sht4x_ng::Precision::High, &mut delay).await.unwrap();
        info!("Hello world! {}: {:?}", serial, measure);
        let temp = (measure.temperature_milli_celsius() / 10).try_into().unwrap_or(0);
        let hum = (measure.humidity_milli_percent() / 10 ).try_into().unwrap_or(0);

        info!("temp conversion! {:?}: {:?}", temp, hum);
         // BTHome v2 packet
        let payload = [
            0x40, // BTHome v2, no encryption
            0x02, // Temperature
            (temp & 0xFF) as u8,
            ((temp >> 8) & 0xFF) as u8,
            0x03, // Humidity
            (hum & 0xFF) as u8,
            ((hum >> 8) & 0xFF) as u8,
        ];
        AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(BTHOME_SVC_UUID.to_be_bytes())]),
                AdStructure::ServiceData16 {
                    uuid: BTHOME_SVC_UUID,
                    data: &payload,
                },
            ],
            &mut adv_data[..],
        ).unwrap();

        // Stop previous advertisement if any
        info!("Starting advertiser");
        let mut adv_param = trouble_host::advertise::AdvertisementParameters::default();
        adv_param.timeout = Some(Duration::from_secs(1));
        let advertiser = peripheral
            .advertise(
            &adv_param,
            Advertisement::NonconnectableScannableUndirected {
                adv_data: &adv_data[..],
                scan_data: &[],
            },
        )
        .await;
        if let Ok(_) = advertiser {
            info!("[adv] advertised");

        } else {
            info!("[adv] advertise error")

        }


        // for hue in 0..=255 {
        //     color.hue = hue;
        //     // Convert from the HSV color space (where we can easily transition from one
        //     // color to the other) to the RGB color space that we can then send to the LED
        //     data = [hsv2rgb(color)];
        //     // When sending to the LED, we do a gamma correction first (see smart_leds
        //     // documentation for details) and then limit the brightness to 10 out of 255 so
        //     // that the output it's not too bright.
        //     led.write(brightness(gamma(data.iter().cloned()), 10))
        //         .unwrap();
        //     Timer::after(Duration::from_millis(10)).await;
        // };
        Timer::after(Duration::from_secs(5)).await;
    }}).await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            //let e = defmt::Debug2Format(&e);
            info!("[ble_task] error {:?}", e);
        }
    }
}
