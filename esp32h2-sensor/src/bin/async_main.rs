#![no_std]
#![no_main]

use embassy_futures::join::join;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use log::info;

use bt_hci::controller::ExternalController;

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

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
}
/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

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
    esp_alloc::heap_allocator!(48 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");
    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    info!("Start wifi init");
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    ).unwrap();
    info!("wifi config initialized");
    let freq = 32.MHz();
    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();
    // TODO: Spawn some tasks
    let _ = spawner;

    info!("Start bluetooth init");

    // Bluetooth
    //
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, mut runner, ..
    } = stack.build();
    //spawner.must_spawn(ble_task(runner));

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let name = "Trouble Example";
    let mut advertiser_data = [0; 31];

    // LED
    //
    let led_pin = peripherals.GPIO8;
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    //let delay = Delay::new();
    // Buffer for BTHome advertisement data
    let mut adv_data = [0; 31];

    info!("About to start embassy join");
    let _ = join(ble_task(runner), async {
    loop {
        info!("beginning loop iter");
        let mut data;
        let serial = sht40.serial_number(&mut delay).await.unwrap();
        let measure = sht40.measure(sht4x_ng::Precision::High, &mut delay).await.unwrap();
        info!("Hello world! {serial}: {measure:?}");
        let temp = (measure.temperature_celsius().to_bits() * 100 >> 20) as i16;
        let hum = (measure.humidity_percent().to_bits() * 100 >> 20) as u16;

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
        Timer::after(Duration::from_secs(15)).await;
    }}).await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}
