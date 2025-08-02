#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
//use {defmt_rtt as _}; // global logger

use bt_hci::controller::ExternalController;
use log::info;

use esp_hal::i2c::master;
use esp_hal::{delay::Delay, main, rmt::Rmt};
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_wifi::{EspWifiController, ble::controller::BleConnector, init};
use sht4x_ng::Sht4x;
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use esp_hal::analog::adc::{AdcCalCurve, Attenuation};
use esp_hal::rtc_cntl::{Rtc, SocResetReason, sleep::TimerWakeupSource};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite, RGB8
};
esp_bootloader_esp_idf::esp_app_desc!();
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
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let i2c = master::I2c::new(peripherals.I2C0, master::Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO7)
        .with_sda(peripherals.GPIO6)
        .into_async();

    let mut delay = embassy_time::Delay;
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut sht40: Sht4x<_, embassy_time::Delay> = Sht4x::new(i2c);
    //esp_alloc::heap_allocator!(73728);

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let timer_group1 = TimerGroup::new(peripherals.TIMG1);
    //let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);

    let mut adc1_config = esp_hal::analog::adc::AdcConfig::new();
    let analog_pin1 = peripherals.GPIO5;
    let mut pin1 = adc1_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(
        analog_pin1,
        esp_hal::analog::adc::Attenuation::_11dB,
    );
    let mut adc1 = esp_hal::analog::adc::Adc::new(peripherals.ADC1, adc1_config);
    let adc1_value: u16 = adc1.read_oneshot(&mut pin1).unwrap_or_else(|_| 0);

    // * 2 for the voltage divider ratio, this should be set somewhere as config or var
    let batt_voltage: u16 = adc1_value * 2;
    log::info!("ADC1 value, battery: {} {}", adc1_value, batt_voltage);

    log::info!("Embassy initialized!");
    log::info!("Start wifi init");
    let maybe_init = esp_wifi::init(timer_group1.timer0, esp_hal::rng::Rng::new(peripherals.RNG));
    let init = if let Ok(i) = maybe_init {
        log::info!("wifi config initialized");
        i
    } else {
        panic!("wifi init failed");
    };
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    //let freq = esp_hal::time
    //let freq =

    info!("Start bluetooth init");

    // Bluetooth
    //
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let address: Address = Address::random([0x00, 0x04, 0x1a, 0x05, 0xe4, 0xfe]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, 0, 0, 27> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();
    //spawner.must_spawn(ble_task(runner));

    // LED
    //
    use esp_hal::time::Rate;
    let led_pin = peripherals.GPIO2;

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt_channel = rmt.channel0;
    let rmt_buffer = smart_led_buffer!(1);

    let mut led = SmartLedsAdapter::new(rmt_channel, led_pin, rmt_buffer);

    //let delay = Delay::new();
    // Buffer for BTHome advertisement data
    let mut adv_data = [0; 31];

    info!("About to start embassy join");
    let _ = join(ble_task(runner), async {
        loop {
            info!("beginning loop iter");
            //let mut data;
            let serial = sht40.serial_number(&mut delay).await.unwrap();
            let measure = sht40
                .measure(sht4x_ng::Precision::High, &mut delay)
                .await
                .unwrap();
            info!("Hello world! {}: {:?}", serial, measure);
            let temp = (measure.temperature_milli_celsius() / 10)
                .try_into()
                .unwrap_or(0);
            let hum = (measure.humidity_milli_percent() / 10)
                .try_into()
                .unwrap_or(0);

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
                0x0C,
                (batt_voltage & 0xFF) as u8,
                ((batt_voltage >> 8) & 0xFF) as u8,
            ];
            AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[BTHOME_SVC_UUID.to_le_bytes()]),
                    AdStructure::ServiceData16 {
                        uuid: BTHOME_SVC_UUID.to_le_bytes(),
                        data: &payload,
                    },
                ],
                &mut adv_data[..],
            )
            .unwrap();

            // Stop previous advertisement if any
            info!("Starting advertiser");
            let mut adv_param = trouble_host::advertise::AdvertisementParameters::default();
            adv_param.tx_power = trouble_host::advertise::TxPower::Plus20dBm;
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
            let blip_light = RGB8{r: 5, g: 0, b: 0};
            led.write([blip_light]).unwrap();
            Timer::after(Duration::from_millis(10)).await;
            let disable_light = RGB8{r: 0, g: 0, b: 0};
            led.write([disable_light]).unwrap();

            let timer = TimerWakeupSource::new(core::time::Duration::from_secs(29));
            rtc.sleep_deep(&[&timer]);
            //Timer::after(Duration::from_secs(5)).await;
        }
    })
    .await;

}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            //let e = defmt::Debug2Format(&e);
            info!("[ble_task] error {:?}", e);
        }
    }
}
