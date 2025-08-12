#![no_std]
#![no_main]

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    info!("{_info}");
    loop {}
}

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
//use {defmt_rtt as _}; // global logger

use bt_hci::controller::ExternalController;
use log::info;

use esp_hal::analog::adc::AdcCalCurve;
use esp_hal::i2c::master;
use esp_hal::rmt::Rmt;
use esp_hal::rtc_cntl::{Rtc, sleep::TimerWakeupSource};
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use esp_wifi::ble::controller::BleConnector;
use sht4x_ng::Sht4x;
use smart_leds::{RGB8, SmartLedsWrite};
esp_bootloader_esp_idf::esp_app_desc!();
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
pub const L2CAP_MTU: usize = 1017;
use trouble_host::prelude::*;
const BTHOME_SVC_UUID: u16 = 0xFCD2; // BTHome service UUID

extern crate alloc;

use scd4x::Scd4xAsync;

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

    use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::mutex::Mutex;
    let i2c_bus: embassy_sync::mutex::Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'_, _>> =
        Mutex::new(i2c);

    let mut delay = embassy_time::Delay;
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut scd = Scd4xAsync::new(I2cDevice::new(&i2c_bus), delay.clone());
    let mut sht40: Sht4x<_, embassy_time::Delay> = Sht4x::new(I2cDevice::new(&i2c_bus));
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

    let power_down_result = scd.power_down().await;
    log::info!("Power down restul {:?}", power_down_result);

    // Bluetooth
    //
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let mac = esp_hal::efuse::Efuse::mac_address();
    let address: Address = Address::random(mac);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, 0, 0, 27> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();
    //spawner.must_spawn(ble_task(runner));
    //
    // Calibrate Button Setup
    let calibrate_button_config = esp_hal::gpio::InputConfig::default();
    let calibrate_button = esp_hal::gpio::Input::new(peripherals.GPIO18, calibrate_button_config);

    // LED
    //
    //
    let red_light = RGB8 { r: 5, g: 0, b: 0 };
    let yellow_light = RGB8 { r: 5, g: 5, b: 0 };
    let green_light = RGB8 { r: 0, g: 5, b: 0 };
    let disable_light = RGB8 { r: 0, g: 0, b: 0 };
    use esp_hal::time::Rate;
    let led_pin = peripherals.GPIO2;

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt_channel = rmt.channel0;
    let rmt_buffer = smart_led_buffer!(2);

    let mut leds = SmartLedsAdapter::new(rmt_channel, led_pin, rmt_buffer);

    // Button is pulled high by external resistor, button to GND
    if calibrate_button.is_high() {
        log::info!("Calibrate button not pressed");
    } else {
        log::info!("Calibrate button pressed, calibration should begin!");
        for _ in 0..3 {
            leds.write([red_light, red_light]).unwrap();
            scd.measure_single_shot_non_blocking().await;

            let co2_timer = TimerWakeupSource::new(core::time::Duration::from_secs(6));
            rtc.sleep_light(&[&co2_timer]);
            leds.write([red_light, yellow_light]).unwrap();
            let scd_measurement_opt = scd.measurement().await;
            let co2_timer = TimerWakeupSource::new(core::time::Duration::from_secs(55));
            rtc.sleep_light(&[&co2_timer]);
            leds.write([red_light, red_light]).unwrap();
        }
        leds.write([yellow_light, yellow_light]).unwrap();
        scd.forced_recalibration(420).await;
        let co2_timer = TimerWakeupSource::new(core::time::Duration::from_secs(1));
        rtc.sleep_light(&[&co2_timer]);
        scd.set_automatic_self_calibration(false).await;
        let co2_timer = TimerWakeupSource::new(core::time::Duration::from_secs(1));
        rtc.sleep_light(&[&co2_timer]);
        leds.write([green_light, green_light]).unwrap();
    }

    //let delay = Delay::new();
    // Buffer for BTHome advertisement data
    let mut adv_data = [0; 31];

    info!("About to start embassy join");
    let _ = join(ble_task(runner), async {
        loop {
            let mut co2: u16 = 0;
            info!("beginning loop iter");
            //let power_down_result = scd.power_down().await;
            //log::info!("Power down result {:?}", power_down_result);
            scd.wake_up().await;
            Timer::after(Duration::from_millis(30)).await;
            match scd.measure_single_shot_non_blocking().await {
                Ok(_) => {
                    log::info!("Measuring single shot");
                    //Timer::after(Duration::from_secs(6)).await;
                    let co2_timer = TimerWakeupSource::new(core::time::Duration::from_secs(6));
                    rtc.sleep_light(&[&co2_timer]);
                    let scd_measurement_opt = scd.measurement().await;
                    if let Ok(scd_measuremenet) = scd_measurement_opt {
                        log::info!("Measure {}", scd_measuremenet.co2);
                        co2 = scd_measuremenet.co2;
                    }
                }

                Err(e) => log::error!("Error with CO2 single shot {e:?}"),
            }
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
                0x0C, // Voltage
                (batt_voltage & 0xFF) as u8,
                ((batt_voltage >> 8) & 0xFF) as u8,
                0x12, // CO2
                (co2 & 0xFF) as u8,
                ((co2 >> 8) & 0xFF) as u8,
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
            //adv_param.tx_power = trouble_host::advertise::TxPower::Plus7dBm;
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
            let blip_light = RGB8 { r: 5, g: 0, b: 0 };
            leds.write([blip_light, disable_light]).unwrap();
            Timer::after(Duration::from_millis(10)).await;
            leds.write([disable_light, disable_light]).unwrap();

            let power_down_result = scd.power_down().await;
            log::info!("Power down result {:?}", power_down_result);
            let mut sleep_time = 59;
            if co2 > 0 {
                sleep_time = sleep_time + 120;
            }

            let timer = TimerWakeupSource::new(core::time::Duration::from_secs(sleep_time));
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
