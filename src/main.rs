#![no_std]
#![no_main]

mod fmt;

use defmt::println;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{task, Spawner};
use embassy_stm32::{gpio::{Level, Output, OutputType, Speed}, time::khz, timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}};
use embassy_time::{Duration, Instant, Timer};
use fmt::info;

#[task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.set_high();
        info!("*blink*");
        Timer::after(Duration::from_millis(800)).await;
        led.set_low();
        Timer::after(Duration::from_millis(800)).await;
    }
}



const DSHOT_FRAME_SIZE: usize = 16; // 16 bits per frame

static mut DSHOT_FRAMES: [u16; DSHOT_FRAME_SIZE + 2] = [0; DSHOT_FRAME_SIZE + 2];

// Create DShot data packet
fn create_dshot_packet(throttle: u16, telemetry: bool) -> u16 {
    let mut packet = (throttle & 0x07FF) << 1; // 11-bit throttle
    if telemetry {
        packet |= 1; // Set telemetry bit if needed
    }
    let crc = ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F) as u16; // 4-bit CRC
    (packet << 4) | crc
}

// Prepare DShot frame as array of pulse widths for each bit
fn prepare_dshot_frame(packet: u16, one_third: u16, two_third: u16, ch: usize) {
    for i in 0..(DSHOT_FRAME_SIZE) {
        // Calculate each bit’s pulse width
        let is_one = (packet & (1 << (15 - i))) != 0;
        unsafe {
            DSHOT_FRAMES[i + ch] = if is_one {
                two_third // Set for ~62.5% high pulse
            } else {
                one_third // Set for ~37.5% high pulse
            };
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());
    println!("hello");

    let blue = Output::new(p.PC13, Level::Low, Speed::Low);
    spawner.spawn(blink(blue).unwrap());

    let pwm_pin = PwmPin::new(p.PA6, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM3, Some(pwm_pin), None, None, None, khz(600), Default::default());

    let max = pwm.max_duty_cycle() as u16;

    let one_third = max / 3;
    let two_third = one_third * 2;
    pwm.ch1().enable();

    let inst = Instant::now();

    let packet0 = create_dshot_packet(10, false);
    
    prepare_dshot_frame(packet0, one_third, two_third, 0);

    unsafe {
        while (Instant::now() - inst) < Duration::from_secs(3) {
            pwm.waveform_up(p.DMA1_CH2.reborrow(), Channel::Ch1, &DSHOT_FRAMES).await;
            Timer::after(Duration::from_micros(50)).await;
        }
    }

    unsafe {
        for thr in (100..500).chain((100..500).rev()).cycle() {
            let packet = create_dshot_packet(thr, false);

            prepare_dshot_frame(packet, one_third, two_third, 0);

            for _ in 0..100 {
                pwm.waveform_up(p.DMA1_CH2.reborrow(), Channel::Ch1, &DSHOT_FRAMES).await;
                Timer::after(Duration::from_micros(25)).await;
            }
        }
    }
}
