#![no_std]
#![no_main]

mod fmt;

use defmt::println;
use dshot_frame::{Command, Frame};
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());
    println!("hello");

    let blue = Output::new(p.PC13, Level::Low, Speed::Low);
    spawner.spawn(blink(blue).unwrap());

    let pwm_pin = PwmPin::new(p.PA6, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM3, Some(pwm_pin), None, None, None, khz(600), Default::default());

    let max = pwm.max_duty_cycle() as u16;

    pwm.ch1().enable();

    let inst = Instant::now();

    let frame = Frame::command(Command::MotorStop, false);

    let ds = &frame.duty_cycles(max);
    
    while (Instant::now() - inst) < Duration::from_secs(3) {
        pwm.waveform_up(p.DMA1_CH2.reborrow(), Channel::Ch1, ds).await;
        Timer::after(Duration::from_micros(50)).await;
    }
    
    for thr in (100..500).chain((100..500).rev()).cycle() {
        let frame = Frame::new(thr, false).unwrap();

        for _ in 0..100 {
            pwm.waveform_up(p.DMA1_CH2.reborrow(), Channel::Ch1, &frame.duty_cycles(max)).await;
            Timer::after(Duration::from_micros(25)).await;
        }
    }
}
