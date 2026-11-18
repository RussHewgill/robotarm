use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicBool, Ordering},
};

use defmt::{debug, error, info, warn};

pub fn hard_fault() -> ! {
    // If `UsageFault` is enabled, we disable that first, since otherwise `udf` will cause that
    // exception instead of `HardFault`.
    #[cfg(not(any(armv6m, armv8m_base)))]
    {
        const SHCSR: *mut u32 = 0xE000ED24usize as _;
        const USGFAULTENA: usize = 18;

        unsafe {
            let mut shcsr = core::ptr::read_volatile(SHCSR);
            shcsr &= !(1 << USGFAULTENA);
            core::ptr::write_volatile(SHCSR, shcsr);
        }
    }

    cortex_m::asm::udf();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();

    // Guard against infinite recursion, just in case.
    if !PANICKED.load(Ordering::Relaxed) {
        PANICKED.store(true, Ordering::Relaxed);

        // print_defmt(info);
        defmt::error!("{}", defmt::Display2Format(info));
    }

    ws2812_panic_red();

    // TODO: loop, hard fault, or other?
    // hard_fault();
    loop {}
}

pub fn ws2812_panic_red() {
    let pin = 25;
    use embassy_rp::pac;

    // set pin to SIO
    pac::IO_BANK0.gpio(pin).ctrl().write(|w| {
        w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::SIOB_PROC_0 as u8);
    });

    pac::PADS_BANK0.gpio(pin).modify(|w| {
        w.set_iso(false); // clear pad isolation
        w.set_od(false); // clear output disable
    });

    assert!(pin < 32);

    let pin_mask = 1u32 << pin;

    // set pin as output in SIO
    pac::SIO.gpio_oe(pin).value_set().write(|w| pin_mask);

    // let out_set_ptr = pac::SIO.gpio_oe(pin).as_ptr();
    // let out_clr_ptr = out_clr.as_ptr();

    //
}
