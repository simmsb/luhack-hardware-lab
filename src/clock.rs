use fugit::{HertzU32, RateExtU32};
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::pll::common_configs::PLL_USB_48MHZ;
use rp2040_hal::pll::{setup_pll_blocking, PLLConfig};
use rp2040_hal::xosc::setup_xosc_blocking;
use rp2040_hal::Watchdog;
use rp_pico::pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC};
use rp_pico::XOSC_CRYSTAL_FREQ;

pub const PLL_SYS_FAST: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1500),
    refdiv: 1,
    post_div1: 3,
    post_div2: 2,
};

pub fn configure_overclock(
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    let xosc = setup_xosc_blocking(xosc_dev, XOSC_CRYSTAL_FREQ.Hz())
        .map_err(|_e| "Can't setup OC (xosc)")
        .unwrap();

    watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        PLL_SYS_FAST,
        &mut clocks,
        resets,
    )
    .map_err(|_e| "Can't setup OC (pll sys)")
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(|_e| "Can't setup OC (pll usb)")
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    clocks
}
