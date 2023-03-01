#![no_main]
#![no_std]
#![feature(alloc_error_handler)]
#![feature(clamp)]
#![feature(const_fn)]
#![feature(integer_atomics)]

//extern crate panic_halt;

extern crate nb;
//use no_std_compat::prelude::v1::*;

extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m_rt as rt; // v0.5.x
extern crate keytokey;

use core::alloc::Layout;

#[alloc_error_handler]
fn oom(_info: Layout, //~ ERROR argument should be `Layout`
) -> ! //~ ERROR return type should be `!`
{
    panic!("OOM");
}

use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: crate::trallocator::Trallocator<CortexMHeap> =
    crate::trallocator::Trallocator::new(CortexMHeap::empty());

//use core::panic::PanicInfo;
//use core::sync::atomic::{self, Ordering};

extern crate panic_semihosting;
/*
use core::sync::atomic;
use core::cmp::Ordering;
use core::panic::PanicInfo;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    resources.DISP.clear().ok();
    resources.DISP.write_str("PANIC").ok();
     loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
*/

pub mod hid;
pub mod keyboard;
pub mod matrix;
mod myhandlers;
mod trallocator;
mod usbout;
use usbout::USBOut;

use crate::keyboard::Keyboard;
use crate::matrix::Matrix;
//use core::convert::TryFrom;
use core::fmt::Write;
use no_std_compat::prelude::v1::*;
use rtfm::app;

//use stm32f1xx_hal::prelude::*; can't use this with v2 digital traits
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
//use stm32f1xx_hal::gpio::{Alternate, Floating, Input, PushPull};
//use stm32f1xx_hal::prelude::*;
pub use stm32f1xx_hal::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use stm32f1xx_hal::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use stm32f1xx_hal::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use stm32f1xx_hal::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
//pub use stm32f1xx_hal::hal::adc::OneShot as _embedded_hal_adc_OneShot;
//pub use stm32f1xx_hal::hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
//pub use stm32f1xx_hal::hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
//pub use stm32f1xx_hal::hal::prelude::*;
use debouncing::{DebounceResult, Debouncer};
use embedded_hal::digital::v2::ToggleableOutputPin;
pub use stm32f1xx_hal::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use stm32f1xx_hal::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use stm32f1xx_hal::dma::WriteDma as _stm32_hal_dma_WriteDma;
//pub use stm32f1xx_hal::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use stm32f1xx_hal::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use stm32f1xx_hal::time::U32Ext as _stm32_hal_time_U32Ext;

use ssd1306::{mode::TerminalMode, Builder};

#[allow(deprecated)]
//use embedded_hal::digital::v1::ToggleableOutputPin;
use embedded_hal::digital::v2::OutputPin;
#[allow(unused_imports)]
use embedded_hal::digital::v2_compat;
//use embedded_hal::serial::Write;

use keytokey::Keyboard as K2KKeyboard;
use keytokey::USBKeyOut;
use stm32f1;
use stm32f1xx_hal::stm32;
use stm32f1xx_hal::{gpio, timer};
use usb_device::bus;
use usb_device::class::UsbClass;
use usb_device::prelude::*;

type KeyboardHidClass = hid::HidClass<'static, UsbBusType, Keyboard>;
type Led = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

// Generic keyboard from
// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
const VID: u16 = 0x27db;
const PID: u16 = 0x16c0;

pub trait StringSender {
    fn writeln(&mut self, s: &str);
}

const TRANSLATION: &[u32] = {
    use keytokey::KeyCode::*;
    use keytokey::UserKey;
    &[
        F.to_u32(),            //0x0
        S.to_u32(),            //0x1
        A.to_u32(),            //0x2
        BSpace.to_u32(),       //0x3
        Down.to_u32(),         //0x4
        Slash.to_u32(),        //0x5
        M.to_u32(),            //0x6
        Comma.to_u32(),        //0x7
        Kb4.to_u32(),          //0x8
        Kb2.to_u32(),          //0x9
        Minus.to_u32(),        //0xa
        PgUp.to_u32(),         //0xb
        Home.to_u32(),         //0xc
        Equal.to_u32(),        //0xd
        Kb7.to_u32(),          //0xe
        Kb9.to_u32(),          //0xf
        V.to_u32(),            //0x10
        X.to_u32(),            //0x11
        Z.to_u32(),            //0x12
        Left.to_u32(),         //0x13
        Enter.to_u32(),        //0x14
        Quote.to_u32(),        //0x15
        J.to_u32(),            //0x16
        L.to_u32(),            //0x17
        R.to_u32(),            //0x18
        E.to_u32(),            //0x19
        Q.to_u32(),            //0x1a
        LAlt.to_u32(),         //0x1b
        Up.to_u32(),           //0x1c
        RShift.to_u32(),       //0x1d
        N.to_u32(),            //0x1e
        Dot.to_u32(),          //0x1f
        B.to_u32(),            //0x20
        C.to_u32(),            //0x21
        LShift.to_u32(),       //0x22
        Right.to_u32(),        //0x23
        Delete.to_u32(),       //0x24
        SColon.to_u32(),       //0x25
        H.to_u32(),            //0x26
        K.to_u32(),            //0x27
        G.to_u32(),            //0x28
        D.to_u32(),            //0x29
        CapsLock.to_u32(),     //0x2a
        Space.to_u32(),        //0x2b
        End.to_u32(),          //0x2c
        Kb0.to_u32(),          //0x2d
        Kb6.to_u32(),          //0x2e
        Kb8.to_u32(),          //0x2f
        T.to_u32(),            //0x30
        W.to_u32(),            //0x31
        Tab.to_u32(),          //0x32
        LCtrl.to_u32(),        //0x33
        LGui.to_u32(),         //0x34
        P.to_u32(),            //0x35
        U.to_u32(),            //0x36
        I.to_u32(),            //0x37
        Kb5.to_u32(),          //0x38
        Kb3.to_u32(),          //0x39
        Kb1.to_u32(),          //0x3a
        PgDown.to_u32(),       //0x3b
        UserKey::UK0.to_u32(), //0x3c
        LBracket.to_u32(),     //0x3d
        Y.to_u32(),            //0x3e
        O.to_u32(),            //0x3f
        UserKey::UK1.to_u32(), //0x40 the reflective light sensor
    ]
    //missing: grave, bslash
};

pub fn get_keytokey<'a, T: USBKeyOut>(output: T) -> K2KKeyboard<'a, T> {
    use handlers::LayerAction::RewriteToShifted as RTS;
    use handlers::LayerAction::SendString;
    use keytokey::{handlers, premade, HandlerID, KeyCode, Keyboard, Modifier, UserKey};

    struct CycleUnicode {}
    impl keytokey::handlers::Action for CycleUnicode {
        fn on_trigger(&mut self, output: &mut dyn USBKeyOut) {
            if output.state().unicode_mode == keytokey::UnicodeSendMode::WinComposeDvorak {
                output.state().unicode_mode = keytokey::UnicodeSendMode::LinuxDvorak;
                output.debug("LinuxDvorak mode");
            } else {
                output.state().unicode_mode = keytokey::UnicodeSendMode::WinComposeDvorak;
                output.debug("WinDvorak mode");
            }
        }
    }
    struct TriggerBootloader {}
    impl keytokey::handlers::Action for TriggerBootloader {
        fn on_trigger(&mut self, output: &mut dyn USBKeyOut) {
            output.bootloader();
            //stm32f1xx_hal::backup_domain1G/1
        }
    }

    let mut k = Keyboard::new(output);
    k.output.state().unicode_mode = keytokey::UnicodeSendMode::LinuxDvorak;

    //the umlaut layer - must come after the tap dance!
    let umlaut_id = k.future_handler_id(1);
    k.add_handler(Box::new(handlers::Layer::new(
        vec![
            (KeyCode::A, RTS(0xE4, 0xC4)),
            (KeyCode::S, RTS(0xF6, 0xD6)),
            (KeyCode::F, RTS(0xFC, 0xDC)),
            (KeyCode::SColon, SendString("ß")),
            (
                KeyCode::Space,
                handlers::LayerAction::Action(Box::new(CycleUnicode {})),
            ),
            (
                KeyCode::Minus,
                handlers::LayerAction::Action(Box::new(TriggerBootloader {})),
            ),
        ],
        handlers::AutoOff::AfterNonModifier,
    )));

    let mut abort = premade::ActionAbort::new();
    //        abort.set_abort_status(dvorak_id, true);
    //       abort.set_abort_status(numpad_id, false);
    abort.set_abort_status(umlaut_id, false);
    k.add_handler(Box::new(handlers::OneShot::new(
        KeyCode::LShift,
        KeyCode::RShift,
        premade::ActionHandler::new(Modifier::Shift as HandlerID),
        keytokey::premade::ActionNone {},
        premade::ActionToggleHandler { id: umlaut_id },
        400,
        1000,
    )));
    k.add_handler(Box::new(handlers::OneShot::new(
        KeyCode::LGui,
        KeyCode::No,
        premade::ActionHandler::new(Modifier::Gui as HandlerID),
        abort,
        keytokey::premade::ActionNone {},
        400,
        1000,
    )));

    const OTHER_MAP: &[(u32, u32)] = &[
        (KeyCode::RBracket.to_u32(), KeyCode::F1.to_u32()),
        (KeyCode::Kb1.to_u32(), KeyCode::F2.to_u32()),
        (KeyCode::Kb2.to_u32(), KeyCode::F3.to_u32()),
        (KeyCode::Kb3.to_u32(), KeyCode::F4.to_u32()),
        (KeyCode::Kb4.to_u32(), KeyCode::F5.to_u32()),
        (KeyCode::Kb5.to_u32(), KeyCode::F6.to_u32()),
        (KeyCode::Kb6.to_u32(), KeyCode::F7.to_u32()),
        (KeyCode::Kb7.to_u32(), KeyCode::F8.to_u32()),
        (KeyCode::Kb8.to_u32(), KeyCode::F9.to_u32()),
        (KeyCode::Kb9.to_u32(), KeyCode::F10.to_u32()),
        (KeyCode::Kb0.to_u32(), KeyCode::F11.to_u32()),
        (KeyCode::Equal.to_u32(), KeyCode::F12.to_u32()),
        (KeyCode::F.to_u32(), KeyCode::BSlash.to_u32()),
        (KeyCode::D.to_u32(), KeyCode::Grave.to_u32()),
        (KeyCode::S.to_u32(), KeyCode::RBracket.to_u32()),
        (KeyCode::BSpace.to_u32(), KeyCode::Delete.to_u32()),
    ];
    let other_layer_id = k.future_handler_id(1);
    k.add_handler(Box::new(handlers::RewriteLayer::new(&OTHER_MAP)));

    k.add_handler(Box::new(handlers::OneShot::new(
        UserKey::UK0,
        KeyCode::No,
        premade::ActionHandler::new(other_layer_id),
        "\u{1F596}",
        "??",
        400,
        1000,
    )));

    let copy_pasta_layer_id = k.future_handler_id(1);
    const COPY_PASTE_MAP: &[(u32, u32)] = &[
        (KeyCode::F.to_u32(), KeyCode::Copy.to_u32()),
        (KeyCode::D.to_u32(), KeyCode::Paste.to_u32()),
        //(KeyCode::G.to_u32(), KeyCode::Cut.to_u32()),
        (KeyCode::B.to_u32(), KeyCode::Stop.to_u32()), // to ctrl-c later on.
        //I really need to move the sensor by 10mm to the right before enabling this.
        //(KeyCode::Space.to_u32(), KeyCode::Enter.to_u32()),
        //(KeyCode::Kb6.to_u32(), KeyCode::VolumeDown.to_u32()),
        //(KeyCode::Kb7.to_u32(), KeyCode::VolumeUp.to_u32()),
        //(KeyCode::Kb1.to_u32(), UserKey::UK51.to_u32()),
        //(KeyCode::Kb2.to_u32(), UserKey::UK52.to_u32()),
        //(KeyCode::Kb3.to_u32(), UserKey::UK53.to_u32()),
        //(KeyCode::Kb4.to_u32(), UserKey::UK54.to_u32()),
        //(KeyCode::Kb5.to_u32(), UserKey::UK55.to_u32()),
        //(KeyCode::Q.to_u32(), UserKey::UK56.to_u32()),
        //(KeyCode::W.to_u32(), UserKey::UK57.to_u32()),
        //(KeyCode::E.to_u32(), UserKey::UK58.to_u32()),
        //(KeyCode::R.to_u32(), UserKey::UK59.to_u32()),
        //
        //num block
        /*
        (KeyCode::M.to_u32(), KeyCode::Kb1.to_u32()),
        (KeyCode::Comma.to_u32(), KeyCode::Kb2.to_u32()),
        (KeyCode::Dot.to_u32(), KeyCode::Kb3.to_u32()),
        (KeyCode::J.to_u32(), KeyCode::Kb4.to_u32()),
        (KeyCode::K.to_u32(), KeyCode::Kb5.to_u32()),
        (KeyCode::L.to_u32(), KeyCode::Kb6.to_u32()),
        (KeyCode::U.to_u32(), KeyCode::Kb7.to_u32()),
        (KeyCode::I.to_u32(), KeyCode::Kb8.to_u32()),
        (KeyCode::O.to_u32(), KeyCode::Kb9.to_u32()),
        (KeyCode::Up.to_u32(), KeyCode::W.to_u32()),
        (KeyCode::Down.to_u32(), KeyCode::E.to_u32()),
        (KeyCode::N.to_u32(), KeyCode::Kb0.to_u32()),
        */
    ];

    k.add_handler(Box::new(handlers::RewriteLayer::new(&COPY_PASTE_MAP)));

    k.add_handler(Box::new(myhandlers::IRPressReleaseMacro::new(
        UserKey::UK1,
        premade::ActionHandler::new(copy_pasta_layer_id),
    )));

    //k.add_handler(Box::new(myhandlers::DesktopSwitcher{}));
    //let grave_layer = vec![(KeyCode::Grave, SendStringShifted("~", "`"))];

    /*
    let grave_layer_id = k.future_handler_id(1);
    k.add_handler(Box::new(
            handlers::Layer::new(grave_layer, handlers::AutoOff::No)
                ));

    k.output.state().enable_handler(grave_layer_id);
    */

    /*
        //k.output.debug(&format!("B{}", ALLOCATOR.get()));
        //one shots must come before space cadets
        //k.add_handler(premade::one_shot_shift(400, 1000));
        //k.add_handler(premade::one_shot_ctrl(400, 1000));
        //k.add_handler(premade::one_shot_alt(400, 1000));
        //k.add_handler(premade::one_shot_gui(400, 1000));
        //k.output.debug(&format!("B1{}", ALLOCATOR.get()));


        //k.add_handler(premade::space_cadet_handler(KeyCode::F, KeyCode::U,
            //k.future_handler_id(2)));
      //  k.add_handler(premade::space_cadet_handler(KeyCode::J, KeyCode::H,
       //     k.future_handler_id(2)));

        struct LayerToggleTapDance {handler_id: HandlerID, toggle: bool}
        impl handlers::TapDanceAction for LayerToggleTapDance {
            fn on_tapdance( &mut self, trigger: u32,
                output: &mut impl USBKeyOut,
                    tap_count: u8,
                    tap_end: handlers::TapDanceEnd){
                        match tap_count {
                            0 => {},
                            1 => output.send_keys(&[KeyCode::try_from(trigger).unwrap()]),
                            _ => {
                                if self.toggle  {
                                    output.state().toggle_handler(self.handler_id);
                                }
                                else {
                                    output.state().enable_handler(self.handler_id);
                                }
                            },
                        }
             }
        }

        k.add_handler(
            Box::new(handlers::TapDance::new(
                KeyCode::F8,
                LayerToggleTapDance{handler_id: umlaut_id, toggle: false},
                100
            )));


        const NUMPAD_MAP: &[(u32, u32)] = &[
                (KeyCode::U.to_u32(), KeyCode::Kb7.to_u32()),
                (KeyCode::I.to_u32(), KeyCode::Kb8.to_u32()),
                (KeyCode::O.to_u32(), KeyCode::Kb9.to_u32()),
                (KeyCode::J.to_u32(), KeyCode::Kb4.to_u32()),
                (KeyCode::K.to_u32(), KeyCode::Kb5.to_u32()),
                (KeyCode::L.to_u32(), KeyCode::Kb6.to_u32()),
                (KeyCode::M.to_u32(), KeyCode::Kb1.to_u32()),
                (KeyCode::Comma.to_u32(), KeyCode::Kb2.to_u32()),
                (KeyCode::Dot.to_u32(), KeyCode::Kb3.to_u32()),
                (KeyCode::N.to_u32(), KeyCode::Kb0.to_u32()),
                (KeyCode::Space.to_u32(), KeyCode::Tab.to_u32()),
                (KeyCode::BSlash.to_u32(), KeyCode::Dot.to_u32()),
                (KeyCode::H.to_u32(), KeyCode::Comma.to_u32()),
            ];
        let numpad_id = k.future_handler_id(2);
        k.add_handler(
            Box::new(handlers::TapDance::new(
                KeyCode::F6,
                LayerToggleTapDance{handler_id: numpad_id, toggle: true},
                100
            )));
        k.add_handler(Box::new(
            handlers::RewriteLayer::new(&NUMPAD_MAP)
        )
        );




        let dvorak_id = k.add_handler(premade::dvorak());

        //k.output.debug(&format!("C{}", ALLOCATOR.get()));
      //  k.output.state().enable_handler(umlaut_id);
        let mut abort = premade::ActionAbort::new();
        abort.set_abort_status(dvorak_id, true);
        abort.set_abort_status(numpad_id, false);
        abort.set_abort_status(umlaut_id , false);
            //k.output.debug(&format!("D{}", ALLOCATOR.get()));

        k.add_handler(
            Box::new(handlers::OneShot::new(
                KeyCode::LShift,
                KeyCode::RShift,
                premade::ActionHandler::new(
                    Modifier::Shift as HandlerID,
                ),
                abort,
                premade::ActionToggleHandler{id: umlaut_id},
                400,
                1000,
            )));

    //$! -> 41, yeah.
        k.add_handler(
            Box::new(handlers::TapDance::new(
                KeyCode::F1,
                LayerToggleTapDance{handler_id: dvorak_id, toggle: true},
                100
            )));

        //k.output.debug(&format!("E{}", ALLOCATOR.get()));


        k.output.state().enable_handler(dvorak_id);

        //k.output.debug(&format!("F{}", ALLOCATOR.get()));
        //k.add_handler(Box::new(premade::CopyPaste{}));
        k.add_handler(Box::new(
               handlers::PressMacro::new(KeyCode::Copy,
                                            vec!(KeyCode::LGui, KeyCode::LCtrl, KeyCode::LShift, KeyCode::C))));

        k.add_handler(Box::new(
               handlers::PressMacro::new(KeyCode::Paste,
                                            vec!(KeyCode::LCtrl, KeyCode::V))));


        //k.output.debug(&format!("G{}", ALLOCATOR.get()));


    */
    //k.output.debug(&format!("I{}", ALLOCATOR.get()));
    //use cortex_m_semihosting::hprintln;

    k.add_handler(Box::new(handlers::UnicodeKeyboard::new()));
    k.add_handler(Box::new(handlers::USBKeyboard::new()));
    //k.add_handler(Box::new(debug_handlers::TranslationHelper {}));
    //k.output.debug(&format!("J{}", ALLOCATOR.get()));

    return k;
}

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    //static mut USB_CLASS: KeyboardHidClass = ();
    static mut TIMER: timer::CountDownTimer<stm32::TIM3> = ();
    static mut TIMER_MS: timer::CountDownTimer<stm32::TIM4> = ();
    static mut LED: Led = ();
    static mut MATRIX: Matrix = ();
    static mut DEBOUNCER: Debouncer = ();
    static mut K2K: K2KKeyboard<'static, USBOut> = ();
    static mut LAST_TIME_MS: u32 = 0;
    static mut CURRENT_TIME_MS: u32 = 0;
    static mut HEAPSIZE: u32 = 0;
    static mut KEYCOUNTER: u32 = 0;
    static mut BACKUP_REGISTER: stm32f1xx_hal::backup_domain::BackupDomain = ();
    static mut DISP: ssd1306::mode::terminal::TerminalMode<
        ssd1306::interface::i2c::I2cInterface<
            stm32f1xx_hal::i2c::BlockingI2c<
                stm32f1::stm32f103::I2C1,
                (
                    stm32f1xx_hal::gpio::gpiob::PB6<
                        stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                    >,
                    stm32f1xx_hal::gpio::gpiob::PB7<
                        stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                    >,
                ),
            >,
        >,
    > = ();

    #[init]
    fn init() -> init::LateResources {
        let start = rt::heap_start() as usize;
        let size = 6 * 1024; // in bytes
        unsafe { ALLOCATOR.0.init(start, size) }

        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let bkp = rcc
            .bkp
            .constrain(device.BKP, &mut rcc.apb1, &mut device.PWR);
        //bkp.write_data_register_low(0, 0);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz()) // use 72mhz to work with stm32duino bootloader
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low().ok();

        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let mut disp: TerminalMode<_> = Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap();
        disp.set_rotation(ssd1306::displayrotation::DisplayRotation::Rotate180)
            .ok();
        disp.clear().ok();
        disp.write_str("hello world!").ok();
        disp.set_position(0, 1).ok();
        led.set_high().ok();

        disp.write_str(" how are you doing today???").ok();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().ok();
        cortex_m::asm::delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS = Some(UsbBus::new(usb));
        }
        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

        let usb_class = hid::HidClass::new(Keyboard::new(), &usb_bus);
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(VID, PID))
            .manufacturer("TyberiusPrime")
            .product("K2KAdvantageQ")
            .serial_number(env!("CARGO_PKG_VERSION"))
            .build();
        disp.write_str("usb init done").ok();

        let mut timer =
            timer::Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1).start_count_down(100.hz());
        timer.listen(timer::Event::Update);

        let mut timer_ms =
            timer::Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1).start_count_down(1000.hz());
        timer_ms.listen(timer::Event::Update);

        //let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        //let pin_rx = gpioa.pa10;
        //let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        use stm32f1xx_hal::gpio::State::High;

        let matrix = Matrix::new(
            //diodes... b15, b3 b4 b5, a5, a4, a3,a2
            vec![
                gpioa.pa2.into_pull_up_input(&mut gpioa.crl).downgrade(), //
                gpioa.pa3.into_pull_up_input(&mut gpioa.crl).downgrade(), //
                gpioa.pa4.into_pull_up_input(&mut gpioa.crl).downgrade(), //
                gpioa.pa5.into_pull_up_input(&mut gpioa.crl).downgrade(), //
            ],
            vec![
                pb3.into_pull_up_input(&mut gpiob.crl).downgrade(), //
                pb4.into_pull_up_input(&mut gpiob.crl).downgrade(), //
                gpiob.pb5.into_pull_up_input(&mut gpiob.crl).downgrade(), //
                gpiob.pb15.into_pull_up_input(&mut gpiob.crh).downgrade(), //

                                                                    //gpiob.pb8.into_pull_up_input(&mut gpiob.crh).downgrade(),
                                                                    //gpiob.pb9.into_pull_up_input(&mut gpiob.crh).downgrade(),
            ],
            vec![
                //b10, b1, b0, a7, a6, a1, a0, c15 = b11
                gpioa
                    .pa0
                    .into_open_drain_output_with_state(&mut gpioa.crl, High)
                    .downgrade(),
                gpioa
                    .pa1
                    .into_open_drain_output_with_state(&mut gpioa.crl, High)
                    .downgrade(),
                gpioa
                    .pa6
                    .into_open_drain_output_with_state(&mut gpioa.crl, High)
                    .downgrade(),
                gpioa
                    .pa7
                    .into_open_drain_output_with_state(&mut gpioa.crl, High)
                    .downgrade(),
            ],
            vec![
                gpiob
                    .pb0
                    .into_open_drain_output_with_state(&mut gpiob.crl, High)
                    .downgrade(),
                gpiob
                    .pb1
                    .into_open_drain_output_with_state(&mut gpiob.crl, High)
                    .downgrade(),
                gpiob
                    .pb10
                    .into_open_drain_output_with_state(&mut gpiob.crh, High)
                    .downgrade(),
                gpiob
                    .pb11
                    .into_open_drain_output_with_state(&mut gpiob.crh, High)
                    .downgrade(),
            ],
            vec![
                gpiob.pb9.into_pull_up_input(&mut gpiob.crh).downgrade(), //
            ],
        );
        let output = USBOut::new(usb_class);
        //output.tx.writeln(&format!("pre_matrix {}", pre_matrix));
        //output.tx.writeln(&format!("matrix {}", ALLOCATOR.get()));
        disp.write_str("matrix init done").ok();

        let debouncer = Debouncer::new(matrix.len());
        //output.tx.writeln(&format!("debouncer {}", ALLOCATOR.get()));

        let k2k = get_keytokey(output);

        disp.write_str("go go go").ok();

        init::LateResources {
            USB_DEV: usb_dev,
            //USB_CLASS: usb_class,
            TIMER: timer,
            TIMER_MS: timer_ms,
            LED: led,
            MATRIX: matrix,
            DEBOUNCER: debouncer,
            K2K: k2k,
            BACKUP_REGISTER: bkp,
            DISP: disp,
        }
    }

    #[interrupt(priority = 3, resources = [USB_DEV, K2K])]
    fn USB_HP_CAN_TX() {
        usb_poll(&mut resources.USB_DEV, &mut resources.K2K.output.usb_class);
    }

    #[interrupt(priority = 3, resources = [USB_DEV, K2K])]
    fn USB_LP_CAN_RX0() {
        usb_poll(&mut resources.USB_DEV, &mut resources.K2K.output.usb_class);
        if let Some(report) = resources.K2K.output.buffer.pop_front() {
            match resources.K2K.output.usb_class.write(report.as_bytes()) {
                Ok(0) => {
                    //try again?
                    resources.K2K.output.buffer.push_front(report); // presumably doesn't happen?
                }
                Ok(_i) => {} //complete report, presumably
                Err(_) => {}
            };
        }
    }

    #[interrupt(priority = 2, resources = [CURRENT_TIME_MS, TIMER_MS])]
    fn TIM4() {
        resources.TIMER_MS.clear_update_interrupt_flag();
        *resources.CURRENT_TIME_MS += 1;
    }

    #[interrupt(priority = 1, resources = [
        CURRENT_TIME_MS,
        DEBOUNCER,
        K2K,
        LAST_TIME_MS,
        LED,
        MATRIX,
        TIMER,
        HEAPSIZE,
        BACKUP_REGISTER,
        DISP,
        KEYCOUNTER,
    ])]
    fn TIM3() {
        resources.TIMER.clear_update_interrupt_flag();
        #[allow(deprecated)]
        resources.MATRIX.read_matrix();
        resources.LED.toggle().ok();

        let states = &resources.MATRIX.output;
        let mut nothing_changed = true;
        let current_time_ms = resources.CURRENT_TIME_MS.lock(|ct| *ct);
        let delta = current_time_ms
            .overflowing_sub(*resources.LAST_TIME_MS)
            .0
            .clamp(0, 2u32.pow(16) - 1);
        let debouncer = &mut *resources.DEBOUNCER;
        let mut update_last_time = false;
        let last_hs = *resources.HEAPSIZE;
        let hs = ALLOCATOR.get();
        let feedback: (Option<String>, bool) = resources.K2K.lock(|k2k| {
            //matrix::Matrix::debug_serial(&states, &mut k2k.output.tx);
            if hs != last_hs {
                //k2k.output.tx.writeln(&format!("heap {}", hs));
            }
            let mut output: Option<String> = None;
            for (ii, pressed) in states.iter().enumerate() {
                match debouncer.update(ii, pressed) {
                    DebounceResult::NoChange => {}
                    DebounceResult::Pressed => {
                        //use cortex_m_semihosting::hprintln;
                        //hprintln!("K: {:x}", ii as u32).ok();
                        nothing_changed = false;
                        k2k.add_keypress(
                            *TRANSLATION.get(ii).unwrap_or(&(ii as u32)),
                            delta as u16,
                        );
                        update_last_time = true;
                        k2k.handle_keys().ok();
                        k2k.clear_unhandled();
                        output = Some(format!("Argh ME: {:x} hs: {}", ii as u32, hs));
                    }
                    DebounceResult::Released => {
                        nothing_changed = false;
                        k2k.add_keyrelease(
                            *TRANSLATION.get(ii).unwrap_or(&(ii as u32)),
                            delta as u16,
                        );
                        update_last_time = true;
                        k2k.handle_keys().ok();
                        k2k.clear_unhandled();
                    }
                }
            }
            if nothing_changed {
                k2k.add_timeout(delta as u16);
                k2k.handle_keys().ok();
                k2k.clear_unhandled();
            }
            output = match output {
                Some(mut x) => {
                    x.push_str(" ");
                    x.push_str(&k2k.output.debug_str);
                    k2k.output.debug_str.clear();
                    Some(x)
                }
                None => {
                    if k2k.output.debug_str.len() > 0 {
                        let mut x = "".to_string();
                        x.push_str(&k2k.output.debug_str);
                        k2k.output.debug_str.clear();
                        Some(x)
                    } else {
                        None
                    }
                }
            };
            (output, k2k.output.enter_bootloader)
        });
        if update_last_time {
            *resources.LAST_TIME_MS = current_time_ms;
        }
        *resources.HEAPSIZE = hs;
        let output = feedback.0;
        let enter_bootloader = feedback.1;
        if enter_bootloader {
            (*resources.BACKUP_REGISTER).write_data_register_low(9, 0x424C);
            resources.DISP.clear().ok();
            resources.DISP.write_str("Bootloader started").ok();
            stm32f1xx_hal::stm32::SCB::sys_reset();
        }
        match output {
            Some(mut output) => {
                *resources.KEYCOUNTER += 1;
                let counter: u32 = *resources.KEYCOUNTER;
                let add = format!("c:{}", counter);
                output.push_str(&add);
                resources.DISP.clear().ok();
                resources.DISP.write_str(&output).ok();
            }
            None => {
                //this does happen.w
            }
        };
    }
};

fn usb_poll(usb_dev: &mut UsbDevice<'static, UsbBusType>, keyboard: &mut KeyboardHidClass) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}
