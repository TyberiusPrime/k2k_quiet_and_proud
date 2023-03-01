use keytokey::handlers::{HandlerResult, OnOff, ProcessKeys};
use keytokey::AcceptsKeycode;
use keytokey::{iter_unhandled_mut, Event, EventStatus, USBKeyOut, KeyCode, UserKey};
use core::convert::TryInto;
use no_std_compat::prelude::v1::*;

//a handler for the IR sensor under the wrist...
pub struct IRPressReleaseMacro<M> {
    keycode: u32,
    callbacks: M,
    waiting: i32,
}
impl<M: OnOff> IRPressReleaseMacro<M> {
    pub fn new(trigger: impl AcceptsKeycode, callbacks: M) -> IRPressReleaseMacro<M> {
        IRPressReleaseMacro {
            keycode: trigger.to_u32(),
            callbacks,
            waiting: 0,
        }
    }
}
impl<T: USBKeyOut, M: OnOff> ProcessKeys<T> for IRPressReleaseMacro<M> {
    fn process_keys(
        &mut self,
        events: &mut Vec<(Event, EventStatus)>,
        output: &mut T,
    ) -> HandlerResult {
        for (event, status) in iter_unhandled_mut(events) {
            match event {
                Event::KeyPress(kc) => {
                    // this is instant
                    if kc.keycode == self.keycode {
                        *status = EventStatus::Handled;
                        self.callbacks.on_deactivate(output);
                        self.waiting = -1;
                    }
                }
                Event::KeyRelease(kc) => {
                    if kc.keycode == self.keycode {
                        *status = EventStatus::Handled;
                        //self.callbacks.on_activate(output); // note inverse
                        self.waiting = 0; //100 = 1s
                    }
                }
                Event::TimeOut(_) => {
                    if self.waiting == 0 {
                        output.debug("ir release timeout");
                        self.callbacks.on_activate(output); // note inverse
                        self.waiting = -1;
                    } else if self.waiting > 0 {
                        self.waiting -= 1;
                    }
                }
            }
        }
        HandlerResult::NoOp
    }
}

//Map UK51-59 to kb1-9 + gui
pub struct DesktopSwitcher {
}
impl<T: USBKeyOut> ProcessKeys<T> for DesktopSwitcher {
    fn process_keys(
        &mut self,
        events: &mut Vec<(Event, EventStatus)>,
        output: &mut T,
    ) -> HandlerResult {
        for (event, status) in iter_unhandled_mut(events) {
            match event {
                Event::KeyPress(kc) => {
                    if kc.keycode >= UserKey::UK51.to_u32() && kc.keycode <= UserKey::UK59.to_u32() {
                        let delta = kc.keycode.to_u32() - UserKey::UK51.to_u32();
                        let keycode = KeyCode::Kb1.to_u32() + delta;
                        *status = EventStatus::Handled;
                        output.send_keys(&[KeyCode::LGui, keycode.try_into().unwrap()]);
                    }
                }
                Event::KeyRelease(_kc) => {
                                    }
                Event::TimeOut(_) => {
                            }
            }
        }
        HandlerResult::NoOp
    }
}
