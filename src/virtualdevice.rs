use evdev::uinput::VirtualDevice;
use evdev::*;

pub fn create_virtual_controller() -> Result<VirtualDevice, Box<dyn std::error::Error>> {
    let mut keys = AttributeSet::<KeyCode>::new();
    keys.insert(KeyCode::BTN_SOUTH);
    keys.insert(KeyCode::BTN_EAST);
    keys.insert(KeyCode::BTN_NORTH);
    keys.insert(KeyCode::BTN_WEST);
    keys.insert(KeyCode::BTN_TL);
    keys.insert(KeyCode::BTN_TR);
    keys.insert(KeyCode::BTN_SELECT);
    keys.insert(KeyCode::BTN_START);
    keys.insert(KeyCode::BTN_THUMBL);
    keys.insert(KeyCode::BTN_THUMBR);
    keys.insert(KeyCode::BTN_MODE);
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY1); // capture
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY2); // gl
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY3); // gr
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY4); // sll
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY5); // slr
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY6); // srl
    keys.insert(KeyCode::BTN_TRIGGER_HAPPY7); // srr

    let abs_setup = |min, max, fuzz, flat| AbsInfo::new(0, min, max, fuzz, flat, 1);

    let device = VirtualDevice::builder()?
        .name("ConHid Virtual Device")
        .input_id(InputId::new(BusType::BUS_USB, 0x045E, 0x028E, 0x110))
        .with_keys(&keys)?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_X,
            abs_setup(-32767, 32767, 16, 128),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Y,
            abs_setup(-32767, 32767, 16, 128),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RX,
            abs_setup(-32767, 32767, 16, 128),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RY,
            abs_setup(-32767, 32767, 16, 128),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Z,
            abs_setup(0, 255, 0, 0),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RZ,
            abs_setup(0, 255, 0, 0),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_HAT0X,
            abs_setup(-1, 1, 0, 0),
        ))?
        .with_absolute_axis(&UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_HAT0Y,
            abs_setup(-1, 1, 0, 0),
        ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_TILT_X,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_TILT_Y,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_MISC,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_BRAKE,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_GAS,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        // .with_absolute_axis(&UinputAbsSetup::new(
        //     AbsoluteAxisCode::ABS_WHEEL,
        //     abs_setup(-32767, 32767, 0, 0),
        // ))?
        .with_ff_effects_max(16)
        .build()?;

    Ok(device)
}
