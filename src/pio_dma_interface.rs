use embedded_hal::digital::v2::OutputPin;

use rp_pico::hal;
use rp_pico::hal::dma::{
    ReadTarget, SingleBuffering, SingleBufferingConfig, SingleChannel, WriteTarget,
};
use rp_pico::hal::pio::{PIOExt, PIO};
use rp_pico::hal::pio::{Running, StateMachine, StateMachineIndex, Tx};
use rp_pico::hal::pio::{Rx, UninitStateMachine};

use crate::array_scaler::LineTransfer;

enum DmaState<
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = u16>,
    TO: WriteTarget<TransmittedWord = u16>,
> {
    IDLE(CH, FROM, TO),
    RUNNING(SingleBuffering<CH, FROM, TO>),
}

pub struct PioInterface<CH: SingleChannel, RS, P: PIOExt, SM: StateMachineIndex> {
    sm: StateMachine<(P, SM), Running>,
    rs: RS,
    rx: Rx<(P, SM)>,
    dma: Option<DmaState<CH, &'static mut [u16], Tx<(P, SM)>>>,
}

impl<CH, RS, P, SM> PioInterface<CH, RS, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
    CH: SingleChannel,
{
    pub fn new(
        clock_divider: f32,
        rs: RS,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        rw: u8,
        pins: (u8, u8),
        buffer: &'static mut [u16],
        dma_channel: CH,
    ) -> Self {
        let video_program = pio_proc::pio_asm!(
            ".side_set 1 opt",
            "jmp start_tx side 1",
            ".wrap_target"

            "public start_tx:"
            "pull side 1",
            "out pins, 24 side 0 [1] ",
            "nop side 1 [1] ",
            "out pins, 8 side 0 [1] ",
            "jmp start_tx side 1 ",

            "public start_8:"
            "pull side 1 ",
            "out pins, 32 side 0 [1]",
            "jmp start_8 side 1",
            ".wrap"
        );

        let video_program_installed = pio.install(&video_program.program).unwrap();

        let (mut video_sm, rx, vid_tx) =
            rp_pico::hal::pio::PIOBuilder::from_program(video_program_installed)
                .out_pins(pins.0, pins.1)
                .side_set_pin_base(5)
                .out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
                .buffers(rp_pico::hal::pio::Buffers::OnlyTx)
                .clock_divisor(clock_divider)
                .build(sm);
        video_sm.set_pindirs((pins.0..pins.1 + 1 as u8).map(|n| (n, hal::pio::PinDir::Output)));
        video_sm.set_pindirs([(rw, hal::pio::PinDir::Output)]);

        Self {
            rs: rs,
            rx: rx,
            sm: video_sm.start(),
            dma: (Some(DmaState::IDLE(dma_channel, buffer, vid_tx))),
        }
    }

    pub fn do_tranfer(&mut self, buffer: &'static mut [u16]) -> &'static mut [u16] {
        let foo = core::mem::replace(&mut self.dma, None).unwrap();

        let (ch, old_buffer, tx) = match foo {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };

        let sbc = SingleBufferingConfig::new(ch, buffer, tx).start();
        self.dma = Some(DmaState::RUNNING(sbc));

        old_buffer
    }

    pub fn free(
        mut self,
        pio: &mut PIO<P>,
    ) -> (CH, UninitStateMachine<(P, SM)>, &'static mut [u16], RS) {
        let foo = core::mem::replace(&mut self.dma, None).unwrap();
        let (ch, old_buffer, tx) = match foo {
            DmaState::IDLE(ch, buff, tx) => (ch, buff, tx),
            DmaState::RUNNING(dma) => dma.wait(),
        };
        let (sm, prg) = self.sm.uninit(self.rx, tx);
        pio.uninstall(prg);

        (ch, sm, old_buffer, self.rs)
    }
}

impl<CH, RS, P, SM> LineTransfer for PioInterface<CH, RS, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
    RS: OutputPin,
    CH: SingleChannel,
{
    fn send_scanline(&mut self, line: &'static mut [u16]) -> &'static mut [u16] {
        self.do_tranfer(line)
    }
}
