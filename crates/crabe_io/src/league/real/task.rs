use std::process::exit;
use std::thread::sleep;
use std::time::Duration;
use bytes::BufMut;
use crate::league::real::RealConfig;
use log::error;
use prost::DecodeError;
use crabe_framework::constant::MAX_ID_ROBOTS;
use crabe_framework::data::output::{Command, CommandMap, Feedback, FeedbackMap, Kick};

use crabe_protocol::protobuf::robot_packet::{BaseCommand, BaseToPc, Kicker, PcToBase};

use crate::communication::UsbTransceiver;
use crate::pipeline::output::CommandSenderTask;

impl CommandSenderTask for Real {
    fn step(&mut self, commands: CommandMap) -> FeedbackMap {
        // dbg!(&commands);
        if commands.len() > 16 {
            error!("Capacity oversize for the commands !");
            Default::default()
        }

        let packet = self.prepare_packet(commands.into_iter());

        if let Err(error) = self.usb.clear(serialport::ClearBuffer::All) {
            error!("Cannot clear buffers : {}", error);
        }
        self.usb.send(packet);
        self.get_feedback_data()
    }

    fn close(&mut self) {
        let mut commands: CommandMap = Default::default();
        for id in 0..MAX_ID_ROBOTS {
            commands.insert(id as u8, Default::default());
        }

        // send at least 2 times to prevent robots from
        // not receiving data when their antenna is in TX mode
        // very important if this is called by a CTRL-C signal
        self.step(commands.clone());
        self.step(commands);
    }
}

pub struct Real {
    usb: UsbTransceiver,
}

// WARNING
// this value MUST be slightly superior to the maximum timeout
// specified in the embedded base station's code
// If you only has "Operation timeouts" errors, try
// resetting the board or replugging it
const MAX_SERIAL_READ_TIMEOUT_MILLIS: u64 = 50;

impl Real {
    pub fn with_config(usb_config: RealConfig) -> Self {
        let usb = UsbTransceiver::new(&usb_config.usb_port,usb_config.usb_baud,
                                      MAX_SERIAL_READ_TIMEOUT_MILLIS)
            .expect("Failed to create usb transceiver");

        Self { usb }
    }

    fn prepare_packet(&mut self, commands: impl Iterator<Item = (u8, Command)>) -> PcToBase {
        let mut packet = PcToBase::default();
        for (id, command) in commands {
            let (kicker_cmd, kick_power) = match command.kick {
                None => (Kicker::NoKick, 0.0_f32),
                Some(Kick::StraightKick { power }) => (Kicker::Flat, power),
                Some(Kick::ChipKick { power }) => (Kicker::Chip, power),
            };

            packet.commands.push(BaseCommand {
                robot_id: id as u32,
                normal_velocity: command.forward_velocity,
                tangential_velocity: command.left_velocity,
                angular_velocity: command.angular_velocity,
                kick: kicker_cmd.into(),
                kick_power,
                charge: command.charge,
                dribbler: command.dribbler,
            });
        }
        packet
    }

    fn get_feedback_data(&mut self) -> FeedbackMap {
        let read_bytes: usize = 0;
        match self.usb.read(read_bytes) {
            Ok(data) => {
                let read_packet: Result<BaseToPc, DecodeError> = prost::Message::decode(data.as_slice());
                let r = match read_packet {
                    Ok(packet) => {
                        packet.feedbacks.iter()
                            .fold(FeedbackMap::new(), |mut feedback_map, base_feedback| {
                                feedback_map.insert(base_feedback.robot_id, Feedback {
                                    has_ball: base_feedback.ir,
                                    voltage: base_feedback.voltage,
                                });
                                feedback_map
                            })
                    }
                    Err(error) => {
                        error!("Couldn't decode packet : {}", error);
                        Default::default()
                    }
                };
                r
            }
            Err(the_error) => {
                error!("{}", the_error);
                Default::default()
            }
        }
    }
}
