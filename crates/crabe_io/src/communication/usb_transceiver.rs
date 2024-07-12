use std::error::Error;
use std::io::Read;
use std::process::exit;
use crate::constant::BUFFER_SIZE;
use log::{debug, error};
use serialport::{ClearBuffer, SerialPort};
use std::time::Duration;

pub type ReadBuffer = [u8; BUFFER_SIZE];

pub struct UsbTransceiver {
    port: Box<dyn SerialPort>,
    read_buffer: ReadBuffer,
}

impl UsbTransceiver {
    pub fn new(port: &str, baud: u32, timeout: u64) -> Result<Self, serialport::Error> {
        let port = serialport::new(port, baud)
            .timeout(Duration::from_millis(timeout))
            .open()?;

        let read_buffer = [0u8; BUFFER_SIZE];

        Ok(Self { port, read_buffer })
    }

    pub fn send<T: prost::Message + Default>(&mut self, packet: T) {
        let mut buf = Vec::new();
        buf.reserve(packet.encoded_len() + 1);
        buf.push(packet.encoded_len() as u8);
        if let Err(err) = packet.encode(&mut buf) {
            error!("Cannot encode the packet, {}", err);
        }

        match self.port.write(&buf[0..packet.encoded_len() + 1]) {
            Ok(_v) => {
                debug!("sent: {:?}", packet);
            }
            Err(e) => {
                error!("send error: {}", e);
            }
        }
    }

    pub fn clear(&self, buffer_type: ClearBuffer) -> serialport::Result<()> {
        self.port.clear(buffer_type)
    }

    pub fn read(&mut self, mut actually_read_bytes: usize) -> Result<Vec<u8>, String> {
        if let Ok(_) = self.port.bytes_to_read() {
            let read_op = self.port.read(self.read_buffer.as_mut_slice());
            match read_op {
                Ok(bytes_read) => {
                    actually_read_bytes = bytes_read;
                    return if bytes_read == 0 {
                        Err(String::from("No bytes could be read from device"))
                    }
                    else {
                        if self.read_buffer[0] == 0 {
                            Err(String::from("No feedback was received from any robot, or they took too long to respond"))
                        } else {
                            let vec = self.read_buffer[1..bytes_read].to_vec();
                            Ok(vec)
                        }
                    }
                }
                Err(v) => Err(v.to_string())
            }
        } else {
            Err(String::from("No bytes available from serial"))
        }

    }

    pub fn read_proto(&mut self) {
        let mut buf = [0u8; 96];

        let bytes_read = self.port.read(&mut buf);
        if let Ok(bytes) = bytes_read {
            println!("Bytes read : {}\n", bytes);
            for i in (0..bytes).rev() {
                print!("{:02x?} ", &buf[i]);
            }
        }
        println!();
    }
}





