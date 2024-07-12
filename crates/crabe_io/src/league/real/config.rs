use clap::Args;
#[derive(Args)]
pub struct RealConfig {
    #[arg(long, default_value = "/dev/ttyUSB0")]
    pub usb_port: String,
    #[arg(long, default_value_t = 460_800)]
    pub usb_baud: u32,
}
