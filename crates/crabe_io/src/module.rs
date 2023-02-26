use crate::league::game_controller::{GameController, GameControllerConfig};
use crate::league::vision::{Vision, VisionConfig};
use clap::Args;
use crabe_framework::component::Receiver;
use crabe_framework::data::output::Feedback;
use crabe_framework::data::receiver::InboundData;

// CrabeIO
#[derive(Args)]
pub struct DataReceiverConfig {
    #[arg(long)]
    gc: bool,

    #[command(flatten)]
    #[command(next_help_heading = "Vision")]
    pub vision_cfg: VisionConfig,

    #[command(flatten)]
    #[command(next_help_heading = "Game Controller")]
    pub gc_cfg: GameControllerConfig,
}

/// CrabeIO
pub trait ReceiverTask {
    fn fetch(&mut self, input: &mut InboundData);
    fn close(&mut self);
}

/// CrabeIO
pub struct DataReceiverPipeline {
    receivers: Vec<Box<dyn ReceiverTask>>,
}

/// CrabeIO
impl DataReceiverPipeline {
    pub fn with_config(config: DataReceiverConfig) -> Self {
        let mut tasks: Vec<Box<dyn ReceiverTask>> =
            vec![Vision::with_config_boxed(config.vision_cfg)];

        if config.gc {
            tasks.push(GameController::with_config_boxed(config.gc_cfg));
        }

        Self {
            receivers: tasks, // How to box ?
        }
    }
}

impl Receiver for DataReceiverPipeline {
    fn step(&mut self, _feedback: &mut Feedback) -> InboundData {
        let mut data = InboundData::default();
        self.receivers.iter_mut().for_each(|x| x.fetch(&mut data));
        data
    }

    fn close(&mut self) {
        self.receivers.iter_mut().for_each(|x| x.close());
    }
}