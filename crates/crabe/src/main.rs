use clap::Parser;
use crabe_filter::{FilterConfig, FilterPipeline};
use crabe_framework::component::{FilterComponent, InputComponent, ToolComponent};
use crabe_framework::config::CommonConfig;
use crabe_framework::data::output::FeedbackMap;
use crabe_framework::data::world::World;
use crabe_io::module::{InputConfig, InputPipeline};
use env_logger::Env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use crabe_framework::data::tool::ToolData;
use crabe_tool::CrabeTool;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
pub struct Cli {
    #[command(flatten)]
    #[command(next_help_heading = "Common")]
    pub common: CommonConfig,

    #[command(flatten)]
    #[command(next_help_heading = "Input")]
    pub input_config: InputConfig,

    #[command(flatten)]
    #[command(next_help_heading = "Filter")]
    pub filter_config: FilterConfig,
}

pub struct System {
    input_component: Box<dyn InputComponent>,
    filter_component: Box<dyn FilterComponent>,
    tool_component: Box<dyn ToolComponent>,
    running: Arc<AtomicBool>,
}

impl System {
    pub fn new(
        input_component: impl InputComponent + 'static,
        filter_component: impl FilterComponent + 'static,
        tool_component: impl ToolComponent + 'static
    ) -> Self {
        let running = Arc::new(AtomicBool::new(true));
        let running_ctrlc = Arc::clone(&running);

        ctrlc::set_handler(move || {
            running_ctrlc.store(false, Ordering::Relaxed);
        })
        .expect("Failed to set Ctrl-C handler");

        Self {
            input_component: Box::new(input_component),
            filter_component: Box::new(filter_component),
            tool_component: Box::new(tool_component),
            running,
        }
    }

    // TODO: Use refresh rate
    pub fn run(&mut self, _refresh_rate: Duration) {
        let mut feedback: FeedbackMap = Default::default();

        let mut world: World = World::default();

        while self.running.load(Ordering::SeqCst) {
            let receive_data = self.input_component.step(&mut feedback);
            let _world = self.filter_component.step(receive_data, &mut world);
            //dbg!(&world);
            let mut tool_data = ToolData {};
            self.tool_component.step(&mut world, &mut tool_data);
            thread::sleep(_refresh_rate);
        }
    }

    pub fn close(&mut self) {
        self.input_component.close();
        self.filter_component.close();
    }
}

fn main() {
    let cli = Cli::parse();
    let env = Env::default()
        .filter_or("CRABE_LOG_LEVEL", "debug")
        .write_style_or("CRABE_LOG_STYLE", "always");
    env_logger::init_from_env(env);

    // DecisionPipeline
    // ToolsPipeline
    // GuardPipeline
    // OutputPipeline

    let mut system = System::new(
        InputPipeline::with_config(cli.input_config, &cli.common),
        FilterPipeline::with_config(cli.filter_config, &cli.common),
        CrabeTool::with_config(), // TODO: Config
    );
    system.run(Duration::from_millis(16));
    system.close();
}
