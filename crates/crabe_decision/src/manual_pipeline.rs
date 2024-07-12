use crabe_framework::component::{Component, DecisionComponent};
use crabe_framework::config::CommonConfig;
use crabe_framework::data::output::{Command, CommandMap};
use crabe_framework::data::tool::ToolData;
use crabe_framework::data::world::World;
use crate::pipeline::DecisionConfig;

pub struct ManualDecisionPipeline {}

impl ManualDecisionPipeline {
    pub fn with_config(_decision_config: DecisionConfig, _common_cfg: &CommonConfig) -> Self {
        Self {}
    }
}

impl Component for ManualDecisionPipeline {
    fn close(self) {}
}

impl DecisionComponent for ManualDecisionPipeline {
    fn step(&mut self, _data: &World) -> (CommandMap, ToolData) {
        let forward_command: Command = Command {
            forward_velocity: 1.0,
            ..Default::default()
        };

        let mut command_map = CommandMap::new();
        command_map.insert(3, forward_command);

        (
            command_map,
            ToolData::default()
        )
    }
}