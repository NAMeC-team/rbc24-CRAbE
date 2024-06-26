// data type to share information from the bot to the manager 
#[derive(Clone)]
pub enum Message {
    WantToGoRight = 0,
    WantToGoLeft = 1,
    WantToBeAligned = 2,
}

// data type to share information from the bot to the manager
#[derive(Clone)]
pub struct MessageData {
    pub message: Message,
    pub id: u8,
}

impl MessageData {
    pub fn new(message: Message, id: u8) -> Self {
        Self { message, id }
    }
}
