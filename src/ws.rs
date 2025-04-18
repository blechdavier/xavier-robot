use std::sync::{Arc, Mutex};

use socketioxide::extract::{Data, SocketRef, State};

use crate::geometry::Twist2d;

#[derive(Clone)]
pub struct WebsocketState {
    pub cmd_vel: Arc<Mutex<Option<Twist2d>>>
}

impl WebsocketState {
    pub fn new() -> Self {
        Self { cmd_vel: Arc::new(Mutex::new(None)) }
    }
}

pub async fn handler(socket: SocketRef, state: State<WebsocketState>) {
    println!("new connection from {}", socket.id);
    socket.on("driveWithSpeeds", move |socket: SocketRef, state: State<WebsocketState>, Data::<Vec<f64>>(data)| {
        *state.cmd_vel.lock().unwrap() = Some(Twist2d::new(data[0], data[1], data[2]));
    })
}