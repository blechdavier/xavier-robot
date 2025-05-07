use std::{future::IntoFuture, sync::{Arc, Mutex}};

use socketioxide::{extract::{Data, SocketRef, State}, SocketIo, SocketIoBuilder};
use tower_http::services::{ServeDir, ServeFile};

use crate::geometry::Twist2d;

pub async fn start_web_server_thread() -> (WebsocketState, SocketIo) {
    let state = WebsocketState::new();
    let (layer, io) = SocketIoBuilder::new().with_state(state.clone()).build_layer();
    io.ns("/", handler);

    let app = axum::Router::new()
        .route_service("/", ServeFile::new("/home/blech/Documents/GitHub/xavier-robot/frontend/dist/index.html"))
        .route_service("/{*wildcard}", ServeDir::new("/home/blech/Documents/GitHub/xavier-robot/frontend/dist"))
        .layer(layer);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    tokio::spawn(axum::serve(listener, app).into_future());
    (state, io)
}

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