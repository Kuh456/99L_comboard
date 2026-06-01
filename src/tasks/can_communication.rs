use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiFrame, StandardId},
};
use esp_println::println;

use crate::{
    constants::{
        CAN_ID_ACCELARATION, CAN_ID_AIR_PRESSURE, CAN_ID_ANGLE_SPEED, CAN_ID_LIFT_OFF,
        CAN_ID_TEST_TO_CAMERA, CAN_ID_TEST_TO_LOG_PARA, CAN_ID_TOP,
    },
    state::{CAN_TX_CHANNEL, IS_CAN_ERROR, PAYLOAD_MUTEX, TRIGGER_SIGNAL},
};

fn create_can_frame_to_send(can_id: u16, cmd: u8) -> Option<EspTwaiFrame> {
    let id = StandardId::new(can_id)?;
    EspTwaiFrame::new(id, &[cmd])
}

#[embassy_executor::task]
pub async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_secs(60));

    loop {
        match select(CAN_TX_CHANNEL.receive(), ticker.next()).await {
            Either::First((can_id, command)) => {
                if let Some(frame) = create_can_frame_to_send(can_id, command) {
                    let result = embassy_time::with_timeout(
                        Duration::from_millis(100),
                        tx.transmit_async(&frame),
                    )
                    .await;

                    if result.is_err() {
                        IS_CAN_ERROR.store(true, Ordering::Relaxed);
                    }
                } else {
                    println!("invalid CAN frame: id=0x{:03x}, cmd={}", can_id, command);
                }
            }
            Either::Second(_) => {
                if let Some(frame) = create_can_frame_to_send(CAN_ID_TEST_TO_LOG_PARA, 0) {
                    let result = embassy_time::with_timeout(
                        Duration::from_millis(100),
                        tx.transmit_async(&frame),
                    )
                    .await;

                    if result.is_err() {
                        IS_CAN_ERROR.store(true, Ordering::Relaxed);
                    }
                }

                if let Some(frame) = create_can_frame_to_send(CAN_ID_TEST_TO_CAMERA, 0) {
                    let result = embassy_time::with_timeout(
                        Duration::from_millis(100),
                        tx.transmit_async(&frame),
                    )
                    .await;

                    if result.is_err() {
                        IS_CAN_ERROR.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        match rx.receive_async().await {
            Ok(payload) => {
                IS_CAN_ERROR.store(false, Ordering::Relaxed);

                match payload.id() {
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_LIFT_OFF => {
                        let mut status_payload = PAYLOAD_MUTEX.lock().await;
                        status_payload.status = (status_payload.status & 0b1011_1111) | 0b0100_0000;
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_ANGLE_SPEED => {
                        if payload.data().len() >= 6 {
                            let mut angle_speed = [0u8; 6];
                            angle_speed.copy_from_slice(&payload.data()[0..6]);
                            let mut angle_speed_payload = PAYLOAD_MUTEX.lock().await;

                            for (i, chunk) in angle_speed.chunks_exact(2).enumerate() {
                                angle_speed_payload.angle_speed[i] =
                                    i16::from_be_bytes(chunk.try_into().unwrap());
                            }
                        }
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_ACCELARATION => {
                        if payload.data().len() >= 6 {
                            let mut acceleration = [0u8; 6];
                            acceleration.copy_from_slice(&payload.data()[0..6]);
                            let mut acceleration_payload = PAYLOAD_MUTEX.lock().await;

                            for (i, chunk) in acceleration.chunks_exact(2).enumerate() {
                                acceleration_payload.acceleration[i] =
                                    i16::from_be_bytes(chunk.try_into().unwrap());
                            }
                        }
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_AIR_PRESSURE => {
                        if payload.data().len() >= 3 {
                            let mut air_pressure = [0u8; 3];
                            air_pressure.copy_from_slice(&payload.data()[0..3]);
                            let mut air_pressure_payload = PAYLOAD_MUTEX.lock().await;
                            air_pressure_payload.air_pressure = air_pressure;
                        }
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_TOP => {
                        TRIGGER_SIGNAL.signal(true);
                        let mut status_payload = PAYLOAD_MUTEX.lock().await;
                        status_payload.status = (status_payload.status & 0b0111_1111) | 0b1000_0000;
                    }
                    _ => {}
                }
            }
            Err(_e) => {
                IS_CAN_ERROR.store(true, Ordering::Relaxed);
            }
        }
    }
}
