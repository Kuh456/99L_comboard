use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use esp_hal::{
    Async,
    gpio::Output,
    uart::{Config as UartConfig, Uart},
};
use esp_println::println;

use crate::{
    gnss::{FixQuality, gnss_setting, parse_gga},
    payload::{decode_height_10m_i16, encode_height_10m_i16},
    state::{
        GNSS_CHANNEL, GNSS_CHANNEL_DROP_COUNT, GNSS_CMD_CHANNEL, GNSS_RX_ERROR_COUNT,
        GNSS_SETTING_ERROR_COUNT, GnssCommand, PAYLOAD_MUTEX,
    },
};

#[embassy_executor::task]
pub async fn gnss_manager_task(mut uart: Uart<'static, Async>, mut gnss_en: Output<'static>) {
    let mut read_buf = [0u8; 90];
    let mut line_buf = [0u8; 90];
    let mut line_len = 0;
    let mut discard_line = false;
    let mut is_on = false;

    loop {
        match select(uart.read_async(&mut read_buf), GNSS_CMD_CHANNEL.receive()).await {
            Either::First(Ok(bytes_read)) => {
                if bytes_read == 0 {
                    continue;
                }

                for &letter in &read_buf[..bytes_read] {
                    if letter == b'$' {
                        line_len = 0;
                        discard_line = false;
                        line_buf[line_len] = letter;
                        line_len += 1;
                        continue;
                    }

                    if letter == b'\r' {
                        continue;
                    }

                    if letter == b'\n' {
                        if !discard_line && line_len > 0 && line_buf[0] == b'$' {
                            let mut send_buf = [0u8; 90];
                            send_buf[..line_len].copy_from_slice(&line_buf[..line_len]);

                            if GNSS_CHANNEL.try_send(send_buf).is_err() {
                                GNSS_CHANNEL_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
                                if GNSS_CHANNEL.try_receive().is_err()
                                    || GNSS_CHANNEL.try_send(send_buf).is_err()
                                {
                                    GNSS_CHANNEL_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }
                        line_len = 0;
                        discard_line = false;
                        continue;
                    }

                    if line_len < line_buf.len() {
                        line_buf[line_len] = letter;
                        line_len += 1;
                    } else {
                        discard_line = true;
                    }
                }
            }
            Either::First(Err(e)) => {
                GNSS_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                println!("UART receive error{:?}", e);
            }
            Either::Second(cmd) => match cmd {
                GnssCommand::TurnOn => {
                    if is_on {
                        continue;
                    }

                    gnss_en.set_high();

                    let config_9600 = UartConfig::default().with_baudrate(9600);
                    if let Err(e) = uart.apply_config(&config_9600) {
                        println!("UART config error (9600baud rate): {:?}", e);
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        gnss_en.set_low();
                        is_on = false;
                        continue;
                    }

                    Timer::after(Duration::from_millis(500)).await;
                    if let Err(error) = gnss_setting(&mut uart).await {
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        println!("GNSS setting failed: {:?}", error);
                        gnss_en.set_low();
                        is_on = false;
                        continue;
                    }
                    Timer::after(Duration::from_millis(50)).await;

                    let config_115200 = UartConfig::default().with_baudrate(115_200);
                    if let Err(e) = uart.apply_config(&config_115200) {
                        println!("UART config error (115200baud rate): {:?}", e);
                        GNSS_SETTING_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        gnss_en.set_low();
                        is_on = false;
                        continue;
                    }
                    is_on = true;
                }
                GnssCommand::TurnOff => {
                    gnss_en.set_low();
                    is_on = false;
                }
            },
        }
    }
}

#[embassy_executor::task]
pub async fn parse_gnss_task() {
    loop {
        let sentence = GNSS_CHANNEL.receive().await;

        if let Ok(gga_data) = parse_gga(sentence.as_slice())
            && gga_data.fix_quality != FixQuality::Invalid
            && let (Some(lat), Some(lon), Some(height_msl)) =
                (gga_data.latitude, gga_data.longitude, gga_data.altitude)
        {
            let encoded_height = encode_height_10m_i16(height_msl);

            let mut payload = PAYLOAD_MUTEX.lock().await;
            payload.gnss_lat = lat;
            payload.gnss_long = lon;
            payload.gnss_height = encoded_height;

            println!(
                "GNSS Lat: {:.6} deg, Lon: {:.6} deg, Height: {:.1} m raw={} altitude_msl={:.1} m",
                lat as f32 / 10_000_000.0,
                lon as f32 / 10_000_000.0,
                decode_height_10m_i16(encoded_height),
                encoded_height,
                height_msl,
            );
        }
    }
}
