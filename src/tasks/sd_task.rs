use core::{fmt::Write, sync::atomic::Ordering};

use embassy_time::{Delay, Duration, Instant, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{Mode, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use esp_hal::{Blocking, gpio::Output, rtc_cntl::Rtc, spi::master::Spi};
use heapless::String;

use crate::{
    constants::{BUF_SIZE, SD_FLUSH_INTERVAL_SECS, SD_LOG_INTERVAL_MS},
    state::{HAS_UNFLUSHED_DATA, IS_LOGGING, PAYLOAD_MUTEX},
};

type SdSpiDevice = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;
type SdBlockDevice = SdCard<SdSpiDevice, Delay>;
pub type SdVolumeManager = VolumeManager<SdBlockDevice, SdTimeSource>;

pub struct SdTimeSource {
    timer: Rtc<'static>,
}

impl SdTimeSource {
    pub fn new(timer: Rtc<'static>) -> Self {
        Self { timer }
    }

    fn current_time(&self) -> u64 {
        // Unless the RTC has been set from an absolute Unix time source, FAT
        // timestamps are only relative to boot and do not represent UTC.
        self.timer.current_time_us()
    }
}

static TZ: jiff::tz::TimeZone = jiff::tz::get!("UTC");

impl TimeSource for SdTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        let now_us = self.current_time();
        let now = jiff::Timestamp::from_microsecond(now_us as i64)
            .unwrap_or_else(|_| jiff::Timestamp::from_second(0).unwrap());
        let now = now.to_zoned(TZ.clone());

        Timestamp {
            year_since_1970: (now.year() - 1970).unsigned_abs() as u8,
            zero_indexed_month: now.month().wrapping_sub(1) as u8,
            zero_indexed_day: now.day().wrapping_sub(1) as u8,
            hours: now.hour() as u8,
            minutes: now.minute() as u8,
            seconds: now.second() as u8,
        }
    }
}

#[embassy_executor::task]
pub async fn sd_write_task(volume_mgr: &'static mut SdVolumeManager) {
    let mut tlm_buffer = [0u8; BUF_SIZE];
    let mut tlm_cursor = 0usize;
    let mut bytes_since_flush = 0usize;
    let mut needs_flush = false;

    let volume0 = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(e) => {
            esp_println::println!("SD open_volume error: {:?}", e);
            return;
        }
    };
    let root_dir = match volume0.open_root_dir() {
        Ok(dir) => dir,
        Err(e) => {
            esp_println::println!("SD open_root_dir error: {:?}", e);
            return;
        }
    };
    let tlm_file = match root_dir.open_file_in_dir("TLM.CSV", Mode::ReadWriteCreateOrAppend) {
        Ok(file) => {
            esp_println::println!("TLM.CSV opened");
            file
        }
        Err(e) => {
            esp_println::println!("SD open_file_in_dir TLM.CSV error: {:?}", e);
            return;
        }
    };

    if tlm_file.length() == 0 {
        let header = b"Time_ms,Status,Lat,Long,Height10m,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,IntegratedAngleX,IntegratedAngleY,IntegratedAngleZ,Press1,Press2,Press3,AirSpeed,FinAngle\n";
        match tlm_file.write(header) {
            Ok(()) => {}
            Err(e) => {
                esp_println::println!("SD TLM header write error: {:?}", e);
                return;
            }
        }
        match tlm_file.flush() {
            Ok(()) => esp_println::println!("TLM header written"),
            Err(e) => {
                esp_println::println!("SD TLM header flush error: {:?}", e);
                return;
            }
        }
    } else if let Err(e) = tlm_file.seek_from_end(0) {
        esp_println::println!("SD TLM seek_from_end error: {:?}", e);
        return;
    }

    let mut ticker = Ticker::every(Duration::from_millis(SD_LOG_INTERVAL_MS));
    let mut last_flush = Instant::now();
    let mut prev_is_logging = false;
    let mut stop_flush_pending = false;

    loop {
        ticker.next().await;

        let is_logging = IS_LOGGING.load(Ordering::Relaxed);
        if is_logging && !prev_is_logging {
            stop_flush_pending = false;
            esp_println::println!("SD logging started");
        } else if !is_logging && prev_is_logging {
            stop_flush_pending = true;
        }
        prev_is_logging = is_logging;

        if is_logging {
            let payload = { *PAYLOAD_MUTEX.lock().await };
            let mut csv_line: String<256> = String::new();
            if writeln!(
                &mut csv_line,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                Instant::now().as_millis(),
                payload.status,
                payload.gnss_lat,
                payload.gnss_long,
                payload.gnss_height,
                payload.angle_speed[0],
                payload.angle_speed[1],
                payload.angle_speed[2],
                payload.acceleration[0],
                payload.acceleration[1],
                payload.acceleration[2],
                payload.integrated_angle[0],
                payload.integrated_angle[1],
                payload.integrated_angle[2],
                payload.air_pressure[0],
                payload.air_pressure[1],
                payload.air_pressure[2],
                payload.air_speed,
                payload.fin_angle,
            )
            .is_err()
            {
                esp_println::println!("TLM CSV line formatting overflow");
            } else {
                let line_bytes = csv_line.as_bytes();

                if tlm_cursor + line_bytes.len() > BUF_SIZE && tlm_cursor > 0 {
                    match tlm_file.write(&tlm_buffer[..tlm_cursor]) {
                        Ok(()) => {
                            bytes_since_flush += tlm_cursor;
                            tlm_cursor = 0;
                            needs_flush = true;
                        }
                        Err(e) => {
                            esp_println::println!("SD TLM buffer-full write error: {:?}", e);
                        }
                    }
                }

                if tlm_cursor + line_bytes.len() <= BUF_SIZE {
                    tlm_buffer[tlm_cursor..tlm_cursor + line_bytes.len()]
                        .copy_from_slice(line_bytes);
                    tlm_cursor += line_bytes.len();
                } else {
                    esp_println::println!(
                        "TLM row skipped: RAM buffer full ({}/{} bytes)",
                        tlm_cursor,
                        BUF_SIZE
                    );
                }
            }
        }

        let periodic_flush_due =
            is_logging && last_flush.elapsed().as_secs() >= SD_FLUSH_INTERVAL_SECS;
        if periodic_flush_due || stop_flush_pending {
            if tlm_cursor > 0 {
                match tlm_file.write(&tlm_buffer[..tlm_cursor]) {
                    Ok(()) => {
                        bytes_since_flush += tlm_cursor;
                        tlm_cursor = 0;
                        needs_flush = true;
                    }
                    Err(e) => esp_println::println!("SD TLM write error: {:?}", e),
                }
            }

            if needs_flush || (stop_flush_pending && tlm_cursor == 0) {
                match tlm_file.flush() {
                    Ok(()) => {
                        esp_println::println!("TLM flush success: {} bytes", bytes_since_flush);
                        bytes_since_flush = 0;
                        needs_flush = false;
                        last_flush = Instant::now();

                        if stop_flush_pending && tlm_cursor == 0 {
                            stop_flush_pending = false;
                            esp_println::println!("SD logging stopped and flushed");
                        }
                    }
                    Err(e) => esp_println::println!("SD TLM flush error: {:?}", e),
                }
            } else if periodic_flush_due {
                last_flush = Instant::now();
            }
        }

        HAS_UNFLUSHED_DATA.store(tlm_cursor > 0, Ordering::Relaxed);
    }
}
