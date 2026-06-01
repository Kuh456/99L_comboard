use core::{fmt::Write, sync::atomic::Ordering};

use embassy_futures::select::{Either, select};
use embassy_time::{Delay, Duration, Instant, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use esp_hal::{Async, gpio::Output, rtc_cntl::Rtc, spi::master::Spi};
use heapless::String;

use crate::{
    constants::BUF_SIZE,
    state::{HAS_UNFLUSHED_DATA, IS_LOGGING, PAYLOAD_MUTEX, RAW_GNSS_CHANNEL},
};

type SdSpiDevice = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
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
    let mut raw_buffer = [0u8; BUF_SIZE];
    let mut tlm_buffer = [0u8; BUF_SIZE];
    let mut raw_cursor = 0;
    let mut tlm_cursor = 0;

    let volume0 = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            esp_println::println!("SD Volume Error: {:?}", e);
            return;
        }
    };
    let root_dir = match volume0.open_root_dir() {
        Ok(d) => d,
        Err(e) => {
            esp_println::println!("SD Root Dir Error: {:?}", e);
            return;
        }
    };

    let raw_gnss_file = match root_dir.open_file_in_dir(
        "GNSS_RAW.TXT",
        embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
    ) {
        Ok(f) => f,
        Err(e) => {
            esp_println::println!("SD File Error (GNSS): {:?}", e);
            return;
        }
    };
    let _ = raw_gnss_file.seek_from_end(0);

    let tlm_file =
        match root_dir.open_file_in_dir("TLM.CSV", embedded_sdmmc::Mode::ReadWriteCreateOrAppend) {
            Ok(f) => f,
            Err(e) => {
                esp_println::println!("SD File Error (TLM): {:?}", e);
                return;
            }
        };

    if tlm_file.length() == 0 {
        let header = b"Time_ms,Status,Lat,Long,Height10m,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Press1,Press2,Press3\n";
        let _ = tlm_file.write(header);
        let _ = tlm_file.flush();
    } else {
        let _ = tlm_file.seek_from_end(0);
    }

    let mut last_flush = Instant::now();
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut prev_is_logging = false;

    loop {
        let is_logging = IS_LOGGING.load(Ordering::Relaxed);

        if !is_logging && prev_is_logging {
            if raw_cursor > 0 && raw_gnss_file.write(&raw_buffer[..raw_cursor]).is_ok() {
                raw_cursor = 0;
            }
            if tlm_cursor > 0 && tlm_file.write(&tlm_buffer[..tlm_cursor]).is_ok() {
                tlm_cursor = 0;
            }
            let _ = raw_gnss_file.flush();
            let _ = tlm_file.flush();
            esp_println::println!("SDカードへの書き込み完了 (ロギング停止)");
        }
        prev_is_logging = is_logging;

        let has_data = raw_cursor > 0 || tlm_cursor > 0;
        HAS_UNFLUSHED_DATA.store(has_data, Ordering::Relaxed);

        match select(RAW_GNSS_CHANNEL.receive(), ticker.next()).await {
            Either::First(raw_gnss_data) => {
                if is_logging {
                    let valid_len = raw_gnss_data
                        .iter()
                        .position(|&x| x == 0)
                        .unwrap_or(raw_gnss_data.len());
                    let valid_bytes = &raw_gnss_data[..valid_len];

                    if valid_bytes.is_empty() {
                        continue;
                    }

                    if raw_cursor + valid_bytes.len() > BUF_SIZE
                        && raw_gnss_file.write(&raw_buffer[..raw_cursor]).is_ok()
                    {
                        let _ = raw_gnss_file.flush();
                        raw_cursor = 0;
                    }

                    if raw_cursor + valid_bytes.len() <= BUF_SIZE {
                        raw_buffer[raw_cursor..raw_cursor + valid_bytes.len()]
                            .copy_from_slice(valid_bytes);
                        raw_cursor += valid_bytes.len();
                    }
                }
            }
            Either::Second(_) => {
                if is_logging {
                    let payload = { *PAYLOAD_MUTEX.lock().await };
                    let now_ms = Instant::now().as_millis();

                    let mut csv_line: String<256> = String::new();
                    let _ = writeln!(
                        &mut csv_line,
                        "{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                        now_ms,
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
                        payload.air_pressure[0],
                        payload.air_pressure[1],
                        payload.air_pressure[2]
                    );

                    let line_bytes = csv_line.as_bytes();
                    if tlm_cursor + line_bytes.len() > BUF_SIZE
                        && tlm_file.write(&tlm_buffer[..tlm_cursor]).is_ok()
                    {
                        let _ = tlm_file.flush();
                        tlm_cursor = 0;
                    }

                    if tlm_cursor + line_bytes.len() <= BUF_SIZE {
                        tlm_buffer[tlm_cursor..tlm_cursor + line_bytes.len()]
                            .copy_from_slice(line_bytes);
                        tlm_cursor += line_bytes.len();
                    }
                }
            }
        }

        if last_flush.elapsed().as_secs() >= 5 {
            if raw_cursor > 0 && raw_gnss_file.write(&raw_buffer[..raw_cursor]).is_ok() {
                let _ = raw_gnss_file.flush();
                raw_cursor = 0;
            }
            if tlm_cursor > 0 {
                match tlm_file.write(&tlm_buffer[..tlm_cursor]) {
                    Ok(size) => {
                        esp_println::println!("TLMCSV: 定期フラッシュ成功 ({:?} bytes)", size);
                        let _ = tlm_file.flush();
                        tlm_cursor = 0;
                    }
                    Err(e) => esp_println::println!("TLM CSV: 定期フラッシュ失敗 {:?}", e),
                }
            }
            last_flush = Instant::now();
        }
    }
}
