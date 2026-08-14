use chrono::{DateTime, Utc};
use config::{Environment, File};
use core::net::Ipv4Addr;
use log::{debug, error, info, trace};
use mavlink::{AsyncMavConnection, dialects::ardupilotmega};
use num_traits::cast::ToPrimitive;
use serde::Deserialize;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

// a mavlink router won't send data over udp if the port hasn't been sent data
// so this will ping the router a handfull of times
async fn wakeup_route(
    conn: &(dyn AsyncMavConnection<ardupilotmega::MavMessage> + Send + Sync),
) -> Result<(), Box<dyn std::error::Error>> {
    let heartbeat = ardupilotmega::MavMessage::HEARTBEAT(ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: ardupilotmega::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: ardupilotmega::MavModeFlag::empty(),
        system_status: ardupilotmega::MavState::MAV_STATE_BOOT,
        mavlink_version: 3,
    });

    for _ in 0..5 {
        conn.send(&mavlink::MavHeader::default(), &heartbeat)
            .await?;
        match tokio::time::timeout(core::time::Duration::from_secs(1), conn.recv()).await {
            Ok(Ok((header, _payload))) => {
                info!(
                    "Port is alive (system_id = {}, component_id = {})",
                    header.system_id, header.component_id
                );
                return Ok(());
            }
            Ok(Err(e)) => {
                error!("Error receiving data: {e}");
            }
            Err(_) => {
                trace!("Timeout while waking port up");
            }
        }
    }
    Ok(())
}

#[derive(Debug)]
struct EventDetails {
    latitude: f64,
    longitude: f64,
    altitude: f32,       // hae
    course: f32,         // 0-360 degrees
    accuracy_horiz: f32, // m
    accuracy_vert: f32,  // m
    speed: f32,          // 3d speed, m/s
    slope: f32,          // degrees
    when: DateTime<Utc>,
}

#[derive(Debug, Deserialize)]
struct VehicleConfig {
    uid: String,
    cot_type: String,
    // vehicles must report a cylinder they occupy
    radius: f32,           // radius in meters of the space the object occupies
    height: f32,           // height in meters of the space the object occupies
    error_confidence: f32, // scalar that can be applied to reported errors, allows to account for error in the fusing process
    sys_id: u8,
    comp_id: u8,
}

#[derive(Debug, Deserialize)]
struct Config {
    cot_port: u16,
    cot_ip: String,
    mavlink: String,
    cot_rate: f32,
    vehicle: VehicleConfig,
}

struct TimestampOffset {
    unix_usec: u64,
    boot_ms: u32,
}

impl TimestampOffset {
    fn new(unix_usec: u64, boot_ms: u32) -> Self {
        Self { boot_ms, unix_usec }
    }

    fn compute_current_unix(&self, boot_ms: u32) -> u64 {
        self.unix_usec + ((boot_ms - self.boot_ms) as u64 * 1000)
    }
}

async fn send_report(
    socket: &UdpSocket,
    config: &VehicleConfig,
    details: EventDetails,
) -> Result<(), std::io::Error> {
    let timestamp_format = "%Y-%m-%dT%H:%M:%S.%fZ";
    let uid = &config.uid;
    let cot_type = &config.cot_type;
    let how = "m-f"; // machine fused
    let lat = details.latitude;
    let lon = details.longitude;
    let hae = details.altitude;
    let course = details.course;
    let speed = details.speed;
    let slope = details.slope;
    let timestamp = details.when.format(timestamp_format);
    let timestamp_stale = (details.when + Duration::from_secs(2)).format(timestamp_format);
    let ce = details
        .accuracy_horiz
        .mul_add(config.error_confidence, config.radius);
    let le = details
        .accuracy_vert
        .mul_add(config.error_confidence, config.height * 0.5f32);
    let output = format!(
        "<?xml_version='1.0' encoding='utf-8'?><event uid=\"{uid}\" type=\"{cot_type}\" how=\"{how}\" time=\"{timestamp}\" start=\"{timestamp}\" stale=\"{timestamp_stale}\"><point lat=\"{lat}\" lon=\"{lon}\" hae=\"{hae}\" ce=\"{ce}\" le=\"{le}\" /><detail><track course=\"{course}\" speed=\"{speed}\" slope=\"{slope}\"/></detail></event>"
    );
    debug!("Sending on {socket:?} {output}");
    socket.send(output.as_bytes()).await?;
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let s = config::Config::builder()
        .add_source(File::with_name("/etc/mavlink-cot.toml").required(false))
        .add_source(File::with_name("config.toml").required(false))
        .add_source(Environment::with_prefix("MAVLINK_TO_COT"))
        .set_default("vehicle.error_confidence", "1.1")?
        .build()?;

    let config: Config = s.try_deserialize()?;

    debug!("Loaded config {config:?}");

    #[cfg(feature = "egm2008")]
    info!("Using the egm2008 model");
    #[cfg(feature = "egm96")]
    info!("Using the egm96 model");

    let address: Ipv4Addr = config.cot_ip.parse()?;
    let port = config.cot_port;
    info!("Joining multicast group: {address}:{port}");
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
    info!("socket is {socket:?}");
    socket.connect((address, port)).await?;
    socket.join_multicast_v4(address, Ipv4Addr::UNSPECIFIED)?;

    let connection_str = &config.mavlink;
    info!("Opening MAVLink connection to {connection_str}");
    let conn = mavlink::connect_async::<ardupilotmega::MavMessage>(connection_str).await?;

    let update_delay = Duration::from_millis((1000f32 / config.cot_rate) as u64);

    wakeup_route(&*conn).await?;

    // quasi reasonable default guesses for accuracy
    let mut h_acc = 20f32;
    let mut v_acc = 50f32;

    let mut last_send = Instant::now();
    let mut timestamps: Option<TimestampOffset> = None;

    // core loop
    loop {
        let (header, msg) = conn.recv().await?;

        // filter out messages from anything but the desired vehicle
        if header.component_id != config.vehicle.comp_id
            || header.system_id != config.vehicle.sys_id
        {
            continue;
        }

        // handle the data
        match msg {
            ardupilotmega::MavMessage::GPS_RAW_INT(data) => {
                trace!("{data:?}");
                if data.fix_type.to_i32().unwrap()
                    >= ardupilotmega::GpsFixType::GPS_FIX_TYPE_2D_FIX
                        .to_i32()
                        .unwrap()
                {
                    if data.h_acc > 0 {
                        h_acc = data.h_acc as f32 * 1e-3;
                    }
                    if data.v_acc > 0 {
                        v_acc = data.v_acc as f32 * 1e-3;
                    }
                }
            }
            ardupilotmega::MavMessage::GLOBAL_POSITION_INT(data) => {
                trace!("{data:?}");

                let when = timestamps.as_ref().and_then(|ts| {
                    DateTime::from_timestamp_micros(
                        ts.compute_current_unix(data.time_boot_ms) as i64
                    )
                });

                if when.is_none() {
                    continue;
                }

                if last_send.elapsed() < update_delay {
                    trace!("dropped data due to being stale");
                    continue;
                }

                last_send = Instant::now();

                let latitude = data.lat as f64 * 1e-7;
                let longitude = data.lon as f64 * 1e-7;
                #[cfg(feature = "egm2008")]
                let undulation = egm2008::geoid_height(latitude as f32, longitude as f32)?;
                #[cfg(feature = "egm96")]
                let undulation = egm96::egm96_compute_altitude_offset(latitude, longitude) as f32;
                let speed = f32::sqrt(
                    (data.vx as f32).powi(2) + (data.vy as f32).powi(2) + (data.vz as f32).powi(2),
                ) * 1e-2f32;
                let course = {
                    let raw = f32::atan2(data.vy as f32, data.vx as f32).to_degrees();
                    if raw < 0f32 { raw + 180f32 } else { raw }
                };
                let slope = f32::atan2(
                    data.vz as f32,
                    f32::sqrt((data.vx as f32).powi(2) + (data.vy as f32).powi(2)),
                )
                .to_degrees()
                .clamp(-90f32, 90f32);
                trace!("Undulation is {undulation} at ({latitude}, {longitude})");
                let event = EventDetails {
                    latitude,
                    longitude,
                    altitude: data.alt as f32 * 1e-3 + undulation,
                    accuracy_horiz: h_acc,
                    accuracy_vert: v_acc,
                    speed,
                    course,
                    slope,
                    when: when.unwrap(),
                };
                send_report(&socket, &config.vehicle, event).await?;
            }
            ardupilotmega::MavMessage::SYSTEM_TIME(data) => {
                trace!("{data:?}");
                timestamps = Some(TimestampOffset::new(data.time_unix_usec, data.time_boot_ms));
            }
            _ => {}
        }
    }
}
