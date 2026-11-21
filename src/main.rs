use chrono::DateTime;
use config::{Environment, File};
use core::net::Ipv4Addr;
use log::{debug, error, info, trace};
use mavlink::{AsyncMavConnection, dialects::ardupilotmega};
use num_traits::cast::ToPrimitive;
use serde::Deserialize;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

const CONFIG_ETC: &str = "/etc/mavlink-to-cot.toml";
const CONFIG_LOCAL: &str = "./config.toml";
const ENV_PREFIX: &str = "MAVLINK_TO_COT";

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

#[derive(Debug, Deserialize)]
struct VehicleConfig {
    uid: String,
    cot_type: Option<String>, // derived from HEARTBEAT MAV_TYPE when unset
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

fn build_event(
    config: &VehicleConfig,
    pos: &ardupilotmega::GLOBAL_POSITION_INT_DATA,
    mav_type: Option<u8>,
    when_us: u64,
    undulation: f32,
    h_acc: f32,
    v_acc: f32,
) -> Option<uas_cot::Event> {
    let gpi = uas_cot::mavlink::GlobalPositionInt {
        time_boot_ms: pos.time_boot_ms,
        lat: pos.lat,
        lon: pos.lon,
        alt: pos.alt,
        relative_alt: pos.relative_alt,
        vx: pos.vx,
        vy: pos.vy,
        vz: pos.vz,
        hdg: pos.hdg,
    };
    let opts = uas_cot::mavlink::Options {
        stale_secs: 2,
        ..Default::default()
    };
    let mut event = uas_cot::mavlink::vehicle(config.sys_id, &gpi, mav_type, when_us, &opts)?;
    event.uid = config.uid.clone();
    event.callsign = Some(config.uid.clone());
    if let Some(cot_type) = &config.cot_type {
        event.kind = cot_type.clone();
    }
    event.how = "m-f".into(); // machine fused
    event.precision_location = None;
    event.point.hae += f64::from(undulation);
    event.point.ce = f64::from(h_acc.mul_add(config.error_confidence, config.radius));
    event.point.le = f64::from(v_acc.mul_add(config.error_confidence, config.height * 0.5f32));
    Some(event)
}

fn sandbox_prep() -> Result<(), Box<dyn std::error::Error>> {
    // use landlock under linux
    #[cfg(target_os = "linux")]
    {
        use landlock::{
            ABI, Access, AccessFs, CompatLevel, Compatible, PathBeneath, PathFd, RestrictSelfAttr,
            Ruleset, RulesetAttr, RulesetCreatedAttr,
        };
        let mut rules = Ruleset::default()
            .set_compatibility(CompatLevel::BestEffort)
            .handle_access(AccessFs::from_all(ABI::V9))?
            .create()?
            .all_threads(true)?;
        if let Ok(fd) = PathFd::new(CONFIG_ETC) {
            trace!("Adding /etc to landlock");
            rules = rules.add_rule(PathBeneath::new(fd, AccessFs::ReadFile))?;
        }
        if let Ok(fd) = PathFd::new(CONFIG_LOCAL) {
            trace!("Adding local config to landlock");
            rules = rules.add_rule(PathBeneath::new(fd, AccessFs::ReadFile))?;
        }
        rules.restrict_self()?;
    }
    Ok(())
}

fn sandbox_tighten() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(target_os = "linux")]
    {
        use landlock::{
            ABI, Access, AccessFs, AccessNet, CompatLevel, Compatible, RestrictSelfAttr, Ruleset,
            RulesetAttr,
        };
        Ruleset::default()
            .set_compatibility(CompatLevel::BestEffort)
            .handle_access(AccessFs::from_all(ABI::V9))?
            .handle_access(AccessNet::from_all(ABI::V9))?
            .create()?
            .all_threads(true)?
            .restrict_self()?;
    }
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let env = env_logger::Env::default()
        .filter_or(ENV_PREFIX.to_owned() + "_LOG", "info");
    env_logger::init_from_env(env);

    sandbox_prep()?;

    let s = config::Config::builder()
        .add_source(File::with_name(CONFIG_ETC).required(false))
        .add_source(File::with_name(CONFIG_LOCAL).required(false))
        .add_source(Environment::with_prefix(ENV_PREFIX).separator("_"))
        .set_default("vehicle.sys_id", "1")?
        .set_default("vehicle.comp_id", "1")?
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

    sandbox_tighten()?;

    let update_delay = Duration::from_millis((1000f32 / config.cot_rate) as u64);

    wakeup_route(&*conn).await?;

    // quasi reasonable default guesses for accuracy
    let mut h_acc = 20f32;
    let mut v_acc = 50f32;

    let mut last_send = Instant::now();
    let mut timestamps: Option<TimestampOffset> = None;
    let mut mav_type: Option<u8> = None;

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
                trace!("Undulation is {undulation} at ({latitude}, {longitude})");
                let when_us = when.unwrap().timestamp_micros() as u64;
                if let Some(event) = build_event(
                    &config.vehicle,
                    &data,
                    mav_type,
                    when_us,
                    undulation,
                    h_acc,
                    v_acc,
                ) {
                    let output = event.to_xml();
                    debug!("Sending on {socket:?} {output}");
                    socket.send(output.as_bytes()).await?;
                }
            }
            ardupilotmega::MavMessage::HEARTBEAT(data) => {
                mav_type = data.mavtype.to_u8();
            }
            ardupilotmega::MavMessage::SYSTEM_TIME(data) => {
                trace!("{data:?}");
                timestamps = Some(TimestampOffset::new(data.time_unix_usec, data.time_boot_ms));
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vehicle(cot_type: Option<&str>) -> VehicleConfig {
        VehicleConfig {
            uid: "UAS <1> & \"co\"".into(),
            cot_type: cot_type.map(Into::into),
            radius: 3.0,
            height: 1.0,
            error_confidence: 1.1,
            sys_id: 1,
            comp_id: 1,
        }
    }

    fn westbound() -> ardupilotmega::GLOBAL_POSITION_INT_DATA {
        ardupilotmega::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 0,
            lat: 473_977_418,
            lon: 85_455_939,
            alt: 488_000,
            relative_alt: 10_000,
            vx: 0,
            vy: -500,
            vz: 0,
            hdg: u16::MAX,
        }
    }

    #[test]
    fn emits_well_formed_cot() {
        let quad = Some(ardupilotmega::MavType::MAV_TYPE_QUADROTOR.to_u8().unwrap());
        let event = build_event(
            &vehicle(None),
            &westbound(),
            quad,
            1_700_000_000_000_000,
            47.5,
            2.0,
            4.0,
        )
        .unwrap();
        let xml = event.to_xml();
        assert!(
            xml.starts_with("<?xml version=\"1.0\" encoding=\"UTF-8\"?><event version=\"2.0\" "),
            "{xml}"
        );
        assert!(
            xml.contains("uid=\"UAS &lt;1&gt; &amp; &quot;co&quot;\""),
            "{xml}"
        );
        assert!(xml.contains("type=\"a-f-A-M-F-Q\" how=\"m-f\""), "{xml}");
        assert!(xml.contains("hae=\"535.5\" ce=\"5.2\" le=\"4.9\""), "{xml}");
        assert!(
            xml.contains("<track course=\"270.0\" speed=\"5.00\"/>"),
            "{xml}"
        );
    }

    #[test]
    fn cot_type_follows_mav_type_unless_configured() {
        let rover = Some(
            ardupilotmega::MavType::MAV_TYPE_GROUND_ROVER
                .to_u8()
                .unwrap(),
        );
        let derived = build_event(&vehicle(None), &westbound(), rover, 0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(derived.kind, "a-f-G-E-V");
        let fixed = build_event(
            &vehicle(Some("a-f-A-M-F-Q")),
            &westbound(),
            rover,
            0,
            0.0,
            1.0,
            1.0,
        )
        .unwrap();
        assert_eq!(fixed.kind, "a-f-A-M-F-Q");
    }
}
