#[path = "../vision_link.rs"]
mod vision_link;

use std::{
    env,
    ffi::{c_char, c_int, c_void, CStr},
    io::{self, Write},
    net::{SocketAddr, UdpSocket},
    ptr,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use vision_link::{VisionFrameSummary, VISION_FLAG_DEPTH_VALID, VISION_FLAG_RGB_VALID};

#[repr(C)]
struct ObContext(c_void);
#[repr(C)]
struct ObDeviceList(c_void);
#[repr(C)]
struct ObDevice(c_void);
#[repr(C)]
struct ObSensorList(c_void);
#[repr(C)]
struct ObSensor(c_void);
#[repr(C)]
struct ObStreamProfileList(c_void);
#[repr(C)]
struct ObStreamProfile(c_void);
#[repr(C)]
struct ObFrame(c_void);
#[repr(C)]
struct ObError(c_void);

type ObFrameCallback = unsafe extern "C" fn(*mut ObFrame, *mut c_void);

const OB_SENSOR_DEPTH: c_int = 3;
const OB_FORMAT_ANY: c_int = 0xff;
const OB_FORMAT_Y11: c_int = 11;
const OB_WIDTH_ANY: c_int = 0;
const OB_HEIGHT_ANY: c_int = 0;
const OB_FPS_ANY: c_int = 0;

#[link(name = "OrbbecSDK")]
extern "C" {
    fn ob_get_major_version() -> c_int;
    fn ob_get_minor_version() -> c_int;
    fn ob_get_patch_version() -> c_int;
    fn ob_create_context(error: *mut *mut ObError) -> *mut ObContext;
    fn ob_delete_context(context: *mut ObContext, error: *mut *mut ObError);
    fn ob_query_device_list(context: *mut ObContext, error: *mut *mut ObError)
        -> *mut ObDeviceList;
    fn ob_device_list_device_count(list: *mut ObDeviceList, error: *mut *mut ObError) -> u32;
    fn ob_device_list_get_device_name(
        list: *mut ObDeviceList,
        index: u32,
        error: *mut *mut ObError,
    ) -> *const c_char;
    fn ob_device_list_get_device_pid(
        list: *mut ObDeviceList,
        index: u32,
        error: *mut *mut ObError,
    ) -> c_int;
    fn ob_device_list_get_device(
        list: *mut ObDeviceList,
        index: u32,
        error: *mut *mut ObError,
    ) -> *mut ObDevice;
    fn ob_delete_device_list(list: *mut ObDeviceList, error: *mut *mut ObError);
    fn ob_delete_device(device: *mut ObDevice, error: *mut *mut ObError);
    fn ob_device_get_sensor_list(
        device: *mut ObDevice,
        error: *mut *mut ObError,
    ) -> *mut ObSensorList;
    fn ob_delete_sensor_list(sensor_list: *mut ObSensorList, error: *mut *mut ObError);
    fn ob_sensor_list_get_sensor_by_type(
        sensor_list: *mut ObSensorList,
        sensor_type: c_int,
        error: *mut *mut ObError,
    ) -> *mut ObSensor;
    fn ob_delete_sensor(sensor: *mut ObSensor, error: *mut *mut ObError);
    fn ob_sensor_get_stream_profile_list(
        sensor: *mut ObSensor,
        error: *mut *mut ObError,
    ) -> *mut ObStreamProfileList;
    fn ob_stream_profile_list_get_video_stream_profile(
        profile_list: *mut ObStreamProfileList,
        width: c_int,
        height: c_int,
        format: c_int,
        fps: c_int,
        error: *mut *mut ObError,
    ) -> *mut ObStreamProfile;
    fn ob_delete_stream_profile_list(
        profile_list: *mut ObStreamProfileList,
        error: *mut *mut ObError,
    );
    fn ob_delete_stream_profile(profile: *mut ObStreamProfile, error: *mut *mut ObError);
    fn ob_video_stream_profile_width(
        profile: *mut ObStreamProfile,
        error: *mut *mut ObError,
    ) -> u32;
    fn ob_video_stream_profile_height(
        profile: *mut ObStreamProfile,
        error: *mut *mut ObError,
    ) -> u32;
    fn ob_video_stream_profile_fps(profile: *mut ObStreamProfile, error: *mut *mut ObError) -> u32;
    fn ob_sensor_start(
        sensor: *mut ObSensor,
        profile: *mut ObStreamProfile,
        callback: ObFrameCallback,
        user_data: *mut c_void,
        error: *mut *mut ObError,
    );
    fn ob_sensor_stop(sensor: *mut ObSensor, error: *mut *mut ObError);
    fn ob_frame_index(frame: *mut ObFrame, error: *mut *mut ObError) -> u64;
    fn ob_frame_data(frame: *mut ObFrame, error: *mut *mut ObError) -> *mut c_void;
    fn ob_frame_data_size(frame: *mut ObFrame, error: *mut *mut ObError) -> u32;
    fn ob_video_frame_width(frame: *mut ObFrame, error: *mut *mut ObError) -> u32;
    fn ob_video_frame_height(frame: *mut ObFrame, error: *mut *mut ObError) -> u32;
    fn ob_depth_frame_get_value_scale(frame: *mut ObFrame, error: *mut *mut ObError) -> f32;
    fn ob_delete_frame(frame: *mut ObFrame, error: *mut *mut ObError);
    fn ob_error_message(error: *mut ObError) -> *const c_char;
    fn ob_error_function(error: *mut ObError) -> *const c_char;
    fn ob_error_args(error: *mut ObError) -> *const c_char;
    fn ob_error_exception_type(error: *mut ObError) -> c_int;
    fn ob_delete_error(error: *mut ObError);
}

#[derive(Clone, Default)]
struct DepthFrame {
    width: usize,
    height: usize,
    data: Vec<u16>,
    frame_index: u64,
    scale: f32,
}

#[derive(Default)]
struct SharedDepth {
    latest: Option<DepthFrame>,
}

struct SdkError(String);

impl std::fmt::Display for SdkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::fmt::Debug for SdkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SdkError {}

#[derive(Debug)]
struct Config {
    serial: Option<String>,
    baud: u32,
    udp: Option<SocketAddr>,
    rate_hz: u32,
    rgb_width: u16,
    rgb_height: u16,
}

enum Transport {
    Serial(Box<dyn serialport::SerialPort>),
    Udp(UdpSocket, SocketAddr),
    Stdout(io::Stdout),
}

impl Transport {
    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        match self {
            Self::Serial(port) => port.write_all(bytes),
            Self::Udp(socket, target) => socket.send_to(bytes, *target).map(|_| ()),
            Self::Stdout(stdout) => {
                stdout.write_all(bytes)?;
                stdout.flush()
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = Config::from_args()?;
    let running = Arc::new(AtomicBool::new(true));
    let stop_flag = Arc::clone(&running);
    ctrlc::set_handler(move || {
        stop_flag.store(false, Ordering::SeqCst);
    })?;

    let mut transport = config.open_transport()?;
    let mut camera = DepthCamera::open()?;
    let frame_interval = Duration::from_millis(1000 / config.rate_hz.max(1) as u64);
    let mut last_send = Instant::now() - frame_interval;
    let mut last_seen_frame_index = None;
    let mut sequence = 1u32;

    eprintln!(
        "sending camera summaries at {} Hz; rgb={}x{}",
        config.rate_hz, config.rgb_width, config.rgb_height
    );

    while running.load(Ordering::SeqCst) {
        if last_send.elapsed() < frame_interval {
            thread::sleep(Duration::from_millis(2));
            continue;
        }

        let frame = camera.latest_frame();
        if let Some(frame) = frame {
            if Some(frame.frame_index) == last_seen_frame_index {
                thread::sleep(Duration::from_millis(1));
                continue;
            }
            last_seen_frame_index = Some(frame.frame_index);

            let summary = summarize_depth(&frame, sequence, config.rgb_width, config.rgb_height);
            let packet = summary.encode();
            transport.send(&packet)?;
            eprintln!(
                "seq={} frame={} depth={}x{} center={}mm mean={}mm min={}mm max={}mm",
                summary.sequence,
                frame.frame_index,
                summary.depth_width,
                summary.depth_height,
                summary.depth_center_mm,
                summary.depth_mean_mm,
                summary.depth_min_mm,
                summary.depth_max_mm
            );
            sequence = sequence.wrapping_add(1).max(1);
            last_send = Instant::now();
        } else {
            thread::sleep(Duration::from_millis(5));
        }
    }

    camera.stop();
    Ok(())
}

impl Config {
    fn from_args() -> Result<Self, Box<dyn std::error::Error>> {
        let mut serial = None;
        let mut baud = 921_600;
        let mut udp = None;
        let mut rate_hz = 10;
        let mut rgb_width = 640;
        let mut rgb_height = 480;

        let mut args = env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--serial" => serial = Some(require_value(&mut args, "--serial")?),
                "--baud" => baud = require_value(&mut args, "--baud")?.parse()?,
                "--udp" => udp = Some(require_value(&mut args, "--udp")?.parse()?),
                "--rate-hz" => rate_hz = require_value(&mut args, "--rate-hz")?.parse()?,
                "--rgb-size" => {
                    let value = require_value(&mut args, "--rgb-size")?;
                    let (width, height) = value
                        .split_once('x')
                        .ok_or("--rgb-size must look like 640x480")?;
                    rgb_width = width.parse()?;
                    rgb_height = height.parse()?;
                }
                "--no-rgb" => {
                    rgb_width = 0;
                    rgb_height = 0;
                }
                "--help" | "-h" => {
                    print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown argument: {arg}").into()),
            }
        }

        Ok(Self {
            serial,
            baud,
            udp,
            rate_hz,
            rgb_width,
            rgb_height,
        })
    }

    fn open_transport(&self) -> Result<Transport, Box<dyn std::error::Error>> {
        match (&self.serial, self.udp) {
            (Some(path), None) => {
                let port = serialport::new(path, self.baud)
                    .timeout(Duration::from_millis(50))
                    .open()?;
                Ok(Transport::Serial(port))
            }
            (None, Some(target)) => {
                let socket = UdpSocket::bind("0.0.0.0:0")?;
                Ok(Transport::Udp(socket, target))
            }
            (None, None) => Ok(Transport::Stdout(io::stdout())),
            (Some(_), Some(_)) => Err("choose only one transport: --serial or --udp".into()),
        }
    }
}

fn print_usage() {
    eprintln!(
        "Usage:
  cargo run --bin send_camera_to_robot -- --serial /dev/ttyUSB0 [--baud 921600] [--rate-hz 10]
  cargo run --bin send_camera_to_robot -- --udp 192.168.1.20:5000 [--rate-hz 10]

Options:
  --rgb-size 640x480   Include RGB stream dimensions in the summary packet
  --no-rgb             Mark RGB as unavailable
"
    );
}

fn require_value(
    args: &mut impl Iterator<Item = String>,
    name: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    args.next()
        .ok_or_else(|| format!("{name} requires a value").into())
}

struct DepthCamera {
    ctx: *mut ObContext,
    list: *mut ObDeviceList,
    dev: *mut ObDevice,
    sensors: *mut ObSensorList,
    depth_sensor: *mut ObSensor,
    profiles: *mut ObStreamProfileList,
    profile: *mut ObStreamProfile,
    shared: Arc<Mutex<SharedDepth>>,
    stopped: bool,
}

impl DepthCamera {
    fn open() -> Result<Self, Box<dyn std::error::Error>> {
        unsafe {
            eprintln!(
                "OrbbecSDK {}.{}.{}, opening depth stream",
                ob_get_major_version(),
                ob_get_minor_version(),
                ob_get_patch_version()
            );
        }

        let shared = Arc::new(Mutex::new(SharedDepth::default()));
        let mut error: *mut ObError = ptr::null_mut();
        let ctx = unsafe { ob_create_context(&mut error) };
        check_error("ob_create_context", &mut error)?;
        let list = unsafe { ob_query_device_list(ctx, &mut error) };
        check_error("ob_query_device_list", &mut error)?;
        let count = unsafe { ob_device_list_device_count(list, &mut error) };
        check_error("ob_device_list_device_count", &mut error)?;
        if count == 0 {
            return Err("No Orbbec device found".into());
        }
        let name = unsafe { cstr(ob_device_list_get_device_name(list, 0, &mut error)) };
        let pid = unsafe { ob_device_list_get_device_pid(list, 0, &mut error) };
        check_error("device list info", &mut error)?;
        eprintln!("device: {name} pid=0x{pid:04x}");

        let dev = unsafe { ob_device_list_get_device(list, 0, &mut error) };
        check_error("ob_device_list_get_device", &mut error)?;
        let sensors = unsafe { ob_device_get_sensor_list(dev, &mut error) };
        check_error("ob_device_get_sensor_list", &mut error)?;
        let depth_sensor =
            unsafe { ob_sensor_list_get_sensor_by_type(sensors, OB_SENSOR_DEPTH, &mut error) };
        check_error("ob_sensor_list_get_sensor_by_type", &mut error)?;
        if depth_sensor.is_null() {
            return Err("Depth sensor not found".into());
        }
        let profiles = unsafe { ob_sensor_get_stream_profile_list(depth_sensor, &mut error) };
        check_error("ob_sensor_get_stream_profile_list", &mut error)?;
        let mut profile = unsafe {
            ob_stream_profile_list_get_video_stream_profile(
                profiles,
                640,
                360,
                OB_FORMAT_Y11,
                30,
                &mut error,
            )
        };
        if !error.is_null() {
            unsafe { ob_delete_error(error) };
            error = ptr::null_mut();
            profile = unsafe {
                ob_stream_profile_list_get_video_stream_profile(
                    profiles,
                    OB_WIDTH_ANY,
                    OB_HEIGHT_ANY,
                    OB_FORMAT_ANY,
                    OB_FPS_ANY,
                    &mut error,
                )
            };
        }
        check_error("select depth profile", &mut error)?;
        let width = unsafe { ob_video_stream_profile_width(profile, &mut error) };
        let height = unsafe { ob_video_stream_profile_height(profile, &mut error) };
        let fps = unsafe { ob_video_stream_profile_fps(profile, &mut error) };
        check_error("stream profile info", &mut error)?;
        eprintln!("depth stream: {width}x{height}@{fps}");

        let shared_ptr = Arc::as_ptr(&shared) as *mut c_void;
        unsafe {
            ob_sensor_start(
                depth_sensor,
                profile,
                on_depth_frame,
                shared_ptr,
                &mut error,
            )
        };
        check_error("ob_sensor_start", &mut error)?;

        Ok(Self {
            ctx,
            list,
            dev,
            sensors,
            depth_sensor,
            profiles,
            profile,
            shared,
            stopped: false,
        })
    }

    fn latest_frame(&self) -> Option<DepthFrame> {
        self.shared
            .lock()
            .ok()
            .and_then(|guard| guard.latest.clone())
    }

    fn stop(&mut self) {
        if self.stopped {
            return;
        }
        self.stopped = true;
        let mut error: *mut ObError = ptr::null_mut();
        unsafe {
            ob_sensor_stop(self.depth_sensor, &mut error);
            let _ = check_error("ob_sensor_stop", &mut error);
            ob_delete_stream_profile(self.profile, &mut error);
            ob_delete_stream_profile_list(self.profiles, &mut error);
            ob_delete_sensor(self.depth_sensor, &mut error);
            ob_delete_sensor_list(self.sensors, &mut error);
            ob_delete_device(self.dev, &mut error);
            ob_delete_device_list(self.list, &mut error);
            ob_delete_context(self.ctx, &mut error);
        }
    }
}

impl Drop for DepthCamera {
    fn drop(&mut self) {
        self.stop();
    }
}

unsafe extern "C" fn on_depth_frame(frame: *mut ObFrame, user_data: *mut c_void) {
    if frame.is_null() || user_data.is_null() {
        return;
    }
    let shared = &*(user_data as *const Mutex<SharedDepth>);
    let mut error: *mut ObError = ptr::null_mut();
    let width = ob_video_frame_width(frame, &mut error) as usize;
    let height = if error.is_null() {
        ob_video_frame_height(frame, &mut error) as usize
    } else {
        0
    };
    let data_size = if error.is_null() {
        ob_frame_data_size(frame, &mut error) as usize
    } else {
        0
    };
    let data_ptr = if error.is_null() {
        ob_frame_data(frame, &mut error) as *const u16
    } else {
        ptr::null()
    };

    if error.is_null()
        && !data_ptr.is_null()
        && data_size >= width * height * std::mem::size_of::<u16>()
    {
        let pixels = std::slice::from_raw_parts(data_ptr, width * height);
        let depth = DepthFrame {
            width,
            height,
            data: pixels.to_vec(),
            frame_index: ob_frame_index(frame, ptr::null_mut()),
            scale: ob_depth_frame_get_value_scale(frame, ptr::null_mut()),
        };
        if let Ok(mut guard) = shared.lock() {
            guard.latest = Some(depth);
        }
    } else if !error.is_null() {
        eprintln!("depth callback error: {}", cstr(ob_error_message(error)));
        ob_delete_error(error);
    }
    ob_delete_frame(frame, ptr::null_mut());
}

fn summarize_depth(
    frame: &DepthFrame,
    sequence: u32,
    rgb_width: u16,
    rgb_height: u16,
) -> VisionFrameSummary {
    let mut min_mm = u16::MAX;
    let mut max_mm = 0u16;
    let mut sum_mm = 0u64;
    let mut count = 0u64;
    for value in &frame.data {
        let mm = scaled_mm(*value, frame.scale);
        if mm == 0 {
            continue;
        }
        min_mm = min_mm.min(mm);
        max_mm = max_mm.max(mm);
        sum_mm += mm as u64;
        count += 1;
    }
    if count == 0 {
        min_mm = 0;
    }

    let center_idx = (frame.height / 2) * frame.width + frame.width / 2;
    let center_mm = frame
        .data
        .get(center_idx)
        .copied()
        .map(|value| scaled_mm(value, frame.scale))
        .unwrap_or(0);

    let mut flags = VISION_FLAG_DEPTH_VALID;
    if rgb_width > 0 && rgb_height > 0 {
        flags |= VISION_FLAG_RGB_VALID;
    }

    VisionFrameSummary {
        sequence,
        captured_at_ms: now_ms_u32(),
        flags,
        depth_width: frame.width.min(u16::MAX as usize) as u16,
        depth_height: frame.height.min(u16::MAX as usize) as u16,
        rgb_width,
        rgb_height,
        depth_min_mm: min_mm,
        depth_max_mm: max_mm,
        depth_center_mm: center_mm,
        depth_mean_mm: if count == 0 {
            0
        } else {
            (sum_mm / count).min(u16::MAX as u64) as u16
        },
        depth_grid_mm: depth_grid(frame),
    }
}

fn depth_grid(frame: &DepthFrame) -> [u16; 16] {
    let mut grid = [0u16; 16];
    for gy in 0..4 {
        for gx in 0..4 {
            let x0 = gx * frame.width / 4;
            let x1 = (gx + 1) * frame.width / 4;
            let y0 = gy * frame.height / 4;
            let y1 = (gy + 1) * frame.height / 4;
            let mut sum = 0u64;
            let mut count = 0u64;
            for y in y0..y1 {
                for x in x0..x1 {
                    let mm = scaled_mm(frame.data[y * frame.width + x], frame.scale);
                    if mm > 0 {
                        sum += mm as u64;
                        count += 1;
                    }
                }
            }
            grid[gy * 4 + gx] = if count == 0 {
                0
            } else {
                (sum / count).min(u16::MAX as u64) as u16
            };
        }
    }
    grid
}

fn scaled_mm(value: u16, scale: f32) -> u16 {
    if value == 0 {
        return 0;
    }
    ((value as f32) * scale).round().clamp(0.0, u16::MAX as f32) as u16
}

fn now_ms_u32() -> u32 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u32
}

fn cstr(ptr: *const c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(ptr).to_string_lossy().into_owned() }
}

fn check_error(step: &str, error: &mut *mut ObError) -> Result<(), SdkError> {
    if error.is_null() {
        return Ok(());
    }
    let message = unsafe {
        format!(
            "{} failed\n  function: {}\n  args: {}\n  message: {}\n  type: {}",
            step,
            cstr(ob_error_function(*error)),
            cstr(ob_error_args(*error)),
            cstr(ob_error_message(*error)),
            ob_error_exception_type(*error)
        )
    };
    unsafe {
        ob_delete_error(*error);
    }
    *error = ptr::null_mut();
    Err(SdkError(message))
}
