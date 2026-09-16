from dataclasses import dataclass
import re
import math

# EKF solution status (euler, quat and nav logs), from sbgECom
# src/logs/sbgEComLogEkf.h. The low 4 bits are the solution mode.
SBG_EKF_SOLUTION_MODE_MASK = 0xF
SBG_EKF_SOL_MODE_NAMES = {
    0: "UNINITIALIZED",
    1: "VERTICAL_GYRO",
    2: "AHRS",
    3: "NAV_VELOCITY",
    4: "NAV_POSITION",
}
SBG_EKF_SOL_ATTITUDE_VALID = 1 << 4
SBG_EKF_SOL_HEADING_VALID = 1 << 5
SBG_EKF_SOL_VELOCITY_VALID = 1 << 6
SBG_EKF_SOL_POSITION_VALID = 1 << 7
SBG_EKF_SOL_MAG_REF_USED = 1 << 9
SBG_EKF_SOL_GPS1_VEL_USED = 1 << 10
SBG_EKF_SOL_GPS1_POS_USED = 1 << 11


# Status log (sbgEComLogStatus.h): general status and aiding reception bits.
SBG_GENERAL_STATUS_BITS = {
    "main_power": 1 << 0,
    "imu_power": 1 << 1,
    "gps_power": 1 << 2,
    "settings": 1 << 3,
    "temperature": 1 << 4,
    "datalogger": 1 << 5,
    "cpu": 1 << 6,
}
SBG_AIDING_RECV_BITS = {
    "gnss1_pos": 1 << 0,
    "gnss1_vel": 1 << 1,
    "gnss1_hdt": 1 << 2,
    "gnss1_utc": 1 << 3,
    "mag": 1 << 8,
    "air_data": 1 << 13,
}


@dataclass
class DeviceStatus:
    general: int
    com: int
    com2: int
    aiding: int
    cpu_pct: int


def parse_status_line(line: str) -> DeviceStatus:
    """Parse a status console line: general, com, com2, aiding, cpu usage."""
    tokens = re.findall(r"\d+", line.split(":", 1)[1])
    if len(tokens) != 5:
        raise ValueError(f"Expected 5 status fields, found {len(tokens)}: {tokens}")
    general, com, com2, aiding, cpu = (int(t) for t in tokens)
    return DeviceStatus(general, com, com2, aiding, cpu)


def ekf_solution_mode(status: int) -> int:
    """EKF solution mode (0 uninitialized ... 4 full navigation)."""
    return status & SBG_EKF_SOLUTION_MODE_MASK


def ekf_solution_mode_name(status: int) -> str:
    mode = ekf_solution_mode(status)
    return SBG_EKF_SOL_MODE_NAMES.get(mode, f"MODE_{mode}")


def heading_valid(status: int) -> bool:
    """True when the EKF flags heading as reliable."""
    return bool(status & SBG_EKF_SOL_HEADING_VALID)


def velocity_valid(status: int) -> bool:
    return bool(status & SBG_EKF_SOL_VELOCITY_VALID)


def position_valid(status: int) -> bool:
    return bool(status & SBG_EKF_SOL_POSITION_VALID)


def yaw_rate(gyro_y: float, gyro_z: float, roll_rad: float, pitch_rad: float):
    """Yaw (heading) rate from body rates and attitude, or None near pitch +-90.

    Euler kinematics for the SBG body frame (X forward, Y right, Z down):
    yaw_rate = (q sin(roll) + r cos(roll)) / cos(pitch). Linear in the rates,
    so the result is in the unit of gyro_y / gyro_z.
    """
    cos_pitch = math.cos(pitch_rad)
    if abs(cos_pitch) < 1e-3:
        return None
    return (gyro_y * math.sin(roll_rad) + gyro_z * math.cos(roll_rad)) / cos_pitch


@dataclass
class SbgEkfNav:
    velN_mps: float
    velE_mps: float
    velD_mps: float
    velStdN_mps: float
    velStdE_mps: float
    velStdD_mps: float
    lat_degrees: float
    lon_degrees: float
    alt_meters: float
    latitudeStd_meters: float
    longitudeStd_meters: float
    altitudeStd_meters: float
    undulation_meters: float
    status: int = 0


def preprocess_nav_line(line: str) -> str:
    # Add a space between concatenated numeric values
    line = re.sub(r"(\d)([-+])", r"\1 \2", line)
    return line


def parse_nav_line(line: str) -> SbgEkfNav:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", line
    )

    if len(tokens) != 14:
        raise ValueError(
            f"Expected 14 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    return SbgEkfNav(
        velN_mps=float(tokens[1]),
        velE_mps=float(tokens[2]),
        velD_mps=float(tokens[3]),
        velStdN_mps=float(tokens[4]),
        velStdE_mps=float(tokens[5]),
        velStdD_mps=float(tokens[6]),
        lat_degrees=float(tokens[7]),
        lon_degrees=float(tokens[8]),
        alt_meters=float(tokens[9]),
        latitudeStd_meters=float(tokens[10]),
        longitudeStd_meters=float(tokens[11]),
        altitudeStd_meters=float(tokens[12]),
        undulation_meters=float(tokens[13]),
        status=int(float(tokens[0])),
    )


@dataclass
class ShipMotion:
    status: str
    heavePeriod_sec: float
    surge_m: float
    sway_m: float
    heave_m: float
    surgeAccel_mps2: float
    swayAccel_mps2: float
    heaveAccel_mps2: float
    surgeVel_mps: float
    swayVel_mps: float
    heaveVel_mps: float


def parse_ship_motion_line(line: str) -> ShipMotion:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", line
    )

    if len(tokens) != 11:
        raise ValueError(
            f"Expected 11 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    return ShipMotion(
        status=tokens[0],
        heavePeriod_sec=float(tokens[1]),
        surge_m=float(tokens[2]),
        sway_m=float(tokens[3]),
        heave_m=float(tokens[4]),
        surgeAccel_mps2=float(tokens[5]),
        swayAccel_mps2=float(tokens[6]),
        heaveAccel_mps2=float(tokens[7]),
        surgeVel_mps=float(tokens[8]),
        swayVel_mps=float(tokens[9]),
        heaveVel_mps=float(tokens[10]),
    )


@dataclass
class AirData:
    status: str
    pressureAbs_pa: float
    altitude_meters: float
    pressureDiff_pa: float
    trueAirspeed_mps: float
    airTemperature_degC: float


def parse_air_data_line(line: str) -> AirData:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", line
    )

    if len(tokens) == 5:
        # Handle 5-field format: [pressureAbs_pa, altitude_meters, pressureDiff_pa, trueAirspeed_mps, airTemperature_degC]
        return AirData(
            status="0",  # Default status when not provided
            pressureAbs_pa=float(tokens[0]),
            altitude_meters=float(tokens[1]),
            pressureDiff_pa=float(tokens[2]),
            trueAirspeed_mps=float(tokens[3]),
            airTemperature_degC=float(tokens[4]),
        )
    elif len(tokens) == 6:
        # Handle 6-field format: [status, pressureAbs_pa, altitude_meters, pressureDiff_pa, trueAirspeed_mps, airTemperature_degC]
        return AirData(
            status=tokens[0],
            pressureAbs_pa=float(tokens[1]),
            altitude_meters=float(tokens[2]),
            pressureDiff_pa=float(tokens[3]),
            trueAirspeed_mps=float(tokens[4]),
            airTemperature_degC=float(tokens[5]),
        )
    else:
        raise ValueError(
            f"Expected 5 or 6 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )


@dataclass
class UtcTime:
    status: str
    gps_time_of_week_ms: int
    gps_year: int
    gps_month: int
    gps_day: int
    gps_hour: int
    gps_minute: int
    gps_second: int
    gps_nanosecond: int
    clkBiasStd_us: int
    clkSfErrorStd_ppm: int
    clkResidualError_us: int


def parse_utc_time_line(line: str) -> UtcTime:
    # Match the full line and extract the fields
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", line
    )

    if len(tokens) != 9:
        raise ValueError(
            f"Expected 9 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    return UtcTime(
        status=tokens[0],
        gps_time_of_week_ms=int(
            float(tokens[1])
        ),  # Convert to float first, then to int
        gps_year=int(float(tokens[2])),
        gps_month=int(float(tokens[3])),
        gps_day=int(float(tokens[4])),
        gps_hour=int(float(tokens[5])),
        gps_minute=int(float(tokens[6])),
        gps_second=int(float(tokens[7])),
        gps_nanosecond=int(float(tokens[8])),
        clkBiasStd_us=0,  # Placeholder, adjust as needed
        clkSfErrorStd_ppm=0,  # Placeholder, adjust as needed
        clkResidualError_us=0,  # Placeholder, adjust as needed
    )


@dataclass
class Quat:
    status: str
    qw: float
    qx: float
    qy: float
    qz: float
    rollStd: float
    pitchStd: float
    yawStd: float
    magDeclination_deg: float
    magInclination_deg: float


def parse_quat_line(line: str) -> Quat:

    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    if len(tokens) != 10:
        raise ValueError(
            f"Expected 10 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return Quat(
        status=tokens[0],
        qw=parse_float(tokens[1]),
        qx=parse_float(tokens[2]),
        qy=parse_float(tokens[3]),
        qz=parse_float(tokens[4]),
        rollStd=parse_float(tokens[5]),
        pitchStd=parse_float(tokens[6]),
        yawStd=parse_float(tokens[7]),
        magDeclination_deg=parse_float(tokens[8]),
        magInclination_deg=parse_float(tokens[9]),
    )


@dataclass
class Euler:
    status: str
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    rollStd_deg: float
    pitchStd_deg: float
    yawStd_deg: float
    magHeading_deg: float
    magDeclination_deg: float
    magInclination_deg: float


def parse_euler_line(line: str) -> Euler:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    if len(tokens) != 10:
        raise ValueError(
            f"Expected 10 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return Euler(
        status=tokens[0],
        roll_deg=parse_float(tokens[1]),
        pitch_deg=parse_float(tokens[2]),
        yaw_deg=parse_float(tokens[3]),
        rollStd_deg=parse_float(tokens[4]),
        pitchStd_deg=parse_float(tokens[5]),
        yawStd_deg=parse_float(tokens[6]),
        magHeading_deg=parse_float(tokens[7]),
        magDeclination_deg=parse_float(tokens[8]),
        magInclination_deg=parse_float(tokens[9]),
    )


@dataclass
class GnssPos:
    status: str
    statusExt: str
    gpsTow_ms: int
    lat_deg: float
    lon_deg: float
    alt_meters: float
    undulation_meters: float
    lat_std_meters: float
    lon_std_meters: float
    alt_std_meters: float
    satellites_tracked: int
    numSatellitesUsed: int
    baseStationId: int = 0  # Default value, adjust as needed
    diffAge: int = 0  # Default value, adjust as needed


def parse_gnss_pos_line(line: str) -> GnssPos:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    # Ignore the first token (header includes a 1 that needs to be ignored)
    tokens = tokens[1:]

    if len(tokens) != 14:
        raise ValueError(
            f"Expected 14 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return GnssPos(
        status=tokens[0],
        statusExt=tokens[1],
        gpsTow_ms=int(float(tokens[2])),
        lat_deg=parse_float(tokens[3]),
        lon_deg=parse_float(tokens[4]),
        alt_meters=parse_float(tokens[5]),
        undulation_meters=parse_float(tokens[6]),
        lat_std_meters=parse_float(tokens[7]),
        lon_std_meters=parse_float(tokens[8]),
        alt_std_meters=parse_float(tokens[9]),
        satellites_tracked=int(float(tokens[10])),
        numSatellitesUsed=int(float(tokens[11])),
        baseStationId=int(float(tokens[12])),
        diffAge=int(float(tokens[13])),  # Adjusted to include the 14th field
    )


@dataclass
class GnssVel:
    status: str
    gpsTow_ms: int
    velN_mps: float
    velE_mps: float
    velD_mps: float
    velStdN_mps: float
    velStdE_mps: float
    velStdD_mps: float
    track_deg: float
    track_std_deg: float


def parse_gnss_vel_line(line: str) -> GnssVel:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    # Ignore the first token (header includes a 1 that needs to be ignored)
    tokens = tokens[1:]

    if len(tokens) != 10:
        raise ValueError(
            f"Expected 10 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return GnssVel(
        status=tokens[0],
        gpsTow_ms=int(float(tokens[1])),
        velN_mps=parse_float(tokens[2]),
        velE_mps=parse_float(tokens[3]),
        velD_mps=parse_float(tokens[4]),
        velStdN_mps=parse_float(tokens[5]),
        velStdE_mps=parse_float(tokens[6]),
        velStdD_mps=parse_float(tokens[7]),
        track_deg=parse_float(tokens[8]),
        track_std_deg=parse_float(tokens[9]),
    )


@dataclass
class ImuData:
    # sbgBasicLogger console prints gyroscopes in deg/s (getGyroscopeDeg).
    status: str
    accX_mps2: float
    accY_mps2: float
    accZ_mps2: float
    gyroX_degps: float
    gyroY_degps: float
    gyroZ_degps: float
    temperature_degC: float


def parse_imu_data_line(line: str) -> ImuData:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    if len(tokens) != 8:
        raise ValueError(
            f"Expected 8 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return ImuData(
        status=tokens[0],
        accX_mps2=parse_float(tokens[1]),
        accY_mps2=parse_float(tokens[2]),
        accZ_mps2=parse_float(tokens[3]),
        gyroX_degps=parse_float(tokens[4]),
        gyroY_degps=parse_float(tokens[5]),
        gyroZ_degps=parse_float(tokens[6]),
        temperature_degC=parse_float(tokens[7]),
    )


@dataclass
class MagData:
    status: str
    magX_au: float
    magY_au: float
    magZ_au: float
    accelerationX_mps2: float
    accelerationY_mps2: float
    accelerationZ_mps2: float


def parse_mag_data_line(line: str) -> MagData:
    # Preprocess the line to handle concatenated numeric values
    line = preprocess_nav_line(line)

    # Match the full line and extract the fields, including 'nan'
    tokens = re.findall(
        r"[-+]?\d*\.\d{1,6}(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?|nan", line
    )

    if len(tokens) != 7:
        raise ValueError(
            f"Expected 7 numeric fields, found {len(tokens)}, tokens: {tokens}"
        )

    def parse_float(val):
        return float("nan") if val == "nan" else float(val)

    return MagData(
        status=tokens[0],
        magX_au=parse_float(tokens[1]),
        magY_au=parse_float(tokens[2]),
        magZ_au=parse_float(tokens[3]),
        accelerationX_mps2=parse_float(tokens[4]),
        accelerationY_mps2=parse_float(tokens[5]),
        accelerationZ_mps2=parse_float(tokens[6]),
    )
