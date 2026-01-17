#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <array>

#include "aquila/config.hpp"
#include "aquila/state.hpp"
#include "aquila/sensors.hpp"
#include "aquila/estimator.hpp"
#include "aquila/controller.hpp"
#include "aquila/modes.hpp"
#include "aquila/logger.hpp"

namespace {

struct CsvRow {
   double t_s{};
   double n_m{};
   double e_m{};
   double d_m{};
   double psi_rad{};
   double qw{};
   double qx{};
   double qy{};
   double qz{};
   double v_mps{};
   double imu_ax{};
   double imu_ay{};
   double imu_az{};
   double imu_gx{};
   double imu_gy{};
   double imu_gz{};
   double gps_x{};
   double gps_y{};
   double gps_z{};
   double gps_vx{};
   double gps_vy{};
   double gps_vz{};
   double baro_alt{};
   double baro_p{};
};

struct Args {
   bool stream = false; // if true: read sensors from stdin, write cmds to stdout
   std::string input_log_path = "logs/sim_sensors.csv";
   std::string output_log_path = "logs/fsw_output.csv";
};

// Set to 1 to bypass estimator and feed controller truth-derived NavState
#define BYPASS_ESTIMATOR_WITH_TRUTH 1
// Set to 1 to force CRUISE mode (ignore ModeManager)
#define FORCE_CRUISE_MODE 1
// Enable integral augmentation block
#define ENABLE_ADAPTIVE_TRIM 0

// -------- Helpers --------

static std::array<double,4> quat_normalize(const std::array<double,4>& q) {
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12) return {1.0, 0.0, 0.0, 0.0};
    return { q[0]/n, q[1]/n, q[2]/n, q[3]/n };
}

// Integrate quaternion for body->NED (n<-b) using body rates (p,q,r) in rad/s
static std::array<double,4> quat_integrate_body_rates(
    const std::array<double,4>& q_nb,
    const std::array<double,3>& omega_body,
    double dt_s) {

    const double qw=q_nb[0], qx=q_nb[1], qy=q_nb[2], qz=q_nb[3];
    const double p=omega_body[0], q=omega_body[1], r=omega_body[2];

    // qdot = 0.5 * Omega(omega) * q
    const double dq_w = 0.5 * (-qx*p - qy*q - qz*r);
    const double dq_x = 0.5 * ( qw*p + qy*r - qz*q);
    const double dq_y = 0.5 * ( qw*q - qx*r + qz*p);
    const double dq_z = 0.5 * ( qw*r + qx*q - qy*p);

    std::array<double,4> q_new{
        qw + dq_w*dt_s,
        qx + dq_x*dt_s,
        qy + dq_y*dt_s,
        qz + dq_z*dt_s
    };
    return quat_normalize(q_new);
}

struct FirstOrderLPF {
    bool init = false;
    double y = 0.0;
    double tau_s = 1.0; // time constant

    explicit FirstOrderLPF(double tau) : tau_s(tau) {}

    double step(double x, double dt_s) {
        if (!init) { y = x; init = true; return y; }
        if (dt_s <= 1e-6) return y;
        const double a = dt_s / (tau_s + dt_s);
        y += a * (x - y);
        return y;
    }
};

static std::array<double,4> yaw_to_quat(double yaw_rad) {
    // body relative to NED, yaw-only
    const double h = 0.5 * yaw_rad;
    return { std::cos(h), 0.0, 0.0, std::sin(h) };
}

static Args parse_args(int argc, char** argv) {
   Args a{};
   for (int i = 1; i < argc; ++i) {
       const std::string s = argv[i];
       if (s == "--stream") {
           a.stream = true;
       } else if (s == "--input" && (i + 1) < argc) {
           a.input_log_path = argv[++i];
       } else if (s == "--output" && (i + 1) < argc) {
           a.output_log_path = argv[++i];
       } else if (s == "--help" || s == "-h") {
           std::cerr
               << "Usage:\n"
               << "  aquila_fsw [--input logs/sim_sensors.csv] [--output logs/fsw_output.csv]\n";
           std::exit(0);
       } else {
           std::cerr << "Unknown arg: " << s << "\n";
           std::exit(1);
       }
   }
   return a;
}

static std::vector<std::string> split_csv(const std::string& line) {
   std::vector<std::string> out;
   std::stringstream ss(line);
   std::string field;
   while (std::getline(ss, field, ',')) out.push_back(field);
   return out;
}

static inline void quat_to_euler_321(const std::array<double,4>& q_nb,
                                     double& roll_phi,
                                     double& pitch_theta,
                                     double& yaw_psi) {

    // q = [qw, qx, qy, qz] for body->NED (n <- b)
    const double qw = q_nb[0];
    const double qx = q_nb[1];
    const double qy = q_nb[2];
    const double qz = q_nb[3];

    // yaw (psi)
    const double siny_cosp = 2.0 * (qw*qz + qx*qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
    yaw_psi = std::atan2(siny_cosp, cosy_cosp);

    // pitch (theta)
    double sinp = 2.0 * (qw*qy - qz*qx);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    pitch_theta = std::asin(sinp);

    // roll (phi)
    const double sinr_cosp = 2.0 * (qw*qx + qy*qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
    roll_phi = std::atan2(sinr_cosp, cosr_cosp);
}

static bool to_double(const std::string& s, double& v) {
   try {
       size_t idx = 0;
       v = std::stod(s, &idx);
       (void)idx;
       return true;
   } catch (...) {
       return false;
   }
}

struct ColumnMap {
   int t_s = -1;

   int n_m = -1;
   int e_m = -1;
   int d_m = -1;

   int psi_rad = -1;
   int qw = -1;
   int qx = -1;
   int qy = -1;
   int qz = -1;
   int v_mps = -1;

   int imu_ax = -1;
   int imu_ay = -1;
   int imu_az = -1;
   int imu_gx = -1;
   int imu_gy = -1;
   int imu_gz = -1;

   int gps_x = -1;
   int gps_y = -1;
   int gps_z = -1;
   int gps_vx = -1;
   int gps_vy = -1;
   int gps_vz = -1;

   int baro_alt = -1;
   int baro_p = -1;
};

static ColumnMap parse_header_map(const std::string& header_line) {
   ColumnMap m;
   auto cols = split_csv(header_line);

   auto find = [&](std::initializer_list<const char*> names) -> int {
       for (size_t i = 0; i < cols.size(); ++i) {
           for (auto* n : names) {
               if (cols[i] == n) return static_cast<int>(i);
           }
       }
       return -1;
   };

   // robust to your evolving naming (x/y/z vs n/e/d)
   m.t_s     = find({"t_s", "t"});

   m.n_m     = find({"x_m", "n_m", "N", "n"});
   m.e_m     = find({"y_m", "e_m", "E", "e"});
   m.d_m     = find({"z_m", "d_m", "D", "d"});

   m.psi_rad = find({"psi_rad", "psi_true_rad", "psi"});
   m.qw = find({"qw", "q_w", "quat_w", "est_qw", "true_qw"});
   m.qx = find({"qx", "q_x", "quat_x", "est_qx", "true_qx"});
   m.qy = find({"qy", "q_y", "quat_y", "est_qy", "true_qy"});
   m.qz = find({"qz", "q_z", "quat_z", "est_qz", "true_qz"});
   m.v_mps   = find({"v_mps", "V", "V_true"});

   m.imu_ax  = find({"imu_ax_mps2", "imu_ax"});
   m.imu_ay  = find({"imu_ay_mps2", "imu_ay"});
   m.imu_az  = find({"imu_az_mps2", "imu_az"});
   m.imu_gx  = find({"imu_gx_radps", "imu_gx"});
   m.imu_gy  = find({"imu_gy_radps", "imu_gy"});
   m.imu_gz  = find({"imu_gz_radps", "imu_gz"});

   m.gps_x   = find({"gps_x_m", "gps_n"});
   m.gps_y   = find({"gps_y_m", "gps_e"});
   m.gps_z   = find({"gps_z_m", "gps_d"});
   m.gps_vx  = find({"gps_vx_mps", "gps_vn"});
   m.gps_vy  = find({"gps_vy_mps", "gps_ve"});
   m.gps_vz  = find({"gps_vz_mps", "gps_vd"});

   m.baro_alt = find({"baro_alt_m", "baro_alt"});
   m.baro_p   = find({"baro_pressure_pa", "baro_p"});

   return m;
}

static bool require_columns(const ColumnMap& m) {
  return (m.t_s >= 0 &&
          m.n_m >= 0 && m.e_m >= 0 && m.d_m >= 0 &&
          m.imu_ax >= 0 && m.imu_ay >= 0 && m.imu_az >= 0 &&
          m.imu_gx >= 0 && m.imu_gy >= 0 && m.imu_gz >= 0 &&
          m.gps_x >= 0 && m.gps_y >= 0 && m.gps_z >= 0 &&
          m.gps_vx >= 0 && m.gps_vy >= 0 && m.gps_vz >= 0 &&
          m.baro_alt >= 0 &&
          m.qw >= 0 && m.qx >= 0 && m.qy >= 0 && m.qz >= 0);
}

// Returns true if the needed baseline fields were parsed.
bool parse_log_line(const std::string& line, const ColumnMap& m, CsvRow& row) {
   if (line.empty()) return false;

   auto fields = split_csv(line);

   auto get = [&](int idx, double& out) -> bool {
       if (idx < 0) return false;
       if (static_cast<size_t>(idx) >= fields.size()) return false;
       return to_double(fields[idx], out);
   };

   // require all baseline fields
   if (!get(m.t_s, row.t_s)) return false;

   if (!get(m.n_m, row.n_m)) return false;
   if (!get(m.e_m, row.e_m)) return false;
   if (!get(m.d_m, row.d_m)) return false;

   get(m.psi_rad, row.psi_rad);
   if (!get(m.qw, row.qw)) return false;
   if (!get(m.qx, row.qx)) return false;
   if (!get(m.qy, row.qy)) return false;
   if (!get(m.qz, row.qz)) return false;
   get(m.v_mps, row.v_mps);

   if (!get(m.imu_ax, row.imu_ax)) return false;
   if (!get(m.imu_ay, row.imu_ay)) return false;
   if (!get(m.imu_az, row.imu_az)) return false;

   if (!get(m.imu_gx, row.imu_gx)) return false;
   if (!get(m.imu_gy, row.imu_gy)) return false;
   if (!get(m.imu_gz, row.imu_gz)) return false;

   if (!get(m.gps_x, row.gps_x)) return false;
   if (!get(m.gps_y, row.gps_y)) return false;
   if (!get(m.gps_z, row.gps_z)) return false;

   if (!get(m.gps_vx, row.gps_vx)) return false;
   if (!get(m.gps_vy, row.gps_vy)) return false;
   if (!get(m.gps_vz, row.gps_vz)) return false;

   if (!get(m.baro_alt, row.baro_alt)) return false;

   // baro pressure is optional
   if (m.baro_p >= 0) {
       get(m.baro_p, row.baro_p);
   }

   return true;
}

} // namespace

// Actuator slew-rate limiter, prevents large jumps in commands
double last_elevator = 0.0;
double last_throttle = 0.0;
bool have_last_cmd = false;

int main(int argc, char** argv) {
   using namespace aquila;
   const Args args = parse_args(argc, argv);

   // IMPORTANT: anything printed to stdout in --stream mode will be interpreted by the Python driver as actuator output. Use stderr for diagnostics.
   std::cerr << "Aquila FSW starting.\n";

   // Output log
   std::ofstream output_file(args.output_log_path);
   if (!output_file) {
       std::cerr << "ERROR: failed to open output log: " << args.output_log_path << "\n";
       return 1;
   }
   CsvLogger logger(output_file);

   // Input: file OR stdin
   std::ifstream input_file;
   std::istream* in = nullptr;

   if (args.stream) {
       in = &std::cin;
   } else {
       input_file.open(args.input_log_path);
       if (!input_file) {
           std::cerr << "ERROR: failed to open input log: " << args.input_log_path << "\n";
           return 1;
       }
       in = &input_file;
   }

   // Read header from the chosen stream
   std::string header_line;
   if (!std::getline(*in, header_line)) {
       std::cerr << "ERROR: empty sensor input stream\n";
       return 1;
   }
   const ColumnMap colmap = parse_header_map(header_line);

   std::cerr << "colmap: qw=" << colmap.qw
            << " qx=" << colmap.qx
            << " qy=" << colmap.qy
            << " qz=" << colmap.qz << "\n";

   if (!require_columns(colmap)) {
        std::cerr << "ERROR: sensor header missing required columns.\n";
        std::cerr << "Header: " << header_line << "\n";
        return 1;
    }

   // In stream mode, emit an output header ONCE
   if (args.stream) {
       std::cout << "t_s,cmd_aileron,cmd_elevator,cmd_rudder,cmd_throttle\n";
       std::cout.flush();
   }

   // Initialize estimator and controller
   #define USE_EKF 1   // 1 = True
   #if USE_EKF
   EkfEstimator estimator;
   #else
   SimpleEstimator estimator;
   #endif

   SimpleController controller;
   ModeManager mode_manager;

   // Initialize state and sensor measurements
   NavState state{};
   ImuMeasurement imu{};
   GpsMeasurement gps{};
   BaroMeasurement baro{};
   ActuatorCommands cmd{};
   ControllerDebug ctrl_dbg{};

   // Persistent attitude state 
   std::array<double,4> att_q_nb = {1.0, 0.0, 0.0, 0.0};
   bool att_init = false;

   // For truth Navstate bypass
   CsvRow prev_row{};
   bool have_prev_row = false;

   // Use PI “adaptive trim” without refactoring your controller class --------
   double i_elevator = 0.0;   // rad bias
   double i_throttle = 0.0;   // throttle bias

   // Conservative starting gains (tune later)
   const double ki_alt_elev = 0.006;   // rad / (m*s)
   const double ki_speed_thr = 0.02;   // throttle / ((m/s)*s)

   // Anti-windup limits (conservative)
   const double i_elevator_max = 0.25;  // rad
   const double i_throttle_max = 0.30;  // throttle units
   // -------------------------------------------------------------------------

   double last_t = 0.0;
   bool first_row = true;
   static int last_mode_index = -999;

   // --- “Sensor-like” filters for testing non-truth values ---
   FirstOrderLPF baro_alt_lpf(0.8);  // tau_alt_s (tune)
   FirstOrderLPF gps_vn_lpf(0.5);   // tau_spd_s (tune)
   FirstOrderLPF gps_ve_lpf(0.5);
   FirstOrderLPF gps_vd_lpf(0.5);


   while (std::getline(*in, header_line)) {
       CsvRow row{};

       if (!parse_log_line(header_line, colmap, row)) {
           std::cerr << "Skipping malformed line: " << header_line << std::endl;
           continue;
       }

       double dt_s = 0.0;
       if (first_row) {
           dt_s = 0.05;   // match scenarios.py dt
           last_t = row.t_s;
           first_row = false;
       } else {
           dt_s = row.t_s - last_t;
           if (dt_s < 1e-3) dt_s = 1e-3;
           last_t = row.t_s;
       }

       // Fill sensor data structures
       imu.timestamp_s = row.t_s;
       imu.accel_mps2  = {row.imu_ax, row.imu_ay, row.imu_az};
       imu.gyro_rads   = {row.imu_gx, row.imu_gy, row.imu_gz};
       imu.valid       = true;

       gps.timestamp_s   = row.t_s;
       gps.position_ned  = {row.gps_x, row.gps_y, row.gps_z};
       gps.velocity_ned  = {row.gps_vx, row.gps_vy, row.gps_vz};
       gps.valid         = true;

       baro.timestamp_s = row.t_s;
       baro.altitude_m  = row.baro_alt;
       baro.valid       = true;

       // Estimation
       #if BYPASS_ESTIMATOR_WITH_TRUTH
            // --- Truth-derived NavState (bypass estimator) ---
            state.position_ned = { row.n_m, row.e_m, row.d_m };

            if (!have_prev_row || dt_s <= 1e-6) {
                // Initialize velocity to commanded cruise speed/set initial velocity to avoid a t=0 “speed=0” spike
                const double v0 = mode_manager.target_speed_mps();  // e.g., ~15 m/s
                state.velocity_ned = { v0, 0.0, 0.0 };
                prev_row = row;
                have_prev_row = true;
            } else {
                state.velocity_ned = {
                    (row.n_m - prev_row.n_m) / dt_s,
                    (row.e_m - prev_row.e_m) / dt_s,
                    (row.d_m - prev_row.d_m) / dt_s
                };
                prev_row = row;
            }

            state.omega_body = { row.imu_gx, row.imu_gy, row.imu_gz };

            // Prefer truth quaternion if present in the log; else fall back to yaw-only
            const bool have_quat =
                std::isfinite(row.qw) && std::isfinite(row.qx) &&
                std::isfinite(row.qy) && std::isfinite(row.qz);

            if (have_quat) {
                const double n = std::sqrt(row.qw*row.qw + row.qx*row.qx + row.qy*row.qy + row.qz*row.qz);
                if (n > 1e-12) {
                    state.quat_nb = { row.qw/n, row.qx/n, row.qy/n, row.qz/n };
                } else {
                    state.quat_nb = { 1.0, 0.0, 0.0, 0.0 };
                }
            } else {
                state.quat_nb = yaw_to_quat(row.psi_rad);
            }

        #else
            // --- Normal EKF path ---
            estimator.predict(dt_s, imu);
            estimator.update_gps(gps);
            estimator.update_baro(baro);
            state = estimator.get_state();
        #endif

        if (row.t_s < 1.0) {
        std::cerr << "t=" << row.t_s
                  << " q=[" << row.qw << "," << row.qx << "," << row.qy << "," << row.qz << "]\n";
        }

       // Mode management (use state for mode logic; ctrl_state is for control robustness testing)
       mode_manager.update(row.t_s, state);
       FlightMode mode = mode_manager.mode();
       #if FORCE_CRUISE_MODE
           mode = FlightMode::CRUISE;
       #endif

       int mode_index = flight_mode_to_int(mode);

       // Reset integrators on mode change
       if (mode_index != last_mode_index) {
           i_elevator = 0.0;
           i_throttle = 0.0;
           last_mode_index = mode_index;
       }

       // Update controller targets from mode manager
       controller.set_target_altitude(mode_manager.target_altitude_m());
       controller.set_desired_speed(mode_manager.target_speed_mps());

       // --- Build a control-view state (ctrl_state) that mimics sensor/estimator lag ---
       NavState ctrl_state = state;

       if (mode == FlightMode::CRUISE || mode == FlightMode::CLIMB || mode == FlightMode::RTL) {
            // 1) Filter altitude using baro (baro.altitude_m is +up)
            const double alt_baro_m = baro.altitude_m;
            const double alt_f_m = baro_alt_lpf.step(alt_baro_m, dt_s);

            // Convert filtered altitude (+up) back into NED down position
            ctrl_state.position_ned[2] = -alt_f_m;

            // 2) Filter GPS velocity components directly (keeps vertical rate consistent with “sensor-view”)
            const double vn_f = gps_vn_lpf.step(gps.velocity_ned[0], dt_s);
            const double ve_f = gps_ve_lpf.step(gps.velocity_ned[1], dt_s);
            const double vd_f = gps_vd_lpf.step(gps.velocity_ned[2], dt_s);

            ctrl_state.velocity_ned = { vn_f, ve_f, vd_f };
       }

       // Control depends on mode
       switch (mode) {
       case FlightMode::STANDBY:
           // Hold trim or steady, level flight
           cmd.aileron  = 0.0;
           cmd.elevator = controller.elevator_trim();
           cmd.rudder   = 0.0;
           cmd.throttle = controller.throttle_trim();
           break;

       case FlightMode::CLIMB:
           // Normal closed-loop control, but ModeManager has set a "climb" target
           ctrl_dbg = ControllerDebug{};    // reset each tick
           cmd = controller.compute_commands(ctrl_state, &ctrl_dbg);
           break;

       case FlightMode::CRUISE:
           // Normal closed-loop cruise (follow square track, etc.)
           ctrl_dbg = ControllerDebug{};    // reset each tick
           cmd = controller.compute_commands(ctrl_state, &ctrl_dbg);
           break;

       case FlightMode::RTL:
           // Return-to-launch
           ctrl_dbg = ControllerDebug{};    // reset each tick
           cmd = controller.compute_commands(ctrl_state, &ctrl_dbg);
           break;

       case FlightMode::FAILSAFE:
       default:
           // Simple FAILSAFE placeholder: cut throttle, neutral surfaces.
           cmd.aileron  = 0.0;
           cmd.elevator = 0.0;
           cmd.rudder   = 0.0;
           cmd.throttle = 0.0;
           break;
       }

       #if ENABLE_ADAPTIVE_TRIM
            // --- Integral augmentation (adaptive trim), migrate to controller later ---
            const bool allow_integrate = (dt_s > 1e-6);
            if (allow_integrate) {
                    if (mode == FlightMode::CRUISE || mode == FlightMode::CLIMB || mode == FlightMode::RTL) {
                        const double alt_m = -ctrl_state.position_ned[2];

                        const double vn = ctrl_state.velocity_ned[0];
                        const double ve = ctrl_state.velocity_ned[1];
                        const double vd = ctrl_state.velocity_ned[2];
                        double speed_mps = std::sqrt(vn*vn + ve*ve); // 2D groundspeed for now


                        // After you compute a "pre-integrator" elevator/throttle command from controller:
                        double elev_pre_i = cmd.elevator;
                        double thr_pre_i  = cmd.throttle;

                        // Compute errors (keep your current sign convention since it works now)
                        const double spd_err_mps = mode_manager.target_speed_mps() - speed_mps;
                        const double alt_err_m   = mode_manager.target_altitude_m() - alt_m;

                        // Candidate integrator updates
                        const double de_i = ki_alt_elev  * alt_err_m   * dt_s;
                        const double dt_i = ki_speed_thr * spd_err_mps * dt_s;

                        // Apply integrators tentatively
                        double i_elevator_new = std::clamp(i_elevator + de_i, -i_elevator_max, i_elevator_max);
                        double i_throttle_new = std::clamp(i_throttle + dt_i, -i_throttle_max, i_throttle_max);

                        // Form unsat commands
                        double elev_unsat = elev_pre_i + i_elevator_new;
                        double thr_unsat  = thr_pre_i  + i_throttle_new;

                        // Saturate
                        double elev_sat = std::clamp(elev_unsat, -1.0, 1.0);
                        double thr_sat  = std::clamp(thr_unsat,  0.0, 1.0);

                        // Anti-windup gating: accept integrator update only if it doesn't “push into” saturation
                        auto would_unwind = [](double u_unsat, double u_sat, double du_i) {
                            // If we're saturated, allow integration only if it moves command toward unsaturation.
                            if (std::abs(u_unsat - u_sat) < 1e-12) return true;      // not saturated
                            return ( (u_unsat > u_sat) && (du_i < 0) ) || ( (u_unsat < u_sat) && (du_i > 0) );
                        };

                        if (would_unwind(elev_unsat, elev_sat, de_i)) i_elevator = i_elevator_new;
                        if (would_unwind(thr_unsat,  thr_sat,  dt_i)) i_throttle = i_throttle_new;

                        // Recompute final commands with accepted integrators
                        cmd.elevator = std::clamp(elev_pre_i + i_elevator, -1.0, 1.0);
                        cmd.throttle = std::clamp(thr_pre_i  + i_throttle,  0.0, 1.0);
                    }
            }
        #endif

       // --- Actuator slew-rate limiting ---
       const double elev_rate = controller.elevator_rate();
       const double thr_rate  = controller.throttle_rate();

       if (!have_last_cmd) {
           last_elevator = cmd.elevator;
           last_throttle = cmd.throttle;
           have_last_cmd = true;
       } else if (dt_s > 1e-6) {
           const double de = std::clamp(cmd.elevator - last_elevator, -elev_rate*dt_s, elev_rate*dt_s);
           const double dt = std::clamp(cmd.throttle - last_throttle, -thr_rate*dt_s,  thr_rate*dt_s);

           last_elevator += de;
           last_throttle += dt;

           cmd.elevator = last_elevator;
           cmd.throttle = last_throttle;
       }

       // Stream mode: write commands to stdout
       if (args.stream) {
           std::cout << row.t_s << ","
                   << cmd.aileron << ","
                   << cmd.elevator << ","
                   << cmd.rudder << ","
                   << cmd.throttle << "\n";
           std::cout.flush();
       }

       // --- Debug telemetry for altitude loop ---
       double dbg_alt_cmd_m        = std::numeric_limits<double>::quiet_NaN();
       double dbg_alt_m            = std::numeric_limits<double>::quiet_NaN();
       double dbg_alt_err_m        = std::numeric_limits<double>::quiet_NaN();
       double dbg_elevator_unsat   = std::numeric_limits<double>::quiet_NaN();

       // Only meaningful in active control modes
       if (mode == FlightMode::CLIMB || mode == FlightMode::CRUISE || mode == FlightMode::RTL) {
           dbg_alt_cmd_m = mode_manager.target_altitude_m();

           // IMPORTANT: debug reflects what controller sees (ctrl_state)
           dbg_alt_m = -ctrl_state.position_ned[2];
           dbg_alt_err_m = dbg_alt_cmd_m - dbg_alt_m;

           // Unsaturated elevator (before clamp) - P-only proxy
           dbg_elevator_unsat = controller.k_alt_p() * dbg_alt_err_m;
       }

       // --- Debug telemetry for sign conventions ---
       double dbg_est_phi_rad   = std::numeric_limits<double>::quiet_NaN();
       double dbg_est_theta_rad = std::numeric_limits<double>::quiet_NaN();
       double dbg_est_psi_rad   = std::numeric_limits<double>::quiet_NaN();
       double dbg_alt_dot_mps   = std::numeric_limits<double>::quiet_NaN();
       double dbg_vd_mps        = std::numeric_limits<double>::quiet_NaN();
       double dbg_q_radps       = std::numeric_limits<double>::quiet_NaN();
       double dbg_elev_times_q  = std::numeric_limits<double>::quiet_NaN();

       {
           quat_to_euler_321(state.quat_nb, dbg_est_phi_rad, dbg_est_theta_rad, dbg_est_psi_rad);

           dbg_vd_mps = ctrl_state.velocity_ned[2];   // NED: +down
           dbg_alt_dot_mps = -dbg_vd_mps;   // +up altitude rate

           dbg_q_radps = state.omega_body[1];    // pitch rate
           dbg_elev_times_q = cmd.elevator * dbg_q_radps; // quick sign check
       }

        // Debug telemetry for controller
        double dbg_theta_rad     = std::numeric_limits<double>::quiet_NaN();
        double dbg_theta_cmd_rad = std::numeric_limits<double>::quiet_NaN();
        double dbg_theta_err_rad = std::numeric_limits<double>::quiet_NaN();
        double dbg_climb_cmd_mps = std::numeric_limits<double>::quiet_NaN();

        if (mode == FlightMode::CLIMB || mode == FlightMode::CRUISE || mode == FlightMode::RTL) {
            dbg_theta_rad     = ctrl_dbg.theta_rad;
            dbg_theta_cmd_rad = ctrl_dbg.theta_cmd_rad;
            dbg_theta_err_rad = ctrl_dbg.theta_err_rad;
            dbg_climb_cmd_mps = ctrl_dbg.climb_cmd_mps;
        }

       // Log everything
       logger.log(row.t_s, state, imu, gps, baro, cmd,
            mode_index,
            dbg_alt_cmd_m, dbg_alt_m, dbg_alt_err_m, dbg_elevator_unsat,
            dbg_est_phi_rad, dbg_est_theta_rad, dbg_est_psi_rad,
            dbg_alt_dot_mps, dbg_vd_mps, dbg_q_radps, dbg_elev_times_q,
            dbg_theta_rad, dbg_theta_cmd_rad, dbg_theta_err_rad, dbg_climb_cmd_mps);
   }

   if (args.stream) {
       std::cerr << "Aquila FSW exiting.\n";
   } else {
       std::cout << "Aquila FSW exiting.\n";
   }

   return 0;
}