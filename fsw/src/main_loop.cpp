#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>

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


// -------- Helpers --------

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
               << "  aquila_fsw [--input logs/sim_sensors.csv] [--output logs/fsw_output.csv]\n"
               << "  aquila_fsw --stream [--output logs/fsw_output.csv]\n";
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
   // baseline fields that FSW needs:
   return (m.t_s     >= 0 &&
           m.n_m     >= 0 && m.e_m >= 0 && m.d_m >= 0 &&
           m.imu_ax  >= 0 && m.imu_ay >= 0 && m.imu_az >= 0 &&
           m.imu_gx  >= 0 && m.imu_gy >= 0 && m.imu_gz >= 0 &&
           m.gps_x   >= 0 && m.gps_y  >= 0 && m.gps_z  >= 0 &&
           m.gps_vx  >= 0 && m.gps_vy >= 0 && m.gps_vz >= 0 &&
           m.baro_alt >= 0);
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
   require_columns(colmap);

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


   double last_t = 0.0;
   bool first_row = true;
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
       estimator.predict(dt_s, imu);
       estimator.update_gps(gps);
       estimator.update_baro(baro);
       state = estimator.get_state();

       // Mode management
       mode_manager.update(row.t_s, state);
       const FlightMode mode = mode_manager.mode();
       int mode_index = flight_mode_to_int(mode);

       // Update controller targets from mode manager
       controller.set_target_altitude(mode_manager.target_altitude_m());
       controller.set_desired_speed(mode_manager.target_speed_mps());

       // Control depends on mode
       switch (mode) {
       case FlightMode::STANDBY:
           // Surfaces neutral, throttle idle
           cmd.aileron  = 0.0;
           cmd.elevator = 0.0;
           cmd.rudder   = 0.0;
           cmd.throttle = 0.0;
           break;

       case FlightMode::CLIMB:
           // Normal closed-loop control, but ModeManager has set a "climb" target
           cmd = controller.compute_commands(state);
           break;

       case FlightMode::CRUISE:
           // Normal closed-loop cruise (follow square track, etc.)
           cmd = controller.compute_commands(state);
           break;

       case FlightMode::RTL:
           // Return-to-launch: still uses the same controller, but ModeManager should have set RTL altitude / heading / speed targets
           cmd = controller.compute_commands(state);
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
           // NOTE: this mirrors what your controller currently does internally:

           // alt command +up
           dbg_alt_cmd_m = mode_manager.target_altitude_m();

           // alt is positive up, so negate to get altitude
           dbg_alt_m = -state.position_ned[2];

           // altitude_control(): alt_err = alt_cmd - state position.
           dbg_alt_err_m = dbg_alt_cmd_m - dbg_alt_m;

           // Unsaturated elevator (before clamp)
           dbg_elevator_unsat = controller.k_alt_p() * dbg_alt_err_m;
       }

       // Log everything
       logger.log(row.t_s, state, imu, gps, baro, cmd, mode_index, dbg_alt_cmd_m, dbg_alt_m, dbg_alt_err_m, dbg_elevator_unsat);

   }

   std::cout << "Aquila FSW exiting." << std::endl;
   return 0;
}