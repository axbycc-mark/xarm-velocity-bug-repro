#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

#include <xarm/wrapper/xarm_api.h>
#include <Eigen/Dense>

//============ CONFIGURATION ====================

const std::string xarm_ip = "192.168.1.208";

// file inputs
const std::string desired_q_path = "/home/axby/qs.txt";
const std::string desired_qdot_path = "/home/axby/qdots.txt";
const std::string times_us_path = "/home/axby/times_us.txt";

// file outputs
const std::string commanded_qdots_path = "/home/axby/commanded_qdots.txt";
const std::string actual_qdots_path = "/home/axby/actual_qdots.txt";

// ==============================================

// type aliases for eigen
namespace Eigen {
using Vector7f = Matrix<float, 7, 1>;
} // namespace Eigen

// the Map allows us to treat std::array as Eigen vector
using CM_Vector7f = Eigen::Map<const Eigen::Vector7f>;
using M_Vector7f = Eigen::Map<Eigen::Vector7f>;

const auto PROCESS_START_TIME = std::chrono::steady_clock::now();
uint64_t get_process_time_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - PROCESS_START_TIME)
        .count();
}

struct TrajectoryPoint {
    uint64_t time_us;
    std::array<float, 7> desired_q = {};
    std::array<float, 7> desired_qdot = {};
};

std::vector<TrajectoryPoint> read_trajectory_points() {
    std::ifstream desiredQFile(desired_q_path);
    std::ifstream desiredQDotFile(desired_qdot_path);
    std::ifstream timeFile(times_us_path);

    // Read time_us.txt
    std::vector<uint64_t> time_us_data;
    uint64_t time;
    while (timeFile >> time) {
        time_us_data.push_back(time);
    }
    std::cout << "Loaded " << time_us_data.size() << " times" << "\n";    

    // Read desired_qs.txt
    std::vector<std::array<float, 7>> desired_qs_data;
    std::string line;
    while (std::getline(desiredQFile, line)) {
        std::istringstream iss(line);
        std::array<float, 7> desired_q;
        for (int i = 0; i < 7; ++i) {
            iss >> desired_q[i];
        }
        desired_qs_data.push_back(desired_q);
    }
    std::cout << "Loaded " << desired_qs_data.size() << " desired qs" << "\n";

    // Read desired_qdots.txt
    std::vector<std::array<float, 7>> desired_qdots_data;
    while (std::getline(desiredQDotFile, line)) {
        std::istringstream iss(line);
        std::array<float, 7> desired_qdot;
        for (int i = 0; i < 7; ++i) {
            iss >> desired_qdot[i];
        }
        desired_qdots_data.push_back(desired_qdot);
    }
    std::cout << "Loaded " << desired_qdots_data.size() << " desired qdots" << "\n";    

    // Combine data into TrajectoryPoint instances
    std::vector<TrajectoryPoint> trajectoryPoints;

    if (time_us_data.size() != desired_qs_data.size()) {
        throw std::runtime_error("data file mismatch");
    }
    if (time_us_data.size() != desired_qdots_data.size()) {
        throw std::runtime_error("data file mismatch");
    }
    
    for (size_t i = 0; i < time_us_data.size(); ++i) {
        TrajectoryPoint point;
        point.time_us = time_us_data[i];
        point.desired_q = desired_qs_data[i];
        point.desired_qdot = desired_qdots_data[i];
        trajectoryPoints.push_back(point);
    }

    return trajectoryPoints;
}

std::array<float,7> array7_minus(const std::array<float,7>& a, const std::array<float,7>& b) {
    std::array<float, 7> delta;
    for (int i = 0; i < 7; ++i) {
        delta[i] = a[i] - b[i];
    }
    return delta;
}

// returns 1 when the arm is not at the target (in this case it inches
// a little closer), and returns 0 when arm is pretty much on the
// target
int go_towards_position(XArmAPI& arm, const std::array<float,7>& target) {
    std::array<float, 7> angles;
    std::array<float, 7> vels;
    std::array<float, 7> torques;

    arm.get_joint_states(angles.data(), vels.data(), torques.data());


    std::array<float, 7> dangles;
    M_Vector7f eigen_dangles(dangles.data());
    eigen_dangles = CM_Vector7f(target.data()) - CM_Vector7f(angles.data());

    const float target_distance = eigen_dangles.cwiseAbs().maxCoeff();
    std::cout << "Max joint distance = " << target_distance << " radians\n";
    if (target_distance < 0.001) {
        std::cout << "Target reached\n";
        return false;
    } else {
        std::cout << "Target not reached yet" << "\n";
    }

    // clip velocities
    const float max_vel = 0.1;
    for (int i = 0; i < 7; ++i) {
        if (dangles[i] > max_vel) {
            dangles[i] = max_vel;
        }
        if (dangles[i] < -max_vel) {
            dangles[i] = -max_vel;
        }
    }

    std::cout << "go_towards_position with vel " << eigen_dangles.transpose() << "\n";
    const int result = arm.vc_set_joint_velocity(
        dangles.data(), /*is_sync=*/false, /*duration=*/0.1);
    std::cout << "got result " << result << "\n";

    return true;
}

// returns the commanded velocity
std::array<float, 7> feedforward_velocity_control(XArmAPI& arm,
                                                  const std::array<float, 7>& actual_angles,
                                                  const std::array<float, 7>& actual_vels,
                                                  const std::array<float, 7>& desired_angles,
                                                  const std::array<float, 7>& desired_vels) {
    const float kp = 0.5;                 
    Eigen::Vector7f angle_err = Eigen::Vector7f::Zero();
    angle_err = CM_Vector7f(desired_angles.data()) - CM_Vector7f(actual_angles.data());

    // sanity check the supplied target angles to see they
    // are close to current angles
    const float max_angle_diff = 0.1;
    const float actual_angle_diff = angle_err.cwiseAbs().maxCoeff();
    if (actual_angle_diff > max_angle_diff) {
        arm.emergency_stop();
        arm.disconnect();
        std::cout << "max angle diff violated " << actual_angle_diff;
        throw std::runtime_error("bad tracking: joint angles");
    }

    Eigen::Vector7f qdot_command = kp*angle_err;
    const Eigen::Vector7f vel_err = CM_Vector7f(desired_vels.data()) - CM_Vector7f(actual_vels.data());

    // sanity check the supplied target vels to see they
    // are close to current vels
    const float max_vel_diff = 1.6;
    const float actual_vel_diff = vel_err.cwiseAbs().maxCoeff();
    if (actual_vel_diff > max_vel_diff) {
        arm.emergency_stop();
        arm.disconnect();
        std::cout << "max vel diff violated " << actual_vel_diff;
        throw std::runtime_error("bad tracking: joint vels");
    }

        
    CM_Vector7f feedforward_qdot(desired_vels.data());
    qdot_command += feedforward_qdot;

    const int result = arm.vc_set_joint_velocity(
        qdot_command.data(), /*is_sync=*/false, /*duration=*/0.1);

    std::array<float, 7> qdot_command_array;
    M_Vector7f(qdot_command_array.data()) = qdot_command;
    return qdot_command_array;
}

struct Recording {
    std::vector<std::array<float, 7>> actual_qdots;
    std::vector<std::array<float, 7>> commanded_qdots;
};

// returns the recording of actual qdots and commanded qdots for later analysis
Recording play_trajectory(XArmAPI& arm, const std::vector<TrajectoryPoint>& trajectory_points) {
    Recording recording;
    recording.actual_qdots.reserve(trajectory_points.size());
    recording.commanded_qdots.reserve(trajectory_points.size());
    
    const uint64_t time_start_us = get_process_time_us();
    const uint64_t trajectory_time_start_us = trajectory_points.front().time_us;

    size_t trajectory_idx = 0;
    while (trajectory_idx < trajectory_points.size()) {
        const auto& trajectory_point = trajectory_points[trajectory_idx];
        const uint64_t dt_us_required = trajectory_point.time_us - trajectory_time_start_us;
        const uint64_t dt_us_actual = get_process_time_us() - time_start_us;

        if (dt_us_actual >= dt_us_required) {
            // play this trajectory point
            std::array<float, 7> actual_angles;
            std::array<float, 7> actual_qdots;
            std::array<float, 7> torques;
            arm.get_joint_states(actual_angles.data(),
                                 actual_qdots.data(),
                                 torques.data());

            auto commanded_qdots = feedforward_velocity_control(
                arm,
                actual_angles, actual_qdots,
                trajectory_point.desired_q,
                trajectory_point.desired_qdot);

            recording.actual_qdots.push_back(actual_qdots);
            recording.commanded_qdots.push_back(commanded_qdots);

            const float progress = float(trajectory_idx+1)/trajectory_points.size();
            std::cout << "Progress " << progress*100 << "%\n";
            trajectory_idx += 1;
        }
    }

    return recording;
}

void save_recording(const Recording& recording) {
    // Serialize actual_qdots
    std::ofstream actualQDotsFile(actual_qdots_path);
    for (const auto& qdots : recording.actual_qdots) {
        for (const auto& qdot : qdots) {
            actualQDotsFile << qdot << " ";
        }
        actualQDotsFile << "\n";
    }

    // Serialize commanded_qdots
    std::ofstream commandedQDotsFile(commanded_qdots_path);
    for (const auto& qdots : recording.commanded_qdots) {
        for (const auto& qdot : qdots) {
            commandedQDotsFile << qdot << " ";
        }
        commandedQDotsFile << "\n";
    }
}

int main(int argc, char *argv[])
{
    const std::vector<TrajectoryPoint> trajectory_points = read_trajectory_points();

    std::cout << "Xarm connecting to " << xarm_ip << "\n";
    XArmAPI arm(xarm_ip, /*is_radian=*/true);

    std::cout << "Enabling motion\n";
    arm.motion_enable(true);

    std::cout << "Setting modes\n";
    constexpr int JOINT_VELOCITY_MODE = 4;
    std::cout << "Setting joint velocity" << "\n";
    arm.set_mode(JOINT_VELOCITY_MODE);
    std::cout << "Setting state" << "\n";
    arm.set_state(0);

    std::string userInput;

    // At the beginning, keep pressing enter until the arm gets into
    // the first joint position of the trajectory. Each time you press
    // enter, the arm moves only a little bit for safety purposes. 
    std::cout << "Going to the start of the trajectory" << "\n";
    while(true) {
        std::cout << "Press enter to inch towards the start of the trajectory" << "\n";
        std::getline(std::cin, userInput);
        if (!go_towards_position(arm, trajectory_points.front().desired_q)) {
            break;
        }
    };

    // After that, pressing enter will execute the trajectory that was loaded from file.
    std::cout << "Trajectory will play after pressing enter." << "\n";
    std::getline(std::cin, userInput);
    const Recording recording = play_trajectory(arm, trajectory_points);

    std::cout << "Saving recording to \n\t"
              << commanded_qdots_path << "\n\t"
              << actual_qdots_path << "\n";
    save_recording(recording);

    return 0;
}
