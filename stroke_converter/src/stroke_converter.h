#pragma once

#include <seed_ros2_controller/stroke_converter/stroke_converter_base.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace seed_converter {

class NoidLifterMover: public StrokeConverter {
public:

    NoidLifterMover() {
    }
    void makeTables() override;
    void Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles) override;
    void Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes) override;
    void setJointNames(const std::vector<std::string> &names) override;

    void calcActuatorVel(std::vector<int16_t> &actuator_vel, const std::vector<double> &joint_vel) override;
    void calcJointVel(std::vector<double> &joint_vel, const std::vector<int16_t> &actuator_vel) override;

private:

    int16_t calcStroke1(double angle, double scale, double offset = 0);

    int16_t calcStroke2(double angle, double scale, int dir, double offset, const std::vector<seed_converter::StrokeMap> &_table);

    void calcStroke3(double angle1, double angle2, double scale, int dir1, int dir2, const std::vector<seed_converter::StrokeMap> &_table1, const std::vector<seed_converter::StrokeMap> &_table2, bool is_pitch, int16_t &out1, int16_t &out2);

    int16_t calcWheelAngDeg(int idx, int16_t raw_ang);

    double quantize(double v, double res);

    double clamp_quantize_inset(double v, double min, double max, double res);

private:
    ConvertTable shoulder_p, shoulder_r, elbow_p, wrist_p, wrist_r, neck_p, neck_r, waist_p, waist_r, leg;
    std::string upper_csv_dir;
    std::string lifter_csv_dir;

    static constexpr double LW_P_MIN = -0.08; //腰、手首差動関節の上限値
    static constexpr double LW_P_MAX =  0.08; //腰、手首差動関節の下限値
    static constexpr double RES = 1e-3; //分解能を下げる

    int16_t zero_ofst[4] = {0}; //ホイール角度ゼロのオフセット
    int16_t prev_wheel[4] = {0}; //ホイール角度ゼロのオフセット
    int16_t cur_wheel[4] = {0}; //ホイール角度ゼロのオフセット

    int idx_waist_y = 0;
    int idx_waist_p = 0;
    int idx_waist_r = 0;
    int idx_l_shoulder_p = 0;
    int idx_l_shoulder_r = 0;
    int idx_l_shoulder_y = 0;
    int idx_l_elbow = 0;
    int idx_l_wrist_y = 0;
    int idx_l_wrist_p = 0;
    int idx_l_wrist_r = 0;
    int idx_l_thumb = 0;
    int idx_neck_y = 0;
    int idx_neck_p = 0;
    int idx_neck_r = 0;
    int idx_r_shoulder_p = 0;
    int idx_r_shoulder_r = 0;
    int idx_r_shoulder_y = 0;
    int idx_r_elbow = 0;
    int idx_r_wrist_y = 0;
    int idx_r_wrist_p = 0;
    int idx_r_wrist_r = 0;
    int idx_r_thumb = 0;
    int idx_knee = 0;
    int idx_ankle = 0;
    int idx_wheel_front_left = 0;
    int idx_wheel_front_right = 0;
    int idx_wheel_rear_left = 0;
    int idx_wheel_rear_right = 0;

};

}

