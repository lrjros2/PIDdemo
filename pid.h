#include <vector>
#include <cmath>
#include <stdint.h>
#include <iostream>
#include <matplotlibcpp.h>

using std::vector;
using std::cout;
using std::endl;
using namespace matplotlibcpp;

#define PI 3.14159

class PID{
private:
    enum RefType{
        INVALID = 0,
        LINE = 1,
        CIRCLE = 2,
        SINE = 3
    };

    struct Ref{
        double x_ref = 0.0;
        double y_ref = 0.0;
        double v_ref = 0.0;
        double head_ref = 0.0;

        double crv_len = 0.0;
        double crv_vature = 0.0;
    };

    struct VehState{
        double x = 0.0;
        double y = 0.0;
        double v = 0.0;
        double head = 0.0;
    };

    struct Param{
        double target_vel = 0.0;
        double dt = 0.0;
        double wheelbase = 0.0;

        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
    };
    Param param_;

    bool is_first_ = true;
    double err_i_ = 0.0;
    double err_d_ = 0.0;

    vector<double> lat_err_vec_;

public:
    PID(){};
    ~PID(){};
    bool Execute(void);

    double GetCrvVature(const PID::Ref &p1, const PID::Ref &p2, const PID::Ref &p3);
    vector<PID::Ref> GetRefCurve(const RefType &ref_type);
    void Init(const vector<PID::Ref> &ref_curve, VehState &cur_state);
    uint32_t GetMinDispIdx(const vector<PID::Ref> &ref_curve, const PID::VehState &cur_state);
    bool GetFrontWheelDelta(const vector<PID::Ref> &ref_curve, 
                        uint32_t &idx, 
                        const PID::VehState &cur_state, 
                        double &delta);
    void UpdateState(const double &delta, const double &lon_acc, VehState &cur_state);
    void Plot(const vector<Ref> &ref_curve, 
            const vector<VehState> &history_state, 
            const bool &to_goal_flag);
};