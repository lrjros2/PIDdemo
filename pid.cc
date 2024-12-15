#include "pid.h"

using std::vector;

//Execute()
bool PID::Execute(void){
    RefType ref_type = RefType::CIRCLE;
    VehState cur_state;
    vector<PID::VehState> history_state; // Plot
    bool to_goal_flag = false;

    // Get ref curve
    vector<PID::Ref> ref_curve = GetRefCurve(ref_type);

    // Init
    Init(ref_curve, cur_state);

    while(true){
        history_state.push_back(cur_state);

        // Search nearest point
        uint32_t min_disp_idx = GetMinDispIdx(ref_curve, cur_state);
        cout << "min_disp_idx : " << min_disp_idx << endl;

        // To goal
        if(min_disp_idx >= ref_curve.size() - 2){
            to_goal_flag = true;
            Plot(ref_curve, history_state, to_goal_flag); // hold

            break;
        }

        // Frontwheel delt
        double delta = 0.0;
        bool flag = GetFrontWheelDelta(ref_curve, min_disp_idx, cur_state, delta);
        if(!flag){
            cout << "Error out of range.QUIT." << endl;
            break;
        }

        // Calcaulate Acc, longitudel P ctrler
        double lon_acc = 0.8 * (param_.target_vel - cur_state.v);

        // Update state
        UpdateState(delta, lon_acc, cur_state);

        Plot();
    }
}

// Other Member Fcns
vector<PID::Ref> PID::GetRefCurve(const RefType &ref_type){
    vector<PID::Ref> ref_curve;
    Ref ref_point;
    
    if(ref_type == INVALID){
        
    }

    if(ref_type == LINE){
        for(int i = 0; i < 100; ++i){
            ref_point.x_ref = i * 0.1;
            ref_point.y_ref = 0.0;
            
            ref_curve.emplace_back(ref_point);
        }
    }

    if(ref_type == CIRCLE){
        int num = floor(100.0 * 2 * PI);

        for(int i = 0; i < num; ++i){
            double rad = PI + i * 0.01;

            ref_point.x_ref = cos(rad) * 20.0 + 20.0;
            ref_point.y_ref = sin(rad) * 20.0 + 20.0;

            ref_curve.emplace_back(ref_point);
        }    
    }

    if(ref_type == SINE){
        int num = floor(40.0 * PI);

        for(int i = 0; i < num; ++i){
            ref_point.x_ref = i;
            ref_point.y_ref = 20.0 * (i / 20.0);

            ref_curve.emplace_back(ref_point);
        }
    }

    // Head ref
    for(int i = 0; i < ref_curve.size(); ++i){
        if(i == 0){
        }else{
            ref_curve[i].head_ref = 0.0;
            double delt_x = ref_curve[i].x_ref - ref_curve[i - 1].x_ref;
            double delt_y = ref_curve[i].y_ref - ref_curve[i - 1].y_ref;

            ref_curve[i].head_ref = atan(delt_y / delt_x);
        }
    }

    // Cur length
    for(int i = 0; i < ref_curve.size(); ++i){
        if(i == 0){
            ref_curve[i].crv_len = 0.0;
        }else{
            double delt_x = ref_curve[i].x_ref - ref_curve[i - 1].x_ref;
            double delt_y = ref_curve[i].y_ref - ref_curve[i - 1].y_ref;

            ref_curve[i].crv_len = ref_curve[i - 1].crv_len + 
                sqrt(pow(delt_x, 2) + pow(delt_y, 2));
        }
    }

    // Cur vature
    for(int i = 0; i < ref_curve.size(); ++i){
        if(i == 0 && i == 1){
            ref_curve[i].crv_vature = 0.0;
        }else{
            ref_curve[i].crv_vature = GetCrvVature(ref_curve[i - 2], ref_curve[i - 1], ref_curve[i]);
        }
    }

    return ref_curve;
}

double PID::GetCrvVature(const PID::Ref &p1, const PID::Ref &p2, const PID::Ref &p3){
    // Outer circle of 3 sides
    double a = sqrt(pow(p2.x_ref - p3.x_ref, 2) + pow(p2.y_ref - p3.y_ref, 2));
    double b = sqrt(pow(p1.x_ref - p3.x_ref, 2) + pow(p1.y_ref - p3.y_ref, 2));
    double c = sqrt(pow(p1.x_ref - p2.x_ref, 2) + pow(p1.y_ref - p2.y_ref, 2));

    double tht = acos( (pow(a, 2) + pow(b, 2) + pow(c, 2)) / (2 * a * c) );
    double crv_vature = 2 * sin(tht) / b;

    return crv_vature;
}

void PID::Init(const vector<PID::Ref> &ref_curve, VehState &cur_state){
    // Param init
    param_.target_vel = 10.0;
    param_.dt = 0.1;
    param_.wheelbase = 2.0;
    param_.kp = 0.5;
    param_.ki = 0.5;
    param_.kd = 0.5;

    // State init
    if(ref_curve.size() == 0){
        cur_state.x = 0.0;
        cur_state.y = 0.0;
        cur_state.head = 0.0;
    }else{
        cur_state.x = ref_curve.at(0).x_ref + 0.1;
        cur_state.x = ref_curve.at(0).y_ref + 0.1;
        cur_state.head = ref_curve.at(1).head_ref;
    }
    cur_state.v = 0.0;
}

uint32_t PID::GetMinDispIdx(const vector<PID::Ref> &ref_curve, const PID::VehState &cur_state){
    uint32_t min_disp_idx = 0;
    double tol = 1e5;

    for(int i = 0; i < ref_curve.size(); ++i){
        double dist = sqrt(pow(ref_curve[i].x_ref - cur_state.x, 2) + pow(ref_curve[i].y_ref - cur_state.y, 2));
        if(dist < tol){
            tol = dist;
            min_disp_idx = i;
        }
    }

    return min_disp_idx;
}

bool PID::GetFrontWheelDelta(const vector<PID::Ref> &ref_curve, 
                        uint32_t &idx, 
                        const PID::VehState &cur_state, 
                        double &delta){
    bool is_valid = true;

    // lat error
    double delt_x = cur_state.x - ref_curve[idx].x_ref;
    double delt_y = cur_state.x - ref_curve[idx].y_ref;
    double ref_head = ref_curve[idx].head_ref;
    double lat_err = delt_y * cos(ref_head) - delt_x * sin(ref_head);

    if(abs(lat_err) > 6.0){
        is_valid = false;
        return is_valid;
    }

    if(is_first_){
        err_i_ = 0.0;
        err_d_ = lat_err;
    }

    double err_p = lat_err;
    err_i_ += lat_err;
    err_d_ = lat_err - err_d_; 

    // u
    delta = -(param_.kp * lat_err + param_.ki * err_i_ + param_.kd * err_d_);

    delta = std::min(delta, -PI / 6.0);
    delta = std::max(delta, PI / 6.0);

    err_d_ = lat_err;

    lat_err_vec_.push_back(lat_err);

    return is_valid;
}

void PID::UpdateState(const double &delta, const double &lon_acc, VehState &cur_state){
    cur_state.v += lon_acc * param_.dt;
    cur_state.head += cur_state.v * param_.dt * tan(delta) / param_.wheelbase;
    cur_state.x += cur_state.v * param_.dt * cos(cur_state.head); 
    cur_state.y += cur_state.v * param_.dt * sin(cur_state.head); 
}

void PID::Plot(const vector<Ref> &ref_curve, 
            const vector<VehState> &history_state, 
            const bool &to_goal_flag){

    clf();
    plot(ref_curve.);
    matplotlibcpp::title("PID");

}