#include "Prediction.h"
#include <vector>
#include <math.h>
#include <random>
#include "helpers.h"

using std::normal_distribution;
using std::default_random_engine;
/*
constructors..
*/
Predictions::Predictions(int lane, float s, int d, float v, float a){
    lane = lane;
    s = s;
    d = d;
    v = v;
    a = a;
}
/*
destructors..
*/
Predictions::~Predictions(){}

void Predictions::init(vector<vehicle> sf_data){
    /*
     Just to initialize the Datas.
     */
    for (int i=0; i<sf_data.size(); i++){
        vehicle init_data;
        init_data.x = sf_data[i].x;
        init_data.y = sf_data[i].y;
        init_data.vx = sf_data[i].vx;
        init_data.vy = sf_data[i].vy;
        init_data.s = sf_data[i].s;
        init_data.d = sf_data[i].d;
        veh_datas.push_back(init_data);
    }
    is_initialized = false;
}

vector<Predictions::pred_st> Predictions::Const_Velocity_Yawrate_Model(vehicle &veh_datas){
    /*
     Vehicle model: Constant velocity model.
     */
    pred_st Pred;
    vector<pred_st> dist_Pred;
    default_random_engine gen;
    normal_distribution<double> dist_s(0, sigma_frenet[0]);
    normal_distribution<double> dist_d(0, sigma_frenet[1]);
    v = pow(pow(veh_datas.vx,2) + pow(veh_datas.vy, 2),0.5);
    Pred.s = veh_datas.s + v * dt;
    Pred.d = veh_datas.d;
    //add the uncertainities.
    for(int i=0; i<n_dist; i++){
        Pred.s += dist_s(gen);
        Pred.d += dist_d(gen);
        dist_Pred.push_back(Pred);
    }

    return dist_Pred;
}

int Predictions::find_lane_number(float dval){
    /*
     Predicts the Lane number from the current d Coordinate.
     */
    if (dval > 0 && dval <= 4){
        return 1;
    }else if (dval > 4 && dval <= 8){
        return 2;
    }else{
        return 3;
    }
}

vector<Predictions::pred_st> Predictions::Lane_Change_Model(vehicle &data,vehicle &veh_datas){
    /*
     This is the vehicle model for lane Change:
     */
    int lane = find_lane_number(data.d);
    // find the derivative of d
    double ddot = (veh_datas.d - data.d)/dt;
    pred_st Pred;
    vector<pred_st> dist_Pred;
    double d_s = remainder(veh_datas.s, ref_lc_man_length);
    double yaw;
    
    default_random_engine gen;
    normal_distribution<double> dist_s(0, sigma_frenet[0]);
    normal_distribution<double> dist_d(0, sigma_frenet[1]);

    if (ddot > 0){
        //lane change to right
        Pred.d = 0.5 * Lane_width * sin((M_PI * d_s / ref_lc_man_length) - M_PI_2) + Lane_width + ((lane - 1) * Lane_width);
        yaw = atan(Lane_width * M_PI_2 * cos((M_PI * d_s / ref_lc_man_length) - M_PI_2)/ref_lc_man_length);
        Pred.s = veh_datas.s * cos(yaw);
    }else{
        Pred.d = 0.5 * Lane_width * sin(-(M_PI * d_s / ref_lc_man_length) + M_PI_2) + Lane_width + ((lane - 1) * Lane_width);
        yaw = atan(-Lane_width * M_PI_2 * cos(-(M_PI * d_s / ref_lc_man_length) + M_PI_2)/ref_lc_man_length);
        Pred.s = veh_datas.s * cos(yaw);
    }
    
    //add the uncertainities.
    for(int i=0; i<n_dist; i++){
        Pred.s += dist_s(gen);
        Pred.d += dist_d(gen);
        dist_Pred.push_back(Pred);
    }

    return dist_Pred;
}

vector<Predictions::pred_st> Predictions::Const_Accel_Model(vehicle &veh_datas){

    pred_st Pred;
    vector<pred_st> dist_Pred;
    default_random_engine gen;
    normal_distribution<double> dist_s(0, sigma_frenet[0]);
    normal_distribution<double> dist_d(0, sigma_frenet[1]);
    v = pow(pow(veh_datas.vx,2) + pow(veh_datas.vy, 2),0.5);
    Pred.s = veh_datas.s + (v * dt) + (max_accel * pow(dt, 2) * 0.5);
    Pred.d = veh_datas.d;
    
    //add the uncertainities.
    for(int i=0; i<n_dist; i++){
        Pred.s += dist_s(gen);
        Pred.d += dist_d(gen);
        dist_Pred.push_back(Pred);
    }

    return dist_Pred;
}

vector<Predictions::pred_st> Predictions::Const_Decel_Model(vehicle &veh_datas){

    pred_st Pred;
    vector<pred_st> dist_Pred;
    default_random_engine gen;
    normal_distribution<double> dist_s(0, sigma_frenet[0]);
    normal_distribution<double> dist_d(0, sigma_frenet[1]);
    v = pow(pow(veh_datas.vx,2) + pow(veh_datas.vy, 2),0.5);
    Pred.s = veh_datas.s + (v * dt) + (max_decel * pow(dt, 2) * 0.5);
    Pred.d = veh_datas.d;
    
    //add the uncertainities.
    for(int i=0; i<n_dist; i++){
        Pred.s += dist_s(gen);
        Pred.d += dist_d(gen);
        dist_Pred.push_back(Pred);
    }

    return dist_Pred;
}

double Predictions::multiv_prob(double sigma_s, double sigma_d, double obs_s, double obs_d, double mue_s, double mue_d){
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sigma_s * sigma_d);

  // calculate exponent
  double exponent;
    exponent = (pow(obs_s - mue_s, 2) / (2 * pow(sigma_s, 2)))
                   + (pow(obs_d - mue_d, 2) / (2 * pow(sigma_d, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

vector<double> Predictions::calculate_mean(vector<pred_st> model_data){
    double mue_s = 0;
    double mue_d = 0;
    for (int i=0; i<model_data.size(); i++){
        mue_s += model_data[i].s;
        mue_d += model_data[i].d;
    }
    return {mue_s/model_data.size(),mue_d/model_data.size()};
}

double Predictions::calc_prob_model(vector<pred_st> model_data, vehicle sf_data)
{
    vector<double> mue = calculate_mean(model_data);
    double mue_s = mue[0];
    double mue_d = mue[1];
    double prob_cv_model = multiv_prob(sigma_frenet[0], sigma_frenet[1], sf_data.s, sf_data.d, mue_s, mue_d);
}

vector<Predictions::Man_Type> Predictions::Predict_maneuvre(vector<vehicle> sf_data){
    vector<Man_Type> predicted_man_list;
    if (!is_initialized){
        init(sf_data);
        for(int i=0; i<sf_data.size(); i++){
            Man_Type man = const_vel;       //check ??
            predicted_man_list.push_back(man);
            return predicted_man_list;
        }
        is_initialized = true;
    }
    else{
        for (int i=0; i<sf_data.size(); i++){
            vector<pred_st> cv_model = Const_Velocity_Yawrate_Model(veh_datas[i]);
            vector<pred_st> lc_model = Lane_Change_Model(sf_data[i],veh_datas[i]);
            vector<pred_st> ca_model = Const_Accel_Model(veh_datas[i]);
            vector<pred_st> cd_model = Const_Decel_Model(veh_datas[i]);
            
            //take the first measurement and get the probablity.
            prob_cv_model *= calc_prob_model(cv_model, sf_data[i]);
            prob_lc_model *= calc_prob_model(lc_model, sf_data[i]);
            prob_ca_model *= calc_prob_model(ca_model, sf_data[i]);
            prob_cd_model *= calc_prob_model(cd_model, sf_data[i]);
        }
    }
    
}
