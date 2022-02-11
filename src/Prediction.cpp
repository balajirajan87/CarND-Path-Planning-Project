#include "Prediction.h"
#include <vector>
#include <math.h>
#include <random>

using std::normal_distribution;
using std::default_random_engine;
using namespace std;
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

Predictions::Predictions(){
    
}
/*
destructors..
*/
Predictions::~Predictions(){}

void Predictions::init(vector<vehicle> sf_data){
    /*
     Just to initialize the Datas.
     */
    no_vehicles = sf_data.size();
    vector<double> prob = {prob_cv_model, prob_lc_model, prob_ca_model, prob_cd_model};
    for (int i=0; i<sf_data.size(); i++){
        vehicle init_data;
        init_data.id = sf_data[i].id;
        init_data.x = sf_data[i].x;
        init_data.y = sf_data[i].y;
        init_data.vx = sf_data[i].vx;
        init_data.vy = sf_data[i].vy;
        init_data.s = sf_data[i].s;
        init_data.d = sf_data[i].d;
        veh_datas.push_back(init_data);
        prob_models.push_back(prob);
        vehicle_id_array.push_back(sf_data[i].id);
    }
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
    Pred.s = veh_datas.s + max_velocity * dt;
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
    v = v + max_accel * dt;
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
    v = v + max_decel * dt;
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
        }
        is_initialized = true;
        return predicted_man_list;
    }
    else{
        if (no_vehicles > sf_data.size()){
            //no of vehicles in the vicinity has decreased.
            vector<int> veh_id_loc;
            for(int i=0; i<sf_data.size(); i++){
                veh_id_loc.push_back(sf_data[i].id);
            }
            //deduce a method to compare the arrays
            for(int i=0; i<veh_id_loc.size(); i++){
                if (veh_id_loc[i] != vehicle_id_array[i]){
                    prob_models.erase(prob_models.begin() + i);
                    veh_datas.erase(veh_datas.begin() + i);         //check ??
                    break;
                }
            }
            no_vehicles = sf_data.size();
        }
        else if (no_vehicles < sf_data.size()){
            //no of vehicles has increased.
            int diff = sf_data.size() - no_vehicles;
            vector<double> prob = {0.25, 0.25, 0.25, 0.25};
            for(int i = 0; i < diff; i++){
                prob_models.push_back(prob);
            }
            no_vehicles = sf_data.size();
        }
        else{/* do nothing */}
        for (int i=0; i<sf_data.size(); i++){
            vector<pred_st> cv_model = Const_Velocity_Yawrate_Model(veh_datas[i]);
            vector<pred_st> lc_model = Lane_Change_Model(sf_data[i],veh_datas[i]);
            vector<pred_st> ca_model = Const_Accel_Model(veh_datas[i]);
            vector<pred_st> cd_model = Const_Decel_Model(veh_datas[i]);
            
            //take the first measurement and get the probablity.
            prob_models[i][0] *= calc_prob_model(cv_model, sf_data[i]);
            prob_models[i][1] *= calc_prob_model(lc_model, sf_data[i]);
            prob_models[i][2] *= calc_prob_model(ca_model, sf_data[i]);
            prob_models[i][3] *= calc_prob_model(cd_model, sf_data[i]);
            
            //for each vehicle calculate the max prob
            int max_index = std::distance(prob_models[i].begin(), max_element(prob_models[i].begin(), prob_models[i].end()));
            Man_Type Pred_loc;
            switch(max_index)
            {
                case 0:
                default:
                    Pred_loc = const_vel;
                    break;
                case 1:
                    Pred_loc = lane_change;
                    break;
                case 2:
                    Pred_loc = const_accel;
                    break;
                case 3:
                    Pred_loc = const_decel;
                    break;
            }
            predicted_man_list.push_back(Pred_loc);
        }
        veh_datas.clear();
        //save the previous vehicle data.
        for (int i=0; i<sf_data.size(); i++){
            vehicle prev_veh_data;
            prev_veh_data.id = sf_data[i].id;
            prev_veh_data.x = sf_data[i].x;
            prev_veh_data.y = sf_data[i].y;
            prev_veh_data.vx = sf_data[i].vx;
            prev_veh_data.vy = sf_data[i].vy;
            prev_veh_data.s = sf_data[i].s;
            prev_veh_data.d = sf_data[i].d;
            veh_datas.push_back(prev_veh_data);
        }
    }
    return predicted_man_list;
}

vector<vector<Predictions::vehicle>> Predictions::generate_predictions(vector<vehicle> sf_data){
    vector<Man_Type> pred_man = Predict_maneuvre(sf_data);
    vector<vector<vehicle>> predicted_trajectory;
    for (int i=0; i<pred_man.size(); i++){
        vector<vehicle> Traj_t;
        vehicle Traj;
        switch(pred_man[i])
        {
            //Predict the trajectory for 1 sec to the future
            case const_vel:
                for (double dt = 0; dt < 1; dt += 0.02){
                    double velocity = pow(pow(veh_datas[i].vx, 2) + pow(veh_datas[i].vy, 2), 0.5);
                    Traj.s = veh_datas[i].s + velocity * dt;
                    Traj.d = veh_datas[i].d;
                    Traj_t.push_back(Traj);
                }
                break;
            case lane_change:
                for (double dt = 0; dt < 1; dt += 0.02){
                    // TODO:
                }
                break;
            case const_accel:
                for (double dt=0; dt<1; dt += 0.02){
                    double velocity = pow(pow(veh_datas[i].vx, 2) + pow(veh_datas[i].vy, 2), 0.5);
                    velocity += max_accel * dt;
                    Traj.s = veh_datas[i].s + (velocity * dt) + (max_accel * dt * dt * 0.5);
                    Traj.d = veh_datas[i].d;
                    Traj_t.push_back(Traj);
                }
                break;
            case const_decel:
                for (double dt=0; dt<1; dt += 0.02){
                    double velocity = pow(pow(veh_datas[i].vx, 2) + pow(veh_datas[i].vy, 2), 0.5);
                    velocity += max_decel * dt;
                    Traj.s = veh_datas[i].s + (velocity * dt) + (max_decel * dt * dt * 0.5);
                    Traj.d = veh_datas[i].d;
                    Traj_t.push_back(Traj);
                }
                break;
        }
        predicted_trajectory.push_back(Traj_t);
    }
    return predicted_trajectory;
}
