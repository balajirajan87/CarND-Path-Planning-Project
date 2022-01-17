//
//  Prediction.h
//  Path_Planning
//
//  Created by Balaji Rajagopalan on 25/12/21.
//

#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>
#include <iostream>
#include "helpers.h"
using std::vector;

class Predictions
{
public:
    //Constructors
    Predictions();
    Predictions(int lane, float s, int d, float v, float a);
    //Destructor.
    virtual ~Predictions();
    
    //member variables.
    int lane;
    int d;
    float s;
    float v;
    float a;
    float dt = 0.02;
    double max_velocity = 22.352;
    double max_accel = 1.0;
    double max_decel = -1.0;
    int Lane_width = 4;
    double ref_lc_man_length = 10.0;
    double sigma_pos[3] = {0.2, 0.1, 0.2};      //x , y , theta
    double sigma_v[2] = {0.3, 0.3};             //vx , vy
    double sigma_frenet[2] = {0.3, 0.2};        //s , d
    int n_dist = 100;
    struct vehicle
    {
        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;
    }sf_vehicles;
    struct pred_st
    {
        double s;
        double d;
    };
    enum Man_Type {const_vel, const_accel, lane_change};
    vector<vehicle> veh_datas;
    //Member functions.
    void increment(int dt);
    float position_at(int t);
    vector<vector<Predictions> > generate_predictions(vector<vehicle> sf_data, int horizon=2);
    vector<pred_st> Lane_Change_Model(vehicle &data,vehicle &veh_datas);
    vector<pred_st> Const_Velocity_Yawrate_Model(vehicle &veh_datas);
    vector<pred_st> Const_Accel_Model(vehicle &veh_datas);
    vector<pred_st> Const_Decel_Model(vehicle &veh_datas);
    vector<Man_Type> Predict_maneuvre(vector<vehicle> sf_data);
    double multiv_prob(double sigma_s, double sigma_d, double obs_s, double obs_d, double mue_s, double mue_d);
    vector<double> calculate_mean(vector<pred_st> model_data);
    void init(vector<vehicle> sf_data);
    double calc_prob_model(vector<pred_st> model_data, vehicle sf_data);
    int find_lane_number(float dval);
private:
    bool is_initialized;
    double prob_cv_model = 0.25;
    double prob_lc_model = 0.25;
    double prob_ca_model = 0.25;
    double prob_cd_model = 0.25;
    
};

#endif /* PREDICTION_H */