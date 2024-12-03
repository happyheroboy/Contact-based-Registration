#include "track_points_KF.h"

track_points_KF::track_points_KF()
{
    
    Mat1d empty_state = Mat1d::zeros(12, 1);
    initial_KF_static(this->KF, empty_state);

}

Mat1d track_points_KF::feed(const MatRT& RT,const vector<cv::Vec4d> model_key_points, const int & isfeed)
{
    Mat1d measure_points = mesure_point(RT,model_key_points);

    //// 判断和上一次的距离
    //Vec6d last_state = Vec6d(KF.statePost);
    //Vec6d diff = last_state  - state;
    //Vec6d a(state(0),state(1), last_state(0), last_state(1), diff(0), diff(1));
    //    cout << a << endl;
    if (isfeed == 1)
    {
        Mat1d diff = state2measure(KF.statePost) - measure_points;
        int pow = (abs(diff(0)) > this->xy_T || abs(diff(1)) > this->xy_T);
        Mat1d new_state = update_KF_static(this->KF, measure_points, pow);

        //cout << new_state << endl;
        Mat1d fliter_sys_key_points = state2measure(new_state);
        //MatRT  RT_KF= state2RT(new_state, model_key_points);
        return fliter_sys_key_points;
    }
    else if (isfeed == 0)
    {
        return measure_points;
    }
    //cout << "measure_points:\n" << measure_points << endl;
    //cout << "measure_fliterRT:\n" << measure_fliterRT << endl;
    
    
}

Mat1d track_points_KF::read()
{

    return state2measure(this->KF.statePost);
}

void track_points_KF::initial_KF_static(KalmanFilter& KF, const Mat1d& initial_state)
{
    double dt = 1;
    KF.init(12, 6, 0, CV_64F);         // init Kalman Filter

    setIdentity(KF.processNoiseCov, Scalar::all(2e-4));       // set process noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));   // set measurement noise
    setIdentity(KF.errorCovPost, Scalar::all(1));             // error covariance

    // trans matrix
    KF.transitionMatrix = Mat::eye(Size(12, 12), CV_64F);
    KF.transitionMatrix.at<double>(0, 3) = 1;
    KF.transitionMatrix.at<double>(1, 4) = 1;
    KF.transitionMatrix.at<double>(2, 5) = 1;
    KF.transitionMatrix.at<double>(6, 9) = 1;
    KF.transitionMatrix.at<double>(7, 10) = 1;
    KF.transitionMatrix.at<double>(8, 11) = 1;

    // measure matrix
    KF.measurementMatrix.at<double>(0, 0) = 1;
    KF.measurementMatrix.at<double>(1, 1) = 1;
    KF.measurementMatrix.at<double>(2, 2) = 1;
    KF.measurementMatrix.at<double>(3, 6) = 1;
    KF.measurementMatrix.at<double>(4, 7) = 1;
    KF.measurementMatrix.at<double>(5, 8) = 1;

    KF.statePost = initial_state;
}

Mat1d track_points_KF::update_KF_static(KalmanFilter& KF, const Mat1d& measure, int pow)
{
    if (pow == 0)
    {
        KF.transitionMatrix.at<double>(0, 3) = 0;
        KF.transitionMatrix.at<double>(1, 4) = 0;
        KF.transitionMatrix.at<double>(2, 5) = 0;
        KF.transitionMatrix.at<double>(6, 9) = 0;
        KF.transitionMatrix.at<double>(7, 10) = 0;
        KF.transitionMatrix.at<double>(8, 11) = 0;
    }
    else
    {
        KF.transitionMatrix.at<double>(0, 3) = 1;
        KF.transitionMatrix.at<double>(1, 4) = 1;
        KF.transitionMatrix.at<double>(2, 5) = 1;
        KF.transitionMatrix.at<double>(6, 9) = 1;
        KF.transitionMatrix.at<double>(7, 10) = 1;
        KF.transitionMatrix.at<double>(8, 11) = 1;
    }
        KF.predict();
        Mat1d s = KF.correct(measure); //会修改statePost
    return s;
}

Mat1d track_points_KF::mesure_point(const MatRT& RT, const vector<cv::Vec4d> model_key_points)
{
    int menber_key_point= model_key_points.size();
    Mat1d measure(menber_key_point*3,1,CV_64F);
    Mat1d modelIDxyz = vec2mat(model_key_points);
    Mat1d modelxyz;
    modelIDxyz.rowRange(1, 4).copyTo(modelxyz);
    cout << RT << endl;
    Mat1d camerframexyz = RT * add_homo(modelxyz);
    for (int i = 0; i < model_key_points.size(); i++)
    {

       measure.at<double>(3 * i, 0) = camerframexyz.at<double>(0, i);
       measure.at<double>(3 * i+1, 0) = camerframexyz.at<double>(1, i);
       measure.at<double>(3 * i+2, 0) = camerframexyz.at<double>(2, i);
    }
    
    return measure;
}

Mat1d track_points_KF::state2measure(const Mat1d& state)
{

    return (Mat1d(6, 1) << state.at<double>(0,0), state.at<double>(1,0), state.at<double>(2,0), state.at<double>(6,0), state.at<double>(7,0), state.at<double>(8,0));
}

MatRT track_points_KF::state2RT(const Mat1d& state, const vector<cv::Vec4d> model_key_points)
{

    MatRT RT;

    // centering
    Mat1d sysF_c, modF_c;
    Mat1d mean_center_model;
    Mat1d mean_center_sys;
    Mat1d model_points(3, 3, CV_64F);
    Mat1d sys_state_points(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        model_points(0, i) = model_key_points[i][1];
        model_points(1, i) = model_key_points[i][2];
        model_points(2, i) = model_key_points[i][3];
        sys_state_points(0, i)=state(3 * i);
        sys_state_points(1, i) = state(3 * i+1);
        sys_state_points(2, i) = state(3 * i+2);
    }
    reduce(model_points, mean_center_model, 2, REDUCE_AVG);
    reduce(sys_state_points, mean_center_sys, 2, REDUCE_AVG);

    repeat(mean_center_sys, 1, sys_state_points.cols, sysF_c);
    repeat(mean_center_model, 1, model_points.cols, modF_c);

    Mat1d sysF_ =  sys_state_points - sysF_c;
    Mat1d modF_ = model_points - modF_c;

    // get RT
    Mat1d w, u, vt;
    SVD::compute(sysF_ * modF_.t(), w, u, vt, SVD::FULL_UV);
    Mat1d R = u * vt;
    double detR = determinant(R);
    if (detR < 0)   R.row(2) = -R.row(2);
    Mat1d T = mean_center_sys - R * mean_center_model;

    hconcat(R, T, RT);
    return RT;
}
