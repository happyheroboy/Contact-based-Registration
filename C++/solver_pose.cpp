#include "solver_pose.h"
#include "ceres/rotation.h"

struct SnavelyReprojectionError
{
    cv::Point2d observed;
    camMatrix camSys_param;
    //CameraParams cam;
    cv::Point3d point_ID;

    SnavelyReprojectionError(Point2d observation, camMatrix camSys_param, Point3d point_ID) :observed(observation), camSys_param(camSys_param), point_ID(point_ID) {}

    template <typename T>
    bool operator()(const T* const rotation,
        const T* const translation,
        T* residuals)const
    {
        T predictions[2], pos_proj1[3], pos_world[3], pos_proj[3];
        pos_world[0] = T(point_ID.x);
        pos_world[1] = T(point_ID.y);
        pos_world[2] = T(point_ID.z);
        //ceres::QuaternionRotatePoint(rotation, pos_world, pos_proj1);
        ceres::AngleAxisRotatePoint(rotation, pos_world, pos_proj1);
        pos_proj1[0] += translation[0];
        pos_proj1[1] += translation[1];
        pos_proj1[2] += translation[2];
        for (int i = 0; i < 3; i++)
        {
            pos_proj[i] = T(camSys_param(i, 0)) * pos_proj1[0] + T(camSys_param(i, 1)) * pos_proj1[1] + T(camSys_param(i, 2)) * pos_proj1[2] + T(camSys_param(i, 3));
        }
        predictions[0] = pos_proj[0] / pos_proj[2];
        predictions[1] = pos_proj[1] / pos_proj[2];
        residuals[0] = predictions[0] - T(observed.x);
        residuals[1] = predictions[1] - T(observed.y);
        //cout << "residuals[0]\n" <<residuals[0] << "\n" << " residuals[1]\n  " << residuals[1] << endl;
        /*residuals[0] = predictions[0] - T(observed.x);
        residuals[1] = predictions[1] - T(observed.y);*/
        return true;
    }

    static ceres::CostFunction* Create(Point2d observed, camMatrix camSys_param, Point3d point_ID) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 3>(
            new SnavelyReprojectionError(observed, camSys_param, point_ID)));
    }
};

solver_pose::solver_pose(const string& camParam_path, const string& model_path)
{
    load_camParam(camParam_path);
    load_model(model_path);
    check_param();
    initial();
}

void solver_pose::initial()
{
    for (int i = 0; i < this->model.fPts.size(); i++)
        this->map_fID2mIdx.insert(pair<int, int>(this->model.fPts[i][0], i));
}

void solver_pose::conclude(const model_info& m_info, const MatRT& RT, vector<vector<Vec3f>>& repro_uv, vector<double>& repro_e)
{
    repro_uv.clear();
    repro_e.clear();

    repro_uv = vector<vector<Vec3f>>(m_info.cam_IDuv.size(), vector<Vec3f>());
    repro_e = vector<double>(m_info.cam_IDuv.size(), -1);

    cout << "repro_e";
    TRY_THROW(m_info.cam_IDuv.size() == m_info.cam_xyz.size(), "the size of camIDuv and camxyz must be the same!");
    for (int i = 0; i < m_info.cam_IDuv.size(); i++)
    {
        if (m_info.cam_IDuv[i].empty())   continue;
        if (m_info.cam_xyz[i].empty())   continue;

        Mat1d mat_cam_IDuv = vec2mat(m_info.cam_IDuv[i]);
        Mat1d mat_est_xyz = vec2mat(m_info.cam_xyz[i]);
        
        TRY_THROW(mat_cam_IDuv.cols == mat_est_xyz.cols, "the size of uv and xyz must be the same!");

        Mat1d uv, e;
        repro(mat_cam_IDuv.rowRange(1, 3), mat_est_xyz, RT, this->camSys_param[i], uv, e);
        
        for (int j = 0; j < m_info.cam_IDuv[i].size(); j++)
            repro_uv[i].push_back(Vec3f(m_info.cam_IDuv[i][j](0), uv(0, j), uv(1, j)));

        repro_e[i] = mean(e)(0);
        cout << ", " << repro_e[i];
    }
    cout << endl;
       Mat1d model_key_IDxyz = vec2mat(model.keyPts);
       Mat1d model_key_xyz;
       model_key_IDxyz.rowRange(1, 4).copyTo(model_key_xyz);
       for (int j = 0; j < model.keyPts.size(); j++)
       {
           Mat1d repro__key_uv0 = remove_homo(camSys_param[0] * add_homo(RT * add_homo(model_key_xyz.col(j))));
           Mat1d repro__key_uv01 = remove_homo(camSys_param[1] * add_homo(RT * add_homo(model_key_xyz.col(j))));
           repro_uv[0].push_back(Vec3f(10000, repro__key_uv0(0, 0), repro__key_uv0(1, 0)));
           repro_uv[1].push_back(Vec3f(10000, repro__key_uv01(0, 0), repro__key_uv01(1, 0)));
       }
       //Mat1d repro__key_uv0 = remove_homo(camSys_param[0] * add_homo(RT * add_homo(model_key_xyz.col(0))));
       //Mat1d repro__key_uv1 = remove_homo(camSys_param[0] * add_homo(RT * add_homo(model_key_xyz.col(1))));
       //Mat1d repro__key_uv01 = remove_homo(camSys_param[1] * add_homo(RT * add_homo(model_key_xyz.col(0))));
       //Mat1d repro__key_uv11 = remove_homo(camSys_param[1] * add_homo(RT * add_homo(model_key_xyz.col(1))));
       //repro_uv[0].push_back(Vec3f(10000, repro__key_uv0(0, 0), repro__key_uv0(1, 0)));
       //repro_uv[0].push_back(Vec3f(20000, repro__key_uv1(0, 0), repro__key_uv1(1, 0)));
       //repro_uv[1].push_back(Vec3f(10000, repro__key_uv01(0, 0), repro__key_uv01(1, 0)));
       //repro_uv[1].push_back(Vec3f(20000, repro__key_uv11(0, 0), repro__key_uv11(1, 0)));
//cout << "P:\n" << camSys_param[0] << endl;
//cout << "RT:\n" << RT << endl;
//cout << "repro__key_uv0:\n" << repro__key_uv0 << endl;
//cout << "repro__key_uv0:\n" << repro__key_uv1 << endl;
}

void solver_pose::load_camParam(const string path)
{
    // 打开文件
    TRY_THROW(check_suffix(path, ".camParam"), "the file type must be .camParam");
    ifstream input_file(path);
    TRY_THROW(input_file.is_open(), "fail to open camParam file!");

    // 读 camParam
    int camNum;
    input_file >> camNum;

    camMatrix P;
    for (int k = 0; k < camNum; k++)
    {
        Mat1d P = Mat1d::zeros(3, 4);
        for (int i = 0; i < 12; i++)
            input_file >> P(i);
        this->camSys_param.push_back(P);
    }
}

void solver_pose::load_model(const string path)
{
    // 打开文件
    TRY_THROW(check_suffix(path, ".model"), "the file type must be .model!");
    ifstream input_file(path);
    TRY_THROW(input_file.is_open(), "fail to open model file!");

    // 读 model
    int fNum, keyNum;
    input_file >> fNum;
    input_file >> keyNum;
    for (int i = 0; i < fNum; i++)
    {
        Vec4d info;
        input_file >> info[0] >> info[1] >> info[2] >> info[3];
        this->model.fPts.push_back(info);
        this->maxID = info[0] > this->maxID ? info[0] : this->maxID;
    }
    for (int i = 0; i < keyNum; i++)
    {
        Vec4d info;
        input_file >> info[0] >> info[1] >> info[2] >> info[3];
        this->model.keyPts.push_back(info);
    }
}

void solver_pose::check_param()
{
    // 文件的行列是否符合事实(尾部数据是否为零等)
    
}

void solver_pose::to_model_sys(const vector<vector<Vec3f>>& cam_fList, map<int, int>& map_fID2mIdx, model_info& info)
{
    // initial
    info.cam_IDuv.clear();
    info.cam_xyz.clear();
    info.model_IDuv.clear();
    info.model_idx.clear();

    for (int i = 0; i < cam_fList.size(); i++)
    {
        info.cam_IDuv.push_back(vector<Vec3f>{});
        info.cam_xyz.push_back(vector<Vec3f>{});
    }
    for (int i = 0; i < this->model.fPts.size(); i++)  
        info.model_IDuv.push_back(vector<Vec4d>{});
        
    // get model_IDuv
    int valid_ob_num = 0;
    for (int cam = 0; cam < cam_fList.size(); cam++)
        for (Vec3d f : cam_fList[cam])
        {
            map<int, int>::iterator it = map_fID2mIdx.find(f[0]);
            if (it == map_fID2mIdx.end())   continue;
            valid_ob_num++;
            info.cam_IDuv[cam].push_back(f);
            info.cam_xyz[cam].push_back(Vec3d(this->model.fPts[it->second][1], this->model.fPts[it->second][2], this->model.fPts[it->second][3]));
            info.model_IDuv[it->second].push_back(Vec4d(cam, f[0], f[1], f[2]));
        }
    info.ob_num = valid_ob_num;

    // get model_idx
    for (int idx = 0; idx < info.model_IDuv.size(); idx++)
    {
        if (info.model_IDuv[idx].empty())    continue;
        info.model_idx.push_back(idx);
    }

    // get model_IDxyz
    info.model_IDxyz = Mat1d::zeros(4, info.model_idx.size());
    for (int i = 0; i < info.model_idx.size(); i++)
    {
        info.model_IDxyz(0, i) = this->model.fPts[info.model_idx[i]](0);
        info.model_IDxyz(1, i) = this->model.fPts[info.model_idx[i]](1);
        info.model_IDxyz(2, i) = this->model.fPts[info.model_idx[i]](2);
        info.model_IDxyz(3, i) = this->model.fPts[info.model_idx[i]](3);
    }
}

void solver_pose::to_cPts_system(const model_info& m_info, cPts_info& info)
{
    // get model_controlPts
    TIME_START(cost_to_get_model_controlPts_time);
    get_model_controlPts(m_info, info.model_controlPts);
    TIME_END(cost_to_get_model_controlPts_time);
    cout << "time get_model_controlPts :" << cost_to_get_model_controlPts_time << endl;
    // get model_observed_IDBC
    TIME_START(cost_to_get_BaryCentricCoordinates_time);
    get_BaryCentricCoordinates(m_info, info, info.model_observed_IDBC, info.map_ID2BC);

    TIME_END(cost_to_get_BaryCentricCoordinates_time);
    cout << "time cost_to_get_BaryCentricCoordinates_time :" << cost_to_get_BaryCentricCoordinates_time << endl;
}

void solver_pose::apply(Data::poseSolver& data,const int isfeed, MatRT & BA_RT)
{
    // 标记执行
    data.reset_output();

    // 检查与预备
    TRY_THROW(data.in_cam_fList.size() == camSys_param.size(), "the size of cam_fList must equal to the number of cameras!");
    if (data.in_cam_fList.empty())  return;

    // 算法计时
    TIME_START(time_all);

    this->beta_iter_num = data.p_iter;
    
    // 转到模型视角、控制点视角
    model_info m_info_single, m_info_both;
    to_model_sys(data.in_cam_fList, this->map_fID2mIdx, m_info_both);
    if (data.in_cam_fList[1].empty())   return;
    to_model_sys(vector<vector<Vec3f>>{vector<Vec3f>{},data.in_cam_fList[1]}, this->map_fID2mIdx, m_info_single);
    
    if (m_info_both.model_idx.size() < 6)  return;

    MatRT RT;
    TIME_START(cost_mpnp_time);
    mPnP(m_info_both, RT);
    TIME_END(cost_mpnp_time);
    cout<<"时间:"<< cost_mpnp_time<<endl;
    //准备BA算法    
    //MatRT BA_RT = bundleAdjustment(m_info_both, RT);
    BA_RT = bundleAdjustment(m_info_both, RT);
    //kalman滤波
    Mat1d fliter_sys_keypoints= tracker.feed(BA_RT, model.keyPts,isfeed);

    conclude(m_info_both, BA_RT, data.d_repro_fList, data.d_repro_e);
     
    TIME_END(time_all);
    get_sys_Key_point(fliter_sys_keypoints, BA_RT, data.sys_key_points);
    // 记录调试用输出
    data.d_time_cost.push_back(Data::time_cost("PnP", time_all));
    cout << "RT\n" << BA_RT << endl;
    cout <<"data.sys_key_point1:\n"<< data.sys_key_points[0] << endl;
    cout << "data.sys_key_point2:\n" << data.sys_key_points[1] << endl;
}

void solver_pose::mPnP(const model_info& m_info, MatRT& RT)
{
    cPts_info c_info;
    TIME_START(cost_to_cPts_system_time);
    to_cPts_system(m_info, c_info);
    TIME_END(cost_to_cPts_system_time);
    cout << "时间 to_cPts_system :" << cost_to_cPts_system_time << endl;
    sys_info s_info;
    TIME_START(cost_to_to_sys_system_time);
    to_sys_system(m_info, c_info, s_info);
    TIME_END(cost_to_to_sys_system_time);
    cout << "时间 to_sys_system :" << cost_to_to_sys_system_time << endl;

    TIME_START(cost_to_RT_time);
    MatRT feed_RT;
    RT = get_RT(m_info, c_info, s_info);
    TIME_END(cost_to_RT_time);
    cout << "时间 to_sys_system :" << cost_to_RT_time << endl;
}

void solver_pose::get_model_controlPts(const model_info& m_info, Mat1d& model_controlPts)
{
    model_controlPts = Mat1d::zeros(3, 4);
    
    // 第一个控制点，重心
    Mat1d mean_center;
    reduce(m_info.model_IDxyz.rowRange(1, 4), mean_center, 1, REDUCE_AVG);
    mean_center.copyTo(model_controlPts.col(0));

    // 重心移动到原点
    Mat1d centered_Pts;
    m_info.model_IDxyz.rowRange(1, 4).copyTo(centered_Pts);
    centered_Pts.row(0) = centered_Pts.row(0) - mean_center(0);
    centered_Pts.row(1) = centered_Pts.row(1) - mean_center(1);
    centered_Pts.row(2) = centered_Pts.row(2) - mean_center(2);

    // 计算特征方向与特征值
    Mat1d covE = centered_Pts * centered_Pts.t();
    Mat1d w, u, vt;
    SVD::compute(covE, w, u, vt, SVD::FULL_UV);

    for (int in = 1; in < 4; in++)
    {
        double k = sqrt(w(in - 1) / m_info.model_idx.size());
        Mat1d pt = (Mat1d(3,1) << mean_center(0) + k * vt(in - 1, 0),
                                  mean_center(1) + k * vt(in - 1, 1),
                                  mean_center(2) + k * vt(in - 1, 2));
        pt.copyTo(model_controlPts.col(in));
    }
}

void solver_pose::get_BaryCentricCoordinates(const model_info& m_info, const cPts_info& c_info, Mat1d& model_IDBC, Mat1d& map_ID2BC)
{
    model_IDBC = Mat1d::zeros(5, m_info.model_idx.size());
    map_ID2BC = Mat1d::zeros(4, this->maxID+1);
    int ptNum = m_info.model_IDxyz.cols;
    Mat1d controlPts = add_homo(c_info.model_controlPts);
    Mat1d pts = add_homo(m_info.model_IDxyz.rowRange(1, 4));

    //invert(controlPts, controlPts, DECOMP_SVD);
    Mat1d BCxyz = (controlPts * pts).t();

    cv::solve(controlPts, pts, BCxyz, DECOMP_LU | DECOMP_NORMAL);

    m_info.model_IDxyz.row(0).copyTo(model_IDBC.row(0));
    BCxyz.copyTo(model_IDBC.rowRange(1, 5));

    for (int i = 0; i < model_IDBC.cols; i++)
    {
        map_ID2BC(0, model_IDBC(0, i)) = model_IDBC(1, i);
        map_ID2BC(1, model_IDBC(0, i)) = model_IDBC(2, i);
        map_ID2BC(2, model_IDBC(0, i)) = model_IDBC(3, i);
        map_ID2BC(3, model_IDBC(0, i)) = model_IDBC(4, i);
    }
}

void solver_pose::to_sys_system(const model_info& m_info, const cPts_info& c_info, sys_info& info)
{
    // get M
    Mat1d Mb;
    get_Mb(m_info, c_info.map_ID2BC, Mb);

    get_sys_controlPts(Mb, c_info.model_controlPts, info.system_controlPts);

    // get sys_IDxyz
    info.sys_IDxyz = Mat1d::zeros(4, c_info.model_observed_IDBC.cols);
    for (int i = 0; i < c_info.model_observed_IDBC.cols; i++)
    {
        Mat1d xyz = c_info.model_observed_IDBC(1, i) * info.system_controlPts.col(0) 
                    + c_info.model_observed_IDBC(2, i) * info.system_controlPts.col(1)
                    + c_info.model_observed_IDBC(3, i) * info.system_controlPts.col(2)
                    + c_info.model_observed_IDBC(4, i) * info.system_controlPts.col(3);
        info.sys_IDxyz(0, i) = c_info.model_observed_IDBC(0, i);
        info.sys_IDxyz(1, i) = xyz(0);
        info.sys_IDxyz(2, i) = xyz(1);
        info.sys_IDxyz(3, i) = xyz(2);
    }
}

void solver_pose::get_Mb(const model_info& m_info, const Mat1d& map_ID2BC, Mat1d& Mb)
{
    Mb = Mat1b::zeros(2 * m_info.ob_num, 13);
    int i = 0;
    for (vector<Vec4d> ptOB : m_info.model_IDuv)
    {
        for (Vec4d info : ptOB)
        {
            double u = info[2];
            double v = info[3];
            camMatrix P = this->camSys_param[info[0]];

            Mat1d u_row = (Mat1d(1, 3) << u * P(2, 0) - P(0, 0), u * P(2, 1) - P(0, 1), u * P(2, 2) - P(0, 2));
            Mat1d v_row = (Mat1d(1, 3) << v * P(2, 0) - P(1, 0), v * P(2, 1) - P(1, 1), v * P(2, 2) - P(1, 2));

            Mat1d rowU(1, 13), rowV(1, 13);
            rowU.colRange(0, 3)  = map_ID2BC(0, info[1]) * u_row;
            rowU.colRange(3, 6)  = map_ID2BC(1, info[1]) * u_row;
            rowU.colRange(6, 9)  = map_ID2BC(2, info[1]) * u_row;
            rowU.colRange(9, 12) = map_ID2BC(3, info[1]) * u_row;
            rowV.colRange(0, 3)  = map_ID2BC(0, info[1]) * v_row;
            rowV.colRange(3, 6)  = map_ID2BC(1, info[1]) * v_row;
            rowV.colRange(6, 9)  = map_ID2BC(2, info[1]) * v_row;
            rowV.colRange(9, 12) = map_ID2BC(3, info[1]) * v_row;
            rowU(12) = -(u * P(2, 3) - P(0, 3));
            rowV(12) = -(v * P(2, 3) - P(1, 3));

            rowU.copyTo(Mb.row(i++));
            rowV.copyTo(Mb.row(i++));
        }
    }
}

void solver_pose::get_sys_controlPts(const Mat1d& Mb, const Mat1d& model_controlPts, Mat1d& sys_controlPts)
{
    sys_controlPts = Mat1d::zeros(3, 4);

    // get solution and right null-space
    Mat1d x;    // 基础解
    cv::solve(Mb.colRange(0, 12), Mb.col(12), x, DECOMP_LU |DECOMP_NORMAL);

    Mat1d covM = Mb.colRange(0, 12).t() * Mb.colRange(0, 12);   // TODO: 右零空间使用EIGEN计算会不会更快更准
    Mat1d w, u, vt;
    SVD::compute(covM, w, u, vt, SVD::FULL_UV);

    Mat1d v = vt.rowRange(8, 12).t();
    flip(v, v, 1);  // v - 4个右零空间，列从左到右、特征值从小到大

    // get S
    Mat1d SxSx = Mat1d::zeros(6, 1);
    Mat1d SxS1 = Mat1d::zeros(6, 1);
    Mat1d SxS2 = Mat1d::zeros(6, 1);
    Mat1d SxS3 = Mat1d::zeros(6, 1);
    Mat1d SxS4 = Mat1d::zeros(6, 1);
    Mat1d S1S1 = Mat1d::zeros(6, 1);
    Mat1d S1S2 = Mat1d::zeros(6, 1);
    Mat1d S1S3 = Mat1d::zeros(6, 1);
    Mat1d S1S4 = Mat1d::zeros(6, 1);
    Mat1d S2S2 = Mat1d::zeros(6, 1);
    Mat1d S2S3 = Mat1d::zeros(6, 1);
    Mat1d S2S4 = Mat1d::zeros(6, 1);
    Mat1d S3S3 = Mat1d::zeros(6, 1);
    Mat1d S3S4 = Mat1d::zeros(6, 1);
    Mat1d S4S4 = Mat1d::zeros(6, 1);
    for (int a = 0, i = 0; a < 3; a++)
    {
        for (int b = a + 1; b < 4; b++, i++)
        {
            Mat1d Sx = x.rowRange(3 * a, 3 * a + 3) - x.rowRange(3 * b, 3 * b + 3);
            Mat1d S1 = v.col(0).rowRange(3 * a, 3 * a + 3) - v.col(0).rowRange(3 * b, 3 * b + 3);
            Mat1d S2 = v.col(1).rowRange(3 * a, 3 * a + 3) - v.col(1).rowRange(3 * b, 3 * b + 3);
            Mat1d S3 = v.col(2).rowRange(3 * a, 3 * a + 3) - v.col(2).rowRange(3 * b, 3 * b + 3);
            Mat1d S4 = v.col(3).rowRange(3 * a, 3 * a + 3) - v.col(3).rowRange(3 * b, 3 * b + 3);
            SxSx(i) = Sx.dot(Sx);
            SxS1(i) = Sx.dot(S1);
            SxS2(i) = Sx.dot(S2);
            SxS3(i) = Sx.dot(S3);
            SxS4(i) = Sx.dot(S4);
            S1S1(i) = S1.dot(S1);
            S1S2(i) = S1.dot(S2);
            S1S3(i) = S1.dot(S3);
            S1S4(i) = S1.dot(S4);
            S2S2(i) = S2.dot(S2);
            S2S3(i) = S2.dot(S3);
            S2S4(i) = S2.dot(S4);
            S3S3(i) = S3.dot(S3);
            S3S4(i) = S3.dot(S4);
            S4S4(i) = S4.dot(S4);
        }
    }

    // get L, rho
    Mat1d L(6,5), rho(6,1);
    for (int a = 0, i = 0; a < 3; a++)
    {
        for (int b = a + 1; b < 4; b++, i++)
        {
            L(i, 0) = 2 * SxS1(i);
            L(i, 1) = 2 * SxS2(i);
            L(i, 2) = S1S1(i);
            L(i, 3) = S2S2(i);
            L(i, 4) = 2 * S1S2(i);

            rho(i) = pow(norm(model_controlPts.col(a) - model_controlPts.col(b)), 2) - SxSx(i);
        }
    }

    // get beta
    Mat1d betaComp;
    cv::solve(L, rho, betaComp, DECOMP_SVD | DECOMP_NORMAL);

    Mat1d beta = Mat1d::zeros(4, 1);
    beta(0) = sqrt(abs(betaComp(2)));
    beta(1) = betaComp(4) / beta(0);
    
    // refine beta
    for (int iter = 0; iter < this->beta_iter_num; iter++)
    {
        Mat1d J(6, 4), e(6, 1);
        for (int a = 0, i = 0; a < 3; a++)
        {
            for (int b = a + 1; b < 4; b++, i++)
            {
                J(i, 0) = 2 * SxS1(i) + 2 * beta(0) * S1S1(i) + 2 * beta(1) * S1S2(i) + 2 * beta(2) * S1S3(i) + 2 * beta(3) * S1S4(i);
                J(i, 1) = 2 * SxS2(i) + 2 * beta(0) * S1S2(i) + 2 * beta(1) * S2S2(i) + 2 * beta(2) * S2S3(i) + 2 * beta(3) * S2S4(i);
                J(i, 2) = 2 * SxS3(i) + 2 * beta(0) * S1S3(i) + 2 * beta(1) * S2S3(i) + 2 * beta(2) * S3S3(i) + 2 * beta(3) * S3S4(i);
                J(i, 3) = 2 * SxS4(i) + 2 * beta(0) * S1S4(i) + 2 * beta(1) * S2S4(i) + 2 * beta(2) * S3S4(i) + 2 * beta(3) * S4S4(i);

                e(i) = SxSx(i) + 2 * beta(0) * SxS1(i) + 2 * beta(1) * SxS2(i) + 2 * beta(2) * SxS3(i) + 2 * beta(3) * SxS4(i) +
                    beta(0) * beta(0) * S1S1(i) + beta(1) * beta(1) * S2S2(i) + beta(2) * beta(2) * S3S3(i) + beta(3) * beta(3) * S4S4(i) +
                    2 * beta(0) * beta(1) * S1S2(i) + 2 * beta(0) * beta(2) * S1S3(i) + 2 * beta(0) * beta(3) * S1S4(i) +
                    2 * beta(1) * beta(2) * S2S3(i) + 2 * beta(1) * beta(3) * S2S4(i) + 2 * beta(2) * beta(3) * S3S4(i) -
                    pow(norm(model_controlPts.col(a) - model_controlPts.col(b)), 2);
            }
        }
        Mat1d beta_shift;
        cv::solve(J, -e, beta_shift, DECOMP_LU | DECOMP_NORMAL);
        beta = beta + beta_shift;
    }

    // 系统控制点必须在系统坐标系z轴的前方（对系统坐标系的设置提出了要求）
    Mat1d controlPts_repo;
    controlPts_repo = x + beta(0) * v.col(0) + beta(1) * v.col(1) + beta(2) * v.col(2) + beta(3) * v.col(3);
    if (controlPts_repo(2) < 0)
        controlPts_repo = x - beta(0) * v.col(0) - beta(1) * v.col(1) - beta(2) * v.col(2) - beta(3) * v.col(3);

    Mat A;
    controlPts_repo.rowRange(0, 3).copyTo(sys_controlPts.col(0));
    controlPts_repo.rowRange(3, 6).copyTo(sys_controlPts.col(1));
    controlPts_repo.rowRange(6, 9).copyTo(sys_controlPts.col(2));
    controlPts_repo.rowRange(9, 12).copyTo(sys_controlPts.col(3));
}

MatRT solver_pose::get_RT(const model_info& m_info, const cPts_info& c_info, const sys_info& s_info)
{    
    MatRT RT;

    // centering
    Mat1d sysF_c, modF_c;
    
    repeat(s_info.system_controlPts.col(0), 1, s_info.sys_IDxyz.cols, sysF_c);
    repeat(c_info.model_controlPts.col(0), 1, m_info.model_IDxyz.cols, modF_c);
    Mat1d sysF_ = s_info.sys_IDxyz.rowRange(1, 4) - sysF_c;
    Mat1d modF_ = m_info.model_IDxyz.rowRange(1, 4) - modF_c;
    
    // get RT
    Mat1d w, u, vt;
    SVD::compute(sysF_ * modF_.t(), w, u, vt, SVD::FULL_UV);
    Mat1d R = u * vt;
    double detR = determinant(R);
    if (detR < 0)
    {
        vt.row(2) = -vt.row(2);
        R = u * vt;
    }
    Mat1d T = s_info.system_controlPts.col(0) - R * c_info.model_controlPts.col(0);

    hconcat(R, T, RT);

    return RT;
}

void solver_pose::repro(const Mat1d& base_uv, const Mat1d& est_xyz, const MatRT& RT, const camMatrix& P, Mat1d& repro_uv, Mat1d& repro_e)
{
    TRY_THROW(est_xyz.cols == base_uv.cols, "the size of base_uv and est_xyz must be the same!");

    // 重投影
    repro_uv = remove_homo(P * add_homo(RT * add_homo(est_xyz)));   

    // 重投影误差
    repro_e = Mat1d(1, repro_uv.cols);
    for (int in = 0; in < repro_uv.cols; in++)
        repro_e(in) = norm(repro_uv.col(in), base_uv.col(in), NORM_L2);
}

Mat1d solver_pose::RT2Rodrigues_t(const MatRT& RT)
{
    Vec3d rvec;
    //double x, y, z;
    Mat1d R(3, 3, CV_64F);
    Mat1d(RT.col(0)).copyTo(R.col(0));
    Mat1d(RT.col(1)).copyTo(R.col(1));
    Mat1d(RT.col(2)).copyTo(R.col(2));
    Mat1d T(3, 1, CV_64F);
    Mat1d(RT.col(3)).copyTo(T);
    Rodrigues(R, rvec);
    return(Mat1d(6, 1) << rvec(0), rvec(1), rvec(2), T(0), T(1), T(2));
}

MatRT solver_pose::bundleAdjustment(const model_info& m_info, const MatRT& RT)
{
    Problem problem;
    euler_trans = RT2Rodrigues_t(RT);
    rot[0] = euler_trans.at<double>(0, 0);
    rot[1] = euler_trans.at<double>(1, 0);
    rot[2] = euler_trans.at<double>(2, 0);
    trans[0] = euler_trans.at<double>(3, 0);
    trans[1] = euler_trans.at<double>(4, 0);
    trans[2] = euler_trans.at<double>(5, 0);
    buildProblem(&problem, m_info);
    TIME_START(time_BA);
    Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
    options.gradient_tolerance = 1e-14;
    options.function_tolerance = 1e-14;
    options.parameter_tolerance = 1e-10;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    TIME_END(time_BA);
    //cout << "timeall:   " << time_BA << endl;
    //std::cout << summary.FullReport() << "\n";
    euler_trans.at<double>(0, 0) = rot[0];
    euler_trans.at<double>(1, 0) = rot[1];
    euler_trans.at<double>(2, 0) = rot[2];
    euler_trans.at<double>(3, 0) = trans[0];
    euler_trans.at<double>(4, 0) = trans[1];
    euler_trans.at<double>(5, 0) = trans[2];
    MatRT rt_mat;
    rt_mat = Rodrigues2RT(euler_trans);
    return rt_mat;
}

void solver_pose::buildProblem(Problem* problem, const model_info& m_info) {
    rot[0] = euler_trans.at<double>(0, 0);
    rot[1] = euler_trans.at<double>(1, 0);
    rot[2] = euler_trans.at<double>(2, 0);
    trans[0] = euler_trans.at<double>(3, 0);
    trans[1] = euler_trans.at<double>(4, 0);
    trans[2] = euler_trans.at<double>(5, 0);

    //Mat1d model_IDXYZ = vec2mat(m_info.model_IDxyz);
    Mat1d model_ID;
    m_info.model_IDxyz.row(0).copyTo(model_ID);

    for (int j = 0; j < m_info.cam_IDuv.size(); j++)
    {
        if (m_info.cam_IDuv[j].size() == 0)
        {
            continue;
        }
        else
        {
            for (int i = 0; i < m_info.cam_IDuv[j].size(); i++)
            {
                double ID = m_info.cam_IDuv[j][i][0];
                Point2d subpoint;
                subpoint.x = m_info.cam_IDuv[j][i][1];
                subpoint.y = m_info.cam_IDuv[j][i][2];
                for (int k = 0; k < m_info.model_IDxyz.cols; k++)
                {
                    if (ID == model_ID.at<double>(k))
                    {
                        CostFunction* cost_function;
                        cost_function = SnavelyReprojectionError::Create((Point2d)subpoint, camSys_param[j], Point3d(m_info.model_IDxyz(1, k), m_info.model_IDxyz(2, k), m_info.model_IDxyz(3, k)));
                        problem->AddResidualBlock(cost_function, NULL, rot, trans);
                    }
                }
            }

        }
    }
}

MatRT solver_pose::Rodrigues2RT(const Mat1d& euler_trans)
{
    Mat r_mat;
    Vec3d Rodrigues_rot;
    Rodrigues_rot(0) = euler_trans.at<double>(0, 0);
    Rodrigues_rot(1) = euler_trans.at<double>(1, 0);
    Rodrigues_rot(2) = euler_trans.at<double>(2, 0);
    Rodrigues(Rodrigues_rot, r_mat);
    //r_mat=getRotation(quater);
    Mat1d t_mat(3, 1, CV_64F);
    t_mat << euler_trans.at<double>(3, 0), euler_trans.at<double>(4, 0), euler_trans.at<double>(5, 0);
    MatRT rt_mat;
    hconcat(r_mat, t_mat, rt_mat);
    return rt_mat;
}

void solver_pose::get_sys_Key_point(const Mat1d fliter_sys_keypoints, const MatRT RT, vector<Mat1d>& sys_key_points)
{
    sys_key_points.clear();
    Mat1d model_key_IDxyz = vec2mat(model.keyPts);
    Mat1d model_key_xyz;
    model_key_IDxyz.rowRange(1, 4).copyTo(model_key_xyz);
    for (int j = 0; j < model.keyPts.size(); j++)
    {
        Mat1d keypoint1 = (Mat1d(3, 1) << fliter_sys_keypoints(3*j, 0), fliter_sys_keypoints(3 * j+1, 0), fliter_sys_keypoints(3 * j+2, 0));
         sys_key_points.push_back(keypoint1);
    }
    //Mat1d keypoint1 = (Mat1d(3, 1) << fliter_sys_keypoints(0, 0), fliter_sys_keypoints(1, 0), fliter_sys_keypoints(2, 0));
    //sys_key_points.push_back(keypoint1);
    //Mat1d keypoint2 = (Mat1d(3, 1) << fliter_sys_keypoints(3, 0), fliter_sys_keypoints(4, 0), fliter_sys_keypoints(5, 0));
    //sys_key_points.push_back(keypoint2);
    //for (int i = 0; i < model_key_xyz.cols; i++)
    //{
    //    Mat1d syspoints= RT * add_homo(model_key_xyz.col(i));
    //    sys_key_points.push_back(syspoints);
    //}

    //cout << "bijianzuobiao:\n" << sys_key_points[1] << endl;
}