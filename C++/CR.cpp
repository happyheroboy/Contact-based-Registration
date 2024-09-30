#include "CR.h"
//using namespace CR;

CR::CR(const string filename)
{
    importModel(filename);
}

CR::~CR()
{

}

//����ģ��
void CR::importModel(const string filename)
{
    // ���ص��ƶ���
    acetabulum_cloud.reset(new PointCloud<PointXYZ>);

    // ����ģ�Ͷ���
    pcl::PolygonMesh mesh;

    // ����STLģ���ļ�
    pcl::io::loadPolygonFileSTL(filename, mesh);

    // ��ģ��ת��Ϊ���ƶ���
    pcl::fromPCLPointCloud2(mesh.cloud, *acetabulum_cloud);

    if (acetabulum_cloud->empty())
    {
        cout << "Fail to load model" << endl;
    }

    cout << "has read��" << acetabulum_cloud->size()<<endl;
}

//�Ƕ�ת��������
void CR::angle2Vector()
{/*
  *
  *
  */
    double x, y, z;
    if (isLeft) {
        x = sin(CV_PI / 2 - ((Alpha_angle * CV_PI) / 180)) * sin((Gamma_angle * CV_PI) / 180);
    }
    else {
        x = -sin(CV_PI / 2 - ((Alpha_angle * CV_PI) / 180)) * sin((Gamma_angle * CV_PI) / 180);
    }
    y = -cos(CV_PI / 2 - ((Alpha_angle * CV_PI) / 180));
    z = -sin(CV_PI / 2 - ((Alpha_angle * CV_PI) / 180)) * cos((Gamma_angle * CV_PI) / 180);

    N_Zaxis.push_back(x);
    N_Zaxis.push_back(y);
    N_Zaxis.push_back(z);
}

//����������ϵ��ʹ�žʿ��ڳ���  ��ת�ž�ģ��ʹ���ڳ���
void CR::buildNewCoordinate()
{
    Mat1d R_rotateModel;

    Mat1d Z_axis = N_Zaxis;
    Mat1d temp_axis;
    if(Gamma_angle==90)
    {
         temp_axis = (Mat1d(3, 1) << 0.0, 1.0, 0.0);
    }
    else
    {
         temp_axis = (Mat1d(3, 1) << 1.0, 0.0, 0.0);
    }

    Mat Y_axis = Z_axis.cross(temp_axis);
    Mat X_axis = Z_axis.cross(Y_axis);

    // ��λ��
    normalize(X_axis, X_axis);
    normalize(Y_axis, Y_axis);
    normalize(Z_axis, Z_axis);

    R_rotateModel.push_back(X_axis.t());
    R_rotateModel.push_back(Y_axis.t());
    R_rotateModel.push_back(Z_axis.t());

    transformMatrix << R_rotateModel(0, 0), R_rotateModel(0, 1), R_rotateModel(0, 2), 0,
        R_rotateModel(1, 0), R_rotateModel(1, 1), R_rotateModel(1, 2), 0,
        R_rotateModel(2, 0), R_rotateModel(2, 1), R_rotateModel(2, 2), 0,
        0, 0, 0, 1;

    rotated_acetabulum_cloud.reset(new PointCloud<PointXYZ>);
    //DisplayPointCloud(acetabulum_cloud);
    //����  ���ƺ�R
    //PointCloud<PointXYZ>::Ptr transformed_cloud_1(new PointCloud<PointXYZ>);

    cout << transformMatrix << endl;
    transformPointCloud(*acetabulum_cloud, *rotated_acetabulum_cloud, transformMatrix);
}

//��֤��ת�Ƿ���ȷ
void CR::apply(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& end_pinpoint , Mat1d& R_rotation, Mat1d& t_trans)
{
    int iter_num = 0;
    Mat1d temp_R;
    angle2Vector(); // ��ʼ��������

    Mat1d real_vec = end_pinpoint - real_CP;
    normalize(real_vec, real_vec);

    while(iter_num<10)
    {
        buildNewCoordinate(); //��תģ��

        getVitualContactPt(); // ����Ӵ���λ��

        getRTMatrix(rLeft_ASIS, rRight_ASIS, real_CP, temp_R); //���㲡�˿ռ��ͼ��ռ��ת����ϵ

        iter_num++;

        //���ݵ�����R���ϸ�����������
        N_Zaxis = temp_R.t() * real_vec;

        cout << N_Zaxis << endl;
        cout << temp_R << endl;
    }
    
    t_trans = v_CP - temp_R*real_CP;
    R_rotation = temp_R;

    //��������ʾ/��׼�����ʾ
    tri_display(rLeft_ASIS, rRight_ASIS, real_CP, R_rotation, t_trans);


}

void CR::tri_display(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, const Mat1d& R_Matrix, const Mat1d& t_Matrix)
{
    Mat1d new_rla = R_Matrix * rLeft_ASIS + t_Matrix;
    Mat1d new_rra = R_Matrix * rRight_ASIS + t_Matrix;
    Mat1d new_rcp = R_Matrix * real_CP + t_Matrix;

    PointXYZ rla_p1;
    PointXYZ rra_p1;
    PointXYZ rcp_p1;

    rla_p1.x = new_rla.at<double>(0, 0);
    rla_p1.y = new_rla.at<double>(1, 0);
    rla_p1.z = new_rla.at<double>(2, 0);

    rra_p1.x = new_rra.at<double>(0, 0);
    rra_p1.y = new_rra.at<double>(1, 0);
    rra_p1.z = new_rra.at<double>(2, 0);

    rcp_p1.x = new_rcp.at<double>(0, 0);
    rcp_p1.y = new_rcp.at<double>(1, 0);
    rcp_p1.z = new_rcp.at<double>(2, 0);

    PointXYZ vla_p;
    PointXYZ vra_p;
    vla_p.x = vLeft_ASIS.at<double>(0, 0);
    vla_p.y = vLeft_ASIS.at<double>(1, 0);
    vla_p.z = vLeft_ASIS.at<double>(2, 0);

    vra_p.x = vRight_ASIS.at<double>(0, 0);
    vra_p.y = vRight_ASIS.at<double>(1, 0);
    vra_p.z = vRight_ASIS.at<double>(2, 0);

    //��ʾ
    // �������ӻ�����
    visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // ���õ�����Ⱦ��ɫΪ��ɫ
    visualization::PointCloudColorHandlerCustom<PointXYZ> color_handler(acetabulum_cloud, 0, 255, 0);
    viewer.addPointCloud<PointXYZ>(acetabulum_cloud, color_handler, "cloud");
    // ���ÿ��ӻ�����
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    //���������
    viewer.addCoordinateSystem(50.0, "cloud", 0);
    viewer.setCameraPosition(0, -100, -300, 1, 0, 0, 0, 0, 1);
    viewer.addLine<PointXYZ>(rcp_p1, rla_p1, 255, 0, 0, "line1");
    viewer.addLine<PointXYZ>(rcp_p1, rra_p1, 255, 0, 0, "line2");
    viewer.addLine<PointXYZ>(L_bPt, vla_p, 0, 255, 0, "line3");
    viewer.addLine<PointXYZ>(L_bPt, vra_p, 0, 255, 0, "line4");

    viewer.spinOnce();

}

//������������
void CR::buildSampleGrid(const PointXYZ initial_P, int n, double edgeLength, const PointCloud<PointXYZ>::Ptr& grid)
{
    /*initial_P ���ĵ�
    * rowsAndCols ������ n*n
    * edgeLength ����Ԫ��С
    */

    //PointCloud<PointXYZ>::Ptr grid(new PointCloud<PointXYZ>);
    // �����������ʼ��
    double startX = initial_P.x - (n/ 2.0) * edgeLength;
    double startY = initial_P.y - (n/ 2.0) * edgeLength;

    // ��������
    for (int i = 0; i < n+1; ++i) 
    {
        for (int j = 0; j < n+1; ++j) 
        {
            PointXYZ gridPoint;
            gridPoint.x = startX + i * edgeLength;
            gridPoint.y = startY + j * edgeLength;
            gridPoint.z = initial_P.z;
            grid->push_back(gridPoint);
        }
    }
}

//������ʾ
void CR::displayPointCloud(const PointCloud<PointXYZ>::Ptr& cloud)
{
        // �������ӻ�����
        visualization::PCLVisualizer viewer("Point Cloud Viewer");

        //// ��ʾ��ʱ��Ҫ��X����ʱ����ת90�Ȳ�����ȷ�ģ������᷽����Ǻ�Mimics��matlabһ��
        //PointCloud<PointXYZ>::Ptr cloud_transformed(new PointCloud<PointXYZ>);
        //Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();//������X�����ת���󣬲���ʼ��Ϊ��λ��
        //double angle_x = -M_PI/2;//��ת90��
        //rotation_x(1, 1) = cos(angle_x);
        //rotation_x(1, 2) = -sin(angle_x);
        //rotation_x(2, 1) = sin(angle_x);
        //rotation_x(2, 2) = cos(angle_x);
        //pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_x);
        
        // ���õ�����Ⱦ��ɫΪ��ɫ
        visualization::PointCloudColorHandlerCustom<PointXYZ> color_handler(cloud, 0, 255, 0);
        viewer.addPointCloud<PointXYZ>(cloud, color_handler, "cloud");
        // ���ÿ��ӻ�����
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        //���������
        viewer.addCoordinateSystem(50.0, "cloud", 0);
        viewer.setCameraPosition(0, -100, -300, 1, 0, 0, 0, 0, 1);
        
        // ѭ����ʾ���ӻ����ڣ�ֱ���ر�
        while (!viewer.wasStopped()) {
            viewer.spinOnce();
        }
}

//�������½�ģ��-���õ��������������ͽӴ���
PointCloud<PointXYZ>::Ptr CR::downSimulationToFindBC(const PointXYZ initial_P, const PointCloud<PointXYZ>::Ptr& cloud, int radius)
{

    double  downDis = std::numeric_limits<double>::max();
    PointCloud<PointXYZ>::Ptr bcPt(new PointCloud<PointXYZ>);
    bcPt->push_back(PointXYZ(0, 0, 0));
    bcPt->push_back(PointXYZ(0, 0, 0));

    PointXYZ contactP;// �Ӵ���
    PointXYZ bottomP;// ����������

    for (const auto& cloudPoint : cloud->points)
    {
        double distance = pcl::euclideanDistance(initial_P, cloudPoint);
        double z_dis = initial_P.z - cloudPoint.z;
        double temp_downDis = z_dis - sqrt(pow(radius, 2) + pow(z_dis, 2) - pow(distance, 2));
        if(temp_downDis < downDis)
        {
            downDis = temp_downDis;
            contactP = cloudPoint;
            bottomP.x = initial_P.x;
            bottomP.y = initial_P.y;
            bottomP.z = initial_P.z - downDis;
            bcPt->points[0] = bottomP;
            bcPt->points[1] = contactP;
        }
    }
    return bcPt;
}

// �������Ż�
void CR::optimizedSamples(const PointXYZ origina_centerP, const PointCloud<PointXYZ>::Ptr& sample_Cloud, const PointXYZ cPt, PointCloud<PointXYZ>::Ptr& newSamplePt, PointXYZ& new_centerP)
{
    /*������һ�νӴ���λ���Ż��������������λ��
    */

    if (sample_Cloud->size()==1)
    {
        newSamplePt->clear();
        new_centerP.x = 0;
        new_centerP.y = 0; // ����������
        new_centerP.z = 0;
    }
    else
    {
        newSamplePt->clear();
        double r_dis = pow(origina_centerP.x - cPt.x, 2) + pow(origina_centerP.y - cPt.y, 2);
        //double r_dis = euclideanDistance(origina_centerP, cPt);
        for (const auto& cloudPoint : sample_Cloud->points)
        {
            //double distance = pcl::euclideanDistance(cPt, cloudPoint);
            double distance = pow(cPt.x - cloudPoint.x, 2) + pow(cPt.y - cloudPoint.y, 2);
            if (distance > r_dis)
            {
                newSamplePt->push_back(cloudPoint);
            }
        }

        //ȷ���µ����ĵ�
        double temp_maxDis = std::numeric_limits<double>::max();
        //PointXYZ new_centerP;
        for (const auto& cloudPoint : newSamplePt->points)
        {
            //double distance1 = euclideanDistance(origina_centerP, cloudPoint);
            double distance1 = pow(origina_centerP.x - cloudPoint.x, 2) + pow(origina_centerP.y - cloudPoint.y, 2);
            if (distance1 < temp_maxDis)
            {
                temp_maxDis = distance1;
                new_centerP.x = cloudPoint.x;
                new_centerP.y = cloudPoint.y;
                new_centerP.z = cloudPoint.z;
            }
        }
    }

}

void CR::getVitualContactPt()
{

    auto start = std::chrono::high_resolution_clock::now();//��ʱ��ʼ
    PointXYZ Lb;
    PointXYZ Lc;

    Eigen::Vector4f centroid;  //��ʼ�����ã���������λ�ã�
    compute3DCentroid(*rotated_acetabulum_cloud, centroid);

    PointXYZ first_P;
    first_P.x = centroid[0];
    first_P.y = centroid[1];
    first_P.z = centroid[2] + 2.5 * radius;
    //cout << first_P.z << endl;

    PointXYZ record_BPt(0.0, 0.0, 0.0);
    PointCloud<PointXYZ>::Ptr transformed_cloud_3(new PointCloud<PointXYZ>);
    // ���
    PointCloud<PointXYZ>::Ptr BPTs(new PointCloud<PointXYZ>);
    // �Ӵ���
    PointCloud<PointXYZ>::Ptr CPTs(new PointCloud<PointXYZ>);
    int num = 1;
    double edgeL = 8.0; // ����߳�
    double err;

    while (num <= 8)
    {
        cout << "����������"<<num << endl;
        buildSampleGrid(first_P, 4, edgeL, transformed_cloud_3);
        edgeL = edgeL / 2;
        computeBottomPoint(rotated_acetabulum_cloud, transformed_cloud_3, radius, Lb, Lc);
        BPTs->points.push_back(Lb);
        CPTs->points.push_back(Lc);

        first_P.x = Lb.x;
        first_P.y = Lb.y;
        err = euclideanDistance(Lb, record_BPt);
        record_BPt = Lb;
        if (err < 1e-5)// ������ֵ������������͵�֮��ľ���С��1e-5 ����ѭ��
            break;
        num++;
    }

    auto end = chrono::high_resolution_clock::now();//��ʱ����
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    std::cout << "��ʱ: " << duration << " ����" << std::endl;

    int minZIndex = 0;
    float minZValue = BPTs->points[0].z;
    for (size_t i = 1; i < BPTs->points.size(); ++i)
    {
        if (BPTs->points[i].z < minZValue)
        {
            minZValue = BPTs->points[i].z;
            minZIndex = i;
        }
    }

    PointCloud<PointXYZ>::Ptr PTs(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr N_PTs(new PointCloud<PointXYZ>);
    PTs->points.push_back(BPTs->points[minZIndex]);
    PTs->points.push_back(CPTs->points[minZIndex]);
 
    cout << transformMatrix << endl;
    transformPointCloud(*PTs, *N_PTs, transformMatrix.transpose());

    L_bPt = N_PTs->points[0];
    L_cPt = N_PTs->points[1];
    v_CP = (Mat1d(3, 1) << L_bPt.x, L_bPt.y, L_bPt.z);

    //auto end = chrono::high_resolution_clock::now();//��ʱ����
    //auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    //std::cout << "��ʱ: " << duration << " ����" << std::endl;

    cout << L_bPt << endl;
    cout << L_cPt << endl;
    //displayPointCloud(transformed_cloud_3);
}

//�����ʼR��t
void CR::getRTMatrix(const Mat1d& rLeft_ASIS, const Mat1d& rRight_ASIS, const Mat1d& real_CP, Mat1d& R_rotation)
{
    // ����ռ�
    Mat1d temp_v1;
    temp_v1.push_back(vLeft_ASIS.t());
    temp_v1.push_back(vRight_ASIS.t());
    temp_v1.push_back(v_CP.t());
    temp_v1 = temp_v1.t();
    
    //��ʵ�ռ�
    Mat1d temp_r1;
    temp_r1.push_back(rLeft_ASIS.t());
    temp_r1.push_back(rRight_ASIS.t());
    temp_r1.push_back(real_CP.t());
    temp_r1 = temp_r1.t();

    //ƽ������
    Mat1d t = v_CP - real_CP;
    Mat1d temp_t1;
    repeat(t, 1, 3, temp_t1);

    //��ʵ����ת������ռ�  **ע����ת����->�Ը�������ϵԭ��
    Mat1d temp_r2 = temp_r1 + temp_t1;
    Mat1d  temp_t2;
    repeat(v_CP, 1, 3, temp_t2);

    // ƽ�Ƶ���ʵ�ռ��ԭ��
    Mat1d temp_v2 = temp_v1 - temp_t2;
    Mat1d temp_r3 = temp_r2 - temp_t2;

    //��������ռ�����
    Mat1d vec1 = temp_v2.col(0);
    Mat1d vec2 = temp_v2.col(1);
    Mat1d cross_v1v2 = vec1.cross(vec2);

    //������ʵ�ռ�����
    Mat1d rVec1 = temp_r3.col(0);
    Mat1d rVec2 = temp_r3.col(1);
    Mat1d cross_r1r2 = rVec1.cross(rVec2);

    //��λ��
    //normalize(vec1, vec1);
    //normalize(vec2, vec2);
    //normalize(rVec1, rVec1);
    //normalize(rVec2, rVec2);
    //normalize(cross_v1v2, cross_v1v2);
    //normalize(cross_r1r2, cross_r1r2);

    //������תR
    Mat1d V_M;
    V_M.push_back(vec1.t());
    V_M.push_back(vec2.t());

    Mat1d R_M;
    R_M.push_back(rVec1.t());
    R_M.push_back(rVec2.t());

    Mat1d H = V_M.t() * R_M;
    Mat1d w, u, vt;
    SVD::compute(H, w, u, vt, SVD::FULL_UV);
    Mat1d R = u * vt;

    if (determinant(R) < 0)
    {
        vt.row(2) = -vt.row(2);
        R = u * vt;
    }

    R_rotation = R;
}

// ɸѡԲ���������
PointCloud<PointXYZ>::Ptr CR::extractPointsInCylinder(const PointCloud<PointXYZ>::Ptr or_cloud, const PointXYZ center, double radius)
{

    PointCloud<PointXYZ>::Ptr cylinder_cloud(new PointCloud<PointXYZ>);
    for (const auto& point : or_cloud->points)
    {
        double distance = pow(point.x - center.x, 2) + pow(point.y - center.y, 2);
        if (distance <= pow(radius, 2))
        {
            cylinder_cloud->points.push_back(point);
        }
    }

    return cylinder_cloud;
}

// ������͵�
void CR::computeBottomPoint(const PointCloud<PointXYZ>::Ptr ac_cloud, const PointCloud<PointXYZ>::Ptr& grid, double radius, PointXYZ &Lowest_bPt, PointXYZ &Lowest_cPt)
{
   /*input:
   * ���������� sample_grid
   * �žʵ��ƣ� cloud
   * ��ͷ�뾶�� radius
   * 
   * output:
   * ���: bottomP
   * �Ӵ��㣺contactP
   */

    //��ʼֵ-�� ��ʼ����������λ�ã������������Ų飩
    PointXYZ or_center = grid->points[12];
    // �²�������λ��
    PointXYZ new_center;
    PointCloud<PointXYZ>::Ptr temp_grid(new PointCloud<PointXYZ>);
    temp_grid = grid;
    int n = 1;// ��¼�Ż�����

    //��״���ƶ���
    PointCloud<PointXYZ>::Ptr columnCloud(new PointCloud<PointXYZ>);
    //��غͽӴ���
    PointCloud<PointXYZ>::Ptr bcPTs(new PointCloud<PointXYZ>);
    // �������
    PointCloud<PointXYZ>::Ptr allBPTs(new PointCloud<PointXYZ>);
    // ���нӴ���
    PointCloud<PointXYZ>::Ptr allCPTs(new PointCloud<PointXYZ>);
    //�µĲ��������
    PointCloud<PointXYZ>::Ptr new_gridPts(new PointCloud<PointXYZ>);
    
    while (1)
    {
        //auto start = std::chrono::high_resolution_clock::now();//��ʱ��ʼ
        columnCloud = extractPointsInCylinder(ac_cloud, or_center, radius);

        //auto end = chrono::high_resolution_clock::now();//��ʱ����
        //auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        //std::cout << "��ʱ: " << duration << " ����" << std::endl;

        bcPTs = downSimulationToFindBC(or_center, columnCloud, radius);
        allBPTs->points.push_back(bcPTs->points[0]);
        allCPTs->points.push_back(bcPTs->points[1]);

        //����sampleGrid
        PointXYZ contactP = bcPTs->points[1];
        optimizedSamples(or_center, temp_grid, contactP, new_gridPts, new_center);
        //cout << n << endl;
        //cout << "size:" << temp_grid->size()<<endl;
        n++;
        if (new_gridPts->empty())
        {
            break;
        }
        or_center = new_center;
        *temp_grid = *new_gridPts;
    }

    if (allBPTs->empty()) 
    {
        cout << "The point cloud is empty." << std::endl;
    }

    //�������в�������������ͽӴ���  Ѱ��������Ͷ�Ӧ�ĽӴ���
    int minZIndex = 0;
    float minZValue = allBPTs->points[0].z;
    for (size_t i = 1; i < allBPTs->points.size(); ++i) 
    {
        if (allBPTs->points[i].z < minZValue) 
        {
            minZValue = allBPTs->points[i].z;
            minZIndex = i;
        }
    }

    Lowest_bPt = allBPTs->points[minZIndex];
    Lowest_cPt = allCPTs->points[minZIndex];
}
